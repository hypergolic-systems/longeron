// Test rig for ABA scenarios. Owns a Longeron World, builds vessels
// via fluent API, runs steps, and exposes per-part / per-edge state
// from drained output records.
//
// Usage:
//   using (var rig = new AbaTestRig()) {
//       var v = rig.Vessel()
//           .At(0, 100, 0)
//           .Root(mass: 1f)
//           .Child(parentIdx: 0, mass: 1f, ...)
//           .Build();
//       for (int i = 0; i < N; ++i) rig.Step(1f/50f);
//       var pose = rig.PartPose(v.Id, 1);
//       // ...
//   }

using System;
using System.Collections.Generic;
using Longeron.Native;

namespace Longeron.Native.Probe
{
    public sealed class AbaTestRig : IDisposable
    {
        private readonly World _world;
        private readonly Dictionary<long, PartPoseRecord>     _partPose    = new Dictionary<long, PartPoseRecord>();
        private readonly Dictionary<long, JointWrenchRecord>  _jointWrench = new Dictionary<long, JointWrenchRecord>();
        private readonly Dictionary<uint, BodyPoseRecord>     _bodyPose    = new Dictionary<uint, BodyPoseRecord>();
        private readonly List<AbaPartDiagRecord>              _abaDiags    = new List<AbaPartDiagRecord>();
        private uint _nextBodyId = 100;

        public AbaTestRig()
        {
            _world = new World(LongeronConfig.Default);
        }

        public World World => _world;

        public uint AllocBodyId() => _nextBodyId++;

        public VesselBuilder Vessel(uint? id = null) =>
            new VesselBuilder(this, id ?? AllocBodyId());

        /// <summary>
        /// Step the world by <paramref name="dt"/> seconds and drain
        /// output records into per-tick state caches.
        /// </summary>
        public void Step(float dt)
        {
            _world.Step(dt);
            DrainOutput();
        }

        /// <summary>Most recent PartPose for (vessel, part), or default if none this tick.</summary>
        public PartPoseRecord PartPose(uint bodyId, ushort partIdx) =>
            _partPose.TryGetValue(Key(bodyId, partIdx), out var v) ? v : default;

        public bool HasPartPose(uint bodyId, ushort partIdx) =>
            _partPose.ContainsKey(Key(bodyId, partIdx));

        public JointWrenchRecord JointWrench(uint bodyId, ushort partIdx) =>
            _jointWrench.TryGetValue(Key(bodyId, partIdx), out var v) ? v : default;

        public BodyPoseRecord BodyPose(uint bodyId) =>
            _bodyPose.TryGetValue(bodyId, out var v) ? v : default;

        public bool HasBodyPose(uint bodyId) => _bodyPose.ContainsKey(bodyId);

        public IReadOnlyList<AbaPartDiagRecord> AbaDiags => _abaDiags;

        public void Dispose() => _world.Dispose();

        private static long Key(uint bodyId, ushort partIdx) =>
            ((long)bodyId << 16) | partIdx;

        private void DrainOutput()
        {
            // Per-tick caches reflect the most recent step only.
            // _abaDiags accumulates across ticks (Aba diag is bounded
            // to the first kAbaDiagWindow ticks per body anyway).
            _partPose.Clear();
            _jointWrench.Clear();
            _bodyPose.Clear();

            RecordType type;
            while ((type = _world.Output.Next()) != RecordType.None)
            {
                switch (type)
                {
                    case RecordType.BodyPose:
                        _world.Output.ReadBodyPose(out var bp);
                        _bodyPose[bp.Body.Id] = bp;
                        break;
                    case RecordType.PartPose:
                        _world.Output.ReadPartPose(out var pp);
                        _partPose[Key(pp.Body.Id, pp.PartIdx)] = pp;
                        break;
                    case RecordType.JointWrench:
                        _world.Output.ReadJointWrench(out var jw);
                        _jointWrench[Key(jw.Body.Id, jw.PartIdx)] = jw;
                        break;
                    case RecordType.AbaPartDiag:
                        _world.Output.ReadAbaPartDiag(out var ad);
                        _abaDiags.Add(ad);
                        break;
                    case RecordType.RneaSummary:
                        _world.Output.ReadRneaSummary(out _);
                        break;
                    case RecordType.BodyMassDiag:
                        _world.Output.ReadBodyMassDiag(out _);
                        break;
                    case RecordType.ContactReport:
                        _world.Output.ReadContactReport(out _);
                        break;
                    default:
                        throw new InvalidOperationException($"AbaTestRig: unhandled record type {type}");
                }
            }
        }
    }

    /// <summary>
    /// Fluent builder for a single vessel. Holds the BodyHandle and
    /// per-part list as the caller adds them; emits BodyCreate +
    /// VesselTreeUpdate + SubShapeMap on <see cref="Build"/>.
    ///
    /// Convention: ComLocal / AttachLocal are in vessel-root frame at
    /// rest (the "rigid rest" frame, where the vessel is unflexed).
    /// AttachLocal for a child is the joint anchor — the same point
    /// on parent and child at rest.
    /// </summary>
    public sealed class VesselBuilder
    {
        private readonly AbaTestRig _rig;
        public BodyHandle Handle { get; }
        private readonly List<PartSpec> _parts = new List<PartSpec>();
        private double _posX, _posY, _posZ;
        private float _rotX, _rotY, _rotZ, _rotW = 1f;
        private bool _kinematic = false;

        internal VesselBuilder(AbaTestRig rig, uint id)
        {
            _rig = rig;
            Handle = new BodyHandle(id);
        }

        public VesselBuilder At(double x, double y, double z)
        {
            _posX = x; _posY = y; _posZ = z;
            return this;
        }

        public VesselBuilder Rotation(float x, float y, float z, float w)
        {
            _rotX = x; _rotY = y; _rotZ = z; _rotW = w;
            return this;
        }

        /// <summary>
        /// Mark the vessel as kinematic. The Jolt body's CoM motion
        /// stays fixed (a_body finite-diff = 0), so per-part forces
        /// flow into ABA's flex residual without polluting the
        /// inertial-residual subtraction. Use for analytic
        /// equilibrium tests.
        /// </summary>
        public VesselBuilder Kinematic()
        {
            _kinematic = true;
            return this;
        }

        /// <summary>Add the root part. Located at the vessel-root origin by convention.</summary>
        public VesselBuilder Root(float mass,
            float halfX = 0.5f, float halfY = 0.5f, float halfZ = 0.5f)
        {
            _parts.Add(new PartSpec
            {
                ParentIdx    = 0xFFFF,
                Mass         = mass,
                ComLocalX    = 0, ComLocalY = 0, ComLocalZ = 0,
                AttachLocalX = 0, AttachLocalY = 0, AttachLocalZ = 0,
                HalfX        = halfX, HalfY = halfY, HalfZ = halfZ,
                InertiaDiag  = ComputeBoxInertia(mass, halfX, halfY, halfZ),
                KAng         = 0f, CAng = 0f,
            });
            return this;
        }

        /// <summary>
        /// Add a child part with explicit attach point.
        /// <paramref name="parentIdx"/> indexes into already-added parts
        /// (root is index 0). <paramref name="comOffset"/> is the child's
        /// CoM in vessel-root frame at rest. <paramref name="attachOffset"/>
        /// is the joint anchor in vessel-root frame at rest — the point
        /// where this child connects to its parent.
        /// </summary>
        public VesselBuilder Child(
            ushort parentIdx, float mass,
            float comX, float comY, float comZ,
            float attachX, float attachY, float attachZ,
            float kAng, float cAng,
            float halfX = 0.5f, float halfY = 0.5f, float halfZ = 0.5f)
        {
            _parts.Add(new PartSpec
            {
                ParentIdx    = parentIdx,
                Mass         = mass,
                ComLocalX    = comX, ComLocalY = comY, ComLocalZ = comZ,
                AttachLocalX = attachX, AttachLocalY = attachY, AttachLocalZ = attachZ,
                HalfX        = halfX, HalfY = halfY, HalfZ = halfZ,
                InertiaDiag  = ComputeBoxInertia(mass, halfX, halfY, halfZ),
                KAng         = kAng, CAng = cAng,
            });
            return this;
        }

        /// <summary>
        /// Build the vessel — emits BodyCreate + VesselTreeUpdate +
        /// SubShapeMap input records.
        /// </summary>
        public BodyHandle Build()
        {
            if (_parts.Count == 0)
                throw new InvalidOperationException("VesselBuilder: no parts added");
            if (_parts.Count > byte.MaxValue)
                throw new InvalidOperationException($"VesselBuilder: too many parts ({_parts.Count}) — BodyCreate uses u8 shape count");

            float totalMass = 0f;
            foreach (var p in _parts) totalMass += p.Mass;

            // BodyCreate — compound shape, one box sub-shape per part
            // located at its com_local. Per-shape density = part_mass /
            // box_volume so each sub-shape carries the right mass; Jolt's
            // CompoundShape::GetMassProperties() then aggregates body
            // mass / CoM / inertia for free. Dynamic by default (so ABA
            // can ModifyShapes per tick); Kinematic when the test wants
            // the Jolt body's motion frozen (analytic equilibrium tests).
            BodyType bodyType = _kinematic ? BodyType.Kinematic : BodyType.Dynamic;
            _rig.World.Input.BeginBodyCreate(
                Handle, bodyType, Layer.Kinematic,
                _posX, _posY, _posZ,
                _rotX, _rotY, _rotZ, _rotW,
                totalMass, (byte)_parts.Count);
            for (int i = 0; i < _parts.Count; ++i)
            {
                var p = _parts[i];
                float volume = 8f * p.HalfX * p.HalfY * p.HalfZ;
                float density = (volume > 1e-9f && p.Mass > 1e-9f)
                    ? p.Mass / volume
                    : 0f;
                _rig.World.Input.AppendShapeBox(
                    p.ComLocalX, p.ComLocalY, p.ComLocalZ,
                    0, 0, 0, 1,
                    p.HalfX, p.HalfY, p.HalfZ,
                    density: density);
            }

            // VesselTreeUpdate — topology + compliance.
            var nodes = new InputBuffer.VesselTreeNode[_parts.Count];
            for (int i = 0; i < _parts.Count; ++i)
            {
                var p = _parts[i];
                nodes[i] = new InputBuffer.VesselTreeNode
                {
                    ParentIdx     = p.ParentIdx,
                    Mass          = p.Mass,
                    ComLocalX     = p.ComLocalX,    ComLocalY    = p.ComLocalY,    ComLocalZ    = p.ComLocalZ,
                    InertiaDiagX  = p.InertiaDiag.x, InertiaDiagY = p.InertiaDiag.y, InertiaDiagZ = p.InertiaDiag.z,
                    AttachLocalX  = p.AttachLocalX, AttachLocalY = p.AttachLocalY, AttachLocalZ = p.AttachLocalZ,
                    KAng          = p.KAng, CAng = p.CAng,
                };
            }
            _rig.World.Input.WriteVesselTreeUpdate(Handle, nodes);

            // SubShapeMap — 1:1 mapping (one collider per part in the
            // test rig; matches the AppendShapeBox loop above).
            var subShapeMap = new ushort[_parts.Count];
            for (int i = 0; i < _parts.Count; ++i) subShapeMap[i] = (ushort)i;
            _rig.World.Input.WriteSubShapeMap(Handle, subShapeMap);

            return Handle;
        }

        // Solid box principal moments about CoM:
        //   I_xx = m/12 × (4·hy² + 4·hz²) etc. Standard parallel-axis form.
        private static (float x, float y, float z) ComputeBoxInertia(
            float m, float hx, float hy, float hz)
        {
            float ix = m / 12f * 4f * (hy * hy + hz * hz);
            float iy = m / 12f * 4f * (hx * hx + hz * hz);
            float iz = m / 12f * 4f * (hx * hx + hy * hy);
            return (ix, iy, iz);
        }

        private struct PartSpec
        {
            public ushort ParentIdx;
            public float  Mass;
            public float  ComLocalX, ComLocalY, ComLocalZ;
            public float  AttachLocalX, AttachLocalY, AttachLocalZ;
            public float  HalfX, HalfY, HalfZ;
            public (float x, float y, float z) InertiaDiag;
            public float  KAng, CAng;
        }
    }
}
