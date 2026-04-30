using System;
using System.Runtime.InteropServices;

namespace Longeron.Native
{
    /// <summary>
    /// Body pose record from <c>longeron_step</c>. Payload only — the
    /// 1-byte tag is consumed by <see cref="OutputBuffer.Next"/> before
    /// the read.
    /// </summary>
    public struct BodyPoseRecord
    {
        public BodyHandle Body;
        public double PosX, PosY, PosZ;
        public float  RotX, RotY, RotZ, RotW;
        public float  LinX, LinY, LinZ;
        public float  AngX, AngY, AngZ;
    }

    /// <summary>
    /// Per-edge joint wrench in the joint's reference frame. Emitted
    /// every tick; the scene driver stashes these on each part's
    /// <see cref="JoltPart"/>.
    ///
    /// <para><c>Force.x</c> = signed axial (+ compression, − tension);
    /// <c>(Force.y, Force.z)</c> is the shear vector with a stable
    /// orthonormal basis perpendicular to the joint axis (parent CoM
    /// → child joint anchor). The Y/Z split is implementation-defined
    /// but stable across ticks for a given joint topology.</para>
    ///
    /// <para><c>Torque.x</c> = signed torsion (twist around the joint
    /// axis); <c>(Torque.y, Torque.z)</c> is the bending moment.</para>
    /// </summary>
    public struct JointWrenchRecord
    {
        public BodyHandle Body;       // vessel body
        public ushort     PartIdx;    // tree index of the child part
        public float      FX, FY, FZ; // joint-frame force: X axial (signed), YZ shear
        public float      TX, TY, TZ; // joint-frame torque: X torsion (signed), YZ bending
        public float      ExtFX, ExtFY, ExtFZ; // diag: per-part external force this tick,
                                                //   body-local axes (gravity + thrust + drag
                                                //   + contact lambdas).
    }

    /// <summary>
    /// Per-vessel RNEA summary, decomposed in each joint's reference
    /// frame (Phase 4 advisory). Joint axis = parent CoM → child
    /// joint anchor in body-local. Compression = F·axis when positive;
    /// tension = -F·axis when negative; shear = perpendicular F.
    /// Torsion = T·axis; bending = perpendicular T. Only tension /
    /// shear / torsion / bending are candidates for breakForce /
    /// breakTorque comparison — compression doesn't break joints.
    /// </summary>
    public struct RneaSummaryRecord
    {
        public BodyHandle Body;
        public ushort PartCount;
        public float  MaxCompression;     public ushort MaxCompressionIdx;
        public float  MaxTension;         public ushort MaxTensionIdx;
        public float  MaxShear;           public ushort MaxShearIdx;
        public float  MaxTorsion;         public ushort MaxTorsionIdx;
        public float  MaxBending;         public ushort MaxBendingIdx;
        public float  AccelMag;
        public float  AlphaMag;
    }

    /// <summary>
    /// Per-part flex pose from the Phase 5 ABA forward pass, expressed
    /// in vessel-body-local axes. Identity when the part is at rest;
    /// non-zero under flex. Scene driver composes with the part's
    /// PartLocalPos/Rot to produce the Unity rb's world pose.
    /// </summary>
    public struct PartPoseRecord
    {
        public BodyHandle Body;       // vessel body
        public ushort     PartIdx;    // tree index of the part
        public float      DeltaPosX, DeltaPosY, DeltaPosZ;
        public float      DeltaRotX, DeltaRotY, DeltaRotZ, DeltaRotW;
    }

    /// <summary>
    /// Contact report record from <c>longeron_step</c>. Phase 1 emits
    /// one report per body pair per tick, at the deepest contact point.
    /// </summary>
    public struct ContactReportRecord
    {
        public BodyHandle BodyA;
        public BodyHandle BodyB;
        public double PointX, PointY, PointZ;
        public float  NormalX, NormalY, NormalZ;
        public float  Depth;
        public float  Impulse;
    }

    /// <summary>
    /// Read-only cursor over the output record stream produced by
    /// <c>longeron_step</c>. Backed by a pinned native buffer; the C#
    /// caller iterates records via <see cref="Next"/> and reads typed
    /// payloads via the corresponding <c>Read…</c> methods. No managed
    /// allocations on the hot path.
    /// </summary>
    public sealed unsafe class OutputBuffer : IDisposable
    {
        private byte* _ptr;
        private int   _capacity;
        private int   _len;     // bytes actually written by the bridge
        private int   _cursor;  // current read offset

        public OutputBuffer(int capacityBytes)
        {
            if (capacityBytes <= 0) throw new ArgumentOutOfRangeException(nameof(capacityBytes));
            _ptr      = (byte*)Marshal.AllocHGlobal(capacityBytes);
            _capacity = capacityBytes;
            _len      = 0;
            _cursor   = 0;
        }

        public byte* Pointer  => _ptr;
        public int   Capacity => _capacity;
        public int   Length   => _len;
        public int   Cursor   => _cursor;

        /// <summary>
        /// Called by <see cref="World.Step"/> to set the actual byte
        /// length the bridge wrote, and reset the read cursor.
        /// </summary>
        internal void OnStepReturned(int actualLen)
        {
            _len    = actualLen;
            _cursor = 0;
        }

        public void Dispose()
        {
            if (_ptr != null)
            {
                Marshal.FreeHGlobal((IntPtr)_ptr);
                _ptr = null;
            }
        }

        /// <summary>
        /// Returns the next record's type and consumes its 1-byte tag,
        /// or <see cref="RecordType.None"/> if the stream is exhausted.
        /// Caller must then call the matching <c>Read…</c> method to
        /// consume the payload, or <see cref="SkipUnknown"/> to skip
        /// (currently throws — schema is not forwards-compatible at the
        /// reader without explicit lengths).
        /// </summary>
        public RecordType Next()
        {
            if (_cursor >= _len) return RecordType.None;
            return (RecordType)_ptr[_cursor++];
        }

        public void ReadBodyPose(out BodyPoseRecord record)
        {
            const int kPayload = 68;  // user_id(4) + pos(24) + rot(16) + lin(12) + ang(12)
            EnsureBytes(kPayload);
            byte* p = _ptr + _cursor;
            record = default;
            record.Body = new BodyHandle(*(uint*)p);   p += 4;
            record.PosX = *(double*)p;                 p += 8;
            record.PosY = *(double*)p;                 p += 8;
            record.PosZ = *(double*)p;                 p += 8;
            record.RotX = *(float*)p;                  p += 4;
            record.RotY = *(float*)p;                  p += 4;
            record.RotZ = *(float*)p;                  p += 4;
            record.RotW = *(float*)p;                  p += 4;
            record.LinX = *(float*)p;                  p += 4;
            record.LinY = *(float*)p;                  p += 4;
            record.LinZ = *(float*)p;                  p += 4;
            record.AngX = *(float*)p;                  p += 4;
            record.AngY = *(float*)p;                  p += 4;
            record.AngZ = *(float*)p;                  p += 4;
            _cursor += kPayload;
        }

        public void ReadJointWrench(out JointWrenchRecord record)
        {
            const int kPayload = 42;  // body_id(4) + part_idx(2)
                                       // + force(float3=12) + torque(float3=12)
                                       // + ext_force(float3=12)
            EnsureBytes(kPayload);
            byte* p = _ptr + _cursor;
            record = default;
            record.Body    = new BodyHandle(*(uint*)p);    p += 4;
            record.PartIdx = *(ushort*)p;                  p += 2;
            record.FX = *(float*)p;                        p += 4;
            record.FY = *(float*)p;                        p += 4;
            record.FZ = *(float*)p;                        p += 4;
            record.TX = *(float*)p;                        p += 4;
            record.TY = *(float*)p;                        p += 4;
            record.TZ = *(float*)p;                        p += 4;
            record.ExtFX = *(float*)p;                     p += 4;
            record.ExtFY = *(float*)p;                     p += 4;
            record.ExtFZ = *(float*)p;                     p += 4;
            _cursor += kPayload;
        }

        public void ReadPartPose(out PartPoseRecord record)
        {
            const int kPayload = 34;  // body_id(4) + part_idx(2)
                                       // + delta_pos(float3=12) + delta_rot(float4=16)
            EnsureBytes(kPayload);
            byte* p = _ptr + _cursor;
            record = default;
            record.Body      = new BodyHandle(*(uint*)p);   p += 4;
            record.PartIdx   = *(ushort*)p;                 p += 2;
            record.DeltaPosX = *(float*)p;                  p += 4;
            record.DeltaPosY = *(float*)p;                  p += 4;
            record.DeltaPosZ = *(float*)p;                  p += 4;
            record.DeltaRotX = *(float*)p;                  p += 4;
            record.DeltaRotY = *(float*)p;                  p += 4;
            record.DeltaRotZ = *(float*)p;                  p += 4;
            record.DeltaRotW = *(float*)p;                  p += 4;
            _cursor += kPayload;
        }

        public void ReadRneaSummary(out RneaSummaryRecord record)
        {
            const int kPayload = 44;  // body_id(4) + part_count(2)
                                       // + 5 × (float(4) + idx(2)) = 30
                                       // + accel(4) + alpha(4)
            EnsureBytes(kPayload);
            byte* p = _ptr + _cursor;
            record = default;
            record.Body              = new BodyHandle(*(uint*)p);   p += 4;
            record.PartCount         = *(ushort*)p;                 p += 2;
            record.MaxCompression    = *(float*)p;                  p += 4;
            record.MaxCompressionIdx = *(ushort*)p;                 p += 2;
            record.MaxTension        = *(float*)p;                  p += 4;
            record.MaxTensionIdx     = *(ushort*)p;                 p += 2;
            record.MaxShear          = *(float*)p;                  p += 4;
            record.MaxShearIdx       = *(ushort*)p;                 p += 2;
            record.MaxTorsion        = *(float*)p;                  p += 4;
            record.MaxTorsionIdx     = *(ushort*)p;                 p += 2;
            record.MaxBending        = *(float*)p;                  p += 4;
            record.MaxBendingIdx     = *(ushort*)p;                 p += 2;
            record.AccelMag          = *(float*)p;                  p += 4;
            record.AlphaMag          = *(float*)p;                  p += 4;
            _cursor += kPayload;
        }

        public void ReadContactReport(out ContactReportRecord record)
        {
            const int kPayload = 52;  // a(4) + b(4) + point(24) + normal(12) + depth(4) + impulse(4)
            EnsureBytes(kPayload);
            byte* p = _ptr + _cursor;
            record = default;
            record.BodyA   = new BodyHandle(*(uint*)p);   p += 4;
            record.BodyB   = new BodyHandle(*(uint*)p);   p += 4;
            record.PointX  = *(double*)p;                 p += 8;
            record.PointY  = *(double*)p;                 p += 8;
            record.PointZ  = *(double*)p;                 p += 8;
            record.NormalX = *(float*)p;                  p += 4;
            record.NormalY = *(float*)p;                  p += 4;
            record.NormalZ = *(float*)p;                  p += 4;
            record.Depth   = *(float*)p;                  p += 4;
            record.Impulse = *(float*)p;                  p += 4;
            _cursor += kPayload;
        }

        private void EnsureBytes(int n)
        {
            if (_cursor + n > _len)
                throw new InvalidOperationException(
                    $"Output stream truncated: need {n} bytes at offset {_cursor}, only {_len - _cursor} available.");
        }
    }
}
