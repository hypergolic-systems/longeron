// LongeronSceneDriver — the per-FixedUpdate driver bracketing every
// flight tick.
//
// Per-tick flow:
//   - Reconcile any pending topology mutations (decouple, dock,
//     joint break, body destroy queue from JoltPart/QuadBody/
//     StaticBody OnDestroy) into the bridge input buffer.
//   - Per-vessel pre-step setup: re-apply kinematic takeover (the
//     OnGoOffRails one-shot can race Part.Unpack and silently miss
//     parts), emit MassUpdates when total mass crosses threshold.
//   - Step the bridge by Time.fixedDeltaTime.
//   - Drain output: per-vessel BodyPose → propagate to all member
//     parts via captured local offsets; refresh derived velocity
//     fields.
//
// Static collision (KSC + PQS terrain) is owned elsewhere:
// PQSStreamer mirrors PQS terrain quads, StaticSceneStreamer mirrors
// PQSCity static prefabs. This driver owns vessel motion only.

using Longeron.Integration;
using Longeron.Native;
using UnityEngine;

namespace Longeron
{
    [DefaultExecutionOrder(10000)]
    public class LongeronSceneDriver : MonoBehaviour
    {
        const string LogPrefix = "[Longeron/driver] ";

        // Phase 4 verbose RNEA logging: dump every per-edge wrench for
        // the active vessel every kVerboseLogStride ticks. Aggressive
        // detail for diagnosing why specific joints show unexpected
        // values (suspicious bending at top of stack, contact-noise
        // jitter, etc.). Set false to silence.
        public static bool _verboseRneaLog = true;
        const int kVerboseLogStride = 5;   // ~10 Hz at 50 Hz physics
        static long _rneaTickCounter = 0;

        // Phase 5 diag: count ticks per body since first BodyMassDiag
        // record (= just after BodyCreate / topology rebuild) so we can
        // log body kinematics for the first ~30 ticks of each body's
        // life. Used to spot post-decouple body torque / angular vel
        // spikes that drive ABA flex divergence.
        static readonly System.Collections.Generic.Dictionary<uint, int> _bodyDiagTickCount =
            new System.Collections.Generic.Dictionary<uint, int>();

        // Per-session contact-report log budget (diagnostic only).
        // 3 s × 50 Hz × ~5 contact pairs = ~750. Bump to 2000 so the
        // verbose-launch window captures every contact.
        static int _contactDiagBudget = 2000;

        // Resolve a Jolt body handle to a human-readable name for
        // contact-report logging. Cached so repeated lookups during
        // the contact-spammy verbose-launch window stay cheap; the
        // cache is invalidated on world rebuild via NotifyWorldCreated.
        static readonly System.Collections.Generic.Dictionary<uint, string> _bodyNameCache =
            new System.Collections.Generic.Dictionary<uint, string>();

        static string ResolveBodyName(uint handleId)
        {
            if (_bodyNameCache.TryGetValue(handleId, out var cached)) return cached;

            string name = "?";
            if (JoltPart.TryGet(handleId, out var jp) && jp != null && jp.Part != null)
            {
                var v = jp.Part.vessel;
                name = "vessel:" + (v != null ? v.vesselName : "?");
            }
            else
            {
                foreach (var sb in Object.FindObjectsOfType<StaticBody>())
                {
                    if (sb != null && sb.Handle.Id == handleId)
                    {
                        name = "static:" + sb.gameObject.name;
                        break;
                    }
                }
            }
            _bodyNameCache[handleId] = name;
            return name;
        }

        // Called by LongeronAddon when a new flight-scene world is
        // created so per-world transient state resets.
        internal void NotifyWorldCreated()
        {
            DiagLogger.Clear();
            _bodyNameCache.Clear();
            _contactDiagBudget = 2000;
        }

        public void FixedUpdate()
        {
            var world = LongeronAddon.ActiveWorld;
            if (world == null) return;
            if (!HighLogic.LoadedSceneIsFlight) return;

            float dt = Time.fixedDeltaTime;
            if (dt <= 0f) return;

            // Tick counter for the verbose RNEA log (used in the
            // JointWrench drain below). Incremented before drain so
            // tick==N matches the records produced by step N.
            ++_rneaTickCounter;

            // Phase 3b: anchor Jolt to the active vessel's mainBody-fixed
            // frame. Every bridge write goes through CbFrame.WorldToCb;
            // every read through CbToWorld. CB-frame is rotating and
            // translating in Unity world, but stationary in itself —
            // terrain doesn't drift, landed vessels don't slide,
            // FloatingOrigin / Krakensbane / planet rotation all fall
            // out of the boundary transform.
            var frame = CbFrame.Current();
            if (!frame.IsValid)
            {
                Debug.LogWarning(LogPrefix + "no active CelestialBody for frame; skipping tick");
                return;
            }

            // Topology mutations queued during the prior frame
            // (decouple, couple, dock, joint break, vessel destroy)
            // and any pending body destroys from JoltPart.OnDestroy
            // get reconciled into the bridge first, before per-tick
            // pose / force records ride the same input buffer.
            TopologyReconciler.Reconcile(world.Input, frame);

            // Diagnostic raycast (1 Hz): vessel → terrain, log Jolt /
            // Unity agreement. Useful for verifying the CB-frame mirror
            // is consistent with stock visual terrain.
            Streamer.LogVesselTerrainDiag();

            // Per-vessel pre-step setup.
            foreach (var mv in SceneRegistry.Vessels)
            {
                if (mv.Vessel == null || mv.Vessel.state == Vessel.State.DEAD) continue;
                LongeronVesselModule.ApplyKinematicTakeover(mv.Vessel);
                EmitMassUpdates(mv, world.Input);

                // Phase 2.1: gravity comes through FlightIntegrator's
                // per-body rb.AddForce(integrationAccel·mass) calls,
                // which our RigidbodyForceHooks Harmony patches now
                // redirect into per-tick ForceDelta records. Jolt's
                // global gravity stays at zero — applying it here
                // would double-count.

                // Phase 2.0: Jolt owns pose. Initial position came in
                // via BodyCreate; Jolt integrates from there each tick
                // and we read the result back below.
            }

            // Step.
            try { world.Step(dt); }
            catch (System.Exception ex)
            {
                Debug.LogError(LogPrefix + "world.Step threw: " + ex.GetType().Name + ": " + ex.Message);
                return;
            }

            // Drain output. Single-body model: one BodyPose per vessel.
            // The handle's reverse lookup gives us the vessel root
            // JoltPart (only the owner is registered in _byHandle); we
            // propagate the vessel's pose to every member part by
            // composing with the captured local offsets.
            RecordType type;
            while ((type = world.Output.Next()) != RecordType.None)
            {
                switch (type)
                {
                    case RecordType.BodyPose:
                        world.Output.ReadBodyPose(out var pose);
                        if (JoltPart.TryGet(pose.Body.Id, out var ownerJb)
                            && ownerJb.Part != null
                            && ownerJb.Part.vessel != null)
                        {
                            // Phase 5 diag: log body angular velocity for the
                            // first 30 ticks after a body comes online so we
                            // can see whether the body itself spins up
                            // post-decouple (= spurious torque) or stays
                            // rotation-stable (= flex divergence is per-part).
                            if (_bodyDiagTickCount.TryGetValue(pose.Body.Id, out int diagTicks))
                            {
                                if (diagTicks < 500)
                                {
                                    float lvm = Mathf.Sqrt(
                                        pose.LinX * pose.LinX
                                        + pose.LinY * pose.LinY
                                        + pose.LinZ * pose.LinZ);
                                    float avm = Mathf.Sqrt(
                                        pose.AngX * pose.AngX
                                        + pose.AngY * pose.AngY
                                        + pose.AngZ * pose.AngZ);
                                    Debug.Log("[Longeron/body-kin] " + string.Format(
                                        "body={0} t={1} v=({2:F2},{3:F2},{4:F2}) |v|={5:F2} "
                                        + "ω=({6:F3},{7:F3},{8:F3}) |ω|={9:F3}",
                                        pose.Body.Id, diagTicks,
                                        pose.LinX, pose.LinY, pose.LinZ, lvm,
                                        pose.AngX, pose.AngY, pose.AngZ, avm));
                                }
                                _bodyDiagTickCount[pose.Body.Id] = diagTicks + 1;
                            }
                            ApplyVesselPose(ownerJb.Part.vessel, pose, frame);
                        }
                        break;
                    case RecordType.ContactReport:
                        world.Output.ReadContactReport(out var cr);
                        // Diag: log contacts with body-identity resolution.
                        if (_contactDiagBudget > 0)
                        {
                            _contactDiagBudget--;
                            string nameA = ResolveBodyName(cr.BodyA.Id);
                            string nameB = ResolveBodyName(cr.BodyB.Id);
                            Debug.Log("[Longeron/contact] a=" + cr.BodyA.Id + "(" + nameA
                                + ") b=" + cr.BodyB.Id + "(" + nameB + ")"
                                + " p=(" + cr.PointX.ToString("F3") + "," + cr.PointY.ToString("F3") + "," + cr.PointZ.ToString("F3")
                                + ") n=(" + cr.NormalX.ToString("F3") + "," + cr.NormalY.ToString("F3") + "," + cr.NormalZ.ToString("F3")
                                + ") depth=" + cr.Depth.ToString("F4") + " imp=" + cr.Impulse.ToString("F2"));
                        }
                        break;
                    case RecordType.AbaPartDiag:
                        world.Output.ReadAbaPartDiag(out var apd);
                        Debug.Log("[Longeron/aba-tick] " + string.Format(
                            "body={0} t={1} part={2} "
                            + "ext=({3:+0.0;-0.0},{4:+0.0;-0.0},{5:+0.0;-0.0})kN "
                            + "iner=({6:+0.0;-0.0},{7:+0.0;-0.0},{8:+0.0;-0.0})kN "
                            + "flex=({9:+0.0;-0.0},{10:+0.0;-0.0},{11:+0.0;-0.0})kN "
                            + "dp=({12:+0.0000;-0.0000},{13:+0.0000;-0.0000},{14:+0.0000;-0.0000})m |dp|={16:F4} "
                            + "dRot={15:F4}°",
                            apd.Body.Id, apd.Tick, apd.PartIdx,
                            apd.ExtFX, apd.ExtFY, apd.ExtFZ,
                            apd.InertialFX, apd.InertialFY, apd.InertialFZ,
                            apd.FlexFX, apd.FlexFY, apd.FlexFZ,
                            apd.DeltaPosX, apd.DeltaPosY, apd.DeltaPosZ,
                            apd.DeltaAngleRad * Mathf.Rad2Deg,
                            Mathf.Sqrt(apd.DeltaPosX*apd.DeltaPosX + apd.DeltaPosY*apd.DeltaPosY + apd.DeltaPosZ*apd.DeltaPosZ)));
                        break;
                    case RecordType.BodyMassDiag:
                        world.Output.ReadBodyMassDiag(out var bmd);
                        // Start tracking body kinematics for this newly-
                        // registered body's first 30 ticks.
                        _bodyDiagTickCount[bmd.Body.Id] = 0;
                        Debug.Log("[Longeron/com-diag] " + string.Format(
                            "body={0} jolt_auto=({1:F3},{2:F3},{3:F3}) "
                            + "real_mass=({4:F3},{5:F3},{6:F3}) "
                            + "diff=({7:F3},{8:F3},{9:F3}) |diff|={10:F3}",
                            bmd.Body.Id,
                            bmd.JoltAutoComX, bmd.JoltAutoComY, bmd.JoltAutoComZ,
                            bmd.RealComX, bmd.RealComY, bmd.RealComZ,
                            bmd.DiffX, bmd.DiffY, bmd.DiffZ,
                            Mathf.Sqrt(bmd.DiffX * bmd.DiffX
                                       + bmd.DiffY * bmd.DiffY
                                       + bmd.DiffZ * bmd.DiffZ)));
                        break;
                    case RecordType.PartPose:
                        world.Output.ReadPartPose(out var pp);
                        if (JoltPart.TryGet(pp.Body.Id, out var ppOwner)
                            && ppOwner.Part != null
                            && ppOwner.Part.vessel != null
                            && SceneRegistry.TryGet(ppOwner.Part.vessel, out var ppMv)
                            && pp.PartIdx < ppMv.PartsByIdx.Count)
                        {
                            var ppPart = ppMv.PartsByIdx[pp.PartIdx];
                            if (ppPart != null && ppPart.gameObject != null)
                            {
                                var ppJp = ppPart.gameObject.GetComponent<JoltPart>();
                                if (ppJp != null)
                                {
                                    ppJp.FlexLocalPos = new Vector3(
                                        pp.DeltaPosX, pp.DeltaPosY, pp.DeltaPosZ);
                                    ppJp.FlexLocalRot = new Quaternion(
                                        pp.DeltaRotX, pp.DeltaRotY, pp.DeltaRotZ, pp.DeltaRotW);
                                }
                            }
                        }
                        break;
                    case RecordType.JointWrench:
                        world.Output.ReadJointWrench(out var jw);
                        if (JoltPart.TryGet(jw.Body.Id, out var jwOwner)
                            && jwOwner.Part != null
                            && jwOwner.Part.vessel != null
                            && SceneRegistry.TryGet(jwOwner.Part.vessel, out var jwMv)
                            && jw.PartIdx < jwMv.PartsByIdx.Count)
                        {
                            var jwPart = jwMv.PartsByIdx[jw.PartIdx];
                            if (jwPart != null && jwPart.gameObject != null)
                            {
                                var jb = jwPart.gameObject.GetComponent<JoltPart>();
                                if (jb != null)
                                {
                                    jb.LastJointForce  = new Vector3(jw.FX, jw.FY, jw.FZ);
                                    jb.LastJointTorque = new Vector3(jw.TX, jw.TY, jw.TZ);
                                }
                                if (_verboseRneaLog
                                    && (_rneaTickCounter % kVerboseLogStride) == 0
                                    && jwPart.vessel == FlightGlobals.ActiveVessel)
                                {
                                    LogVerboseEdge(_rneaTickCounter, jwPart, jw);
                                }
                            }
                        }
                        break;
                    case RecordType.RneaSummary:
                        world.Output.ReadRneaSummary(out var rnea);
                        if (JoltPart.TryGet(rnea.Body.Id, out var rneaOwner)
                            && rneaOwner.Part != null
                            && rneaOwner.Part.vessel != null)
                        {
                            // Joint forces decomposed in each edge's
                            // reference frame. Compression is benign
                            // (gravity loading a stack); tension /
                            // shear / bending / torsion are the
                            // break-relevant channels.
                            Debug.Log("[Longeron/rnea] " + string.Format(
                                "v='{0}' parts={1} |a|={2:F2}m/s² |α|={3:F3}rad/s² | " +
                                "compr={4:F1}kN@{5} tens={6:F1}kN@{7} shear={8:F1}kN@{9} | " +
                                "tors={10:F2}kN·m@{11} bend={12:F2}kN·m@{13}",
                                rneaOwner.Part.vessel.vesselName, rnea.PartCount,
                                rnea.AccelMag, rnea.AlphaMag,
                                rnea.MaxCompression, rnea.MaxCompressionIdx,
                                rnea.MaxTension,     rnea.MaxTensionIdx,
                                rnea.MaxShear,       rnea.MaxShearIdx,
                                rnea.MaxTorsion,     rnea.MaxTorsionIdx,
                                rnea.MaxBending,     rnea.MaxBendingIdx));
                        }
                        break;
                    default:
                        Debug.LogWarning(LogPrefix + "unexpected output record type " + type);
                        break;
                }
            }

            // After pose readback, refresh each managed vessel's
            // derived velocity fields so the navball / SAS / aero /
            // orbit displays read correct values this same tick.
            foreach (var mv in SceneRegistry.Vessels)
            {
                RefreshVesselVelocityFields(mv.Vessel);
            }

            // Diagnostic capture of part-pose drift relative to the
            // vessel root. Tracks first ~5 s after each vessel registers
            // — useful for spotting whether constraints record the wrong
            // rest pose at construction or leak over time.
            DiagLogger.OnTickStart();
            foreach (var mv in SceneRegistry.Vessels)
            {
                DiagLogger.LogVessel(mv.Vessel);
            }
        }

        // Threshold below which a mass change is too small to be
        // worth round-tripping through the bridge. 1 g (0.001 kg
        // = 1e-6 t) — typical engine fuel-burn deltas are
        // milligrams to grams per tick, so this lets sub-tick
        // accumulation bunch up before we emit.
        const float kMassChangeThresholdTonnes = 1e-6f;

        // Single-body mass aggregation: sum every part's mass +
        // resources, compare to the per-vessel cached total, emit one
        // Verbose per-edge dump for the active vessel. Logged at
        // ~10 Hz so a 30-second flight produces ~300 lines per joint
        // — enough density to spot transients without drowning the
        // log. Format is one line per joint per cadence-tick:
        //   [Longeron/rnea-e] tick=N 'PartName'(←'ParentName')
        //     F(ax,sh1,sh2)=(…)kN T(tor,bn1,bn2)=(…)kN·m
        //     bF=… bT=… fR=… tR=…
        //
        // Where:
        //   ax  = signed axial force (joint frame X) (+compression / -tension).
        //   sh1, sh2 = perpendicular shear components (joint frame Y, Z).
        //   tor = signed torsion (joint frame X torque).
        //   bn1, bn2 = perpendicular bending components.
        //   bF, bT = stock break thresholds for context.
        //   fR, tR = ratios that the StressGauge UI displays.
        //
        // grep "Longeron/rnea-e" KSP.log to extract a flight's worth.
        static void LogVerboseEdge(long tick, Part p, Native.JointWrenchRecord jw)
        {
            string parentName = (p.parent != null)
                ? (p.parent.partInfo != null ? p.parent.partInfo.name : p.parent.name)
                : "<root>";
            string partName = p.partInfo != null ? p.partInfo.name : p.name;

            // Stock-equivalent break thresholds cached on JoltPart.
            // Matches the live ConfigurableJoint.breakForce stock sets
            // (verified via kspcli). Falls back to Part.breakingForce
            // when not yet computed.
            var jp = p.gameObject != null
                ? p.gameObject.GetComponent<JoltPart>() : null;
            float bf = (jp != null && jp.EffectiveBreakForce  > 0f)
                ? jp.EffectiveBreakForce  : p.breakingForce;
            float bt = (jp != null && jp.EffectiveBreakTorque > 0f)
                ? jp.EffectiveBreakTorque : p.breakingTorque;

            float tens  = jw.FX < 0f ? -jw.FX : 0f;
            float shear = Mathf.Sqrt(jw.FY * jw.FY + jw.FZ * jw.FZ);
            float fr = bf > 0f
                ? Mathf.Sqrt(tens * tens + shear * shear) / bf
                : 0f;

            float tors = Mathf.Abs(jw.TX);
            float bend = Mathf.Sqrt(jw.TY * jw.TY + jw.TZ * jw.TZ);
            float tr = bt > 0f
                ? Mathf.Sqrt(tors * tors + bend * bend) / bt
                : 0f;

            float extMag = Mathf.Sqrt(jw.ExtFX * jw.ExtFX
                                       + jw.ExtFY * jw.ExtFY
                                       + jw.ExtFZ * jw.ExtFZ);

            Debug.Log("[Longeron/rnea-e] " + string.Format(
                "tick={0} '{1}'(←'{2}') F(ax,s1,s2)=({3:+0.0;-0.0},{4:+0.0;-0.0},{5:+0.0;-0.0})kN " +
                "T(tor,b1,b2)=({6:+0.00;-0.00},{7:+0.00;-0.00},{8:+0.00;-0.00})kN·m " +
                "ext=({13:+0.0;-0.0},{14:+0.0;-0.0},{15:+0.0;-0.0})kN |ext|={16:F1} " +
                "bF={9:F0} bT={10:F0} fR={11:F2} tR={12:F2}",
                tick, partName, parentName,
                jw.FX, jw.FY, jw.FZ,
                jw.TX, jw.TY, jw.TZ,
                bf, bt, fr, tr,
                jw.ExtFX, jw.ExtFY, jw.ExtFZ, extMag));
        }

        // MassUpdate when the delta crosses the threshold. Per-part
        // jb.LastMass is also updated so the per-tick delta detection
        // remains stable across topology changes.
        static void EmitMassUpdates(Integration.ManagedVessel mv, Native.InputBuffer input)
        {
            if (!mv.Body.IsValid || mv.Vessel == null) return;

            float total = 0f;
            foreach (var part in mv.Vessel.parts)
            {
                if (part == null) continue;
                float m = part.mass + part.GetResourceMass();
                if (m > 0f) total += m;

                var jb = part.gameObject != null
                    ? part.gameObject.GetComponent<JoltPart>()
                    : null;
                if (jb != null) jb.LastMass = m;
            }
            if (total < 1e-6f) total = 0.01f;

            if (System.Math.Abs(total - mv.LastMass) < kMassChangeThresholdTonnes)
                return;

            input.WriteMassUpdate(mv.Body, total);
            mv.LastMass = total;
        }

        // Single-body pose propagation. The BodyPose record carries the
        // vessel-level pose at the body's anchor (= vessel root part's
        // transform position at body-create time). We compose this
        // pose with each part's frozen LocalPos / LocalRot offset to
        // derive the per-part Unity rb pose for stock-code consumers.
        //
        // The vessel is rigid by construction (single body, no internal
        // DOF), so part offsets stay constant until the next topology
        // rebuild.
        //
        // Velocity propagation: rb at offset r from the body's reference
        // point has v(r) = v_anchor + ω × r — standard rigid-body
        // kinematics. We use the vessel's body-frame pose to express r
        // in world axes.
        static void ApplyVesselPose(Vessel v, BodyPoseRecord pose, CbFrame frame)
        {
            Vector3d posCb = new Vector3d(pose.PosX, pose.PosY, pose.PosZ);
            Vector3d posWorld = frame.CbToWorld(posCb);
            QuaternionD rotCb = new QuaternionD(pose.RotX, pose.RotY, pose.RotZ, pose.RotW);
            QuaternionD rotWorldD = frame.CbToWorld(rotCb);
            Quaternion rotWorld = (Quaternion)rotWorldD;

            Vector3d velCb = new Vector3d(pose.LinX, pose.LinY, pose.LinZ);
            Vector3d velWorld = frame.CbVelToWorld(velCb, posWorld);

            Vector3d angCb = new Vector3d(pose.AngX, pose.AngY, pose.AngZ);
            Vector3d angWorld = frame.CbAngVelToWorld(angCb);
            Vector3 angWorldF = (Vector3)angWorld;

            Vector3 anchorWorld = (Vector3)posWorld;
            Vector3 anchorVel = (Vector3)velWorld;

            // Phase 5 diag: log Unity-side anchor + per-part values
            // for the first 500 ticks per body so we can compare
            // Jolt's body kinematics against what stock physics sees.
            bool diagThisTick = false;
            int diagTickIdx = 0;
            if (_bodyDiagTickCount.TryGetValue(pose.Body.Id, out int dtc) && dtc < 500)
            {
                diagThisTick = true;
                diagTickIdx = dtc;
                Debug.Log("[Longeron/unity-anchor] " + string.Format(
                    "body={0} t={1} pose_cb=({2:F2},{3:F2},{4:F2}) "
                    + "anchor_world=({5:F2},{6:F2},{7:F2}) "
                    + "anchor_vel=({8:F2},{9:F2},{10:F2}) |v|={11:F2} "
                    + "alt={12:F1} mainBodyPos=({13:F1},{14:F1},{15:F1})",
                    pose.Body.Id, diagTickIdx,
                    pose.PosX, pose.PosY, pose.PosZ,
                    anchorWorld.x, anchorWorld.y, anchorWorld.z,
                    anchorVel.x, anchorVel.y, anchorVel.z,
                    anchorVel.magnitude,
                    v.altitude,
                    v.mainBody != null ? v.mainBody.position.x : 0.0,
                    v.mainBody != null ? v.mainBody.position.y : 0.0,
                    v.mainBody != null ? v.mainBody.position.z : 0.0));
            }

            foreach (var part in v.parts)
            {
                if (part == null) continue;
                var rb = part.rb;
                if (rb == null) continue;
                var jb = part.gameObject != null
                    ? part.gameObject.GetComponent<JoltPart>()
                    : null;
                if (jb == null) continue;

                // Phase 5 flex composition: vessel pose drives the
                // rigid base; FlexLocalPos / FlexLocalRot (from native
                // ABA) add the per-part deflection on top.
                //   r_world = vesselRot · (PartLocalPos + FlexLocalPos)
                //   rot     = vesselRot · FlexLocalRot · PartLocalRot
                Vector3 localCom = jb.PartLocalPos + jb.FlexLocalPos;
                Vector3 rWorld = rotWorld * localCom;
                Vector3 partPos = anchorWorld + rWorld;
                Quaternion partRot = rotWorld * jb.FlexLocalRot * jb.PartLocalRot;
                Vector3 partVel = anchorVel + Vector3.Cross(angWorldF, rWorld);

                // Capture pre-write rb state so we can compare against
                // what we're about to write — desync between Unity rb
                // and computed pose would show up as a delta here.
                Vector3 rbPosBefore = rb.position;
                Vector3 rbVelBefore = rb.velocity;

                rb.position        = partPos;
                rb.rotation        = partRot;
                rb.velocity        = partVel;
                rb.angularVelocity = angWorldF;
                // With Physics.autoSimulation = false, Unity no longer
                // syncs rb.position → transform.position at the end of
                // the physics step (because the step doesn't run). The
                // renderer reads transform.position, so without an
                // explicit transform write the meshes would freeze in
                // place while the rb pose advances. Write the transform
                // directly to keep the renderer in sync.
                part.transform.position = partPos;
                part.transform.rotation = partRot;

                jb.LastVelocity        = partVel;
                jb.LastAngularVelocity = angWorldF;

                if (diagThisTick && jb.PartIdx != 0xFFFF)
                {
                    Vector3 dPos = partPos - rbPosBefore;
                    Vector3 dVel = partVel - rbVelBefore;
                    Debug.Log("[Longeron/unity-part] " + string.Format(
                        "body={0} t={1} part={2} '{3}' "
                        + "compute_pos=({4:F4},{5:F4},{6:F4}) "
                        + "rb_pos_pre=({7:F4},{8:F4},{9:F4}) "
                        + "Δpos=({10:+0.0000;-0.0000},{11:+0.0000;-0.0000},{12:+0.0000;-0.0000}) |Δp|={28:F4} "
                        + "compute_vel=({13:F2},{14:F2},{15:F2}) "
                        + "rb_vel_pre=({16:F2},{17:F2},{18:F2}) "
                        + "Δvel=({19:+0.00;-0.00},{20:+0.00;-0.00},{21:+0.00;-0.00}) "
                        + "PartLocal=({22:F4},{23:F4},{24:F4}) "
                        + "FlexLocal=({25:F4},{26:F4},{27:F4}) "
                        + "FlexRot=({29:F4},{30:F4},{31:F4},{32:F4}) flexAngDeg={33:F2}",
                        pose.Body.Id, diagTickIdx, jb.PartIdx,
                        part.partInfo != null ? part.partInfo.name : part.name,
                        partPos.x, partPos.y, partPos.z,
                        rbPosBefore.x, rbPosBefore.y, rbPosBefore.z,
                        dPos.x, dPos.y, dPos.z,
                        partVel.x, partVel.y, partVel.z,
                        rbVelBefore.x, rbVelBefore.y, rbVelBefore.z,
                        dVel.x, dVel.y, dVel.z,
                        jb.PartLocalPos.x, jb.PartLocalPos.y, jb.PartLocalPos.z,
                        jb.FlexLocalPos.x, jb.FlexLocalPos.y, jb.FlexLocalPos.z,
                        dPos.magnitude,
                        jb.FlexLocalRot.x, jb.FlexLocalRot.y, jb.FlexLocalRot.z, jb.FlexLocalRot.w,
                        Quaternion.Angle(Quaternion.identity, jb.FlexLocalRot)));
                }
            }
        }


        // Refresh vessel-level derived velocity fields from rb state
        // after pose readback. Stock VesselPrecalculate.CalculatePhysicsStats
        // + Vessel.UpdatePosVel runs at execution order 0 — i.e. before
        // our +10000 pose writeback — and again from per-frame Update,
        // so the values would lag at best and get clobbered between our
        // write and the navball read. Replay KSP's logic from
        // ~/dev/ksp-reference/source/Assembly-CSharp/Vessel.cs:3530+
        // and VesselPrecalculate.cs:540+ with current rb state.
        // Public static so VesselUpdatePosVelPatch can call it as a
        // postfix on every stock UpdatePosVel invocation.
        public static void RefreshVesselVelocityFields(Vessel v)
        {
            if (v == null || !v.loaded || v.packed) return;
            if (v.rootPart == null || v.rootPart.rb == null) return;
            if (v.orbit == null || v.mainBody == null) return;

            // Read velocity from JoltPart rather than rb.velocity:
            // Harmony postfixes on extern get_velocity aren't always
            // honored under Mono, so rb.velocity may still return 0 for
            // kinematic bodies. JoltPart.LastVelocity reflects what
            // ApplyPoseToRigidbody set on rb.velocity.
            //
            // With Krakensbane patched out (FrameVel ≡ 0):
            //   rb.velocity == velocityD (stock formula reduces)
            //
            // The frame interpretation depends on body.inverseRotation:
            //   inverseRotation=true (low alt):  rb.velocity is in the
            //     surface-rotating frame. Stock convention here is
            //     velocityD = rb.velocity = "surface velocity"
            //     (rotating frame); obt_velocity = velocityD + ω×r.
            //   inverseRotation=false (high alt): rb.velocity is in the
            //     inertial frame. velocityD = rb.velocity = inertial;
            //     obt_velocity = velocityD; srf_velocity = velocityD - ω×r.
            //
            // Stock derives srf/obt via the orbit (Vessel.cs:3530); we
            // shortcut from rb.velocity since the orbit isn't always
            // current at our +10000 execution order.
            var rootRb = v.rootPart.rb;
            var rootJb = rootRb.GetComponent<JoltPart>();
            Vector3d rbVel = rootJb != null
                ? (Vector3d)rootJb.LastVelocity
                : (Vector3d)rootRb.velocity;
            v.rb_velocity = (Vector3)rbVel;
            v.rb_velocityD = rbVel;
            v.velocityD = rbVel;

            Vector3d rotVelAtCoM = v.mainBody.getRFrmVel(v.CoMD);
            if (v.mainBody.inverseRotation)
            {
                // Unity world is the rotating frame; rb.velocity is
                // surface velocity directly.
                v.srf_velocity = v.velocityD;
                v.obt_velocity = v.velocityD + rotVelAtCoM;
            }
            else
            {
                // Unity world is inertial; subtract surface rotation.
                v.obt_velocity = v.velocityD;
                v.srf_velocity = v.velocityD - rotVelAtCoM;
            }
            v.obt_speed = v.obt_velocity.magnitude;
            v.upAxis = (v.CoMD - v.mainBody.position).normalized;
            v.verticalSpeed = Vector3d.Dot(v.srf_velocity, v.upAxis);
            double sqrMag = v.srf_velocity.sqrMagnitude;
            if (sqrMag > 0.0)
            {
                v.srfSpeed = System.Math.Sqrt(sqrMag);
                v.srf_vel_direction = v.srf_velocity / v.srfSpeed;
                double horiz = sqrMag - v.verticalSpeed * v.verticalSpeed;
                v.horizontalSrfSpeed = (horiz > 0.0 && !double.IsNaN(horiz))
                                        ? System.Math.Sqrt(horiz) : 0.0;
            }
            else
            {
                v.srfSpeed = 0.0;
                v.horizontalSrfSpeed = 0.0;
                v.srf_vel_direction = Vector3d.zero;
            }

            // Angular velocity in vessel-local frame (matches
            // VesselPrecalculate:645). Same JoltPart-direct approach as
            // linear velocity above.
            Vector3 rootAngV = rootJb != null
                ? rootJb.LastAngularVelocity
                : rootRb.angularVelocity;
            v.angularVelocity = Quaternion.Inverse(v.ReferenceTransform.rotation)
                              * rootAngV;
            v.angularVelocityD = v.angularVelocity;
        }

    }
}
