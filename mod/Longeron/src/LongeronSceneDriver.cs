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

        // Called by LongeronAddon when a new flight-scene world is
        // created so per-world transient state resets.
        internal void NotifyWorldCreated()
        {
            DiagLogger.Clear();
        }

        public void FixedUpdate()
        {
            var world = LongeronAddon.ActiveWorld;
            if (world == null) return;
            if (!HighLogic.LoadedSceneIsFlight) return;

            float dt = Time.fixedDeltaTime;
            if (dt <= 0f) return;

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
                            ApplyVesselPose(ownerJb.Part.vessel, pose, frame);
                        break;
                    case RecordType.ContactReport:
                        world.Output.ReadContactReport(out _);
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

            foreach (var part in v.parts)
            {
                if (part == null) continue;
                var rb = part.rb;
                if (rb == null) continue;
                var jb = part.gameObject != null
                    ? part.gameObject.GetComponent<JoltPart>()
                    : null;
                if (jb == null) continue;

                Vector3 rWorld = rotWorld * jb.PartLocalPos;
                Vector3 partPos = anchorWorld + rWorld;
                Quaternion partRot = rotWorld * jb.PartLocalRot;
                Vector3 partVel = anchorVel + Vector3.Cross(angWorldF, rWorld);

                rb.position        = partPos;
                rb.rotation        = partRot;
                rb.velocity        = partVel;
                rb.angularVelocity = angWorldF;

                jb.LastVelocity        = partVel;
                jb.LastAngularVelocity = angWorldF;
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
