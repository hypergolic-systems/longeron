// LongeronSceneDriver — the per-FixedUpdate driver bracketing every
// flight tick.
//
// Phase 1.5 responsibilities:
//   - Mint a synthetic static ground body the first time we see a
//     managed vessel (no real terrain mirroring yet — that's Phase 3).
//   - Idempotently re-apply rb.isKinematic = true / useGravity = false
//     on every managed part (the OnGoOffRails one-shot can race
//     Part.Unpack and silently miss parts).
//   - Build the per-tick input buffer:
//       SetGravity from vessel.precalc.integrationAccel
//       SetKinematicPose for each managed body, mirrored from the
//         part's Unity Transform
//   - Step the bridge by Time.fixedDeltaTime.
//   - Drain output: log contacts (rate-limited).
//
// Force redirect, mass updates, and pose readback are Phase 2.

using Longeron.Integration;
using Longeron.Native;
using UnityEngine;

namespace Longeron
{
    [DefaultExecutionOrder(10000)]
    public class LongeronSceneDriver : MonoBehaviour
    {
        const string LogPrefix = "[Longeron/driver] ";

        long _tick;
        bool _groundSpawned;

        // Called by LongeronAddon when a new flight-scene world is
        // created so per-world transient state resets.
        internal void NotifyWorldCreated()
        {
            _groundSpawned = false;
        }

        // Contact-log throttle: log at most kLogEveryTicks ticks worth
        // of contacts in detail; otherwise emit a one-line summary
        // each tick. KSP's FixedUpdate is 50 Hz, so 50 → ~1s.
        const int kLogEveryTicks = 50;

        public void FixedUpdate()
        {
            var world = LongeronAddon.ActiveWorld;
            if (world == null) return;
            if (!HighLogic.LoadedSceneIsFlight) return;

            _tick++;
            float dt = Time.fixedDeltaTime;
            if (dt <= 0f) return;

            // Spawn the synthetic ground once we have a vessel anchor.
            if (!_groundSpawned)
                TrySpawnSyntheticGround(world);

            // Per-vessel pre-step setup.
            foreach (var mv in SceneRegistry.Vessels)
            {
                if (mv.Vessel == null || mv.Vessel.state == Vessel.State.DEAD) continue;
                LongeronVesselModule.ApplyKinematicTakeover(mv.Vessel);

                // Phase 2.1: gravity comes through FlightIntegrator's
                // per-body rb.AddForce(integrationAccel·mass) calls,
                // which our RigidbodyForceHooks Harmony patches now
                // redirect into per-tick ForceDelta records. Jolt's
                // global gravity stays at zero — applying it here
                // would double-count.
                if (mv.Vessel == FlightGlobals.ActiveVessel
                    && mv.Vessel.rootPart != null
                    && (_tick % kLogEveryTicks) == 0)
                {
                    var fv = Krakensbane.GetFrameVelocity();
                    var rootPos = mv.Vessel.rootPart.transform.position;
                    Debug.Log(LogPrefix + string.Format(
                        "tick={0} fv=({1:F2},{2:F2},{3:F2}) rootPos=({4:F2},{5:F2},{6:F2})",
                        _tick, fv.x, fv.y, fv.z,
                        rootPos.x, rootPos.y, rootPos.z));
                }

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

            // Drain output. BodyPose records → write each part's
            // Unity rigidbody (kinematic, so position/rotation
            // assignments take effect immediately). ContactReport
            // records → tally + sample-log.
            int contactCount = 0;
            ContactReportRecord firstContact = default;
            bool haveFirst = false;
            int posesWritten = 0;
            RecordType type;
            while ((type = world.Output.Next()) != RecordType.None)
            {
                switch (type)
                {
                    case RecordType.BodyPose:
                        world.Output.ReadBodyPose(out var pose);
                        if (SceneRegistry.TryGetPart(pose.Body.Id, out var part) && part?.rb != null)
                        {
                            var jb = part.gameObject != null
                                       ? part.gameObject.GetComponent<JoltBody>()
                                       : null;
                            ApplyPoseToRigidbody(part.rb, pose, jb);
                            posesWritten++;
                        }
                        break;
                    case RecordType.ContactReport:
                        world.Output.ReadContactReport(out var c);
                        contactCount++;
                        if (!haveFirst) { firstContact = c; haveFirst = true; }
                        break;
                    default:
                        Debug.LogWarning(LogPrefix + "unexpected output record type " + type);
                        break;
                }
            }
            // Suppress unused warning until the Phase 2 pose-readback log lands.
            _ = posesWritten;

            // After pose readback, rb.velocity is current — refresh
            // each managed vessel's derived velocity fields so the
            // navball / SAS / aero / orbit displays read correct
            // values this same tick.
            foreach (var mv in SceneRegistry.Vessels)
            {
                RefreshVesselVelocityFields(mv.Vessel);
            }

            if (contactCount > 0 && (_tick % kLogEveryTicks) == 0)
            {
                Debug.Log(LogPrefix + string.Format(
                    "tick={0} contacts={1} sample: a={2} b={3} pt=({4:F2},{5:F2},{6:F2}) " +
                    "n=({7:F2},{8:F2},{9:F2}) depth={10:F4}",
                    _tick, contactCount,
                    firstContact.BodyA.Id, firstContact.BodyB.Id,
                    firstContact.PointX, firstContact.PointY, firstContact.PointZ,
                    firstContact.NormalX, firstContact.NormalY, firstContact.NormalZ,
                    firstContact.Depth));
            }
        }

        // Write Jolt's integrated pose + velocity onto the Unity
        // Rigidbody. With rb.isKinematic = true, position/rotation
        // assignments take effect on the next physics step (Unity
        // doesn't try to integrate). Velocity writes feed
        // vessel.velocityD / orbit-driver via the
        // OrbitDriverKinematicBypass patch.
        static void ApplyPoseToRigidbody(Rigidbody rb, BodyPoseRecord pose, JoltBody jb)
        {
            rb.position = new Vector3((float)pose.PosX, (float)pose.PosY, (float)pose.PosZ);
            rb.rotation = new Quaternion(pose.RotX, pose.RotY, pose.RotZ, pose.RotW);

            // Unity silently drops rb.velocity / rb.angularVelocity
            // writes on kinematic rigidbodies. Stash the analytic
            // Jolt velocity on the JoltBody component instead — our
            // refresh path and any reader-patches read from there.
            var vel = new Vector3(pose.LinX, pose.LinY, pose.LinZ);
            var angVel = new Vector3(pose.AngX, pose.AngY, pose.AngZ);
            if (jb != null)
            {
                jb.LastVelocity = vel;
                jb.LastAngularVelocity = angVel;
            }
            // Try the rb.velocity write anyway — for non-kinematic rbs
            // (e.g., if a future Phase moves to dynamic rbs) this is
            // the right thing; for kinematic, it's a silent no-op.
            rb.velocity = vel;
            rb.angularVelocity = angVel;
        }

        // Diagnostic: instance-level so we can throttle a per-second log
        // without spamming.
        static long _diagTick;
        public static void DiagPostfixVelocity(Vessel v)
        {
            _diagTick++;
            if ((_diagTick % 50) != 0) return;
            if (v?.rootPart?.rb == null) return;
            var rb = v.rootPart.rb;
            Debug.Log(string.Format(
                "[Longeron/diag-vel] v={0} rb.vel=({1:F3},{2:F3},{3:F3}) " +
                "isKin={4} velD=({5:F2},{6:F2},{7:F2}) srf=({8:F2},{9:F2},{10:F2}) " +
                "srfSpeed={11:F2} obtSpeed={12:F2} fv=({13:F2},{14:F2},{15:F2})",
                v.vesselName,
                rb.velocity.x, rb.velocity.y, rb.velocity.z,
                rb.isKinematic,
                v.velocityD.x, v.velocityD.y, v.velocityD.z,
                v.srf_velocity.x, v.srf_velocity.y, v.srf_velocity.z,
                v.srfSpeed, v.obt_speed,
                Krakensbane.GetFrameVelocity().x,
                Krakensbane.GetFrameVelocity().y,
                Krakensbane.GetFrameVelocity().z));
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

            // Source the velocity from JoltBody.LastVelocity — Unity
            // discards rb.velocity writes on kinematic rbs, so reading
            // rb.velocity directly always gives 0. JoltBody stashed
            // the analytic Jolt velocity at pose readback time.
            //
            // Jolt integrates in the rotating frame (Krakensbane
            // keeps Unity world rotating with the surface), so this
            // is surface-frame velocity. velocityD = rotating-frame
            // velocity + Krakensbane.FrameVelocity (the inertial
            // component drained at high speeds). Inertial-frame
            // velocity = velocityD + body's surface rotation at our
            // position.
            var rootJb = v.rootPart.gameObject != null
                            ? v.rootPart.gameObject.GetComponent<JoltBody>()
                            : null;
            Vector3d rbVel = rootJb != null
                              ? (Vector3d)rootJb.LastVelocity
                              : (Vector3d)v.rootPart.rb.velocity;
            v.rb_velocity = (Vector3)rbVel;
            v.rb_velocityD = rbVel;
            v.velocityD = rbVel + Krakensbane.GetFrameVelocity();

            // srf_velocity = velocityD itself (rotating-frame motion).
            // obt_velocity = inertial-frame velocity = surface motion +
            // body's surface rotation at our position.
            // Both formulas via Kraken (~/dev/hgs2/mod/KrakenClient.cs:368).
            Vector3d rotVelAtCoM = v.mainBody.getRFrmVel(v.CoMD);
            v.srf_velocity = v.velocityD;
            v.obt_velocity = v.velocityD + rotVelAtCoM;
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
            // VesselPrecalculate:645).
            v.angularVelocity = Quaternion.Inverse(v.ReferenceTransform.rotation)
                              * v.rootPart.rb.angularVelocity;
            v.angularVelocityD = v.angularVelocity;
        }

        // Synthetic launchpad surrogate: a 50 m × 0.5 m × 50 m static
        // box oriented perpendicular to gravity, placed just below
        // the lowest part along the gravity direction. KSP's flight
        // scene doesn't reorient the world to put +Y as "up" — Unity
        // axes are absolute Kerbin-centric coords. So we anchor
        // everything in the gravity vector, not Unity Y.
        //
        // Phase 3 replaces this with PQS streaming.
        void TrySpawnSyntheticGround(World world)
        {
            // Pick the active vessel (or the first registered) as the
            // gravity reference point — gravity varies with position,
            // but for a single launchpad vessel this is a constant.
            Vessel anchorVessel = null;
            foreach (var mv in SceneRegistry.Vessels)
            {
                if (mv.Vessel != null && mv.Vessel.rootPart != null)
                {
                    anchorVessel = mv.Vessel;
                    break;
                }
            }
            if (anchorVessel == null) return;

            Vector3 rootPos = anchorVessel.rootPart.transform.position;
            Vector3d gd = FlightGlobals.getGeeForceAtPosition(rootPos);
            Vector3 g = new Vector3((float)gd.x, (float)gd.y, (float)gd.z);
            float gMag = g.magnitude;
            if (gMag < 1e-3f)
            {
                Debug.LogWarning(LogPrefix + "no gravity at vessel position — skipping synthetic ground");
                return;
            }
            Vector3 gDir = g / gMag;          // points "down"
            Vector3 upDir = -gDir;            // points "up"

            // Find the lowest collider point across all parts'
            // bounds, projected along the gravity-up direction.
            // Using collider AABBs (not just transform pivots)
            // matches the actual geometry — engine bells extend well
            // below their transform, and KSP placed the vessel so
            // the lowest collider point IS at the launchpad surface.
            // Spawning the synthetic ground exactly there puts the
            // vessel at its natural rest height — no fall, no
            // penetration, no buried-in-pad.
            float lowestProj = float.PositiveInfinity;
            foreach (var mv in SceneRegistry.Vessels)
            {
                if (mv.Vessel == null) continue;
                foreach (var part in mv.Vessel.parts)
                {
                    if (part == null) continue;
                    foreach (var col in part.GetComponentsInChildren<Collider>(includeInactive: false))
                    {
                        if (col == null || col.isTrigger || !col.enabled) continue;
                        Bounds b = col.bounds;
                        // The bounds are an AABB in world coords. The
                        // extreme along -upDir is the most negative
                        // dot of any of the 8 corners with upDir.
                        // Cheap upper-bound: use bounds.center ±
                        // bounds.extents projected onto upDir.
                        float centerProj = Vector3.Dot(b.center - rootPos, upDir);
                        float extentProj = Mathf.Abs(b.extents.x * upDir.x)
                                         + Mathf.Abs(b.extents.y * upDir.y)
                                         + Mathf.Abs(b.extents.z * upDir.z);
                        float minProj = centerProj - extentProj;
                        if (minProj < lowestProj) lowestProj = minProj;
                    }
                }
            }
            if (float.IsInfinity(lowestProj) || float.IsNaN(lowestProj)) lowestProj = 0f;

            const float kHalfY = 0.25f;
            // Ground top face exactly at the lowest collider point.
            // Body center sits halfY below that.
            Vector3 lowestPos    = rootPos + upDir * lowestProj;
            Vector3 groundCenter = lowestPos + gDir * kHalfY;

            // Rotation that maps the box's natural up (+Y_local) to
            // the world's gravity-up direction.
            Quaternion groundRot = Quaternion.FromToRotation(Vector3.up, upDir);

            world.Input.WriteBodyCreateBox(
                new BodyHandle(SceneRegistry.SyntheticGroundBodyId),
                BodyType.Static, Layer.Static,
                halfX: 25f, halfY: kHalfY, halfZ: 25f,
                posX: groundCenter.x, posY: groundCenter.y, posZ: groundCenter.z,
                rotX: groundRot.x, rotY: groundRot.y, rotZ: groundRot.z, rotW: groundRot.w,
                mass: 0f);

            _groundSpawned = true;
            Debug.Log(LogPrefix + string.Format(
                "synthetic ground spawned: center=({0:F2},{1:F2},{2:F2}) up=({3:F2},{4:F2},{5:F2}) lowestProj={6:F2} — vessel '{7}'",
                groundCenter.x, groundCenter.y, groundCenter.z,
                upDir.x, upDir.y, upDir.z,
                lowestProj,
                anchorVessel.vesselName));
        }
    }
}
