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

        bool _groundSpawned;

        // Called by LongeronAddon when a new flight-scene world is
        // created so per-world transient state resets.
        internal void NotifyWorldCreated()
        {
            _groundSpawned = false;
        }

        public void FixedUpdate()
        {
            var world = LongeronAddon.ActiveWorld;
            if (world == null) return;
            if (!HighLogic.LoadedSceneIsFlight) return;

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
            // Unity rigidbody. ContactReport records are read but not
            // currently consumed downstream — Phase 4 will wire them
            // into ABA's external-wrench input.
            RecordType type;
            while ((type = world.Output.Next()) != RecordType.None)
            {
                switch (type)
                {
                    case RecordType.BodyPose:
                        world.Output.ReadBodyPose(out var pose);
                        if (SceneRegistry.TryGetPart(pose.Body.Id, out var part) && part?.rb != null)
                            ApplyPoseToRigidbody(part.rb, pose);
                        break;
                    case RecordType.ContactReport:
                        world.Output.ReadContactReport(out _);
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
        }

        // Apply Jolt's integrated pose + velocity to a Unity Rigidbody.
        //
        // Velocity convention: Jolt integrates in absolute world coords
        // (V_abs). Stock KSP expects rb.velocity to be the
        // *Krakensbane-adjusted* velocity (V_abs − FrameVel), so that
        // velocityD = rb.velocity + FrameVel = V_abs round-trips. We
        // subtract FrameVel here.
        //
        // Why this matters: if we wrote V_abs into rb.velocity, then on
        // every tick where speed > MaxV, Krakensbane would drain
        // rb_velocityD (= V_abs) into FrameVel, but Jolt has no notion
        // of FrameVel and would re-emit V_abs next tick — FrameVel
        // grows by V_abs every tick → runaway acceleration → vessel
        // explodes from thermal damage. Subtracting FrameVel here makes
        // Krakensbane's drain idempotent: at steady state rb.velocity
        // ≈ 0, FrameVel holds the orbital velocity, no further excess.
        //
        // The setter goes through Patch_Rigidbody_SetVelocity into
        // JoltBody.LastVelocity, so subsequent stock reads return the
        // adjusted velocity too. JoltBody.LastVelocity here stores
        // *adjusted*, not absolute — RefreshVesselVelocityFields adds
        // FrameVel back when computing velocityD.
        static void ApplyPoseToRigidbody(Rigidbody rb, BodyPoseRecord pose)
        {
            Vector3 frameVel = Krakensbane.GetFrameVelocityV3f();
            Vector3 vAbs = new Vector3(pose.LinX, pose.LinY, pose.LinZ);
            rb.position = new Vector3((float)pose.PosX, (float)pose.PosY, (float)pose.PosZ);
            rb.rotation = new Quaternion(pose.RotX, pose.RotY, pose.RotZ, pose.RotW);
            rb.velocity = vAbs - frameVel;
            rb.angularVelocity = new Vector3(pose.AngX, pose.AngY, pose.AngZ);
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

            // Read velocity from JoltBody directly rather than through
            // rb.velocity. Why: Harmony postfixes on Unity's
            // extern Rigidbody.get_velocity property aren't always
            // honored under Mono, so rb.velocity may still return 0 for
            // kinematic bodies. JoltBody.LastVelocity is what we wrote
            // in ApplyPoseToRigidbody — it's the Krakensbane-adjusted
            // velocity (V_abs − FrameVel). velocityD = adjusted +
            // FrameVel = V_abs round-trips back to absolute.
            var rootRb = v.rootPart.rb;
            var rootJb = rootRb.GetComponent<JoltBody>();
            Vector3d rbVel = rootJb != null
                ? (Vector3d)rootJb.LastVelocity
                : (Vector3d)rootRb.velocity;
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
            // VesselPrecalculate:645). Same JoltBody-direct approach as
            // linear velocity above.
            Vector3 rootAngV = rootJb != null
                ? rootJb.LastAngularVelocity
                : rootRb.angularVelocity;
            v.angularVelocity = Quaternion.Inverse(v.ReferenceTransform.rotation)
                              * rootAngV;
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
