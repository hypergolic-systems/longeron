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

                // Gravity: use FlightGlobals.getGeeForceAtPosition for
                // a clean gravity-only vector at the vessel's CoM.
                // Pure gravity (no rotating-frame fictitious forces).
                // Phase 2.0 sets Jolt's global gravity once per tick;
                // Phase 4 will make this per-body for multi-planet
                // scenes via per-tick ForceDelta records.
                if (mv.Vessel == FlightGlobals.ActiveVessel && mv.Vessel.rootPart != null)
                {
                    Vector3d g = FlightGlobals.getGeeForceAtPosition(mv.Vessel.rootPart.transform.position);
                    world.Input.WriteSetGravity(g.x, g.y, g.z);

                    if ((_tick % kLogEveryTicks) == 0)
                    {
                        var fv = Krakensbane.GetFrameVelocity();
                        var rootPos = mv.Vessel.rootPart.transform.position;
                        Debug.Log(LogPrefix + string.Format(
                            "tick={0} grav=({1:F3},{2:F3},{3:F3}) |g|={4:F2} fv=({5:F2},{6:F2},{7:F2}) rootPos=({8:F2},{9:F2},{10:F2})",
                            _tick,
                            g.x, g.y, g.z, g.magnitude,
                            fv.x, fv.y, fv.z,
                            rootPos.x, rootPos.y, rootPos.z));
                    }
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
                            ApplyPoseToRigidbody(part.rb, pose);
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
        static void ApplyPoseToRigidbody(Rigidbody rb, BodyPoseRecord pose)
        {
            rb.position = new Vector3((float)pose.PosX, (float)pose.PosY, (float)pose.PosZ);
            rb.rotation = new Quaternion(pose.RotX, pose.RotY, pose.RotZ, pose.RotW);
            rb.velocity = new Vector3(pose.LinX, pose.LinY, pose.LinZ);
            rb.angularVelocity = new Vector3(pose.AngX, pose.AngY, pose.AngZ);
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

            // Find the part farthest along -gDir (the deepest part).
            // Project each part's offset from rootPos onto -gDir and
            // pick the smallest projection — that's the lowest along
            // the gravity axis.
            float lowestProj = 0f;
            foreach (var mv in SceneRegistry.Vessels)
            {
                if (mv.Vessel == null) continue;
                foreach (var part in mv.Vessel.parts)
                {
                    if (part?.transform == null) continue;
                    Vector3 offset = part.transform.position - rootPos;
                    float proj = Vector3.Dot(offset, upDir);
                    if (proj < lowestProj) lowestProj = proj;
                }
            }

            // Ground top face sits 0.5 m below the lowest part along
            // the gravity axis. Box halfY = 0.25 m, so center is
            // halfY further down.
            const float kHalfY     = 0.25f;
            const float kClearance = 0.5f;
            Vector3 lowestPos = rootPos + upDir * lowestProj;
            Vector3 groundCenter = lowestPos + gDir * (kClearance + kHalfY);

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
                "synthetic ground spawned: center=({0:F2},{1:F2},{2:F2}) up=({3:F2},{4:F2},{5:F2}) — vessel '{6}'",
                groundCenter.x, groundCenter.y, groundCenter.z,
                upDir.x, upDir.y, upDir.z,
                anchorVessel.vesselName));
        }
    }
}
