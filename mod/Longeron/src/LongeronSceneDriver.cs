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

                // Gravity (Jolt's global gravity, set per-tick from
                // active vessel's integrationAccel — Phase 4 may make
                // this per-body for multi-planet scenes).
                if (mv.Vessel == FlightGlobals.ActiveVessel
                    && mv.Vessel.precalc != null)
                {
                    var a = mv.Vessel.precalc.integrationAccel;
                    world.Input.WriteSetGravity(a.x, a.y, a.z);
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
        // box anchored just below the lowest managed part. Picking
        // "lowest part" rather than "1 m below root" guarantees a
        // small overlap with the bottom part's collider regardless of
        // whether the vessel is one part or a tall stack — the smoke
        // test wants a contact, not a free-fall debugging exercise.
        // Phase 3 replaces this with PQS streaming.
        void TrySpawnSyntheticGround(World world)
        {
            float lowestY = float.MaxValue;
            float anchorX = 0, anchorZ = 0;
            string anchorName = null;
            foreach (var mv in SceneRegistry.Vessels)
            {
                if (mv.Vessel == null) continue;
                foreach (var part in mv.Vessel.parts)
                {
                    if (part?.transform == null) continue;
                    float py = part.transform.position.y;
                    if (py < lowestY)
                    {
                        lowestY = py;
                        anchorX = part.transform.position.x;
                        anchorZ = part.transform.position.z;
                        anchorName = mv.Vessel.vesselName;
                    }
                }
            }
            if (anchorName == null) return;

            // Box halfY = 0.25 m, top face at (centerY + 0.25). Place
            // top face 0.5 m below the lowest part's transform pivot
            // — Phase 2.0 has Dynamic bodies that gravity will pull
            // down onto the ground, so we want clearance for them to
            // fall through, not overlap.
            const float kHalfY      = 0.25f;
            const float kClearance  = 0.5f;
            float topY = lowestY - kClearance;
            float centerY = topY - kHalfY;

            world.Input.WriteBodyCreateBox(
                new BodyHandle(SceneRegistry.SyntheticGroundBodyId),
                BodyType.Static, Layer.Static,
                halfX: 25f, halfY: kHalfY, halfZ: 25f,
                posX: anchorX, posY: centerY, posZ: anchorZ,
                rotX: 0, rotY: 0, rotZ: 0, rotW: 1,
                mass: 0f);

            _groundSpawned = true;
            Debug.Log(LogPrefix + string.Format(
                "synthetic ground spawned: top@y={0:F2} ({1:F2} m below lowest part) — vessel '{2}'",
                topY, kClearance, anchorName));
        }
    }
}
