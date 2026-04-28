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

                // SetKinematicPose for each part: mirror the Unity
                // Transform straight into Jolt. World-frame → Jolt
                // absolute coords (which match Unity world coords for
                // Phase 1.5; Krakensbane offset handling is Phase 2).
                for (int i = 0; i < mv.Parts.Count; i++)
                {
                    var part = mv.Parts[i];
                    if (part == null) continue;
                    var t = part.transform;
                    var p = t.position;
                    var r = t.rotation;
                    world.Input.WriteSetKinematicPose(
                        mv.BodyHandles[i],
                        posX: p.x, posY: p.y, posZ: p.z,
                        rotX: r.x, rotY: r.y, rotZ: r.z, rotW: r.w,
                        linX: 0, linY: 0, linZ: 0,
                        angX: 0, angY: 0, angZ: 0);
                }
            }

            // Step.
            try { world.Step(dt); }
            catch (System.Exception ex)
            {
                Debug.LogError(LogPrefix + "world.Step threw: " + ex.GetType().Name + ": " + ex.Message);
                return;
            }

            // Drain output. For Phase 1.5: count contacts; log first
            // ContactReport per kLogEveryTicks ticks for sanity check.
            int contactCount = 0;
            ContactReportRecord firstContact = default;
            bool haveFirst = false;
            RecordType type;
            while ((type = world.Output.Next()) != RecordType.None)
            {
                switch (type)
                {
                    case RecordType.BodyPose:
                        world.Output.ReadBodyPose(out _);
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

        // Synthetic launchpad surrogate: a 50 m × 0.5 m × 50 m static
        // box centred 1 m below the first managed vessel's root part.
        // Phase 3 replaces this with PQS streaming.
        void TrySpawnSyntheticGround(World world)
        {
            Vessel anchor = null;
            foreach (var mv in SceneRegistry.Vessels)
            {
                if (mv.Vessel != null && mv.Vessel.rootPart != null)
                {
                    anchor = mv.Vessel;
                    break;
                }
            }
            if (anchor == null) return;

            var root = anchor.rootPart.transform;
            var p = root.position;

            world.Input.WriteBodyCreateBox(
                new BodyHandle(SceneRegistry.SyntheticGroundBodyId),
                BodyType.Static, Layer.Static,
                halfX: 25f, halfY: 0.25f, halfZ: 25f,
                posX: p.x, posY: p.y - 1.0, posZ: p.z,
                rotX: 0, rotY: 0, rotZ: 0, rotW: 1,
                mass: 0f);

            _groundSpawned = true;
            Debug.Log(LogPrefix + string.Format(
                "synthetic ground spawned at ({0:F1}, {1:F1}, {2:F1}) — anchor vessel '{3}'",
                p.x, p.y - 1.0, p.z, anchor.vesselName));
        }
    }
}
