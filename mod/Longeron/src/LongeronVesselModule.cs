// Per-vessel lifecycle: register the vessel with SceneRegistry when it
// goes off-rails, queue BodyCreate records into the bridge for each
// part, queue BodyDestroy + unregister on go-on-rails.
//
// VesselModule is auto-attached by KSP to every Vessel; OnStart fires
// after the vessel is fully constructed but before the first tick.
// OnGoOffRails fires whenever a packed vessel becomes physics-active.

using Longeron.Integration;
using Longeron.Native;
using UnityEngine;

namespace Longeron
{
    public class LongeronVesselModule : VesselModule
    {
        const string LogPrefix = "[Longeron/Vessel] ";

        // Per-part KSP mass conversion. KSP exposes Part.mass +
        // GetResourceMass() in tonnes; the bridge expects kilograms.
        const float TonneToKg = 1000f;

        public override void OnGoOffRails()
        {
            if (vessel == null) return;
            if (LongeronAddon.ActiveWorld == null) return;
            if (SceneRegistry.TryGet(vessel, out _)) return;

            var input = LongeronAddon.ActiveWorld.Input;
            var managed = SceneRegistry.Register(vessel);

            int created = 0, skipped = 0;
            foreach (var part in vessel.parts)
            {
                if (part == null) continue;

                // Phase 1.5: every part is a kinematic body in the
                // Kinematic layer. Mass is informational only —
                // kinematic bodies don't respond to forces — but we
                // pass it through for symmetry with future phases.
                float mass = (part.mass + part.GetResourceMass()) * TonneToKg;
                if (mass < 1e-3f) mass = 10f;

                var handle = SceneRegistry.MintBodyHandle();
                bool ok = ColliderWalker.WriteBodyFor(
                    input, handle, part,
                    BodyType.Dynamic, Layer.Kinematic, mass);

                if (ok)
                {
                    managed.Add(part, handle);
                    created++;
                }
                else
                {
                    skipped++;
                }
            }

            // Stop stock physics from doing anything to managed parts'
            // rigidbodies. Kinematic + no gravity = inert under PhysX.
            // Driver re-applies these idempotently each tick (the rb
            // may not exist yet at this point — Vessel.Unpack runs
            // after OnGoOffRails).
            ApplyKinematicTakeover(vessel);

            Debug.Log(LogPrefix + $"managing '{vessel.vesselName}': {created} body create(s) queued, {skipped} skipped");
        }

        public override void OnGoOnRails()
        {
            if (vessel == null) return;
            if (!SceneRegistry.Unregister(vessel, out var managed)) return;

            var world = LongeronAddon.ActiveWorld;
            if (world != null)
            {
                foreach (var h in managed.BodyHandles)
                    world.Input.WriteBodyDestroy(h);
            }

            // Restore rigidbodies to stock-physics defaults.
            foreach (var part in vessel.parts)
            {
                var rb = part?.rb;
                if (rb == null) continue;
                rb.isKinematic = false;
                // useGravity stays false — FlightIntegrator owns
                // gravity via direct rb.AddForce; stock sets
                // useGravity = false everywhere anyway.
            }

            Debug.Log(LogPrefix + $"released '{vessel.vesselName}': {managed.BodyHandles.Count} body destroy(s) queued");
        }

        public static void ApplyKinematicTakeover(Vessel v)
        {
            if (v == null) return;
            foreach (var part in v.parts)
            {
                var rb = part?.rb;
                if (rb == null) continue;
                if (!rb.isKinematic) rb.isKinematic = true;
                if (rb.useGravity)   rb.useGravity = false;
            }
        }
    }
}
