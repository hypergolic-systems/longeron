// Per-vessel lifecycle hooks. KSP auto-attaches a VesselModule to every
// Vessel at construction time.
//
// OnGoOffRails: vessel transitions from packed (rails) to physics-active.
// We just mark it dirty for the reconciler — TopologyReconciler will
// register the vessel and create bodies/constraints on the next
// FixedUpdate.
//
// OnGoOnRails: vessel transitions from physics-active to packed. We
// tear down synchronously here because the vessel is leaving physics
// and we want the bodies gone immediately, not next tick.
//
// Mid-flight topology mutations (decouple, couple, dock, joint break)
// don't go through these hooks — they fire onVesselWasModified /
// onVesselCreate / onVesselDestroy, which LongeronAddon subscribes to
// and routes into TopologyReconciler.MarkDirty.

using Longeron.Integration;
using Longeron.Native;
using UnityEngine;

namespace Longeron
{
    public class LongeronVesselModule : VesselModule
    {
        const string LogPrefix = "[Longeron/Vessel] ";

        // KSP's internal force API operates in kN with mass in tonnes
        // (kN / t = m/s² so F = m·a works dimensionally without an
        // explicit unit conversion). When stock code calls
        // Rigidbody.AddForce it passes a kN value with the rb.mass
        // also in tonnes — Unity's integrator just sees a consistent
        // pair. We mirror the same convention into Jolt: pass the
        // body mass in tonnes, leave forces in kN, and Jolt's
        // integrator does kN / t = m/s² exactly the same way.
        // No 1000× scaling on either side.

        public override void OnGoOffRails()
        {
            if (vessel == null) return;
            if (LongeronAddon.ActiveWorld == null) return;

            // Stamp every part's rigidbody as kinematic *before* the
            // reconciler runs in the next FixedUpdate. Stock KSP just
            // set isKinematic = false during Vessel.Unpack and created
            // ConfigurableJoints between parts — without immediate
            // takeover, Unity's PhysX gets one step with non-kinematic
            // rbs + compliant joints, settling the rocket into stock-
            // wobble equilibrium (e.g., side boosters tilt 15° outward
            // before we capture poses). Reconciler then sends BodyCreate
            // at those drifted poses; FixedConstraint records the drift
            // as the rest pose, locking the wobble in.
            ApplyKinematicTakeover(vessel);

            TopologyReconciler.MarkDirty(vessel);
        }

        public override void OnGoOnRails()
        {
            if (vessel == null) return;
            if (!SceneRegistry.Unregister(vessel, out var managed)) return;

            var world = LongeronAddon.ActiveWorld;
            if (world != null && managed.Body.IsValid)
            {
                // Single-body model: one BodyDestroy per vessel.
                world.Input.WriteBodyDestroy(managed.Body);
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

            Debug.Log(LogPrefix + $"released '{vessel.vesselName}': {managed.LastParts.Count} part(s)");
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
