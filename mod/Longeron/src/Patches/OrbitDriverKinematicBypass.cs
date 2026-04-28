// OrbitDriver.TrackRigidbody sits behind an isKinematic guard: for
// kinematic rigidbodies, it never re-sets the orbit's velocity field
// (`vel`) from `vessel.velocityD`. What it *does* do is Swizzle()
// both `pos` and `vel` in-place at the end of each call — and Swizzle
// is a y↔z swap. So for a kinematic vessel in TRACK_Phys mode, `vel`
// flips components every tick.
//
// `vessel.srf_velocity = orbit.GetVel() - mainBody.getRFrmVelOrbit(orbit)`
// then oscillates between two values, visible as the navball
// flickering and (more importantly for the smoke test) audible as the
// wind sound effects retriggering at every speed-threshold crossing.
//
// Our fix: for Longeron-managed vessels, temporarily clear isKinematic
// while TrackRigidbody runs. The non-kinematic path updates `vel`
// from `vessel.velocityD`, and the swizzles remain symmetric because
// `vel` is being overwritten fresh each call.

using HarmonyLib;
using Longeron.Integration;

namespace Longeron.Patches
{
    [HarmonyPatch(typeof(OrbitDriver), nameof(OrbitDriver.TrackRigidbody))]
    static class OrbitDriverKinematicBypass
    {
        [HarmonyPrefix]
        static void Prefix(OrbitDriver __instance, out bool __state)
        {
            __state = false;
            var v = __instance.vessel;
            if (v == null || v.rootPart == null || v.rootPart.rb == null) return;
            if (!SceneRegistry.TryGet(v, out _)) return;
            if (v.rootPart.rb.isKinematic)
            {
                v.rootPart.rb.isKinematic = false;
                __state = true;
            }
        }

        [HarmonyPostfix]
        static void Postfix(OrbitDriver __instance, bool __state)
        {
            if (!__state) return;
            var v = __instance.vessel;
            if (v != null && v.rootPart != null && v.rootPart.rb != null)
                v.rootPart.rb.isKinematic = true;
        }
    }
}
