// Patch Unity's Rigidbody velocity / angularVelocity property
// accessors so reads/writes route through a JoltPart backing field
// when one's attached to the rb's GameObject.
//
// Why: Unity silently no-ops rb.velocity / rb.angularVelocity writes
// on kinematic rigidbodies, and reads always return 0. Stock code
// (VesselPrecalculate sums rb.velocity into velocityD; ModuleParachute
// reads rb.velocity directly; Krakensbane.ChangeWorldVelocity sets
// rb.velocity += offset) assumes the property roundtrips. With the
// patch, every reader/writer in stock + modded code "just works"
// against our analytic Jolt velocity — we don't have to chase down
// each call site, and Krakensbane's drain math (FrameVel +=
// excess; rb.velocity -= excess) becomes self-consistent again.
//
// The setter still lets the original run so non-managed rbs (other
// vessels, Unity-PhysX-owned objects) behave normally.

using HarmonyLib;
using UnityEngine;

namespace Longeron.Patches
{
    [HarmonyPatch(typeof(Rigidbody), "get_velocity")]
    static class Patch_Rigidbody_GetVelocity
    {
        [HarmonyPostfix]
        static void Postfix(Rigidbody __instance, ref Vector3 __result)
        {
            var jb = __instance.GetComponent<JoltPart>();
            if (jb != null) __result = jb.LastVelocity;
        }
    }

    [HarmonyPatch(typeof(Rigidbody), "set_velocity")]
    static class Patch_Rigidbody_SetVelocity
    {
        [HarmonyPrefix]
        static void Prefix(Rigidbody __instance, Vector3 value)
        {
            var jb = __instance.GetComponent<JoltPart>();
            if (jb != null) jb.LastVelocity = value;
            // Let the original run too. For kinematic rbs it's a
            // silent no-op (no harm); for any future non-kinematic
            // path it correctly drives Unity-PhysX integration.
        }
    }

    [HarmonyPatch(typeof(Rigidbody), "get_angularVelocity")]
    static class Patch_Rigidbody_GetAngularVelocity
    {
        [HarmonyPostfix]
        static void Postfix(Rigidbody __instance, ref Vector3 __result)
        {
            var jb = __instance.GetComponent<JoltPart>();
            if (jb != null) __result = jb.LastAngularVelocity;
        }
    }

    [HarmonyPatch(typeof(Rigidbody), "set_angularVelocity")]
    static class Patch_Rigidbody_SetAngularVelocity
    {
        [HarmonyPrefix]
        static void Prefix(Rigidbody __instance, Vector3 value)
        {
            var jb = __instance.GetComponent<JoltPart>();
            if (jb != null) jb.LastAngularVelocity = value;
        }
    }
}
