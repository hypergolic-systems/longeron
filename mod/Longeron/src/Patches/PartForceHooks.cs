// Harmony hooks that intercept per-part force API calls and redirect into
// Longeron's external-wrench accumulator.
//
// Stock KSP modules (engines, RCS, reaction wheels, aero surfaces) call
// Part.AddForce / AddForceAtPosition / AddTorque / AddImpulse. Each writes
// into RigidBodyPart.force / .torque / .forces, which FlightIntegrator
// drains once per physics tick. We short-circuit those writes for
// Longeron-managed vessels by returning `false` from the prefix; the
// equivalent wrench is recorded on the VesselScene's fExt buffer instead.
//
// Non-Longeron vessels fall through unchanged (prefix returns `true`).
//
// Two external-force paths that do NOT route through Part.Add* (gravity,
// stock drag cubes) are handled elsewhere: gravity via GravitySkip +
// SetGravity in the scene driver; stock drag is deferred out of spike scope.

using HarmonyLib;
using Longeron.Integration;
using Longeron.Physics;
using UnityEngine;

namespace Longeron.Patches
{
    [HarmonyPatch(typeof(Part), nameof(Part.AddForce))]
    static class PartAddForcePatch
    {
        [HarmonyPrefix]
        static bool Prefix(Part __instance, Vector3d vec)
        {
            if (!SceneRegistry.Instance.TryGet(__instance, out var scene, out var id)) return true;
            scene.AddWorldForce(id, Convert(vec));
            return false;
        }

        static float3 Convert(Vector3d v) => new float3((float)v.x, (float)v.y, (float)v.z);
    }

    [HarmonyPatch(typeof(Part), nameof(Part.AddForceAtPosition))]
    static class PartAddForceAtPositionPatch
    {
        [HarmonyPrefix]
        static bool Prefix(Part __instance, Vector3d vec, Vector3d pos)
        {
            if (!SceneRegistry.Instance.TryGet(__instance, out var scene, out var id)) return true;
            scene.AddWorldForceAtPosition(id, Convert(vec), Convert(pos));
            return false;
        }

        static float3 Convert(Vector3d v) => new float3((float)v.x, (float)v.y, (float)v.z);
    }

    [HarmonyPatch(typeof(Part), nameof(Part.AddTorque))]
    static class PartAddTorquePatch
    {
        [HarmonyPrefix]
        static bool Prefix(Part __instance, Vector3d vec)
        {
            if (!SceneRegistry.Instance.TryGet(__instance, out var scene, out var id)) return true;
            scene.AddWorldTorque(id, Convert(vec));
            return false;
        }

        static float3 Convert(Vector3d v) => new float3((float)v.x, (float)v.y, (float)v.z);
    }

    [HarmonyPatch(typeof(Part), nameof(Part.AddImpulse))]
    static class PartAddImpulsePatch
    {
        [HarmonyPrefix]
        static bool Prefix(Part __instance, Vector3d vec)
        {
            if (!SceneRegistry.Instance.TryGet(__instance, out var scene, out var id)) return true;
            // Stock's AddImpulse divides by TimeWarp.fixedDeltaTime to convert
            // to a force-equivalent for one tick. Match that.
            float dt = TimeWarp.fixedDeltaTime;
            if (dt < 1e-8f) return false;
            scene.AddWorldForce(id, Convert(vec) * (1f / dt));
            return false;
        }

        static float3 Convert(Vector3d v) => new float3((float)v.x, (float)v.y, (float)v.z);
    }
}
