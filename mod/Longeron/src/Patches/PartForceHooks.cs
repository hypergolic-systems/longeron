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
//
// Unit conversion: KSP uses tonnes for mass so its force-bearing API takes
// kilonewtons (e.g., an LV-T45 registers thrust ~200 "units" in Part.AddForce,
// which paired with a 1 t rigidbody mass gives 200 m/s² — that only works if
// the unit is kN). Our solver is SI (N, kg) so we scale by 1000 at the hook.

using HarmonyLib;
using Longeron.Integration;
using Longeron.Physics;
using UnityEngine;

namespace Longeron.Patches
{
    static class ForceUnits
    {
        // KSP force API → solver: multiply by 1000 (kN → N).
        public const float KN_TO_N = 1000f;

        public static float3 FromKspForce(Vector3d v) =>
            new float3((float)v.x * KN_TO_N, (float)v.y * KN_TO_N, (float)v.z * KN_TO_N);

        // Positions stay in meters either way.
        public static float3 Pos(Vector3d v) =>
            new float3((float)v.x, (float)v.y, (float)v.z);
    }

    [HarmonyPatch(typeof(Part), nameof(Part.AddForce))]
    static class PartAddForcePatch
    {
        [HarmonyPrefix]
        static bool Prefix(Part __instance, Vector3d vec)
        {
            if (!SceneRegistry.Instance.TryGet(__instance, out var scene, out var id)) return true;
            scene.AddWorldForce(id, ForceUnits.FromKspForce(vec));
            return false;
        }
    }

    [HarmonyPatch(typeof(Part), nameof(Part.AddForceAtPosition))]
    static class PartAddForceAtPositionPatch
    {
        [HarmonyPrefix]
        static bool Prefix(Part __instance, Vector3d vec, Vector3d pos)
        {
            if (!SceneRegistry.Instance.TryGet(__instance, out var scene, out var id)) return true;
            scene.AddWorldForceAtPosition(id, ForceUnits.FromKspForce(vec), ForceUnits.Pos(pos));
            return false;
        }
    }

    [HarmonyPatch(typeof(Part), nameof(Part.AddTorque))]
    static class PartAddTorquePatch
    {
        [HarmonyPrefix]
        static bool Prefix(Part __instance, Vector3d vec)
        {
            if (!SceneRegistry.Instance.TryGet(__instance, out var scene, out var id)) return true;
            // kN·m torque → N·m.
            scene.AddWorldTorque(id, ForceUnits.FromKspForce(vec));
            return false;
        }
    }

    [HarmonyPatch(typeof(Part), nameof(Part.AddImpulse))]
    static class PartAddImpulsePatch
    {
        [HarmonyPrefix]
        static bool Prefix(Part __instance, Vector3d vec)
        {
            if (!SceneRegistry.Instance.TryGet(__instance, out var scene, out var id)) return true;
            float dt = TimeWarp.fixedDeltaTime;
            if (dt < 1e-8f) return false;
            // Stock's AddImpulse is a kN·s impulse applied over one tick.
            // Divide by dt → kN, then scale to N.
            scene.AddWorldForce(id, ForceUnits.FromKspForce(vec) * (1f / dt));
            return false;
        }
    }
}
