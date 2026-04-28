// Harmony postfix on Vessel.UpdatePosVel — re-applies our refresh of
// srf_velocity / verticalSpeed / srfSpeed / obt_velocity / etc.
// AFTER stock has finished its own (stale, kinematic-rb-confused)
// computation. UpdatePosVel runs from both FixedUpdate (via
// VesselPrecalculate.MainPhysics) and per-frame Update, so just
// writing in our +10000 SceneDriver wasn't enough — Update would
// overwrite our values before the navball read them.

using HarmonyLib;
using Longeron.Integration;

namespace Longeron.Patches
{
    [HarmonyPatch(typeof(Vessel), nameof(Vessel.UpdatePosVel))]
    static class VesselUpdatePosVelPatch
    {
        [HarmonyPostfix]
        static void Postfix(Vessel __instance)
        {
            if (LongeronAddon.ActiveWorld == null) return;
            if (__instance == null) return;
            if (!SceneRegistry.TryGet(__instance, out _)) return;
            LongeronSceneDriver.RefreshVesselVelocityFields(__instance);
            LongeronSceneDriver.DiagPostfixVelocity(__instance);
        }
    }
}
