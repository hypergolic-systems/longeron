// KrakensbaneDisable — patch out Krakensbane entirely.
//
// Krakensbane existed to keep PhysX's velocity integrator within
// single-precision float range: when vessel velocity got too high,
// FrameVel absorbed the excess so rb.velocity stayed small. We
// replaced PhysX with Jolt (built JPH_DOUBLE_PRECISION=ON), which
// integrates in double. The justification is gone.
//
// We patch:
//   - Krakensbane.FixedUpdate     → no-op (skip engagement state machine)
//   - GetFrameVelocity / V3f      → return zero
//
// With FrameVel ≡ 0:
//   rb.velocity                  is the actual Unity-world velocity
//   vessel.velocityD             == rb.velocity (stock formula)
//   vessel.obt_velocity          == velocityD
//   vessel.srf_velocity          == velocityD - mainBody.getRFrmVel(p)
// All stock readers that derive from these continue to work; the only
// path that would have broken is direct reads of FrameVel, which now
// observe zero.
//
// AddFrameVelocity / AddExcess / Zero are not patched: if a
// modded code path explicitly wants to set FrameVel they can; we just
// don't drive the state machine ourselves.

using HarmonyLib;
using UnityEngine;

namespace Longeron.Patches
{
    [HarmonyPatch(typeof(Krakensbane), "FixedUpdate")]
    static class Krakensbane_FixedUpdate_Skip
    {
        static bool Prefix() => false;
    }

    [HarmonyPatch(typeof(Krakensbane), nameof(Krakensbane.GetFrameVelocity))]
    static class Krakensbane_GetFrameVelocity_Zero
    {
        static bool Prefix(ref Vector3d __result)
        {
            __result = Vector3d.zero;
            return false;
        }
    }

    [HarmonyPatch(typeof(Krakensbane), nameof(Krakensbane.GetFrameVelocityV3f))]
    static class Krakensbane_GetFrameVelocityV3f_Zero
    {
        static bool Prefix(ref Vector3 __result)
        {
            __result = Vector3.zero;
            return false;
        }
    }
}
