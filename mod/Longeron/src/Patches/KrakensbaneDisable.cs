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
//   - Krakensbane.AddExcess       → no-op (the engage helper that
//                                          mutates FrameVel and
//                                          calls vessel.ChangeWorldVelocity
//                                          on every loaded vessel)
//   - GetFrameVelocity / V3f      → return zero
//
// AddFrameVelocity calls AddExcess after a SafeToEngage check; for
// landed/PRELAUNCH vessels SafeToEngage returns false so it's already
// a no-op there, but for vessels unpacking in flight it would
// otherwise modify rb.velocity on every loaded vessel. Patching
// AddExcess covers all paths.
//
// With FrameVel ≡ 0:
//   rb.velocity                  is the actual Unity-world velocity
//   vessel.velocityD             == rb.velocity (stock formula)
//   vessel.obt_velocity          == velocityD
//   vessel.srf_velocity          == velocityD - mainBody.getRFrmVel(p)

using HarmonyLib;
using UnityEngine;

namespace Longeron.Patches
{
    [HarmonyPatch(typeof(Krakensbane), "FixedUpdate")]
    static class Krakensbane_FixedUpdate_Skip
    {
        static bool Prefix() => false;
    }

    [HarmonyPatch(typeof(Krakensbane), nameof(Krakensbane.AddExcess))]
    static class Krakensbane_AddExcess_Skip
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
