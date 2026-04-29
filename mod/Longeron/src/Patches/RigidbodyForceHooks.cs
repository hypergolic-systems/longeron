// Harmony prefixes on Rigidbody.AddForce* — every stock/modded engine,
// aero surface, RCS thruster, reaction wheel etc. eventually lands at
// one of these calls. Per-part Unity rbs are kinematic for vessels we
// manage, so the original AddForce path is a no-op anyway. We
// intercept, convert each ForceMode to an equivalent world-frame
// force, and queue a record into the bridge's input buffer.
//
// Single-body model: every part in a vessel shares the SAME vessel
// BodyHandle. A force on any per-part rb routes to the vessel body
// at the application point (in world coords) — Jolt's
// BodyInterface::AddForce(force, point) computes the lever-arm
// torque internally as (point − vesselCoM) × force.
//
// Pure AddTorque has no point — uses ForceDelta with f=0.
//
// Phase 3b: Jolt's coordinate frame is the active vessel's mainBody-
// fixed (rotating) frame, not Unity world. Force vectors transform
// via CbFrame.WorldDirToCb; world points via CbFrame.WorldToCb.

using HarmonyLib;
using Longeron.Integration;
using Longeron.Native;
using UnityEngine;

namespace Longeron.Patches
{
    static class ForceConversion
    {
        // Convert any ForceMode into a continuous world-frame force
        // suitable for the bridge's input record. The bridge applies
        // this as F·dt during the next physics step.
        //
        //   Force            — F (N)        → F as-is
        //   Acceleration     — a (m/s²)     → F = a · m
        //   Impulse          — J (N·s)      → F = J / dt
        //   VelocityChange   — Δv (m/s)     → F = Δv · m / dt
        //
        // Phase 2.1 collapses everything to Force — Impulse is a
        // 1-frame approximation rather than a true instant impulse;
        // proper impulse handling can land later via a separate
        // record type / Jolt's BodyInterface::AddImpulse.
        public static Vector3 ToForce(Vector3 input, ForceMode mode, Rigidbody rb)
        {
            switch (mode)
            {
                case ForceMode.Force:          return input;
                case ForceMode.Acceleration:   return input * rb.mass;
                case ForceMode.Impulse:        return input / Time.fixedDeltaTime;
                case ForceMode.VelocityChange: return input * rb.mass / Time.fixedDeltaTime;
                default:                       return input;
            }
        }
    }

    [HarmonyPatch(typeof(Rigidbody))]
    static class RigidbodyForceHooks
    {
        // Apply force at a specific world-space point on the vessel
        // body. Jolt computes the implicit torque as
        // (point - vesselCoM) × force.
        //
        // Returns true if redirected; false if rb isn't Longeron-
        // managed (let stock physics run) or the bridge isn't ready.
        static bool TryRedirectAt(Rigidbody rb, Vector3 forceWorld, Vector3 pointWorld)
        {
            if (LongeronAddon.ActiveWorld == null) return false;
            if (rb == null) return false;
            var jb = rb.GetComponent<JoltBody>();
            if (jb == null) return false;

            var frame = CbFrame.Current();
            if (!frame.IsValid) return false;

            Vector3d fCb = frame.WorldDirToCb(new Vector3d(
                forceWorld.x, forceWorld.y, forceWorld.z));
            Vector3d pCb = frame.WorldToCb(new Vector3d(
                pointWorld.x, pointWorld.y, pointWorld.z));

            LongeronAddon.ActiveWorld.Input.WriteForceAtPosition(
                jb.Handle,
                fCb.x, fCb.y, fCb.z,
                pCb.x, pCb.y, pCb.z,
                partIdx: jb.PartIdx);
            return true;
        }

        // Pure torque on the vessel body (no point). Routes through
        // the existing ForceDelta record (f=0, τ given).
        static bool TryRedirectTorque(Rigidbody rb, Vector3 torqueWorld)
        {
            if (LongeronAddon.ActiveWorld == null) return false;
            if (rb == null) return false;
            var jb = rb.GetComponent<JoltBody>();
            if (jb == null) return false;

            var frame = CbFrame.Current();
            if (!frame.IsValid) return false;

            Vector3d tCb = frame.WorldDirToCb(new Vector3d(
                torqueWorld.x, torqueWorld.y, torqueWorld.z));

            LongeronAddon.ActiveWorld.Input.WriteForceDelta(
                jb.Handle, 0.0, 0.0, 0.0, tCb.x, tCb.y, tCb.z);
            return true;
        }

        // -- AddForce --------------------------------------------------
        // No explicit point — applies at the part's CoM. With single-
        // body, that means apply on the vessel body at the part's
        // worldCenterOfMass so off-CoM forces still produce the right
        // lever arm relative to the vessel CoM.

        [HarmonyPatch(nameof(Rigidbody.AddForce), typeof(Vector3))]
        [HarmonyPrefix]
        static bool AddForce_V(Rigidbody __instance, Vector3 force) =>
            !TryRedirectAt(__instance, force, __instance.worldCenterOfMass);

        [HarmonyPatch(nameof(Rigidbody.AddForce), typeof(Vector3), typeof(ForceMode))]
        [HarmonyPrefix]
        static bool AddForce_V_M(Rigidbody __instance, Vector3 force, ForceMode mode) =>
            !TryRedirectAt(__instance,
                ForceConversion.ToForce(force, mode, __instance),
                __instance.worldCenterOfMass);

        [HarmonyPatch(nameof(Rigidbody.AddForce), typeof(float), typeof(float), typeof(float))]
        [HarmonyPrefix]
        static bool AddForce_F(Rigidbody __instance, float x, float y, float z) =>
            !TryRedirectAt(__instance, new Vector3(x, y, z), __instance.worldCenterOfMass);

        [HarmonyPatch(nameof(Rigidbody.AddForce), typeof(float), typeof(float), typeof(float), typeof(ForceMode))]
        [HarmonyPrefix]
        static bool AddForce_F_M(Rigidbody __instance, float x, float y, float z, ForceMode mode) =>
            !TryRedirectAt(__instance,
                ForceConversion.ToForce(new Vector3(x, y, z), mode, __instance),
                __instance.worldCenterOfMass);

        // -- AddRelativeForce ------------------------------------------
        // Force given in body-local axes. Rotate into world, then apply
        // at the part's CoM.

        [HarmonyPatch(nameof(Rigidbody.AddRelativeForce), typeof(Vector3))]
        [HarmonyPrefix]
        static bool AddRelForce_V(Rigidbody __instance, Vector3 force) =>
            !TryRedirectAt(__instance,
                __instance.rotation * force,
                __instance.worldCenterOfMass);

        [HarmonyPatch(nameof(Rigidbody.AddRelativeForce), typeof(Vector3), typeof(ForceMode))]
        [HarmonyPrefix]
        static bool AddRelForce_V_M(Rigidbody __instance, Vector3 force, ForceMode mode) =>
            !TryRedirectAt(__instance,
                ForceConversion.ToForce(__instance.rotation * force, mode, __instance),
                __instance.worldCenterOfMass);

        // -- AddForceAtPosition ----------------------------------------
        // Explicit world-space point — apply force there.

        [HarmonyPatch(nameof(Rigidbody.AddForceAtPosition), typeof(Vector3), typeof(Vector3))]
        [HarmonyPrefix]
        static bool AddForceAtPos_V_V(Rigidbody __instance, Vector3 force, Vector3 position) =>
            !TryRedirectAt(__instance, force, position);

        [HarmonyPatch(nameof(Rigidbody.AddForceAtPosition), typeof(Vector3), typeof(Vector3), typeof(ForceMode))]
        [HarmonyPrefix]
        static bool AddForceAtPos_V_V_M(Rigidbody __instance, Vector3 force, Vector3 position, ForceMode mode) =>
            !TryRedirectAt(__instance,
                ForceConversion.ToForce(force, mode, __instance),
                position);

        // -- AddTorque -------------------------------------------------
        // No application point. Pure torque on the vessel body.

        [HarmonyPatch(nameof(Rigidbody.AddTorque), typeof(Vector3))]
        [HarmonyPrefix]
        static bool AddTorque_V(Rigidbody __instance, Vector3 torque) =>
            !TryRedirectTorque(__instance, torque);

        [HarmonyPatch(nameof(Rigidbody.AddTorque), typeof(Vector3), typeof(ForceMode))]
        [HarmonyPrefix]
        static bool AddTorque_V_M(Rigidbody __instance, Vector3 torque, ForceMode mode) =>
            !TryRedirectTorque(__instance,
                ForceConversion.ToForce(torque, mode, __instance));

        [HarmonyPatch(nameof(Rigidbody.AddRelativeTorque), typeof(Vector3))]
        [HarmonyPrefix]
        static bool AddRelTorque_V(Rigidbody __instance, Vector3 torque) =>
            !TryRedirectTorque(__instance, __instance.rotation * torque);

        [HarmonyPatch(nameof(Rigidbody.AddRelativeTorque), typeof(Vector3), typeof(ForceMode))]
        [HarmonyPrefix]
        static bool AddRelTorque_V_M(Rigidbody __instance, Vector3 torque, ForceMode mode) =>
            !TryRedirectTorque(__instance,
                ForceConversion.ToForce(__instance.rotation * torque, mode, __instance));
    }
}
