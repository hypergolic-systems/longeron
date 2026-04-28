// Harmony prefixes on Rigidbody.AddForce* — every stock/modded engine,
// aero surface, RCS thruster, reaction wheel etc. eventually lands at
// one of these calls. The Unity rb is kinematic for parts we manage,
// so the original AddForce path is a no-op anyway. We intercept,
// convert each ForceMode to an equivalent world-frame force, and
// queue a single ForceDelta record into the bridge's input buffer.
//
// JoltBody.GetComponent dispatch keeps the patch hot path to a single
// O(1) Unity component lookup — no global dictionary mutation.

using HarmonyLib;
using Longeron.Native;
using UnityEngine;

namespace Longeron.Patches
{
    static class ForceConversion
    {
        // Convert any ForceMode into a continuous world-frame force
        // suitable for the bridge's ForceDelta record. The bridge
        // applies this as F·dt during the next physics step.
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
        // Returns true if the call was redirected and the original should be
        // skipped. Returns false if the rb isn't Longeron-managed (let stock
        // physics handle it) or the bridge isn't ready.
        static bool TryRedirect(Rigidbody rb, Vector3 force, Vector3 torque)
        {
            if (LongeronAddon.ActiveWorld == null) return false;
            if (rb == null) return false;
            var jb = rb.GetComponent<JoltBody>();
            if (jb == null) return false;
            LongeronAddon.ActiveWorld.Input.WriteForceDelta(
                jb.Handle,
                force.x, force.y, force.z,
                torque.x, torque.y, torque.z);
            return true;
        }

        // -- AddForce --------------------------------------------------

        [HarmonyPatch(nameof(Rigidbody.AddForce), typeof(Vector3))]
        [HarmonyPrefix]
        static bool AddForce_V(Rigidbody __instance, Vector3 force) =>
            !TryRedirect(__instance, force, Vector3.zero);

        [HarmonyPatch(nameof(Rigidbody.AddForce), typeof(Vector3), typeof(ForceMode))]
        [HarmonyPrefix]
        static bool AddForce_V_M(Rigidbody __instance, Vector3 force, ForceMode mode) =>
            !TryRedirect(__instance, ForceConversion.ToForce(force, mode, __instance), Vector3.zero);

        [HarmonyPatch(nameof(Rigidbody.AddForce), typeof(float), typeof(float), typeof(float))]
        [HarmonyPrefix]
        static bool AddForce_F(Rigidbody __instance, float x, float y, float z) =>
            !TryRedirect(__instance, new Vector3(x, y, z), Vector3.zero);

        [HarmonyPatch(nameof(Rigidbody.AddForce), typeof(float), typeof(float), typeof(float), typeof(ForceMode))]
        [HarmonyPrefix]
        static bool AddForce_F_M(Rigidbody __instance, float x, float y, float z, ForceMode mode) =>
            !TryRedirect(__instance, ForceConversion.ToForce(new Vector3(x, y, z), mode, __instance), Vector3.zero);

        // -- AddRelativeForce ------------------------------------------

        [HarmonyPatch(nameof(Rigidbody.AddRelativeForce), typeof(Vector3))]
        [HarmonyPrefix]
        static bool AddRelForce_V(Rigidbody __instance, Vector3 force) =>
            !TryRedirect(__instance, __instance.rotation * force, Vector3.zero);

        [HarmonyPatch(nameof(Rigidbody.AddRelativeForce), typeof(Vector3), typeof(ForceMode))]
        [HarmonyPrefix]
        static bool AddRelForce_V_M(Rigidbody __instance, Vector3 force, ForceMode mode) =>
            !TryRedirect(__instance,
                ForceConversion.ToForce(__instance.rotation * force, mode, __instance),
                Vector3.zero);

        // -- AddForceAtPosition ----------------------------------------

        [HarmonyPatch(nameof(Rigidbody.AddForceAtPosition), typeof(Vector3), typeof(Vector3))]
        [HarmonyPrefix]
        static bool AddForceAtPos_V_V(Rigidbody __instance, Vector3 force, Vector3 position)
        {
            // Convert "force at world point" into force-at-COM + torque
            // around COM. Equivalent for rigid-body dynamics.
            Vector3 r = position - __instance.worldCenterOfMass;
            Vector3 torque = Vector3.Cross(r, force);
            return !TryRedirect(__instance, force, torque);
        }

        [HarmonyPatch(nameof(Rigidbody.AddForceAtPosition), typeof(Vector3), typeof(Vector3), typeof(ForceMode))]
        [HarmonyPrefix]
        static bool AddForceAtPos_V_V_M(Rigidbody __instance, Vector3 force, Vector3 position, ForceMode mode)
        {
            Vector3 worldForce = ForceConversion.ToForce(force, mode, __instance);
            Vector3 r = position - __instance.worldCenterOfMass;
            Vector3 torque = Vector3.Cross(r, worldForce);
            return !TryRedirect(__instance, worldForce, torque);
        }

        // -- AddTorque -------------------------------------------------

        [HarmonyPatch(nameof(Rigidbody.AddTorque), typeof(Vector3))]
        [HarmonyPrefix]
        static bool AddTorque_V(Rigidbody __instance, Vector3 torque) =>
            !TryRedirect(__instance, Vector3.zero, torque);

        [HarmonyPatch(nameof(Rigidbody.AddTorque), typeof(Vector3), typeof(ForceMode))]
        [HarmonyPrefix]
        static bool AddTorque_V_M(Rigidbody __instance, Vector3 torque, ForceMode mode) =>
            !TryRedirect(__instance,
                Vector3.zero,
                ForceConversion.ToForce(torque, mode, __instance));

        [HarmonyPatch(nameof(Rigidbody.AddRelativeTorque), typeof(Vector3))]
        [HarmonyPrefix]
        static bool AddRelTorque_V(Rigidbody __instance, Vector3 torque) =>
            !TryRedirect(__instance, Vector3.zero, __instance.rotation * torque);

        [HarmonyPatch(nameof(Rigidbody.AddRelativeTorque), typeof(Vector3), typeof(ForceMode))]
        [HarmonyPrefix]
        static bool AddRelTorque_V_M(Rigidbody __instance, Vector3 torque, ForceMode mode) =>
            !TryRedirect(__instance,
                Vector3.zero,
                ForceConversion.ToForce(__instance.rotation * torque, mode, __instance));
    }
}
