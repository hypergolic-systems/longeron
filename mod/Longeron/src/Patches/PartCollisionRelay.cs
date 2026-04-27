// Harmony postfixes that turn stock Part's PhysX collision events into
// Longeron contact constraints.
//
// Why event-driven instead of query-driven: PhysX already runs the broadphase
// and the per-pair narrowphase against KSP's actual collider geometry —
// concave MeshColliders, terrain heightfields, building meshes, all of it.
// Our previous attempt — building per-body capsule primitives, registering
// world-AABBs for the launchpad mesh, and running an analytic capsule-vs-AABB
// narrowphase each tick — went wrong precisely because it had to *re-derive*
// shape information PhysX already knows. The launchpad's MeshCollider is
// concave, ~51×41×71 m, and the rocket spawns *inside* its world-space AABB:
// our analytic narrowphase reported 10–12 m of penetration on the X- face
// every tick, the PGS Baumgarte stabilization ejected the rocket at hundreds
// of m/s, and it self-destructed in the atmosphere. No amount of AABB tuning
// fixes that, because AABBs are the wrong primitive for concave geometry.
//
// PhysX gives us actual contact points on the actual mesh surface. We just
// have to receive them.
//
// Mechanism: Part is a MonoBehaviour, so Unity already calls Part's
// OnCollisionStay / OnCollisionEnter every FixedUpdate that PhysX detects
// a contact. Stock's bodies are mostly trivial (they update one bool used
// for a reset-collisions edge case), so the events are essentially "free"
// for us to harvest via a Harmony postfix. No new MonoBehaviour, no new
// PartModule, no per-part component injection — just a passive observer
// on existing dispatch.
//
// Lifecycle (per FixedUpdate):
//
//   t = -10000   LongeronScenePreTick.FixedUpdate   →  scene.ClearContacts()
//   t = ...      Unity physics step                 →  Part.OnCollisionStay
//                                                      fires per contact pair;
//                                                      our postfix appends to
//                                                      scene.Contacts
//   t = +10000   LongeronSceneDriver.FixedUpdate    →  StepWithContacts drains
//                                                      the buffer
//
// We hook OnCollisionStay (recurring, every tick a contact persists) and
// OnCollisionEnter (first-tick of a contact). OnCollisionExit isn't needed
// because the buffer is cleared at -10000 each tick: a contact that exits
// simply stops being re-added.
//
// Intra-vessel filtering: Longeron owns inter-part dynamics directly through
// the articulated tree, so collisions between two parts of the same managed
// vessel are noise. We drop any ContactPoint whose otherCollider is in the
// receiving scene's OwnColliders set (built once at VesselScene.Build).

using System.Collections.Generic;
using HarmonyLib;
using Longeron.Integration;
using UnityEngine;

namespace Longeron.Patches
{
    static class PartCollisionRelay
    {
        // First-time per-otherCollider diagnostic. One log line each distinct
        // external collider that ever fires a contact event into our relay —
        // enough to confirm the launchpad / runway / terrain are reaching us,
        // without spamming the log at 50 Hz. Mirrors the _loggedAccepted
        // pattern from the prior query-based discovery path.
        static readonly HashSet<Collider> _loggedAccepted = new HashSet<Collider>();

        internal static void Ingest(Part part, Collision c)
        {
            if (part == null || c == null) return;
            var vessel = part.vessel;
            if (vessel == null) return;
            if (!SceneRegistry.Instance.TryGet(vessel, out var scene)) return;

            var contacts = c.contacts;
            if (contacts == null) return;

            for (int i = 0; i < contacts.Length; i++)
            {
                var cp = contacts[i];
                var other = cp.otherCollider;
                if (other == null) continue;
                // Drop intra-vessel pairs — Longeron's articulated solver
                // owns inter-part dynamics; PhysX should not be telling us
                // about parts of the same vessel touching each other.
                if (scene.OwnColliders.Contains(other)) continue;

                if (_loggedAccepted.Add(other))
                {
                    Debug.Log(string.Format(
                        "[Longeron/contact] event accepted: vessel={0} part={1} " +
                        "other={2} layer={3} kind={4}",
                        vessel.vesselName,
                        part.partInfo != null ? part.partInfo.name : part.name,
                        other.name, other.gameObject.layer, other.GetType().Name));
                }

                scene.AddContact(part, cp);
            }
        }
    }

    [HarmonyPatch(typeof(Part), nameof(Part.OnCollisionStay))]
    static class PartOnCollisionStayPatch
    {
        [HarmonyPostfix]
        static void Postfix(Part __instance, Collision c)
        {
            PartCollisionRelay.Ingest(__instance, c);
        }
    }

    [HarmonyPatch(typeof(Part), nameof(Part.OnCollisionEnter))]
    static class PartOnCollisionEnterPatch
    {
        [HarmonyPostfix]
        static void Postfix(Part __instance, Collision c)
        {
            PartCollisionRelay.Ingest(__instance, c);
        }
    }
}
