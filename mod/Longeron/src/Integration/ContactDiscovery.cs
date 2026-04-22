// Query-based contact discovery.
//
// Two parallel paths feed the Contacts buffer:
//
//   1. Per-part OverlapSphere + ComputePenetration. Works for convex colliders
//      and convex-marked MeshColliders. For concave MeshColliders Unity's
//      ComputePenetration is documented as undefined and in practice returns
//      garbage depths (kilometers), so we hard-reject any result above a
//      sane-depth threshold and log the offending collider once so we can
//      diagnose what KSP geometry is concave.
//
//   2. Downward raycast along local-gravity from each part's bounds center.
//      This is the robust path for ground / launchpad contact — MeshCollider
//      raycasts work against concave meshes. Single ray per part covers the
//      "rocket resting on a flat pad" case; side contacts are the convex
//      path's job.
//
// Intra-vessel hits are filtered via VesselScene.OwnColliders (populated at
// Build from every Collider on every managed part).

using System.Collections.Generic;
using Longeron.Physics;
using UnityEngine;

namespace Longeron.Integration
{
    internal static class ContactDiscovery
    {
        static readonly Collider[] overlapBuffer = new Collider[32];

        static readonly HashSet<Collider> _loggedBogus = new HashSet<Collider>();

        // Exclude-list mask. Much safer than a whitelist — first-launch data
        // showed that whitelisting LocalScenery (layer 15) misses whichever
        // layer the launchpad is actually on. Exclude only the layers we
        // *know* are wrong:
        //   10 = ScaledSpace (planet proxies — 200 m Kerbin SphereCollider;
        //        hitting this produced the relativistic expulsions)
        //   11 = ScaledScenery
        //   19 = PartTriggers (ladders, airlocks, sensors)
        //   20 = InternalSpace (IVA)
        //   1  = TransparentFX / 2 = IgnoreRaycast (as named)
        const int CONTACT_LAYER_MASK =
            ~((1 << 1) | (1 << 2) | (1 << 10) | (1 << 11) | (1 << 19) | (1 << 20));

        // Diagnostic: log each distinct collider we successfully pull contact
        // data from, once, so we learn which layer the launchpad / terrain /
        // etc. actually live on.
        static readonly HashSet<Collider> _loggedAccepted = new HashSet<Collider>();

        // 0.2 m cap: tight enough that a one-contact-per-part surprise can't
        // inject catastrophic energy, loose enough to cover real landing-speed
        // impacts in a single tick. Bogus-pen hits get rejected above 0.2 m.
        const float MAX_SANE_DEPTH = 0.2f;

        // Lever arm (contact point minus part origin) that exceeds a part's
        // plausible size is a red flag — the contact point is on a far-away
        // collider we shouldn't be reacting to. 10 m is bigger than any stock
        // part and still much smaller than planet-scale geometry.
        const float MAX_SANE_LEVER = 10f;

        // Ground raycast parameters (MVP defaults tuned for the launchpad).
        const float RAY_CAST_BUFFER = 0.5f;
        const float RAY_CONTACT_GAP = 0.05f;

        public static void Discover(VesselScene scene)
        {
            var parts = scene.BodyToPart;

            // Local "down" — toward planet center. For the launchpad this is
            // ~(0,-1,0) after FloatingOrigin; we read it off the vessel so
            // this still works for landings on arbitrary planet latitudes.
            Vector3 down = Vector3.down;
            if (scene.Vessel != null)
            {
                var up = scene.Vessel.upAxis;
                if (up.sqrMagnitude > 1e-6)
                    down = -(Vector3)up.normalized;
            }

            for (int i = 0; i < parts.Length; i++)
            {
                var part = parts[i];
                var partCol = part.collider;
                if (partCol == null) continue;

                DiscoverViaPenetration(scene, part, partCol);
                DiscoverViaDownRay(scene, part, partCol, down);
            }
        }

        static void DiscoverViaPenetration(VesselScene scene, Part part, Collider partCol)
        {
            Bounds b = partCol.bounds;
            float radius = b.extents.magnitude + 0.05f;

            int hitCount = UnityEngine.Physics.OverlapSphereNonAlloc(
                b.center, radius, overlapBuffer, CONTACT_LAYER_MASK);

            for (int h = 0; h < hitCount; h++)
            {
                var other = overlapBuffer[h];
                if (other == null) continue;
                if (scene.OwnColliders.Contains(other)) continue;

                Vector3 dir;
                float dist;
                bool overlap = UnityEngine.Physics.ComputePenetration(
                    partCol, partCol.transform.position, partCol.transform.rotation,
                    other,   other.transform.position,   other.transform.rotation,
                    out dir, out dist);
                if (!overlap || dist <= 0f) continue;

                if (dist > MAX_SANE_DEPTH)
                {
                    if (_loggedBogus.Add(other))
                    {
                        var mc = other as MeshCollider;
                        string kind = mc != null ? ("MeshCol convex=" + mc.convex) : other.GetType().Name;
                        Debug.Log($"[Longeron/contact] rejecting bogus penetration: part={part.partName} " +
                                  $"other={other.name} kind={kind} layer={other.gameObject.layer} " +
                                  $"rawdist={dist:F2} bounds={other.bounds.size}");
                    }
                    continue;
                }

                Vector3 pt = other.ClosestPoint(b.center);

                // Lever-arm sanity: if the contact point is wildly far from
                // the part's own collider, something in our broadphase /
                // narrowphase is lying — skip.
                if ((pt - partCol.transform.position).sqrMagnitude > MAX_SANE_LEVER * MAX_SANE_LEVER)
                    continue;

                if (_loggedAccepted.Add(other))
                    Debug.Log($"[Longeron/contact] pen path accepted: part={part.partName} other={other.name} " +
                              $"layer={other.gameObject.layer} kind={other.GetType().Name} dist={dist:F3}");

                scene.Contacts.Add(new ContactEntry
                {
                    bodyId     = scene.PartToBody[part],
                    point      = new float3(pt.x, pt.y, pt.z),
                    normal     = new float3(dir.x, dir.y, dir.z),
                    separation = -dist,
                });
            }
        }

        static void DiscoverViaDownRay(VesselScene scene, Part part, Collider partCol, Vector3 down)
        {
            Bounds b = partCol.bounds;
            // Project extents onto -down to get distance from bounds center to
            // the "bottom" of the AABB along the local gravity direction.
            float halfAlongDown =
                Mathf.Abs(b.extents.x * down.x) +
                Mathf.Abs(b.extents.y * down.y) +
                Mathf.Abs(b.extents.z * down.z);
            float castDist = halfAlongDown + RAY_CAST_BUFFER;

            // 4-corner sampling: rays from the four corners of the AABB's
            // world-XZ footprint, plus one from the center. Single-center
            // ray gave horizontal parts a "tightrope walker" balance — any
            // small torque tipped them over. Spreading rays across the AABB
            // footprint gives a stable support polygon for both vertical
            // and horizontal orientations (the 4 corners sit outside a
            // cylinder's base but hit the pad just past the edge, which is
            // fine — they still resist roll).
            //
            // Offsets are in world X / Z. For an upright vessel with down
            // ≈ -Y that's perpendicular-to-down. For a tilted vessel it's
            // approximate but still gives useful spread.
            float ox = b.extents.x * 0.9f;
            float oz = b.extents.z * 0.9f;
            Vector3 c0 = b.center;
            Vector3 c1 = b.center + new Vector3(+ox, 0f, +oz);
            Vector3 c2 = b.center + new Vector3(+ox, 0f, -oz);
            Vector3 c3 = b.center + new Vector3(-ox, 0f, +oz);
            Vector3 c4 = b.center + new Vector3(-ox, 0f, -oz);
            TryDownRay(scene, part, c0, down, halfAlongDown, castDist);
            TryDownRay(scene, part, c1, down, halfAlongDown, castDist);
            TryDownRay(scene, part, c2, down, halfAlongDown, castDist);
            TryDownRay(scene, part, c3, down, halfAlongDown, castDist);
            TryDownRay(scene, part, c4, down, halfAlongDown, castDist);
        }

        static void TryDownRay(VesselScene scene, Part part, Vector3 origin, Vector3 down,
                               float halfAlongDown, float castDist)
        {
            RaycastHit hit;
            if (!UnityEngine.Physics.Raycast(origin, down, out hit, castDist, CONTACT_LAYER_MASK))
                return;
            if (hit.collider == null) return;
            if (scene.OwnColliders.Contains(hit.collider)) return;

            float depth = (halfAlongDown + RAY_CONTACT_GAP) - hit.distance;
            if (depth <= 0f) return;
            if (depth > MAX_SANE_DEPTH) depth = MAX_SANE_DEPTH;

            if (_loggedAccepted.Add(hit.collider))
                Debug.Log($"[Longeron/contact] ray path accepted: part={part.partName} " +
                          $"other={hit.collider.name} layer={hit.collider.gameObject.layer} " +
                          $"kind={hit.collider.GetType().Name} depth={depth:F3}");

            scene.Contacts.Add(new ContactEntry
            {
                bodyId     = scene.PartToBody[part],
                point      = new float3(hit.point.x, hit.point.y, hit.point.z),
                normal     = new float3(hit.normal.x, hit.normal.y, hit.normal.z),
                separation = -depth,
            });
        }
    }
}
