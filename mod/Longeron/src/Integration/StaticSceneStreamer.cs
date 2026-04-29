// StaticSceneStreamer — mirror KSP's PQSCity static prefab geometry
// (KSC: launchpad concrete, runway tarmac, VAB / SPH / tracking
// station / etc. shells) into Jolt as static bodies.
//
// PQS terrain (the Kerbin sphere itself) is handled by PQSStreamer.
// PQSCity[2] is *not* PQS terrain — it's a static prefab parented
// under pqsController.transform, anchored at the launch site's
// lat/lon, with regular Unity Colliders on its children. The stock
// game uses MeshColliders for the launchpad / runway surfaces and
// BoxColliders for building shells.
//
// We walk once per world-create, mirror every Collider that passes
// our filter into Jolt as a static body (one body per collider,
// matching the QuadBody pattern), and rely on Unity's component
// lifecycle for teardown via the StaticBody MonoBehaviour.
//
// LOD note: PQSCity2 deactivates distant LOD groups via
// SetActive(false). includeInactive: false in the walk means we only
// pick up the currently-loaded LOD. KSC LOD0 is always active when
// the player is on the pad, so this is fine for phase 1; if the
// camera moves and a different LOD activates, we won't pick up its
// colliders. Phase later if it bites.

using System;
using System.Collections.Generic;
using Longeron.Native;
using UnityEngine;

namespace Longeron.Integration
{
    internal static class StaticSceneStreamer
    {
        const string LogPrefix = "[Longeron/static] ";

        // Unity layer 15 = "Local Scenery" — KSC buildings, runway,
        // launchpad. PQS terrain quads are also on this layer but
        // they're handled by PQSStreamer; we filter PQSCity descendants
        // by parent type, not layer, so the layer filter is a sanity
        // check that excludes weirdly-tagged children.
        const int kLocalSceneryLayer = 15;

        public static void MirrorAllPQSCities()
        {
            if (FlightGlobals.Bodies == null) return;
            if (LongeronAddon.ActiveWorld == null) return;

            int citiesVisited = 0;
            int collidersMirrored = 0;
            int collidersSkipped = 0;

            foreach (var body in FlightGlobals.Bodies)
            {
                if (body == null) continue;
                var pqs = body.pqsController;
                if (pqs == null) continue;

                var cities = pqs.GetComponentsInChildren<PQSCity2>(includeInactive: false);
                for (int i = 0; i < cities.Length; ++i)
                {
                    if (cities[i] == null) continue;
                    citiesVisited++;
                    MirrorCity(cities[i].gameObject,
                                ref collidersMirrored, ref collidersSkipped);
                }

                // Legacy PQSCity (no stock use in 1.12.5 — Squad
                // migrated KSC to PQSCity2 — but Kopernicus / KK mods
                // may still use it).
                var legacy = pqs.GetComponentsInChildren<PQSCity>(includeInactive: false);
                for (int i = 0; i < legacy.Length; ++i)
                {
                    if (legacy[i] == null) continue;
                    citiesVisited++;
                    MirrorCity(legacy[i].gameObject,
                                ref collidersMirrored, ref collidersSkipped);
                }
            }

            Debug.Log(LogPrefix + string.Format(
                "MirrorAllPQSCities: {0} cities walked, {1} colliders mirrored, {2} skipped",
                citiesVisited, collidersMirrored, collidersSkipped));
        }

        static void MirrorCity(GameObject city,
                                ref int mirrored, ref int skipped)
        {
            var colliders = city.GetComponentsInChildren<Collider>(includeInactive: false);
            for (int i = 0; i < colliders.Length; ++i)
            {
                var c = colliders[i];
                if (c == null) { skipped++; continue; }
                if (TryMirrorCollider(c)) mirrored++;
                else skipped++;
            }
        }

        // Returns true if a Jolt body was minted for this collider.
        static bool TryMirrorCollider(Collider c)
        {
            if (c.isTrigger) return false;
            if (!c.enabled) return false;
            if (c.gameObject == null || !c.gameObject.activeInHierarchy) return false;
            if (c.gameObject.layer != kLocalSceneryLayer)
            {
                // Surfaces wires / detail meshes occasionally land on
                // other layers; log so we can spot patterns if KSC
                // colliders are silently being skipped.
                if (Debug.isDebugBuild)
                    Debug.Log(LogPrefix + string.Format(
                        "skip {0}: layer {1} != Local Scenery",
                        c.name, c.gameObject.layer));
                return false;
            }

            // Idempotency — re-entry on world rebuild walks the same
            // hierarchy. If a StaticBody is already present, skip.
            if (c.gameObject.GetComponent<StaticBody>() != null) return false;

            var frame = CbFrame.Current();
            if (!frame.IsValid) return false;

            var world = LongeronAddon.ActiveWorld;
            if (world == null) return false;

            try
            {
                if (c is MeshCollider mc)   return MirrorMesh(world, frame, mc);
                if (c is BoxCollider box)   return MirrorBox(world, frame, box);
                if (c is SphereCollider sp) return MirrorSphere(world, frame, sp);
                // CapsuleCollider, WheelCollider, TerrainCollider:
                // skip — none in stock KSC. Wheel handled by Phase 3.5.
                if (Debug.isDebugBuild)
                    Debug.Log(LogPrefix + string.Format(
                        "skip {0}: unsupported collider type {1}",
                        c.name, c.GetType().Name));
                return false;
            }
            catch (Exception ex)
            {
                Debug.LogError(LogPrefix + string.Format(
                    "mirror failed for {0}: {1}: {2}",
                    c.name, ex.GetType().Name, ex.Message));
                return false;
            }
        }

        // -- MeshCollider -------------------------------------------------

        static bool MirrorMesh(World world, CbFrame frame, MeshCollider mc)
        {
            var mesh = mc.sharedMesh;
            if (mesh == null) return false;
            // Some mod meshes ship without CPU-side read access. We
            // can't pack their vertices; skip with a warning.
            if (!mesh.isReadable)
            {
                Debug.LogWarning(LogPrefix + "mesh not readable: " + mc.name);
                return false;
            }

            var localVerts = mesh.vertices;
            var triangles  = mesh.triangles;
            if (localVerts.Length == 0 || triangles.Length == 0) return false;

            // PQSCity prefabs frequently sit under scaled parent
            // transforms (Squad scales building shells per LOD, and
            // PQSCity2's Orientate path applies a scale to anchor the
            // city to the surface). Bake lossyScale into the packed
            // vertex array — Jolt's MeshShape doesn't carry a per-
            // sub-shape scale.
            Vector3 scale = mc.transform.lossyScale;
            int n = localVerts.Length;
            var packed = new float[n * 3];
            for (int i = 0; i < n; ++i)
            {
                packed[i * 3 + 0] = localVerts[i].x * scale.x;
                packed[i * 3 + 1] = localVerts[i].y * scale.y;
                packed[i * 3 + 2] = localVerts[i].z * scale.z;
            }

            // World-space pose of the collider's transform; the
            // packed vertices are already in (scaled) local axes, so
            // body pose = transform.position / .rotation in CB-frame.
            Vector3 worldPos = mc.transform.position;
            Quaternion worldRot = mc.transform.rotation;
            Vector3d posCb = frame.WorldToCb(new Vector3d(worldPos.x, worldPos.y, worldPos.z));
            QuaternionD rotCb = frame.WorldToCb((QuaternionD)worldRot);

            var handle = SceneRegistry.MintBodyHandle();
            world.Input.WriteBodyCreateTriangleMesh(
                handle, Layer.Static,
                packed, triangles,
                posX: posCb.x, posY: posCb.y, posZ: posCb.z,
                rotX: (float)rotCb.x, rotY: (float)rotCb.y,
                rotZ: (float)rotCb.z, rotW: (float)rotCb.w,
                groupId: 0);

            StaticBody.AttachTo(mc.gameObject, handle);
            return true;
        }

        // -- BoxCollider --------------------------------------------------

        static bool MirrorBox(World world, CbFrame frame, BoxCollider box)
        {
            // Box pose: collider's local center, transformed to world.
            // BoxCollider.center is in the transform's local frame
            // (already accounts for the transform but NOT the box's
            // own size — size is half-extents × 2).
            Vector3 worldCenter = box.transform.TransformPoint(box.center);
            Quaternion worldRot = box.transform.rotation;

            // Half-extents in world units: size × 0.5 × lossyScale,
            // axis-by-axis. BoxCollider doesn't support non-uniform
            // rotation between collider and transform; the local axes
            // align with the transform's, scaled.
            Vector3 scale = box.transform.lossyScale;
            float halfX = Mathf.Abs(box.size.x * 0.5f * scale.x);
            float halfY = Mathf.Abs(box.size.y * 0.5f * scale.y);
            float halfZ = Mathf.Abs(box.size.z * 0.5f * scale.z);
            // Jolt rejects degenerate boxes; sanity-clamp to a tiny
            // positive minimum.
            if (halfX < 1e-4f || halfY < 1e-4f || halfZ < 1e-4f)
            {
                Debug.LogWarning(LogPrefix + string.Format(
                    "degenerate box on {0}: half=({1},{2},{3}) — skipping",
                    box.name, halfX, halfY, halfZ));
                return false;
            }

            Vector3d posCb = frame.WorldToCb(new Vector3d(worldCenter.x, worldCenter.y, worldCenter.z));
            QuaternionD rotCb = frame.WorldToCb((QuaternionD)worldRot);

            var handle = SceneRegistry.MintBodyHandle();
            world.Input.WriteBodyCreateBox(
                handle, BodyType.Static, Layer.Static,
                halfX: halfX, halfY: halfY, halfZ: halfZ,
                posX: posCb.x, posY: posCb.y, posZ: posCb.z,
                rotX: (float)rotCb.x, rotY: (float)rotCb.y,
                rotZ: (float)rotCb.z, rotW: (float)rotCb.w,
                mass: 0f, groupId: 0);

            StaticBody.AttachTo(box.gameObject, handle);
            return true;
        }

        // -- SphereCollider -----------------------------------------------

        static bool MirrorSphere(World world, CbFrame frame, SphereCollider sp)
        {
            Vector3 worldCenter = sp.transform.TransformPoint(sp.center);
            // Spheres don't deform under non-uniform scale in Unity's
            // collision math; the largest-axis scale is what stock
            // PhysX uses for the effective radius.
            Vector3 scale = sp.transform.lossyScale;
            float maxScale = Mathf.Max(Mathf.Abs(scale.x),
                                Mathf.Max(Mathf.Abs(scale.y), Mathf.Abs(scale.z)));
            float radius = sp.radius * maxScale;
            if (radius < 1e-4f)
            {
                Debug.LogWarning(LogPrefix + string.Format(
                    "degenerate sphere on {0}: r={1} — skipping",
                    sp.name, radius));
                return false;
            }

            Vector3d posCb = frame.WorldToCb(new Vector3d(worldCenter.x, worldCenter.y, worldCenter.z));
            // Sphere rotation is irrelevant; use identity for clarity.
            var handle = SceneRegistry.MintBodyHandle();
            world.Input.WriteBodyCreateSphere(
                handle, BodyType.Static, Layer.Static,
                radius: radius,
                posX: posCb.x, posY: posCb.y, posZ: posCb.z,
                rotX: 0f, rotY: 0f, rotZ: 0f, rotW: 1f,
                mass: 0f, groupId: 0);

            StaticBody.AttachTo(sp.gameObject, handle);
            return true;
        }
    }
}
