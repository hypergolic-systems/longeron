// ColliderWalker — KSP Part → Jolt sub-shape list.
//
// Walks every Collider under a Part's transform hierarchy, classifies
// each into a Jolt-supported shape kind, and emits a BodyCreate record
// (multi-sub-shape via the InputBuffer Begin/Append protocol).
//
// Phase 1.5 supported shapes:
//   - BoxCollider     → ShapeKind.Box
//   - SphereCollider  → ShapeKind.Sphere
//   - MeshCollider    → ShapeKind.ConvexHull (vertices baked into part frame)
//
// Skipped (warned) for now:
//   - any collider with isTrigger == true (game-logic, not physics)
//   - disabled collider GameObjects
//   - WheelCollider (Phase 3.5 owns wheels)
//   - CapsuleCollider (Phase 2)
//   - MeshCollider with convex == false (no equivalent within Phase 1.5)
//
// Coordinate convention: each sub-shape's local transform places the
// shape's natural frame inside the body's frame (= the part's
// transform). For Box/Sphere we send the natural half-extents/radius
// and rely on sub_pos/sub_rot. For ConvexHull we bake the full
// collider→part transform (including lossyScale) into the vertices and
// send sub_pos = (0,0,0), sub_rot = identity. This sidesteps the
// non-uniform-scale problem on hulls — at the cost of N matrix
// multiplications per build, which is fine since vessel-build is rare.

using System.Collections.Generic;
using Longeron.Native;
using UnityEngine;

namespace Longeron.Integration
{
    internal static class ColliderWalker
    {
        const string LogPrefix = "[Longeron/colliders] ";

        // Returns true if at least one sub-shape was emitted; false if
        // the part contributed no usable colliders (caller should not
        // create a body in that case).
        //
        // The part's anchor pose is captured in Unity world and converted
        // to CB-frame via the supplied CbFrame before submission. Sub-
        // shape transforms are part-local and frame-invariant.
        // Single-body-per-vessel: build one Jolt body whose compound
        // shape contains every part's colliders, all transformed into
        // the vessel root part's local frame. Body anchor pose = root
        // part's transform (CB-frame).
        //
        // Caller passes a pre-allocated dict that will be populated
        // with per-part (LocalPos, LocalRot) offsets relative to the
        // root — used by LongeronSceneDriver to derive each part's
        // Unity-world pose from the single body's pose readback.
        //
        // Returns true if the body was emitted; false if the vessel
        // had no usable colliders (caller should skip registration).
        public static bool WriteBodyForVessel(
            InputBuffer input,
            BodyHandle handle,
            Vessel vessel,
            BodyType bodyType,
            Layer layer,
            float totalMassKg,
            uint groupId,
            CbFrame frame,
            Dictionary<Part, ManagedVessel.PartOffset> outPartOffsets)
        {
            if (vessel == null || vessel.rootPart == null) return false;
            var root = vessel.rootPart;
            var rootXform = root.transform;
            if (rootXform == null) return false;

            // Body anchor = root part's transform in CB-frame.
            Vector3d rootCbPos = frame.WorldToCb(new Vector3d(
                rootXform.position.x, rootXform.position.y, rootXform.position.z));
            QuaternionD rootCbRot = frame.WorldToCb((QuaternionD)rootXform.rotation);

            // Initial vessel velocity in CB-frame. Stock unpack
            // (Vessel.GoOffRails → Part.SetWorldVelocity) writes
            // rb.velocity / rb.angularVelocity from orbit propagation,
            // so by the time the reconciler runs these reflect the
            // pre-pack velocity. Without this, every rails round-trip
            // resets the body to rest in Jolt and the vessel re-falls
            // from gravity — visible as "I lost all my velocity coming
            // out of time warp".
            Vector3 rootRbVel = Vector3.zero;
            Vector3 rootRbAngVel = Vector3.zero;
            if (root.rb != null)
            {
                rootRbVel    = root.rb.velocity;
                rootRbAngVel = root.rb.angularVelocity;
            }
            Vector3d linCb = frame.WorldVelToCb(
                new Vector3d(rootRbVel.x, rootRbVel.y, rootRbVel.z),
                new Vector3d(rootXform.position.x, rootXform.position.y, rootXform.position.z));
            Vector3d angCb = frame.WorldAngVelToCb(
                new Vector3d(rootRbAngVel.x, rootRbAngVel.y, rootRbAngVel.z));

            var subShapes = new List<SubShape>(64);
            outPartOffsets.Clear();

            // Phase 5 ABA: walk parts in BFS order (matching
            // EmitVesselTree) and emit EXACTLY ONE SubShape per part so
            // sub_shape_index == part_idx in the tree. The ABA forward
            // pass writes per-part poses by index via
            // MutableCompoundShape::ModifyShape on this aligned mapping.
            //
            // v1 limitation (TODO): multi-collider parts (Mk1 pod hatch,
            // some engines, fairings) lose their secondary colliders —
            // only the first usable collider on each part participates.
            // For collision fidelity on those parts, build an inner
            // StaticCompoundShape per part as a Phase 5.x extension.
            // Parts with no usable collider get a tiny sphere placeholder
            // so the index alignment holds.
            var bfsOrder = ListPartsBfs(vessel.rootPart);
            foreach (var part in bfsOrder)
            {
                if (part == null) continue;
                var partXform = part.transform;
                if (partXform == null) continue;

                // Record this part's offset from root, captured in
                // root-local space and frozen until the next rebuild.
                var offset = new ManagedVessel.PartOffset
                {
                    LocalPos = rootXform.InverseTransformPoint(partXform.position),
                    LocalRot = Quaternion.Inverse(rootXform.rotation) * partXform.rotation,
                };
                outPartOffsets[part] = offset;

                // Find this part's first usable collider; project into
                // ROOT-local space so the shape's pose is in the body's
                // anchor frame.
                SubShape sub = default;
                bool found = false;
                var colliders = part.GetComponentsInChildren<Collider>(includeInactive: true);
                foreach (var col in colliders)
                {
                    if (col == null) continue;
                    if (col.isTrigger) continue;
                    if (!col.enabled) continue;
                    if (col.gameObject == null || !col.gameObject.activeInHierarchy) continue;
                    if (col.GetType().Name == "WheelCollider") continue;  // Phase 3.5

                    if (TryClassify(col, rootXform, out sub))
                    {
                        found = true;
                        break;
                    }
                }
                if (!found)
                {
                    // Tiny placeholder so SubShape index aligns with
                    // part BFS index. Located at the part's CoM in
                    // root-local space; 1 cm sphere is small enough to
                    // never trigger meaningful contact for parts that
                    // wouldn't have had collision anyway.
                    sub.Kind = ShapeKind.Sphere;
                    sub.PosX = offset.LocalPos.x;
                    sub.PosY = offset.LocalPos.y;
                    sub.PosZ = offset.LocalPos.z;
                    sub.RotX = 0f; sub.RotY = 0f; sub.RotZ = 0f; sub.RotW = 1f;
                    sub.P0 = 0.01f;
                }
                subShapes.Add(sub);
            }

            if (subShapes.Count == 0)
            {
                Debug.Log(LogPrefix + $"vessel '{vessel.vesselName}' has no parts — skipping body");
                return false;
            }

            if (subShapes.Count > 255)
            {
                Debug.LogWarning(LogPrefix + $"vessel '{vessel.vesselName}' has {subShapes.Count} parts, capping at 255 (ABA index alignment will fail beyond this)");
                subShapes.RemoveRange(255, subShapes.Count - 255);
            }

            input.BeginBodyCreate(
                handle, bodyType, layer,
                posX: rootCbPos.x, posY: rootCbPos.y, posZ: rootCbPos.z,
                rotX: (float)rootCbRot.x, rotY: (float)rootCbRot.y,
                rotZ: (float)rootCbRot.z, rotW: (float)rootCbRot.w,
                mass: totalMassKg,
                shapeCount: (byte)subShapes.Count,
                linVx: (float)linCb.x, linVy: (float)linCb.y, linVz: (float)linCb.z,
                angVx: (float)angCb.x, angVy: (float)angCb.y, angVz: (float)angCb.z,
                groupId: groupId);

            foreach (var sub in subShapes)
                AppendSubShape(input, sub);

            return true;
        }

        public static bool WriteBodyFor(
            InputBuffer input,
            BodyHandle handle,
            Part part,
            BodyType bodyType,
            Layer layer,
            float massKg,
            uint groupId,
            CbFrame frame)
        {
            // Part transform anchors the body's frame, expressed in
            // CB-fixed coords.
            var partXform = part.transform;
            Vector3d partCbPos = frame.WorldToCb(new Vector3d(
                partXform.position.x, partXform.position.y, partXform.position.z));
            QuaternionD partCbRot = frame.WorldToCb((QuaternionD)partXform.rotation);

            // First pass: enumerate + classify, compute total count for
            // BeginBodyCreate's shape_count byte. Re-walking is cheap;
            // it lets us stream into the buffer in a single Append loop
            // without a back-patch.
            var colliders = part.GetComponentsInChildren<Collider>(includeInactive: true);
            var subShapes = new List<SubShape>(colliders.Length);

            foreach (var col in colliders)
            {
                if (col == null) continue;
                if (col.isTrigger) continue;
                if (!col.enabled) continue;
                if (col.gameObject == null || !col.gameObject.activeInHierarchy) continue;

                // WheelCollider lives in UnityEngine.VehiclesModule
                // (which we don't reference). Identify by type name to
                // avoid the assembly dependency.
                if (col.GetType().Name == "WheelCollider")
                {
                    Debug.Log(LogPrefix + $"skipping WheelCollider on '{part.partInfo?.name ?? "?"}' (Phase 3.5)");
                    continue;
                }

                SubShape sub;
                if (TryClassify(col, partXform, out sub))
                    subShapes.Add(sub);
            }

            if (subShapes.Count == 0)
            {
                Debug.Log(LogPrefix + $"part '{part.partInfo?.name ?? "?"}' has no usable colliders — skipping body");
                return false;
            }

            if (subShapes.Count > 255)
            {
                Debug.LogWarning(LogPrefix + $"part '{part.partInfo?.name ?? "?"}' has {subShapes.Count} colliders, capping at 255");
                subShapes.RemoveRange(255, subShapes.Count - 255);
            }

            input.BeginBodyCreate(
                handle, bodyType, layer,
                posX: partCbPos.x, posY: partCbPos.y, posZ: partCbPos.z,
                rotX: (float)partCbRot.x, rotY: (float)partCbRot.y,
                rotZ: (float)partCbRot.z, rotW: (float)partCbRot.w,
                mass: massKg,
                shapeCount: (byte)subShapes.Count,
                groupId: groupId);

            foreach (var sub in subShapes)
            {
                AppendSubShape(input, sub);
            }

            return true;
        }

        // BFS-flatten the part tree starting from root. Same order as
        // TopologyReconciler.EmitVesselTree, so the SubShape index
        // matches the VesselTree's part_idx 1:1 — needed by the ABA
        // forward pass to ModifyShape per part by tree index.
        static List<Part> _bfsScratch = new List<Part>(128);
        static HashSet<Part> _bfsSeen = new HashSet<Part>();
        internal static List<Part> ListPartsBfs(Part root)
        {
            _bfsScratch.Clear();
            _bfsSeen.Clear();
            if (root == null) return _bfsScratch;
            _bfsScratch.Add(root);
            _bfsSeen.Add(root);
            for (int head = 0; head < _bfsScratch.Count; ++head)
            {
                var p = _bfsScratch[head];
                if (p == null) continue;
                foreach (var child in p.children)
                {
                    if (child == null || _bfsSeen.Contains(child)) continue;
                    _bfsSeen.Add(child);
                    _bfsScratch.Add(child);
                }
            }
            return _bfsScratch;
        }

        // -- Internal: typed sub-shape record built per collider ---------

        struct SubShape
        {
            public ShapeKind Kind;
            public float PosX, PosY, PosZ;
            public float RotX, RotY, RotZ, RotW;
            // Box: half-extents.  Sphere: P0 = radius.
            public float P0, P1, P2;
            // ConvexHull: vertices baked into part-local frame, xyz packed.
            public float[] Vertices;
        }

        static bool TryClassify(Collider col, Transform partXform, out SubShape sub)
        {
            sub = default;

            // ---- Box ----------------------------------------------------
            if (col is BoxCollider box)
            {
                // Box center in world space; project into part-local frame.
                Vector3 worldCenter = box.transform.TransformPoint(box.center);
                Vector3 localPos    = partXform.InverseTransformPoint(worldCenter);
                Quaternion localRot = Quaternion.Inverse(partXform.rotation) * box.transform.rotation;

                Vector3 lossyScale = box.transform.lossyScale;
                Vector3 halfExtents = new Vector3(
                    Mathf.Abs(box.size.x * lossyScale.x) * 0.5f,
                    Mathf.Abs(box.size.y * lossyScale.y) * 0.5f,
                    Mathf.Abs(box.size.z * lossyScale.z) * 0.5f);

                sub.Kind = ShapeKind.Box;
                sub.PosX = localPos.x; sub.PosY = localPos.y; sub.PosZ = localPos.z;
                sub.RotX = localRot.x; sub.RotY = localRot.y; sub.RotZ = localRot.z; sub.RotW = localRot.w;
                sub.P0 = halfExtents.x; sub.P1 = halfExtents.y; sub.P2 = halfExtents.z;
                return true;
            }

            // ---- Sphere -------------------------------------------------
            if (col is SphereCollider sph)
            {
                Vector3 worldCenter = sph.transform.TransformPoint(sph.center);
                Vector3 localPos    = partXform.InverseTransformPoint(worldCenter);

                Vector3 ls = sph.transform.lossyScale;
                float scale = Mathf.Max(Mathf.Abs(ls.x), Mathf.Abs(ls.y), Mathf.Abs(ls.z));
                if (Mathf.Abs(ls.x - ls.y) > 1e-3f || Mathf.Abs(ls.y - ls.z) > 1e-3f)
                {
                    Debug.LogWarning(LogPrefix + $"non-uniform scale on SphereCollider; using max-axis scale (radius will be inflated)");
                }
                float radius = Mathf.Abs(sph.radius * scale);

                sub.Kind = ShapeKind.Sphere;
                sub.PosX = localPos.x; sub.PosY = localPos.y; sub.PosZ = localPos.z;
                sub.RotX = 0; sub.RotY = 0; sub.RotZ = 0; sub.RotW = 1;
                sub.P0 = radius;
                return true;
            }

            // ---- Mesh (convex only) ------------------------------------
            if (col is MeshCollider mesh)
            {
                if (!mesh.convex)
                {
                    Debug.LogWarning(LogPrefix + $"non-convex MeshCollider on a managed part — skipping (Phase 1.5)");
                    return false;
                }
                var sharedMesh = mesh.sharedMesh;
                if (sharedMesh == null) return false;
                var verts = sharedMesh.vertices;
                if (verts == null || verts.Length == 0) return false;

                // Bake the full mesh→part transform into each vertex —
                // includes lossyScale, so non-uniformly scaled hulls are
                // correctly reshaped before reaching Jolt.
                int n = verts.Length;
                if (n > ShapeLimits.MaxConvexHullVertices)
                {
                    Debug.LogWarning(LogPrefix + $"MeshCollider has {n} verts; truncating to {ShapeLimits.MaxConvexHullVertices}");
                    n = ShapeLimits.MaxConvexHullVertices;
                }
                var packed = new float[n * 3];
                for (int i = 0; i < n; i++)
                {
                    Vector3 worldV = mesh.transform.TransformPoint(verts[i]);
                    Vector3 localV = partXform.InverseTransformPoint(worldV);
                    packed[i * 3 + 0] = localV.x;
                    packed[i * 3 + 1] = localV.y;
                    packed[i * 3 + 2] = localV.z;
                }

                sub.Kind = ShapeKind.ConvexHull;
                // Vertices are baked into part-local frame already → identity sub-transform.
                sub.PosX = 0; sub.PosY = 0; sub.PosZ = 0;
                sub.RotX = 0; sub.RotY = 0; sub.RotZ = 0; sub.RotW = 1;
                sub.Vertices = packed;
                return true;
            }

            // ---- Capsule ------------------------------------------------
            if (col is CapsuleCollider)
            {
                Debug.LogWarning(LogPrefix + "skipping CapsuleCollider (Phase 2)");
                return false;
            }

            // ---- Anything else -----------------------------------------
            Debug.LogWarning(LogPrefix + $"unknown collider type {col.GetType().Name} — skipping");
            return false;
        }

        static void AppendSubShape(InputBuffer input, SubShape s)
        {
            switch (s.Kind)
            {
                case ShapeKind.Box:
                    input.AppendShapeBox(
                        s.PosX, s.PosY, s.PosZ,
                        s.RotX, s.RotY, s.RotZ, s.RotW,
                        s.P0, s.P1, s.P2);
                    break;
                case ShapeKind.Sphere:
                    input.AppendShapeSphere(
                        s.PosX, s.PosY, s.PosZ,
                        s.RotX, s.RotY, s.RotZ, s.RotW,
                        s.P0);
                    break;
                case ShapeKind.ConvexHull:
                    input.AppendShapeConvexHull(
                        s.PosX, s.PosY, s.PosZ,
                        s.RotX, s.RotY, s.RotZ, s.RotW,
                        s.Vertices);
                    break;
            }
        }
    }
}
