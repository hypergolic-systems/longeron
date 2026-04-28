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
        public static bool WriteBodyFor(
            InputBuffer input,
            BodyHandle handle,
            Part part,
            BodyType bodyType,
            Layer layer,
            float massKg)
        {
            // Part transform anchors the body's frame.
            var partXform = part.transform;
            var partWorldPos = partXform.position;
            var partWorldRot = partXform.rotation;

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
                posX: partWorldPos.x, posY: partWorldPos.y, posZ: partWorldPos.z,
                rotX: partWorldRot.x, rotY: partWorldRot.y, rotZ: partWorldRot.z, rotW: partWorldRot.w,
                mass: massKg,
                shapeCount: (byte)subShapes.Count);

            foreach (var sub in subShapes)
            {
                AppendSubShape(input, sub);
            }

            return true;
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
