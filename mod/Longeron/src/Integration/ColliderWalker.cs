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
            Dictionary<Part, ManagedVessel.PartOffset> outPartOffsets,
            List<ushort> outSubShapeMap,
            List<string> outSubShapeColliderNames)
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
            var subShapeToPart = new List<ushort>(64);
            var subShapeColliderNames = new List<string>(64);
            outPartOffsets.Clear();
            if (outSubShapeMap != null) outSubShapeMap.Clear();
            if (outSubShapeColliderNames != null) outSubShapeColliderNames.Clear();

            // Phase 5 ABA: walk parts in BFS order (matching
            // EmitVesselTree) so the per-shape part_idx is the BFS
            // index. Each part may emit MULTIPLE SubShapes — one per
            // valid collider — preserving Phase 4's mass distribution
            // for Jolt's auto-CoM computation. ABA's ModifyShapes
            // applies the same flex transform to every SubShape that
            // shares a part_idx (see native side's subShapeToPart map).
            var bfsOrder = ListPartsBfs(vessel.rootPart);
            ushort partIdx = 0;
            foreach (var part in bfsOrder)
            {
                if (part == null) { partIdx++; continue; }
                var partXform = part.transform;
                if (partXform == null) { partIdx++; continue; }

                // Record this part's offset from root, captured in
                // root-local space and frozen until the next rebuild.
                var offset = new ManagedVessel.PartOffset
                {
                    LocalPos = rootXform.InverseTransformPoint(partXform.position),
                    LocalRot = Quaternion.Inverse(rootXform.rotation) * partXform.rotation,
                };
                outPartOffsets[part] = offset;

                // Walk this part's colliders and project them into
                // ROOT-local space so all sub-shapes share a single
                // body frame. We classify into the partSubs scratch
                // so we can compute total volume across this part's
                // sub-shapes once, then stamp the same density on
                // every sub-shape (so per-part mass distributes
                // proportionally to volume across its colliders).
                _partSubsScratch.Clear();
                _partSubColliderNamesScratch.Clear();
                var colliders = part.GetComponentsInChildren<Collider>(includeInactive: true);
                foreach (var col in colliders)
                {
                    if (col == null) continue;
                    if (col.isTrigger) continue;
                    if (!col.enabled) continue;
                    if (col.gameObject == null || !col.gameObject.activeInHierarchy) continue;
                    if (col.GetType().Name == "WheelCollider") continue;  // Phase 3.5

                    if (TryClassify(col, rootXform, out var sub))
                    {
                        _partSubsScratch.Add(sub);
                        _partSubColliderNamesScratch.Add(col.gameObject.name);
                    }
                }

                if (_partSubsScratch.Count > 0)
                {
                    // density (tonnes/m³) = part_mass / Σ sub-shape volume.
                    // All sub-shapes of this part share the same density,
                    // so per-sub-shape mass = sub_volume × density adds
                    // back up to part_mass, with the volume-weighted CoM
                    // Jolt computes via the compound aggregation.
                    float partMass = part.mass + part.GetResourceMass();
                    float partVol = 0f;
                    for (int i = 0; i < _partSubsScratch.Count; ++i)
                        partVol += SubShapeVolume(_partSubsScratch[i]);
                    float density = (partMass > 1e-9f && partVol > 1e-9f)
                        ? partMass / partVol
                        : 0f;  // sentinel: native uses Jolt default

                    for (int i = 0; i < _partSubsScratch.Count; ++i)
                    {
                        var sub = _partSubsScratch[i];
                        sub.Density = density;
                        subShapes.Add(sub);
                        subShapeToPart.Add(partIdx);
                        subShapeColliderNames.Add(_partSubColliderNamesScratch[i]);
                    }
                }
                // Spawn snapshot: log this part's transform pose and
                // each collider's WORLD AABB, so we can correlate per-
                // part placement with the launch-time pad geometry.
                Debug.Log(LogPrefix + string.Format(
                    "  part[{0}] '{1}': {2} colliders → {3} sub-shapes, mass={4:F3}t, transform={5}, partLocal={6}",
                    partIdx,
                    part.partInfo != null ? part.partInfo.name : part.name,
                    colliders != null ? colliders.Length : 0,
                    _partSubsScratch.Count,
                    part.mass + part.GetResourceMass(),
                    part.transform.position.ToString("F4"),
                    offset.LocalPos.ToString("F4")));
                if (colliders != null)
                {
                    for (int ci = 0; ci < colliders.Length; ++ci)
                    {
                        var c = colliders[ci];
                        if (c == null) continue;
                        bool used = c.enabled && !c.isTrigger
                                    && c.gameObject != null
                                    && c.gameObject.activeInHierarchy
                                    && c.GetType().Name != "WheelCollider";
                        Debug.Log(LogPrefix + string.Format(
                            "    [{0}] {1} type={2} convex={3} used={4} bounds.center={5} extents={6}",
                            ci, c.name, c.GetType().Name,
                            (c is MeshCollider mci) ? mci.convex.ToString() : "n/a",
                            used,
                            c.bounds.center.ToString("F3"),
                            c.bounds.extents.ToString("F3")));
                    }
                }
                partIdx++;
            }

            if (subShapes.Count == 0)
            {
                Debug.Log(LogPrefix + $"vessel '{vessel.vesselName}' has no usable colliders — skipping body");
                return false;
            }

            if (subShapes.Count > 255)
            {
                Debug.LogWarning(LogPrefix + $"vessel '{vessel.vesselName}' has {subShapes.Count} colliders, capping at 255");
                subShapes.RemoveRange(255, subShapes.Count - 255);
                subShapeToPart.RemoveRange(255, subShapeToPart.Count - 255);
            }

            // Phase 5 diag: log spawn position + initial velocity on
            // every body create so we can correlate post-decouple
            // surprise trajectories with what the bridge was actually
            // told. Remove once decouple flow stabilizes.
            Debug.Log(LogPrefix + string.Format(
                "spawn '{0}' root='{1}' rootWorld=({2:F2},{3:F2},{4:F2}) altWorld={5:F2} rb.vel=({6:F2},{7:F2},{8:F2}) |v|={9:F2}",
                vessel.vesselName,
                root.partInfo != null ? root.partInfo.name : root.name,
                rootXform.position.x, rootXform.position.y, rootXform.position.z,
                rootXform.position.y,
                rootRbVel.x, rootRbVel.y, rootRbVel.z,
                rootRbVel.magnitude));

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

            // Phase 5 ABA: SubShape→part_idx so the native side can
            // apply each part's flex transform to every collider that
            // belongs to it. Sent right after BodyCreate so the
            // ordering matches the per-shape iteration order.
            input.WriteSubShapeMap(handle, subShapeToPart.ToArray());
            if (outSubShapeMap != null)
            {
                for (int i = 0; i < subShapeToPart.Count; ++i)
                    outSubShapeMap.Add(subShapeToPart[i]);
            }
            if (outSubShapeColliderNames != null)
            {
                for (int i = 0; i < subShapeColliderNames.Count; ++i)
                    outSubShapeColliderNames.Add(subShapeColliderNames[i]);
            }

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
            // Convex-hull volume cached at TryClassify time (computed
            // from the source mesh's triangles via the divergence
            // theorem). Avoids C# polyhedron-volume code in the hot
            // density loop.
            public float HullVolume;
            // Density set per part in WriteBodyForVessel after all this
            // part's sub-shapes are classified (density = part_mass /
            // Σ sub-shape volume so the per-part mass distributes
            // proportionally to volume across colliders). Wire format
            // attaches it to each AppendShape so Jolt's CompoundShape
            // aggregates body mass + CoM + inertia from per-sub-shape
            // MassProperties. Density 0 = use Jolt default 1000 kg/m³.
            public float Density;
        }

        // Per-part scratch list — collects this part's classified sub-
        // shapes so we can compute Σ volume → density before pushing
        // them onto the body's flat sub-shape stream. Cleared per part.
        static readonly List<SubShape> _partSubsScratch = new List<SubShape>(8);
        // Parallel: collider GameObject names for the same per-part
        // scratch entries. Used for contact-diag resolution.
        static readonly List<string> _partSubColliderNamesScratch = new List<string>(8);

        // Sub-shape volume (m³ in part-local frame after lossyScale
        // bake). Used by WriteBodyForVessel to compute per-part
        // density. Returns 0 for unsupported shape kinds; caller
        // treats that as "skip volume contribution".
        static float SubShapeVolume(SubShape s)
        {
            switch (s.Kind)
            {
                case ShapeKind.Box:
                    // Box half-extents → 8 · hx · hy · hz.
                    return 8f * s.P0 * s.P1 * s.P2;
                case ShapeKind.Sphere:
                    // 4/3 · π · r³.
                    return (4f / 3f) * Mathf.PI * s.P0 * s.P0 * s.P0;
                case ShapeKind.ConvexHull:
                    return s.HullVolume;
                default:
                    return 0f;
            }
        }

        // Closed-mesh volume via the divergence theorem: sum of signed
        // tetrahedra (origin, v0, v1, v2) over every triangle. For a
        // closed surface with outward-oriented faces this gives the
        // enclosed volume; for nearly-convex KSP collider meshes it
        // matches the convex hull's volume to within float precision.
        // Returns the absolute value to be sign-agnostic about winding.
        // Skips triangles whose indices reference outside `verts` —
        // defensive against a caller that truncates verts but passes
        // the original triangle indices.
        static float ClosedMeshVolume(Vector3[] verts, int[] tris)
        {
            if (verts == null || tris == null) return 0f;
            int vn = verts.Length;
            float vol = 0f;
            for (int i = 0; i + 2 < tris.Length; i += 3)
            {
                int i0 = tris[i], i1 = tris[i + 1], i2 = tris[i + 2];
                if ((uint)i0 >= (uint)vn || (uint)i1 >= (uint)vn || (uint)i2 >= (uint)vn)
                    continue;
                Vector3 v0 = verts[i0];
                Vector3 v1 = verts[i1];
                Vector3 v2 = verts[i2];
                vol += Vector3.Dot(v0, Vector3.Cross(v1, v2));
            }
            return Mathf.Abs(vol) / 6f;
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

                // Compute hull volume on the FULL vertex set first —
                // the triangle indices reference the original mesh, so
                // truncating verts before this would IndexOutOfRange
                // on any index above the cap. Transform every vert
                // into part-local frame (the volume is invariant under
                // rigid transforms but scales by lossyScale, so we
                // need the same frame Jolt sees).
                int rawCount = verts.Length;
                var localVertsFull = new Vector3[rawCount];
                for (int i = 0; i < rawCount; i++)
                {
                    Vector3 worldV = mesh.transform.TransformPoint(verts[i]);
                    localVertsFull[i] = partXform.InverseTransformPoint(worldV);
                }
                int[] tris = sharedMesh.triangles;
                float hullVol = ClosedMeshVolume(localVertsFull, tris);

                // Now truncate for the wire format. Jolt's convex-hull
                // builder caps vertex count for performance; the
                // truncated set still bounds an approximate hull. The
                // volume (above) is from the FULL set, so density
                // stays accurate even when the wire-side hull is
                // simplified.
                int n = rawCount;
                if (n > ShapeLimits.MaxConvexHullVertices)
                {
                    Debug.LogWarning(LogPrefix + $"MeshCollider has {n} verts; truncating to {ShapeLimits.MaxConvexHullVertices}");
                    n = ShapeLimits.MaxConvexHullVertices;
                }
                var packed = new float[n * 3];
                for (int i = 0; i < n; i++)
                {
                    packed[i * 3 + 0] = localVertsFull[i].x;
                    packed[i * 3 + 1] = localVertsFull[i].y;
                    packed[i * 3 + 2] = localVertsFull[i].z;
                }

                sub.Kind = ShapeKind.ConvexHull;
                // Vertices are baked into part-local frame already → identity sub-transform.
                sub.PosX = 0; sub.PosY = 0; sub.PosZ = 0;
                sub.RotX = 0; sub.RotY = 0; sub.RotZ = 0; sub.RotW = 1;
                sub.Vertices = packed;
                sub.HullVolume = hullVol;
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
                        s.P0, s.P1, s.P2,
                        density: s.Density);
                    break;
                case ShapeKind.Sphere:
                    input.AppendShapeSphere(
                        s.PosX, s.PosY, s.PosZ,
                        s.RotX, s.RotY, s.RotZ, s.RotW,
                        s.P0,
                        density: s.Density);
                    break;
                case ShapeKind.ConvexHull:
                    input.AppendShapeConvexHull(
                        s.PosX, s.PosY, s.PosZ,
                        s.RotX, s.RotY, s.RotZ, s.RotW,
                        s.Vertices,
                        density: s.Density);
                    break;
            }
        }
    }
}
