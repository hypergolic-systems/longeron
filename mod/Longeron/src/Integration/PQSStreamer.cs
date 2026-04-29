// PQSStreamer — mirror KSP's PQS terrain quads into Jolt static
// MeshShape bodies as the player moves around.
//
// Two pieces:
//   1. LongeronPQSMod : PQSMod — added to every loaded PQS so we
//      receive OnQuadBuilt / OnQuadDestroy callbacks naturally,
//      without Harmony patches on stock PQS internals.
//   2. Streamer — static helpers handling per-quad lifecycle. Mints
//      BodyHandles, packs vertex / triangle data into the bridge
//      input buffer, and attaches a QuadBody MonoBehaviour to each
//      mirrored quad's GameObject.
//
// Stock PQSCache POOLS quads (PQSCache.DestroyQuad SetActive(false)s
// the GameObject and returns the PQ to a free list). Unity's
// OnDestroy doesn't fire on quad recycling, so we can't rely on
// QuadBody.OnDestroy alone — Streamer.OnQuadDestroy explicitly
// destroys the QuadBody component, which DOES fire OnDestroy and
// queues the BodyDestroy.

using System;
using System.Collections.Generic;
using System.Reflection;
using Longeron.Native;
using UnityEngine;

namespace Longeron.Integration
{
    /// <summary>
    /// Per-PQS hook receiver. One instance per loaded CelestialBody's
    /// PQS. All work delegates to the static <see cref="Streamer"/>.
    /// </summary>
    public sealed class LongeronPQSMod : PQSMod
    {
        public override void OnQuadBuilt(PQ quad) => Streamer.OnQuadBuilt(quad);
        public override void OnQuadUpdate(PQ quad) => Streamer.OnQuadUpdate(quad);
        public override void OnQuadDestroy(PQ quad) => Streamer.OnQuadDestroy(quad);
    }

    internal static class Streamer
    {
        const string LogPrefix = "[Longeron/pqs] ";

        // Initial sweep at AttachToAllPQS time is bounded; track to
        // log a one-line summary rather than per-quad spam.
        static int _initialMirrorCount;

        /// <summary>
        /// Walk every loaded CelestialBody, find its
        /// <c>pqsController</c>, and inject a <see cref="LongeronPQSMod"/>
        /// so we receive lifecycle callbacks. Then walk the existing
        /// quad tree and mirror any already-built quads — without this,
        /// terrain quads that built before our hook was installed
        /// (most of the near-vessel terrain at scene load) would never
        /// be mirrored until the player moves enough to retrigger
        /// OnQuadBuilt.
        /// </summary>
        public static void AttachToAllPQS()
        {
            if (FlightGlobals.Bodies == null) return;

            int pqsAttached = 0;
            _initialMirrorCount = 0;

            foreach (var body in FlightGlobals.Bodies)
            {
                if (body == null) continue;
                var pqs = body.pqsController;
                if (pqs == null) continue;

                if (TryInjectMod(pqs))
                {
                    pqsAttached++;
                    SweepExistingQuads(pqs);
                }
            }

            Debug.Log(LogPrefix + string.Format(
                "AttachToAllPQS: injected mod into {0} PQS, mirrored {1} existing quads",
                pqsAttached, _initialMirrorCount));
        }

        // PQS.mods and PQS.ResetModList() are both private/internal.
        // Stock 1.12.5 doesn't expose a public path to inject a mod
        // post-setup, so we use reflection. KSP is a frozen target —
        // the byte layout of these private members is permanent.
        static readonly FieldInfo s_modsField =
            typeof(PQS).GetField("mods", BindingFlags.Instance | BindingFlags.NonPublic);
        static readonly MethodInfo s_resetModList =
            typeof(PQS).GetMethod("ResetModList", BindingFlags.Instance | BindingFlags.NonPublic);

        static bool TryInjectMod(PQS pqs)
        {
            // Already injected? (re-entry on scene reload)
            if (s_modsField != null
                && s_modsField.GetValue(pqs) is PQSMod[] existing)
            {
                for (int i = 0; i < existing.Length; ++i)
                {
                    if (existing[i] is LongeronPQSMod) return false;
                }
            }

            // Stock PQSMods are MonoBehaviours on child GameObjects of
            // the PQS transform. Drop our component on a child holder;
            // the PQS picks up child PQSMods via GetComponents in
            // SetupMods / ResetModList.
            var holder = new GameObject("Longeron_PQSMod");
            holder.transform.SetParent(pqs.transform, worldPositionStays: false);
            var mod = holder.AddComponent<LongeronPQSMod>();
            mod.sphere = pqs;
            mod.modEnabled = true;
            // Run later than stock mods so the mesh is finalized when
            // OnQuadBuilt fires for us.
            mod.order = 1000;
            mod.OnSetup();

            // Force PQS to rebuild its private mods array including
            // the new child component. ResetModList is internal so
            // reflection again.
            if (s_resetModList != null)
            {
                s_resetModList.Invoke(pqs, null);
            }
            else
            {
                Debug.LogWarning(LogPrefix + "ResetModList reflection lookup failed; "
                                 + "OnQuadBuilt callbacks may not fire");
            }
            return true;
        }

        static void SweepExistingQuads(PQS pqs)
        {
            if (pqs.quads == null) return;
            for (int i = 0; i < pqs.quads.Length; ++i)
                SweepQuadRecursive(pqs.quads[i]);
        }

        static void SweepQuadRecursive(PQ quad)
        {
            if (quad == null) return;
            if (quad.isBuilt && quad.meshCollider != null && quad.meshCollider.sharedMesh != null)
            {
                if (TryMirrorQuad(quad))
                    _initialMirrorCount++;
            }
            if (quad.subNodes != null)
            {
                for (int i = 0; i < quad.subNodes.Length; ++i)
                    SweepQuadRecursive(quad.subNodes[i]);
            }
        }

        // ---- PQSMod callbacks ----

        public static void OnQuadBuilt(PQ quad)
        {
            if (LongeronAddon.ActiveWorld == null) return;
            if (quad == null || quad.gameObject == null) return;

            // PQSMod_QuadMeshColliders runs at minLevel <= subdivision.
            // If this quad doesn't carry a collider, skip — Jolt only
            // cares about collidable surfaces.
            var mc = quad.gameObject.GetComponent<MeshCollider>();
            if (mc == null || !mc.enabled || mc.sharedMesh == null) return;

            // If we already mirrored this quad and the mesh hasn't
            // changed shape, no work to do. (PQSMod_QuadMeshColliders
            // sometimes re-fires OnQuadBuilt on the same mesh data.)
            var existing = quad.gameObject.GetComponent<QuadBody>();
            if (existing != null
                && existing.LastVertexCount == mc.sharedMesh.vertexCount
                && existing.LastTriangleCount == mc.sharedMesh.triangles.Length / 3)
            {
                return;
            }

            // Stale QuadBody from a different mesh / pooled GameObject —
            // destroy synchronously so Unity fires its OnDestroy and
            // queues the old BodyDestroy before we attach the new one.
            if (existing != null) UnityEngine.Object.DestroyImmediate(existing);

            TryMirrorQuad(quad);
        }

        public static void OnQuadUpdate(PQ quad)
        {
            // OnQuadUpdate fires every PQS update tick. Most calls
            // don't change the mesh shape — short-circuit on
            // unchanged vertex / triangle counts. (For terrain that
            // animates vertices but keeps topology, this misses
            // updates; PQS doesn't do that today.)
            if (LongeronAddon.ActiveWorld == null) return;
            if (quad == null || quad.gameObject == null) return;

            var qb = quad.gameObject.GetComponent<QuadBody>();
            if (qb == null) return;  // not mirrored — was below minLevel

            var mc = quad.gameObject.GetComponent<MeshCollider>();
            if (mc == null || !mc.enabled || mc.sharedMesh == null) return;

            int verts = mc.sharedMesh.vertexCount;
            int tris  = mc.sharedMesh.triangles.Length / 3;
            if (verts == qb.LastVertexCount && tris == qb.LastTriangleCount) return;

            // Topology changed — destroy the old body and remirror.
            UnityEngine.Object.DestroyImmediate(qb);
            TryMirrorQuad(quad);
        }

        public static void OnQuadDestroy(PQ quad)
        {
            // PQSCache pools the GameObject (SetActive false) without
            // destroying it. Unity's OnDestroy won't fire — explicitly
            // destroy our QuadBody so its OnDestroy queues the
            // BodyDestroy.
            if (quad == null || quad.gameObject == null) return;
            var qb = quad.gameObject.GetComponent<QuadBody>();
            if (qb != null) UnityEngine.Object.DestroyImmediate(qb);
        }

        // ---- Mirror logic ----

        static bool TryMirrorQuad(PQ quad)
        {
            var world = LongeronAddon.ActiveWorld;
            if (world == null) return false;

            var mc = quad.gameObject.GetComponent<MeshCollider>();
            if (mc == null || mc.sharedMesh == null) return false;

            var mesh = mc.sharedMesh;
            var localVerts = mesh.vertices;
            var triangles  = mesh.triangles;
            if (localVerts.Length == 0 || triangles.Length == 0) return false;

            // Bake quad-local vertices to Unity-world space at the
            // current transform. We pass world-space verts with body
            // pos = (0,0,0) rather than tracking per-quad transforms
            // on the native side; Krakensbane / FloatingOrigin shifts
            // already translate every Jolt body uniformly via
            // WriteShiftWorld, so terrain stays in lockstep with
            // vessels.
            var xform = quad.transform;
            var worldXyz = new float[localVerts.Length * 3];
            for (int i = 0; i < localVerts.Length; ++i)
            {
                Vector3 w = xform.TransformPoint(localVerts[i]);
                worldXyz[i * 3 + 0] = w.x;
                worldXyz[i * 3 + 1] = w.y;
                worldXyz[i * 3 + 2] = w.z;
            }

            var handle = SceneRegistry.MintBodyHandle();
            try
            {
                world.Input.WriteBodyCreateTriangleMesh(
                    handle, Layer.Static,
                    worldXyz, triangles,
                    posX: 0, posY: 0, posZ: 0,
                    rotX: 0, rotY: 0, rotZ: 0, rotW: 1,
                    groupId: 0);
            }
            catch (Exception ex)
            {
                Debug.LogError(LogPrefix + "WriteBodyCreateTriangleMesh threw: "
                                + ex.GetType().Name + ": " + ex.Message);
                return false;
            }

            var qb = QuadBody.AttachTo(quad, handle);
            if (qb != null)
            {
                qb.LastVertexCount = localVerts.Length;
                qb.LastTriangleCount = triangles.Length / 3;
                qb.BakedWorldPosition = xform.position;
            }
            return true;
        }

        // Threshold (m) above which a drifted quad gets re-mirrored.
        // PQS independently translates quad transforms each tick via
        // CelestialBody.position → PQS.PrecisePosition setter →
        // FastUpdateQuadsPosition (PQS.cs:1261) → FastUpdateSubQuadsPosition
        // (PQ.cs:272). That path bypasses FloatingOrigin's uniform
        // shift, so our Jolt static mesh — baked once at OnQuadBuilt —
        // doesn't follow. Threshold trades off "more re-mirrors per
        // second" against "vessel can clip through stale terrain by
        // up to this distance before we catch it." 0.5 m is well below
        // CollisionEnhancer's ~10 cm punch-through distance budget.
        const float kQuadDriftThresholdSq = 0.25f;  // 0.5 m^2

        // Re-mirror every active terrain body whose Unity-side
        // transform has drifted more than the threshold from our
        // last bake. Called from LongeronSceneDriver.FixedUpdate
        // each tick (cheap — typical ~1000 active quads, one
        // Vector3 distance check each).
        // Drift telemetry — accumulated across ticks, flushed at most
        // once per second so a fast-rotating planet doesn't produce
        // 50 lines per second.
        static int _driftRemirrorAccum;
        static float _driftLogLastTime;
        static float _driftMaxThisWindow;

        public static void UpdateActive()
        {
            if (LongeronAddon.ActiveWorld == null) return;
            var list = QuadBody.Active;
            for (int i = list.Count - 1; i >= 0; i--)
            {
                var qb = list[i];
                if (qb == null) { list.RemoveAt(i); continue; }
                var quad = qb.Quad;
                if (quad == null || quad.gameObject == null) continue;

                Vector3 now = quad.transform.position;
                Vector3 delta = now - qb.BakedWorldPosition;
                float dSq = delta.sqrMagnitude;
                if (dSq < kQuadDriftThresholdSq) continue;

                if (dSq > _driftMaxThisWindow) _driftMaxThisWindow = dSq;

                // Drifted — destroy the stale Jolt body and re-mirror
                // with current vertex coords. DestroyImmediate avoids
                // the [DisallowMultipleComponent] race when AddComponent
                // runs in the same frame.
                UnityEngine.Object.DestroyImmediate(qb);
                TryMirrorQuad(quad);
                _driftRemirrorAccum++;
            }

            if (_driftRemirrorAccum > 0 && Time.realtimeSinceStartup - _driftLogLastTime > 1.0f)
            {
                Debug.Log(LogPrefix + string.Format(
                    "drift remirror: {0} quad(s) in last second, max drift {1:F2} m",
                    _driftRemirrorAccum, Mathf.Sqrt(_driftMaxThisWindow)));
                _driftRemirrorAccum = 0;
                _driftMaxThisWindow = 0;
                _driftLogLastTime = Time.realtimeSinceStartup;
            }
        }

        // -- Per-second diagnostic: raycast vessel→terrain, compare
        //    Unity's reported hit point against our Jolt mirror's
        //    state for that quad. Lets us see whether visual terrain
        //    and Jolt terrain agree, and at what magnitude they differ.

        // Layer 15 = "Local Scenery" — PQS quads, KSC static, asteroids.
        // Mask bit: 1 << 15 = 32768.
        const int kTerrainLayerMask = 1 << 15;

        static float _diagLastTime;

        public static void LogVesselTerrainDiag()
        {
            if (Time.realtimeSinceStartup - _diagLastTime < 1.0f) return;
            _diagLastTime = Time.realtimeSinceStartup;

            var v = FlightGlobals.ActiveVessel;
            if (v == null || v.rootPart == null) return;
            var rb = v.rootPart.rb;
            if (rb == null) return;

            // Ray straight down along gravity. Use a generous range so
            // we still catch terrain when descending from altitude.
            Vector3 origin = v.rootPart.transform.position;
            Vector3 down = -v.upAxis;
            const float kRayLen = 5000f;

            if (!Physics.Raycast(origin, down, out var hit, kRayLen,
                                  kTerrainLayerMask, QueryTriggerInteraction.Ignore))
            {
                Debug.Log(LogPrefix + string.Format(
                    "diag: vessel='{0}' pos=({1:F2},{2:F2},{3:F2}) — no terrain hit within {4}m",
                    v.vesselName, origin.x, origin.y, origin.z, kRayLen));
                return;
            }

            var go = hit.collider != null ? hit.collider.gameObject : null;
            var pq = go != null ? go.GetComponent<PQ>() : null;
            var qb = go != null ? go.GetComponent<QuadBody>() : null;

            // Mesh corners in world coords for a sense of quad scale +
            // exact world geometry. PQS quads use a fixed N×N vertex
            // grid where N = sphereRoot.cacheSideVertCount (default 15);
            // corners are indices 0, N-1, N*(N-1), N*N-1.
            string cornersStr = "<no mesh>";
            if (pq != null && pq.verts != null && pq.verts.Length > 0)
            {
                int total = pq.verts.Length;
                // Detect grid side length by sqrt; clamp safely.
                int n = Mathf.Max(1, (int)Mathf.Round(Mathf.Sqrt(total)));
                int sw = 0;
                int se = (n - 1).Clamp(0, total - 1);
                int nw = (n * (n - 1)).Clamp(0, total - 1);
                int ne = (total - 1).Clamp(0, total - 1);

                Vector3 cSW = pq.transform.TransformPoint(pq.verts[sw]);
                Vector3 cSE = pq.transform.TransformPoint(pq.verts[se]);
                Vector3 cNW = pq.transform.TransformPoint(pq.verts[nw]);
                Vector3 cNE = pq.transform.TransformPoint(pq.verts[ne]);
                cornersStr = string.Format(
                    "SW=({0:F2},{1:F2},{2:F2}) SE=({3:F2},{4:F2},{5:F2}) NW=({6:F2},{7:F2},{8:F2}) NE=({9:F2},{10:F2},{11:F2})",
                    cSW.x, cSW.y, cSW.z, cSE.x, cSE.y, cSE.z,
                    cNW.x, cNW.y, cNW.z, cNE.x, cNE.y, cNE.z);
            }

            string quadInfo;
            if (pq != null)
            {
                Vector3 qPos = pq.transform.position;
                if (qb != null)
                {
                    Vector3 baked = qb.BakedWorldPosition;
                    Vector3 drift = qPos - baked;
                    quadInfo = string.Format(
                        "lod={0} body={1} quadPos=({2:F2},{3:F2},{4:F2}) baked=({5:F2},{6:F2},{7:F2}) drift={8:F3}m",
                        pq.subdivision, qb.Handle.Id,
                        qPos.x, qPos.y, qPos.z,
                        baked.x, baked.y, baked.z,
                        drift.magnitude);
                }
                else
                {
                    quadInfo = string.Format(
                        "lod={0} body=<unmirrored> quadPos=({1:F2},{2:F2},{3:F2})",
                        pq.subdivision, qPos.x, qPos.y, qPos.z);
                }
            }
            else
            {
                quadInfo = string.Format("<not a PQ — collider={0}>",
                                          hit.collider != null ? hit.collider.name : "<null>");
            }

            Debug.Log(LogPrefix + string.Format(
                "diag: vessel='{0}' pos=({1:F2},{2:F2},{3:F2}) altAGL={4:F2} hit=({5:F2},{6:F2},{7:F2}) dist={8:F2} {9} corners=[{10}]",
                v.vesselName,
                origin.x, origin.y, origin.z,
                v.heightFromTerrain,
                hit.point.x, hit.point.y, hit.point.z,
                hit.distance,
                quadInfo,
                cornersStr));
        }
    }
}

internal static class IntClampExtensions
{
    public static int Clamp(this int v, int min, int max) =>
        v < min ? min : (v > max ? max : v);
}
