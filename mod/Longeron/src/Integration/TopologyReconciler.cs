// TopologyReconciler — observes vessel structural mutations via
// GameEvents and reconciles each dirty vessel's part tree against our
// recorded ManagedVessel state, emitting the minimal bridge
// mutations to make Jolt match.
//
// Why diff-based reconciliation over per-method Harmony postfixes:
//   - One subscription set covers every stock and modded mutation
//     path. Decouple, Couple, dock, joint break, KAS hooks all funnel
//     through Part.Couple / Part.decouple / PartJoint.DestroyJoint
//     which fire onVesselWasModified.
//   - Idempotent: multiple events on the same vessel within one tick
//     collapse to one diff.
//   - Order-independent: onVesselWasModified(old) and
//     onVesselCreate(new) from a decouple can fire in either order;
//     the reconciler observes final state.
//
// Body destruction is NOT this class's concern — JoltBody.OnDestroy
// queues BodyDestroy records when Unity destroys a Part GameObject
// (crash, splash damage, fairing despawn). Reconcile() drains that
// queue at the start of each pass.

using System.Collections.Generic;
using Longeron.Native;
using UnityEngine;

namespace Longeron.Integration
{
    internal static class TopologyReconciler
    {
        const string LogPrefix = "[Longeron/Reconciler] ";

        static readonly HashSet<Vessel> _dirty = new HashSet<Vessel>();

        public static void MarkDirty(Vessel v)
        {
            if (v != null) _dirty.Add(v);
        }

        public static void Clear()
        {
            _dirty.Clear();
        }

        // Drain pending body destroys (from JoltBody.OnDestroy) plus
        // process every dirty vessel. Called from
        // LongeronSceneDriver.FixedUpdate before per-vessel pre-step
        // setup so all topology mutations land in the same input
        // buffer as the per-tick pose / force records.
        //
        // CbFrame is passed through to ColliderWalker so part poses
        // get baked in CB-frame coords before submission.
        public static void Reconcile(InputBuffer input, CbFrame frame)
        {
            JoltBody.DrainPendingDestroys(input);

            if (_dirty.Count == 0) return;

            // Snapshot to a buffer so MarkDirty can fire mid-iteration
            // (e.g., reconciling vessel A could trigger something that
            // queues vessel B for next tick — we don't want to process
            // B in this same call).
            var pending = new List<Vessel>(_dirty);
            _dirty.Clear();

            foreach (var v in pending) ReconcileVessel(v, input, frame);
        }

        static void ReconcileVessel(Vessel v, InputBuffer input, CbFrame frame)
        {
            // End-of-life: vessel is dead or fully destroyed. Tear down
            // any bodies/constraints we held for it.
            if (v == null || v.state == Vessel.State.DEAD)
            {
                if (v != null && SceneRegistry.TryGet(v, out var mvDead))
                    TearDown(mvDead, input, v);
                return;
            }

            // Not physics-active: don't touch it. If we held state
            // previously, OnGoOnRails has the synchronous tear-down
            // path and would have run already.
            if (!v.loaded || v.packed) return;

            bool firstTime = !SceneRegistry.TryGet(v, out var mv);
            if (firstTime) mv = SceneRegistry.Register(v);

            int bodiesAdded = 0, bodiesMigrated = 0, bodiesRemoved = 0;
            int edgesAdded = 0, edgesRemoved = 0;

            // ----- Diff parts -----
            var expectedParts = new HashSet<Part>(v.parts);

            // Remove parts no longer in this vessel. Don't issue
            // BodyDestroy here — the JoltBody MonoBehaviour stays alive
            // on the GameObject and another reconciliation may claim
            // the part (decouple → new vessel). If the part is truly
            // destroyed, JoltBody.OnDestroy queues the BodyDestroy.
            // We just relinquish the (Part → BodyHandle) mapping in
            // this ManagedVessel.
            var staleParts = new List<Part>();
            foreach (var kv in mv.Bodies)
                if (!expectedParts.Contains(kv.Key)) staleParts.Add(kv.Key);
            foreach (var p in staleParts)
            {
                mv.RemoveBody(p);
                bodiesRemoved++;
            }

            // Add parts new to this vessel. Migrated parts (those whose
            // GameObject already has a JoltBody from a previous
            // ManagedVessel) re-use the existing BodyHandle — no
            // BodyCreate emitted. Truly new parts mint a fresh body.
            foreach (var part in v.parts)
            {
                if (part == null) continue;
                if (mv.Bodies.ContainsKey(part)) continue;

                var existingJb = part.gameObject != null
                    ? part.gameObject.GetComponent<JoltBody>()
                    : null;
                if (existingJb != null && existingJb.Handle.IsValid)
                {
                    // Migrated part: re-use the existing Jolt body but
                    // update its collision filter group to this
                    // vessel's. Without this, decoupled parts retain
                    // their original vessel's group ID and the
                    // GroupFilterTable rejects collisions with the
                    // post-decouple parent vessel ("same group +
                    // same SubGroupID = no collide").
                    mv.AddBody(part, existingJb.Handle);
                    input.WriteSetBodyGroup(existingJb.Handle, mv.GroupId);
                    bodiesMigrated++;
                }
                else
                {
                    float mass = part.mass + part.GetResourceMass();
                    if (mass < 1e-6f) mass = 0.01f;

                    var handle = SceneRegistry.MintBodyHandle();
                    bool ok = ColliderWalker.WriteBodyFor(
                        input, handle, part,
                        BodyType.Dynamic, Layer.Kinematic, mass,
                        groupId: mv.GroupId,
                        frame: frame);
                    if (!ok) continue;

                    mv.AddBody(part, handle);
                    var jb = JoltBody.AttachTo(part, handle);
                    if (jb != null) jb.LastMass = mass;
                    bodiesAdded++;
                }
            }

            // ----- Diff edges (parent-child only for Phase 2) -----
            var expectedEdges = new HashSet<(Part, Part)>();
            foreach (var part in v.parts)
            {
                if (part?.parent == null) continue;
                if (!expectedParts.Contains(part.parent)) continue;
                expectedEdges.Add((part.parent, part));
            }

            // Edges no longer present (parent re-assigned, child gone, etc.)
            var staleEdges = new List<(Part, Part)>();
            foreach (var key in mv.ConstraintEdges.Keys)
                if (!expectedEdges.Contains(key)) staleEdges.Add(key);
            foreach (var key in staleEdges)
            {
                if (mv.RemoveEdge(key.Item1, key.Item2, out var cid))
                {
                    input.WriteConstraintDestroy(cid);
                    edgesRemoved++;
                }
            }

            // Edges newly present
            foreach (var edge in expectedEdges)
            {
                if (mv.ConstraintEdges.ContainsKey(edge)) continue;
                if (!mv.Bodies.TryGetValue(edge.Item1, out var parentH)) continue;
                if (!mv.Bodies.TryGetValue(edge.Item2, out var childH)) continue;

                uint cid = SceneRegistry.MintConstraintId();
                if (AttachAnchors.TryResolveWorld(edge.Item2, edge.Item1, out var anchorWorld))
                {
                    // Convert Unity-world anchor → CB-frame, matching
                    // the body coordinate system the native side uses.
                    Vector3d anchorCb = frame.WorldToCb(new Vector3d(
                        anchorWorld.x, anchorWorld.y, anchorWorld.z));
                    input.WriteConstraintCreateFixedAt(
                        cid, parentH, childH,
                        anchorCb.x, anchorCb.y, anchorCb.z);
                    Debug.Log(LogPrefix + string.Format(
                        "edge cid={0} {1} -> {2} anchorWorld=({3:F2},{4:F2},{5:F2}) anchorCb=({6:F2},{7:F2},{8:F2})",
                        cid,
                        edge.Item1?.partInfo?.name ?? "?",
                        edge.Item2?.partInfo?.name ?? "?",
                        anchorWorld.x, anchorWorld.y, anchorWorld.z,
                        anchorCb.x, anchorCb.y, anchorCb.z));
                }
                else
                {
                    // Fallback: modded attachment paths (KAS, etc.) that
                    // don't populate AttachNode pairings. Auto-detect
                    // anchor at midpoint-of-CoMs preserves prior behavior.
                    Debug.LogWarning(LogPrefix + $"no attach-node anchor for edge "
                                     + $"{edge.Item1?.partInfo?.name ?? "?"} -> "
                                     + $"{edge.Item2?.partInfo?.name ?? "?"} — "
                                     + "falling back to autoDetect constraint");
                    input.WriteConstraintCreateFixed(cid, parentH, childH);
                }
                mv.AddEdge(edge.Item1, edge.Item2, cid);
                edgesAdded++;
            }

            // Idempotently re-apply kinematic takeover. Newly-added
            // parts may not have had their rigidbodies stamped yet
            // (Vessel.Unpack races OnGoOffRails for the first frame).
            LongeronVesselModule.ApplyKinematicTakeover(v);

            if (firstTime || bodiesAdded + bodiesMigrated + bodiesRemoved + edgesAdded + edgesRemoved > 0)
            {
                Debug.Log(LogPrefix + string.Format(
                    "{0} '{1}': +{2} body, ~{3} migrated, -{4} body, +{5} edge, -{6} edge",
                    firstTime ? "first-register" : "reconcile",
                    v.vesselName,
                    bodiesAdded, bodiesMigrated, bodiesRemoved,
                    edgesAdded, edgesRemoved));
            }
        }

        static void TearDown(ManagedVessel mv, InputBuffer input, Vessel v)
        {
            foreach (var cid in mv.ConstraintIds) input.WriteConstraintDestroy(cid);
            foreach (var h in mv.BodyHandles) input.WriteBodyDestroy(h);
            SceneRegistry.Unregister(v, out _);
            Debug.Log(LogPrefix + $"teardown '{(v != null ? v.vesselName : "<dead>")}'");
        }
    }
}
