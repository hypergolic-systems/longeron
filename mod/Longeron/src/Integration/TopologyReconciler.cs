// TopologyReconciler — observes vessel structural mutations via
// GameEvents and reconciles each dirty vessel against our recorded
// ManagedVessel state. Single-body model: one Jolt body per vessel.
//
// Why diff-based reconciliation:
//   - One subscription set covers every stock and modded mutation
//     path (decouple, couple, dock, joint break, fairing eject all
//     funnel through onVesselWasModified).
//   - Idempotent: multiple events on the same vessel within one tick
//     collapse to one diff.
//   - Order-independent: onVesselWasModified(old) and onVesselCreate
//     (new) from a decouple can fire in either order; reconciliation
//     observes final state.
//
// On any topology change (parts added or removed from a vessel since
// last reconcile), the entire vessel body is rebuilt: BodyDestroy +
// BodyCreate with the new compound shape. Cheap relative to the
// dynamic-step cost on a typical KSP rocket. Phase 4 (Featherstone
// ABA) computes joint forces post-hoc from the same single body's
// motion, decomposing into per-edge stresses for break detection.
//
// Body destruction for unexpected GameObject teardown (crash, splash
// damage, scene exit) is handled by JoltBody.OnDestroy → pending
// destroy queue, drained at the start of every reconcile pass.

using System.Collections.Generic;
using Longeron.Native;
using UnityEngine;

namespace Longeron.Integration
{
    internal static class TopologyReconciler
    {
        const string LogPrefix = "[Longeron/Reconciler] ";

        static readonly HashSet<Vessel> _dirty = new HashSet<Vessel>();

        // Reused per-call to avoid allocations on the hot path.
        static readonly Dictionary<Part, ManagedVessel.PartOffset> _offsetScratch =
            new Dictionary<Part, ManagedVessel.PartOffset>(64);

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
        public static void Reconcile(InputBuffer input, CbFrame frame)
        {
            JoltBody.DrainPendingDestroys(input);

            if (_dirty.Count == 0) return;

            var pending = new List<Vessel>(_dirty);
            _dirty.Clear();

            foreach (var v in pending) ReconcileVessel(v, input, frame);
        }

        static void ReconcileVessel(Vessel v, InputBuffer input, CbFrame frame)
        {
            // End-of-life: vessel is dead. Tear down its body if held.
            if (v == null || v.state == Vessel.State.DEAD)
            {
                if (v != null && SceneRegistry.TryGet(v, out var mvDead))
                    TearDown(mvDead, input, v);
                return;
            }

            // Not physics-active: don't touch it. OnGoOnRails has the
            // synchronous tear-down path and would have run already.
            if (!v.loaded || v.packed) return;

            bool firstTime = !SceneRegistry.TryGet(v, out var mv);
            if (firstTime) mv = SceneRegistry.Register(v);

            // Compare current parts to last-seen.
            var current = new HashSet<Part>();
            foreach (var p in v.parts) if (p != null) current.Add(p);

            bool topologyChanged = firstTime
                                   || !current.SetEquals(mv.LastParts);

            if (!topologyChanged) return;

            // Topology diff detected → rebuild the vessel body from
            // scratch. Destroy any prior body (the reconciler-level
            // destroy is for the case where the body still exists but
            // its part composition has changed; the JoltBody.OnDestroy
            // queue handles the case where the owning Part GameObject
            // itself was destroyed).
            if (mv.Body.IsValid)
            {
                input.WriteBodyDestroy(mv.Body);
                mv.Body = default;
                // Detach the JoltBody handle on every former member so
                // any further AddForce calls land nowhere. The MonoBe-
                // haviours stay on the Part GameObjects; they get
                // re-attached below for the new body.
                //
                // Skip parts that have migrated to a different vessel
                // (decouple, undock). Those parts already have a valid
                // handle stamped by the new vessel's reconcile pass;
                // clearing here would clobber it. Reconcile order
                // across `_dirty` is HashSet-derived, so the parent
                // can run after a child vessel and would otherwise
                // race-clear its handles — manifests as decoupled
                // boosters losing their separation impulse and
                // "hanging in midair".
                foreach (var p in mv.LastParts)
                {
                    if (p == null || p.gameObject == null) continue;
                    if (p.vessel != null && p.vessel != v) continue;
                    var jb = p.gameObject.GetComponent<JoltBody>();
                    if (jb != null) { jb.Handle = default; jb.OwnsBody = false; }
                }
            }

            // Compute aggregate mass and emit a fresh body.
            float totalMass = 0f;
            foreach (var p in current)
            {
                float m = p.mass + p.GetResourceMass();
                if (m > 0f) totalMass += m;
            }
            if (totalMass < 1e-6f) totalMass = 0.01f;

            var newHandle = SceneRegistry.MintBodyHandle();
            bool ok = ColliderWalker.WriteBodyForVessel(
                input, newHandle, v,
                BodyType.Dynamic, Layer.Kinematic,
                totalMass, mv.GroupId,
                frame, _offsetScratch);
            if (!ok)
            {
                Debug.LogWarning(LogPrefix + $"vessel '{v.vesselName}' produced no usable colliders; skipping body create");
                mv.LastParts.Clear();
                mv.PartOffsets.Clear();
                mv.LastMass = 0f;
                return;
            }

            // Record state and stamp the handle on every part. Root
            // part owns the body (its OnDestroy will queue the
            // BodyDestroy if the GameObject dies); other parts share
            // the handle for force redirection + pose propagation.
            mv.Body = newHandle;
            mv.LastParts.Clear();
            foreach (var p in current) mv.LastParts.Add(p);
            mv.PartOffsets.Clear();
            foreach (var kv in _offsetScratch) mv.PartOffsets[kv.Key] = kv.Value;
            mv.LastMass = totalMass;

            var root = v.rootPart;
            foreach (var p in current)
            {
                if (p == null || p.gameObject == null) continue;
                bool isRoot = (p == root);
                var jb = JoltBody.AttachTo(p, newHandle, ownsBody: isRoot);
                if (jb == null) continue;
                if (mv.PartOffsets.TryGetValue(p, out var off))
                {
                    jb.PartLocalPos = off.LocalPos;
                    jb.PartLocalRot = off.LocalRot;
                }
                jb.LastMass = p.mass + p.GetResourceMass();
            }

            // Idempotently re-apply kinematic takeover for any newly-
            // unpacked rigidbodies that haven't been stamped yet.
            LongeronVesselModule.ApplyKinematicTakeover(v);

            Debug.Log(LogPrefix + string.Format(
                "{0} '{1}': {2} part(s), mass={3:F2}t, body={4}",
                firstTime ? "first-register" : "rebuild",
                v.vesselName, current.Count, totalMass, newHandle.Id));
        }

        static void TearDown(ManagedVessel mv, InputBuffer input, Vessel v)
        {
            if (mv.Body.IsValid)
            {
                input.WriteBodyDestroy(mv.Body);
                // Same migration guard as ReconcileVessel: don't clear
                // handles for parts that have been adopted by a
                // different vessel since LastParts was recorded.
                foreach (var p in mv.LastParts)
                {
                    if (p == null || p.gameObject == null) continue;
                    if (p.vessel != null && p.vessel != v) continue;
                    var jb = p.gameObject.GetComponent<JoltBody>();
                    if (jb != null) { jb.Handle = default; jb.OwnsBody = false; }
                }
            }
            SceneRegistry.Unregister(v, out _);
            Debug.Log(LogPrefix + $"teardown '{(v != null ? v.vesselName : "<dead>")}'");
        }
    }
}
