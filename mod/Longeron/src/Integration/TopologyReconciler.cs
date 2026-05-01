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
// damage, scene exit) is handled by JoltPart.OnDestroy → pending
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

        // Drain pending body destroys (from JoltPart.OnDestroy) plus
        // process every dirty vessel. Called from
        // LongeronSceneDriver.FixedUpdate before per-vessel pre-step
        // setup so all topology mutations land in the same input
        // buffer as the per-tick pose / force records.
        public static void Reconcile(InputBuffer input, CbFrame frame)
        {
            JoltPart.DrainPendingDestroys(input);

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
            // its part composition has changed; the JoltPart.OnDestroy
            // queue handles the case where the owning Part GameObject
            // itself was destroyed).
            if (mv.Body.IsValid)
            {
                input.WriteBodyDestroy(mv.Body);
                mv.Body = default;
                // Detach the JoltPart handle on every former member so
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
                    var jb = p.gameObject.GetComponent<JoltPart>();
                    if (jb != null) { jb.Handle = default; jb.OwnsBody = false; }
                }
            }

            // Aggregate vessel mass for the BodyCreate header. Per-
            // part density is computed inside ColliderWalker and
            // attached to each AppendShape; Jolt's CompoundShape then
            // aggregates mass / CoM / inertia from per-sub-shape
            // MassProperties. The header mass is still useful as a
            // safety override for parts that contribute mass without
            // colliders (rare; CalculateInertia rescales to it).
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
                var jb = JoltPart.AttachTo(p, newHandle, ownsBody: isRoot);
                if (jb == null) continue;
                if (mv.PartOffsets.TryGetValue(p, out var off))
                {
                    jb.PartLocalPos = off.LocalPos;
                    jb.PartLocalRot = off.LocalRot;
                }
                jb.LastMass = p.mass + p.GetResourceMass();
                // Pre-compute stock-equivalent joint break thresholds for
                // this part's joint-to-parent. Used by StressGauge and
                // (eventually) joint-break detection in place of the much
                // stricter Part.breakingForce.
                jb.RecomputeBreakThresholds();
            }

            // Idempotently re-apply kinematic takeover for any newly-
            // unpacked rigidbodies that haven't been stamped yet.
            LongeronVesselModule.ApplyKinematicTakeover(v);

            // Stock physics easing: at first registration, force gravity
            // off (vessel.gravityMultiplier = 0), then let stock's
            // VesselPrecalculate.CalculateGravity ramp it back to 1
            // over ~16 ticks (easingFrameIncrease = 0.0625).
            //
            // Why we need this: stock's PutShipToGround places the
            // vessel using world Y, which doesn't perfectly align with
            // upAxis at the launchpad's location. The engine's lowest
            // collider often spawns slightly INSIDE the launchpad mesh
            // (~25cm penetration). With Physics.autoSimulation=false
            // and no PhysX depenetration, gravity yanks the vessel
            // through the pad faster than Jolt's iterative contact
            // resolution can push it back out — vessel tunnels and
            // ends up resting on the ground plane below the pad.
            //
            // The easing window gives Jolt 16 gravity-free ticks to
            // resolve the spawn-time penetration, then gravity ramps
            // in and the vessel rests cleanly on the actual pad.
            // Stock's gravity force flows through FlightIntegrator's
            // AddForce(integrationAccel) which is already pre-
            // multiplied by gravityMultiplier — our RigidbodyForceHooks
            // intercepts the scaled value, so no separate scaling is
            // needed on our side.
            if (firstTime && v.precalc != null)
            {
                v.precalc.isEasingGravity = true;
            }

            // Send the vessel's spanning tree to the native RNEA pass
            // (Phase 4 advisory — logs per-edge transmitted wrench;
            // doesn't drive simulation). Built in topology order from
            // the root walk so parent indices are always < child. Also
            // populates mv.PartsByIdx so SceneDriver can map incoming
            // per-edge JointWrench records back to Parts.
            EmitVesselTree(v, newHandle, input, mv);

            Debug.Log(LogPrefix + string.Format(
                "{0} '{1}': {2} part(s), mass={3:F2}t, body={4}",
                firstTime ? "first-register" : "rebuild",
                v.vesselName, current.Count, totalMass, newHandle.Id));
        }

        // Walk the part tree rooted at v.rootPart in BFS order so each
        // child's parent is already in the list before the child itself.
        // Build per-part {parent_idx, mass, com_local, inertia_diag,
        // attach_local} in vessel-root frame and send as a
        // VesselTreeUpdate record. Inertia uses a uniform-density box
        // approximation around each part's collider AABB — good enough
        // for advisory magnitudes; precise inertia is Phase 4.1.
        static readonly List<Part> _bfsScratch = new List<Part>(128);
        static readonly Dictionary<Part, ushort> _idxScratch = new Dictionary<Part, ushort>(128);
        static InputBuffer.VesselTreeNode[] _treeNodeScratch =
            new InputBuffer.VesselTreeNode[128];

        static void EmitVesselTree(Vessel v, BodyHandle vesselBody, InputBuffer input,
                                    ManagedVessel mv)
        {
            if (v == null || v.rootPart == null) return;
            var rootXform = v.rootPart.transform;
            if (rootXform == null) return;

            _bfsScratch.Clear();
            _idxScratch.Clear();
            _bfsScratch.Add(v.rootPart);
            _idxScratch[v.rootPart] = 0;

            // BFS to flatten the part tree in topology order.
            for (int head = 0; head < _bfsScratch.Count; ++head)
            {
                var p = _bfsScratch[head];
                if (p == null) continue;
                foreach (var child in p.children)
                {
                    if (child == null || _idxScratch.ContainsKey(child)) continue;
                    _idxScratch[child] = (ushort)_bfsScratch.Count;
                    _bfsScratch.Add(child);
                }
            }

            int n = _bfsScratch.Count;
            if (n > 1024) n = 1024;  // matches kMaxPartsPerVessel native-side

            // Snapshot BFS order onto the ManagedVessel so SceneDriver
            // can resolve per-edge JointWrench records (which carry
            // part_idx) back to a Part for stashing on JoltPart.
            mv.PartsByIdx.Clear();
            for (int i = 0; i < n; ++i) mv.PartsByIdx.Add(_bfsScratch[i]);

            if (_treeNodeScratch.Length < n)
            {
                _treeNodeScratch = new InputBuffer.VesselTreeNode[
                    System.Math.Max(n, _treeNodeScratch.Length * 2)];
            }

            const ushort kInvalidIdx = 0xFFFF;
            for (int i = 0; i < n; ++i)
            {
                var p = _bfsScratch[i];
                ref var node = ref _treeNodeScratch[i];

                // Stamp the per-part index on the JoltPart so
                // RigidbodyForceHooks can attribute force records
                // back to this position in the tree.
                if (p.gameObject != null)
                {
                    var jbStamp = p.gameObject.GetComponent<JoltPart>();
                    if (jbStamp != null) jbStamp.PartIdx = (ushort)i;
                }

                // Parent index in the BFS order. Root has no parent.
                if (i == 0 || p.parent == null || !_idxScratch.TryGetValue(p.parent, out var parentIdx))
                    node.ParentIdx = kInvalidIdx;
                else
                    node.ParentIdx = parentIdx;

                node.Mass = p.mass + p.GetResourceMass();

                // Part transforms expressed in vessel-root local frame.
                Vector3 comLocal = rootXform.InverseTransformPoint(p.transform.position);
                node.ComLocalX = comLocal.x;
                node.ComLocalY = comLocal.y;
                node.ComLocalZ = comLocal.z;

                // Inertia: prefer the rb's principal moments
                // (Unity computes these from collider geometry +
                // density when the rb is created). For kinematic
                // rbs that haven't been initialized non-kinematic
                // first, this can return Vector3.zero — fall back
                // to a uniform-density box from collider AABBs.
                //
                // Project the principal moments onto vessel-root
                // axes via the part's relative rotation. We only
                // store the diagonal (Phase 4 advisory); the
                // off-diagonal terms are dropped.
                ComputePartInertiaDiag(p, rootXform, node.Mass,
                    out node.InertiaDiagX, out node.InertiaDiagY, out node.InertiaDiagZ);

                // Joint anchor in vessel-root frame. For child parts,
                // use the part's attachJoint anchor if available; else
                // fall back to the part's transform position. Root part
                // has no joint — use (0,0,0).
                Vector3 attachLocal;
                if (i == 0)
                {
                    attachLocal = Vector3.zero;
                }
                else
                {
                    var aj = p.attachJoint != null ? p.attachJoint.Joint : null;
                    if (aj != null)
                    {
                        // ConfigurableJoint.connectedAnchor is in the
                        // connected body's local frame. Transform to
                        // world via the parent's rb, then to vessel-root.
                        Vector3 worldAnchor = aj.connectedBody != null
                            ? aj.connectedBody.transform.TransformPoint(aj.connectedAnchor)
                            : p.transform.position;
                        attachLocal = rootXform.InverseTransformPoint(worldAnchor);
                    }
                    else
                    {
                        attachLocal = comLocal;
                    }
                }
                node.AttachLocalX = attachLocal.x;
                node.AttachLocalY = attachLocal.y;
                node.AttachLocalZ = attachLocal.z;

                // Phase 5.1 ABA: per-edge spherical-joint compliance.
                // Translation between anchors is rigidly pinned (no
                // K_lin); only angular spring + damper. Mirrors
                // PartJoint.SetupJoint's stiffness formula:
                //
                //   K_ang = rigidity (1500) × num3 × {Stack,Srf}AttachStiffNess
                //   num3  = (size + 1) × {stack:2, srf:0.8}
                //
                // multiplied by kStiffnessScale to make rockets more
                // rigid than stock (visible flex only on heavy bending
                // loads, not on every joint).
                //
                // Damping = critical, where critical for the angular
                // oscillator on this part is 2 × sqrt(K_ang × I). We
                // approximate I with the average of the part's
                // diagonal inertia.
                ComputeJointCompliance(p, i, ref node);
            }

            // Slice to the actual node count.
            var slice = new InputBuffer.VesselTreeNode[n];
            System.Array.Copy(_treeNodeScratch, slice, n);
            input.WriteVesselTreeUpdate(vesselBody, slice);
        }

        // Inertia tensor for the part's RNEA contribution. We want
        // the diagonal in vessel-root axes. Unity exposes principal
        // moments (rb.inertiaTensor) in part-local axes plus a
        // rotation (rb.inertiaTensorRotation) from those principal
        // axes to part-local. Compose with the part-to-root rotation,
        // then take the diagonal of R · diag(I) · R^T:
        //   I_root_kk = Σ_j (R_kj)² · I_j
        // (Off-diagonals discarded — Phase 4 wire format only carries
        // the diagonal. Good enough for advisory magnitudes; Phase 4.x
        // can extend to a full symmetric tensor.)
        // Phase 5 ABA stiffness scale — multiplier on top of stock's
        // PartJoint.SetupJoint formula. The pad-rest transient
        // instability that prompted the original drop to 5× was
        // ultimately caused by the NotifyShapeChanged CoM-mismatch
        // bug (now fixed). With that fix in place, low stiffness
        // also creates a thrust-feedback runaway: as the engine
        // flexes angularly under thrust, the thrust direction tilts
        // in body frame → lateral force grows → more flex → divergence.
        // Near-welded stiffness: K is sized so that even at the
        // breaking-force threshold, joint angular deflection stays
        // under ~1°. For breakingTorque ~100 kN·m and 1° = 0.017 rad,
        // K_required ≈ 100 / 0.017 ≈ 6000 kN·m/rad on a size-1 stack.
        // That's ~33× the prior 180 kN·m/rad (kStiffnessScale=30),
        // so kStiffnessScale = 1000 puts us in that regime. Stiff
        // joints + cumulative chain effect (4-part stack has 3 joints
        // in series, lateral offset at the leaf grows with chain
        // length × Σθ_i) is what lets the leaf engine stay visually
        // attached to its parent during normal flight. ω_n = √(K/I)
        // ≈ 110 rad/s for I≈0.5 t·m²; substep dt = 1ms gives margin
        // ~6× critical-stable for explicit Euler.
        const float kStiffnessScale = 1000f;

        // Damping ratio (fraction of critical). 1.0 = critical (fastest
        // non-overshoot decay). Super-critical (>1) introduces a fast
        // eigenvalue at -(ζ + sqrt(ζ²-1))·ω that needs dt < 0.76/ω for
        // semi-implicit Euler stability. Critical gives stable dt < 2/ω
        // margin without overshoot.
        const float kDampingRatio = 1.0f;

        // Compute the per-edge spherical-joint compliance for the joint
        // connecting `p` (BFS index `idx`) to its parent. Writes the
        // 2 angular values into `node`. For the root (idx == 0) writes
        // zeros. Phase 5.1: linear DOF was eliminated (anchors rigidly
        // pinned), so K_lin / C_lin no longer exist on the wire — see
        // splendid-dancing-flute.md.
        static void ComputeJointCompliance(Part p, int idx,
            ref InputBuffer.VesselTreeNode node)
        {
            if (idx == 0 || p.parent == null)
            {
                node.KAng = 0f; node.CAng = 0f;
                return;
            }

            // Stock's num3 from PartJoint.SetupJoint:320-329.
            // attachMode determines stack vs srf node factor.
            float nodeFactor = (p.attachMode == AttachModes.STACK) ? 2f : 0.8f;

            // Resolve effective node size, mirroring PartJoint.Create.
            int size = 1;
            AttachNode nodeToParent = p.FindAttachNodeByPart(p.parent);
            AttachNode nodeFromParent = p.parent.FindAttachNodeByPart(p);
            if (nodeToParent != null && nodeFromParent != null)
                size = Mathf.Min(nodeToParent.size, nodeFromParent.size);
            else if (nodeToParent != null) size = nodeToParent.size;
            else if (nodeFromParent != null) size = nodeFromParent.size;
            else if (p.parent.srfAttachNode != null) size = p.parent.srfAttachNode.size;

            float num3 = (size + 1) * nodeFactor;

            // Stock's stiffness in N·m/rad (units: positionSpring on
            // a ConfigurableJoint angular drive). Native side expects
            // kN·m/rad for K_ang since our forces and masses are
            // already in kN/tonnes.
            float stockRigidity = 1500f;
            float stiffStock_Nm = stockRigidity * num3 * FlightGlobals.StackAttachStiffNess;
            float kAng_Nm = stiffStock_Nm * kStiffnessScale;
            // Convert N·m/rad → kN·m/rad: divide by 1000.
            float kAng_kNm = kAng_Nm * 0.001f;

            // Critical damping for the angular oscillator on this part:
            // critical = 2 × sqrt(K × I), with I approximated as the
            // mean of the part's diagonal inertia.
            float inertiaBallpark = (node.InertiaDiagX + node.InertiaDiagY + node.InertiaDiagZ) / 3f;
            if (inertiaBallpark <= 1e-9f) inertiaBallpark = 1e-3f;
            float cAng_kNm = kDampingRatio * 2f * Mathf.Sqrt(kAng_kNm * inertiaBallpark);

            node.KAng = kAng_kNm;
            node.CAng = cAng_kNm;
        }

        static void ComputePartInertiaDiag(
            Part p, Transform rootXform, float mass,
            out float ix, out float iy, out float iz)
        {
            Vector3 principal = Vector3.zero;
            Quaternion principalToPartLocal = Quaternion.identity;
            bool haveTensor = false;

            if (p.rb != null)
            {
                var t = p.rb.inertiaTensor;
                // For kinematic-from-creation rbs Unity sometimes
                // hands back a near-zero tensor. Treat anything
                // smaller than ~1e-7 as missing and fall back.
                if (t.x > 1e-7f || t.y > 1e-7f || t.z > 1e-7f)
                {
                    principal = t;
                    principalToPartLocal = p.rb.inertiaTensorRotation;
                    haveTensor = true;
                }
            }

            if (!haveTensor)
            {
                // AABB box fallback: solid uniform-density box from
                // collider AABB half-extents. I = m·(b²+c²)/3 along
                // each axis (matches stock for parts with no explicit
                // rb-side tensor — typically jointless flora).
                Vector3 ext = ApproxAABBHalfExtents(p);
                float bx = ext.x, by = ext.y, bz = ext.z;
                ix = (mass / 3f) * (by * by + bz * bz);
                iy = (mass / 3f) * (bx * bx + bz * bz);
                iz = (mass / 3f) * (bx * bx + by * by);
                return;
            }

            // Compose: principal axes → part-local → vessel-root.
            Quaternion partToRoot = Quaternion.Inverse(rootXform.rotation) * p.transform.rotation;
            Quaternion principalToRoot = partToRoot * principalToPartLocal;

            // Build the 3 column basis of principalToRoot, then the
            // diagonal of R · diag(I) · R^T per-axis.
            Vector3 cx = principalToRoot * Vector3.right;
            Vector3 cy = principalToRoot * Vector3.up;
            Vector3 cz = principalToRoot * Vector3.forward;
            ix = cx.x * cx.x * principal.x + cy.x * cy.x * principal.y + cz.x * cz.x * principal.z;
            iy = cx.y * cx.y * principal.x + cy.y * cy.y * principal.y + cz.y * cz.y * principal.z;
            iz = cx.z * cx.z * principal.x + cy.z * cy.z * principal.y + cz.z * cz.z * principal.z;
        }

        static Vector3 ApproxAABBHalfExtents(Part p)
        {
            Bounds b = default;
            bool any = false;
            foreach (var col in p.GetComponentsInChildren<Collider>(includeInactive: false))
            {
                if (col == null || col.isTrigger || !col.enabled) continue;
                if (!any) { b = col.bounds; any = true; }
                else b.Encapsulate(col.bounds);
            }
            if (!any) return new Vector3(0.5f, 0.5f, 0.5f);  // fallback
            return b.extents;
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
                    var jb = p.gameObject.GetComponent<JoltPart>();
                    if (jb != null) { jb.Handle = default; jb.OwnsBody = false; }
                }
            }
            SceneRegistry.Unregister(v, out _);
            Debug.Log(LogPrefix + $"teardown '{(v != null ? v.vesselName : "<dead>")}'");
        }
    }
}
