// JoltPart — MonoBehaviour attached to every Part GameObject Longeron
// manages. Carries the BodyHandle that maps the part to its vessel's
// Jolt body, plus per-part state (BFS index, local offset from vessel
// root, last-tick velocity, last-tick joint wrench from RNEA).
//
// Naming note: in the single-body model every part shares one Jolt body
// (the vessel's), so this component represents the "Jolt-aware part",
// not a Jolt body of its own. A future per-vessel JoltPart component
// (root-only) may take over the body-handle ownership semantics.
//
// Why a component instead of a Dictionary<Rigidbody, BodyHandle> in
// SceneRegistry: the lookup is on the hot path (every Rigidbody.AddForce
// call from stock and modded code goes through it). A
// Rigidbody.GetComponent<JoltPart>() is one Unity-native O(1) probe and
// cleans itself up automatically when the GameObject is destroyed.
//
// ECS-style ownership: when Unity destroys a Part GameObject (crash,
// splash damage, fairing despawn, scene exit) JoltPart.OnDestroy fires
// automatically and queues a BodyDestroy for our handle.
// TopologyReconciler drains the queue at the start of each FixedUpdate
// before processing dirty vessels, so the body cleanup rides the same
// input buffer as everything else. No periodic sweep, no manual cleanup
// at every call site that might destroy a part.

using System.Collections.Generic;
using Longeron.Native;
using UnityEngine;

namespace Longeron
{
    [DisallowMultipleComponent]
    public sealed class JoltPart : MonoBehaviour
    {
        // Single-body model: every Part in a vessel shares the SAME
        // BodyHandle (the vessel's). Only the OwnsBody == true part
        // (the vessel root at body-create time) is responsible for
        // BodyDestroy on cleanup; non-owners' OnDestroy is a no-op
        // because their resource is co-owned by the vessel body.
        public BodyHandle Handle;
        public Part Part;
        public bool OwnsBody;

        // Offset from the vessel root at body-create time, in the
        // root's local frame. Used by LongeronSceneDriver to derive
        // each part's Unity-world pose from the single Jolt body's
        // pose. Captured once when the part joins a vessel; refreshed
        // when the vessel body is rebuilt (decouple, dock, etc.).
        public Vector3 PartLocalPos;
        public Quaternion PartLocalRot = Quaternion.identity;

        // The latest analytic Jolt-integrated velocity for this part,
        // stored at pose readback. Unity silently discards rb.velocity
        // writes on kinematic rigidbodies, so reading rb.velocity gives
        // 0 — useless for navball / orbit / aero / parachute deploy
        // gates. Anywhere stock reads rb.velocity, we either patch the
        // reader or compute the same field from this stored value.
        public Vector3 LastVelocity;
        public Vector3 LastAngularVelocity;

        // Tonnes (KSP convention). Tracked per-part so the driver can
        // detect mass deltas; aggregated to per-vessel for the actual
        // MassUpdate record (single-body model uses one vessel mass,
        // not per-part).
        public float LastMass;

        // Index into the vessel's Phase 4 RNEA tree. Stamped during
        // TopologyReconciler.EmitVesselTree's BFS walk. Used by the
        // force-redirect Harmony hook to attribute each force record
        // to the originating part so the native side can subtract
        // external wrenches per part during the RNEA pass.
        // 0xFFFF = unattributed (force was applied to a part the
        // tree doesn't track, e.g. a vessel without a sent tree).
        public ushort PartIdx = 0xFFFF;

        // Phase 4 joint wrench in the joint's own reference frame.
        // Refreshed every tick by SceneDriver from native JointWrench
        // records. PartModules read these on the next tick's
        // OnFixedUpdate to decide whether to break.
        //
        // Frame convention:
        //   X axis = joint axial direction (parent CoM → child joint
        //            anchor, normalized in vessel-body frame).
        //   Y, Z axes = orthonormal pair perpendicular to the joint
        //               axis (stable Gram-Schmidt basis). Only the
        //               (Y, Z) magnitude is physically meaningful;
        //               the Y/Z split itself is implementation-defined.
        //
        // KSP convention: forces in kN, torques in kN·m.
        //
        // LastJointForce.x:    signed axial → +compression / -tension.
        // LastJointForce.yz:   shear vector (magnitude = perpendicular
        //                      load on the joint).
        // LastJointTorque.x:   signed torsion (twist around the axis).
        // LastJointTorque.yz:  bending moment (e.g. a radial decoupler
        //                      holding a heavy booster against gravity).
        public Vector3 LastJointForce;    // X = axial (signed), YZ = shear
        public Vector3 LastJointTorque;   // X = torsion (signed), YZ = bending

        /// <summary>Compressive (≥ 0) component of the joint axial force.
        /// 0 if the joint is in tension. Doesn't break joints.</summary>
        public float LastJointCompression =>
            LastJointForce.x > 0f ? LastJointForce.x : 0f;

        /// <summary>Tensile (≥ 0) component of the joint axial force.
        /// 0 if the joint is in compression. Compare against
        /// <c>part.breakingForce</c> together with <see cref="LastJointShear"/>.</summary>
        public float LastJointTension =>
            LastJointForce.x < 0f ? -LastJointForce.x : 0f;

        /// <summary>Magnitude of the perpendicular-to-axis force (shear).</summary>
        public float LastJointShear =>
            Mathf.Sqrt(LastJointForce.y * LastJointForce.y +
                       LastJointForce.z * LastJointForce.z);

        /// <summary>Signed torsion (twist around the joint axis).</summary>
        public float LastJointTorsion => LastJointTorque.x;

        /// <summary>Magnitude of the bending moment (perpendicular
        /// torque component). Compare against <c>part.breakingTorque</c>.</summary>
        public float LastJointBending =>
            Mathf.Sqrt(LastJointTorque.y * LastJointTorque.y +
                       LastJointTorque.z * LastJointTorque.z);

        // Pending body destroys from GameObjects Unity has destroyed
        // since the last drain. Static so the queue survives even when
        // a JoltPart's component is gone — the queue is keyed by
        // BodyHandle, not by component reference.
        static readonly Queue<BodyHandle> _pendingDestroy = new Queue<BodyHandle>();

        // BodyHandle.Id → JoltPart reverse lookup. The bridge emits
        // output records (BodyPose, ContactReport) keyed by uint body
        // id; the driver uses this map to find the corresponding Part
        // and rigidbody. Forward lookup (Part → BodyHandle) is just
        // `part.gameObject.GetComponent<JoltPart>().Handle`.
        static readonly Dictionary<uint, JoltPart> _byHandle =
            new Dictionary<uint, JoltPart>();

        // Attach a JoltPart component to the given Part. Pass
        // ownsBody=true on the vessel root (the part whose transform
        // anchored the BodyCreate); pass false on every other part in
        // the vessel — they share the handle but don't own the
        // lifetime. Local pose offsets default to identity; caller
        // should set them after attach.
        //
        // For single-body, multiple parts register against the same
        // handle. _byHandle stores only the *owner* JoltPart so the
        // reverse lookup (handle → JoltPart → Vessel) goes through
        // the root part — output records like BodyPose are
        // vessel-level and only the owner needs to receive them.
        public static JoltPart AttachTo(Part part, BodyHandle handle, bool ownsBody)
        {
            if (part == null || part.gameObject == null) return null;
            var jb = part.gameObject.GetComponent<JoltPart>();
            if (jb == null) jb = part.gameObject.AddComponent<JoltPart>();
            jb.Handle = handle;
            jb.Part = part;
            jb.OwnsBody = ownsBody;
            // Reset to unattributed; EmitVesselTree restamps with the
            // correct index immediately after attach.
            jb.PartIdx = 0xFFFF;
            if (ownsBody) _byHandle[handle.Id] = jb;
            return jb;
        }

        public static bool TryGet(uint bodyId, out JoltPart jb) =>
            _byHandle.TryGetValue(bodyId, out jb);

        // Public hook for sibling component-owners (QuadBody for PQS
        // terrain quads, StaticBody for KSC static colliders) that
        // participate in the same per-tick BodyDestroy drain. Reusing
        // the queue keeps a single drain site in
        // TopologyReconciler.Reconcile rather than scattering bridge
        // bookkeeping across resource owners.
        public static void EnqueuePendingDestroy(BodyHandle handle)
        {
            if (handle.IsValid) _pendingDestroy.Enqueue(handle);
        }

        void OnDestroy()
        {
            // Only the owning part's destruction tears down the Jolt
            // body — non-owner parts share the resource and shouldn't
            // free it. Topology mutations (decouple, joint break) go
            // through TopologyReconciler which destroys + rebuilds the
            // body explicitly, so this path only fires when the entire
            // owning part GameObject is destroyed (crash, splash damage,
            // scene exit).
            if (OwnsBody && Handle.IsValid)
            {
                _pendingDestroy.Enqueue(Handle);
                _byHandle.Remove(Handle.Id);
            }
        }

        // Drain pending destroys into the bridge input buffer. Called
        // by TopologyReconciler.Reconcile at the start of each tick.
        // The drain stays here (rather than on SceneRegistry) so the
        // resource lifecycle stays attached to the component that owns
        // the resource.
        internal static void DrainPendingDestroys(InputBuffer input)
        {
            while (_pendingDestroy.Count > 0)
                input.WriteBodyDestroy(_pendingDestroy.Dequeue());
        }

        // Flushed at flight-scene exit — the world is being disposed,
        // so any pending destroys are moot and we'd otherwise leak the
        // queue across scene transitions. The handle-lookup map is
        // also cleared because GameObject destruction (and the
        // associated OnDestroy → _byHandle.Remove) happens
        // asynchronously after Object.Destroy and may not fire before
        // the next world's bodies start re-using ids.
        internal static void ClearPendingDestroys()
        {
            _pendingDestroy.Clear();
            _byHandle.Clear();
        }
    }
}
