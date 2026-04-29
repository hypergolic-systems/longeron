// JoltBody — MonoBehaviour attached to every Part GameObject Longeron
// manages. Carries the BodyHandle that maps the part to its Jolt body.
//
// Why a component instead of a Dictionary<Rigidbody, BodyHandle> in
// SceneRegistry: the lookup is on the hot path (every Rigidbody.AddForce
// call from stock and modded code goes through it). A
// Rigidbody.GetComponent<JoltBody>() is one Unity-native O(1) probe and
// cleans itself up automatically when the GameObject is destroyed.
//
// ECS-style ownership: when Unity destroys a Part GameObject (crash,
// splash damage, fairing despawn, scene exit) JoltBody.OnDestroy fires
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
    public sealed class JoltBody : MonoBehaviour
    {
        public BodyHandle Handle;
        public Part Part;

        // The latest analytic Jolt-integrated velocity for this part,
        // stored at pose readback. Unity silently discards rb.velocity
        // writes on kinematic rigidbodies, so reading rb.velocity gives
        // 0 — useless for navball / orbit / aero / parachute deploy
        // gates. Anywhere stock reads rb.velocity, we either patch the
        // reader or compute the same field from this stored value.
        public Vector3 LastVelocity;
        public Vector3 LastAngularVelocity;

        // Tonnes (KSP convention). Compared against
        // `part.mass + part.GetResourceMass()` each tick by the
        // driver; when it differs by more than a threshold, a
        // MassUpdate record is queued and this is updated.
        public float LastMass;

        // Pending body destroys from GameObjects Unity has destroyed
        // since the last drain. Static so the queue survives even when
        // a JoltBody's component is gone — the queue is keyed by
        // BodyHandle, not by component reference.
        static readonly Queue<BodyHandle> _pendingDestroy = new Queue<BodyHandle>();

        // BodyHandle.Id → JoltBody reverse lookup. The bridge emits
        // output records (BodyPose, ContactReport) keyed by uint body
        // id; the driver uses this map to find the corresponding Part
        // and rigidbody. Forward lookup (Part → BodyHandle) is just
        // `part.gameObject.GetComponent<JoltBody>().Handle`.
        static readonly Dictionary<uint, JoltBody> _byHandle =
            new Dictionary<uint, JoltBody>();

        public static JoltBody AttachTo(Part part, BodyHandle handle)
        {
            if (part == null || part.gameObject == null) return null;
            var jb = part.gameObject.AddComponent<JoltBody>();
            jb.Handle = handle;
            jb.Part = part;
            _byHandle[handle.Id] = jb;
            return jb;
        }

        public static bool TryGet(uint bodyId, out JoltBody jb) =>
            _byHandle.TryGetValue(bodyId, out jb);

        // Public hook for sibling component-owners (QuadBody for PQS
        // terrain quads, future SceneObjectBody for KSC static, etc.)
        // that participate in the same per-tick BodyDestroy drain.
        // Reusing the queue keeps a single drain site in
        // TopologyReconciler.Reconcile rather than scattering bridge
        // bookkeeping across resource owners.
        public static void EnqueuePendingDestroy(BodyHandle handle)
        {
            if (handle.IsValid) _pendingDestroy.Enqueue(handle);
        }

        void OnDestroy()
        {
            if (Handle.IsValid)
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
