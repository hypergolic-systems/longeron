// JoltBody — MonoBehaviour attached to every Part GameObject Longeron
// manages. Carries the BodyHandle that maps the part to its Jolt body.
//
// Why a component instead of a Dictionary<Rigidbody, BodyHandle> in
// SceneRegistry: the lookup is on the hot path (every Rigidbody.AddForce
// call from stock and modded code goes through it). A
// Rigidbody.GetComponent<JoltBody>() is one Unity-native O(1) probe and
// cleans itself up automatically when the GameObject is destroyed. No
// manual registry mutation on vessel destruction, no concurrent-access
// hazards, no leaks if a Part disappears unexpectedly.

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

        public static JoltBody AttachTo(Part part, BodyHandle handle)
        {
            if (part == null || part.gameObject == null) return null;
            var jb = part.gameObject.AddComponent<JoltBody>();
            jb.Handle = handle;
            jb.Part = part;
            return jb;
        }
    }
}
