// StaticBody — MonoBehaviour attached to every Unity Collider we've
// mirrored into Jolt as a static body for KSC / launch-site geometry
// (PQSCity[2] children).
//
// Same shape as QuadBody (PQS terrain quads): the resource (Jolt body)
// is owned by the GameObject, and OnDestroy queues the matching
// BodyDestroy via JoltPart.EnqueuePendingDestroy. No periodic sweep,
// no side-table bookkeeping.
//
// Unlike PQS quads (which are pooled by PQSCache and don't fire Unity
// OnDestroy on recycle), KSC GameObjects aren't pooled — scene exit
// destroys them and OnDestroy fires naturally. The DestroyAll sweep
// is still required at world teardown so OnDestroy lands while
// ActiveWorld is still live (otherwise the BodyDestroy enqueue is
// silently lost and the next world inherits stale handle IDs).

using System.Collections.Generic;
using Longeron.Native;
using UnityEngine;

namespace Longeron.Integration
{
    [DisallowMultipleComponent]
    public sealed class StaticBody : MonoBehaviour
    {
        public BodyHandle Handle;

        // Live set, walked by DestroyAll on world teardown.
        internal static readonly List<StaticBody> Active = new List<StaticBody>();

        public static StaticBody AttachTo(GameObject go, BodyHandle handle)
        {
            if (go == null) return null;
            var sb = go.AddComponent<StaticBody>();
            sb.Handle = handle;
            Active.Add(sb);
            return sb;
        }

        void OnDestroy()
        {
            JoltPart.EnqueuePendingDestroy(Handle);
            Active.Remove(this);
        }

        // Explicitly destroy every live StaticBody. Called from
        // LongeronAddon.DisposeWorld before the World is freed so the
        // OnDestroy queue-into-live-Input path fires while it can still
        // do useful work. Snapshot the list first because OnDestroy
        // mutates Active.
        public static void DestroyAll()
        {
            if (Active.Count == 0) return;
            var snapshot = Active.ToArray();
            for (int i = 0; i < snapshot.Length; ++i)
            {
                var sb = snapshot[i];
                if (sb != null) UnityEngine.Object.DestroyImmediate(sb);
            }
            Active.Clear();
        }
    }
}
