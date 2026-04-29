// QuadBody — MonoBehaviour attached to every PQS quad GameObject we've
// mirrored into Jolt as a static MeshShape body.
//
// Mirrors the JoltBody pattern from Phase 2.5: the resource (Jolt
// body) is owned by the GameObject, and the GameObject's destruction
// queues the matching BodyDestroy via Unity's component lifecycle. No
// periodic sweep, no per-quad bookkeeping in a side dictionary.
//
// LastVertexCount / LastTriangleCount are sampled on each
// BodyCreate to detect in-place mesh edits between PQSMod
// OnQuadUpdate fires (PQS reuses the same MeshCollider component
// across LOD swaps; only sharedMesh.vertices / .triangles change).

using System.Collections.Generic;
using Longeron.Native;
using UnityEngine;

namespace Longeron.Integration
{
    [DisallowMultipleComponent]
    public sealed class QuadBody : MonoBehaviour
    {
        public BodyHandle Handle;
        public PQ Quad;
        public int LastVertexCount;
        public int LastTriangleCount;

        // CB-frame position of the quad at mirror time, captured for
        // diagnostic comparison against a fresh frame.WorldToCb
        // computation. With the frame redesign, the Jolt body sits at
        // this CB-frame position permanently — drift in Unity world
        // (planet rotation, orbital motion) doesn't move it.
        public Vector3d BakedCbPosition;

        // Used to walk the live set for diagnostic traversal and SOI-
        // change cleanup. Maintained in AttachTo / OnDestroy.
        internal static readonly List<QuadBody> Active =
            new List<QuadBody>();

        public static QuadBody AttachTo(PQ quad, BodyHandle handle)
        {
            if (quad == null || quad.gameObject == null) return null;
            var qb = quad.gameObject.AddComponent<QuadBody>();
            qb.Handle = handle;
            qb.Quad = quad;
            Active.Add(qb);
            return qb;
        }

        void OnDestroy()
        {
            JoltBody.EnqueuePendingDestroy(Handle);
            Active.Remove(this);
        }
    }
}
