// Per-scene list of static-world AABB primitives. Populated at VesselScene
// build by scanning the launchpad hierarchy in the flight scene. At this
// stage one AABB is enough to demo a rocket standing on its engine bell;
// runway / PQS terrain / KSC buildings will register additional AABBs when
// we cover them.
//
// KSP's "Launch Pad" GameObject carries the MeshCollider we care about
// (identified during Phase A diagnostics: layer=15, concave MeshCollider).
// We find it by name in the active scene — if the scene loaded something
// else (runway, non-KSC launch site), Count = 0 and the vessel will fall
// through until we extend the discovery strategy.

using System.Collections.Generic;
using Longeron.Physics;
using Longeron.Physics.Contact;
using UnityEngine;

namespace Longeron.Integration
{
    internal sealed class StaticWorld
    {
        readonly List<AabbShape> aabbs = new List<AabbShape>();

        public int Count => aabbs.Count;
        public AabbShape this[int i] => aabbs[i];

        public void Add(AabbShape aabb) => aabbs.Add(aabb);
        public void Clear() => aabbs.Clear();

        // Scan the flight scene for the launchpad and add its collider bounds.
        // Stock KSP names the GameObject "Launch Pad"; we walk its entire
        // child hierarchy so we pick up sub-colliders (lightning rods, structural
        // elements) that also belong to the pad.
        public static StaticWorld BuildForLaunchpad()
        {
            var sw = new StaticWorld();

            var pad = GameObject.Find("Launch Pad");
            if (pad != null)
            {
                foreach (var c in pad.GetComponentsInChildren<Collider>(includeInactive: false))
                {
                    var b = c.bounds;
                    sw.Add(new AabbShape(
                        new float3(b.min.x, b.min.y, b.min.z),
                        new float3(b.max.x, b.max.y, b.max.z)));
                }
            }

            return sw;
        }
    }
}
