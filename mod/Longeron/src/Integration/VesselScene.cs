// VesselScene — wraps an ArticulatedScene together with a Part↔BodyId map,
// plus enough bookkeeping to build the scene from a live KSP vessel.
//
// Responsibilities:
//   - Enumerate vessel.parts in topological order (parent index < child index).
//   - Install a Floating root at body[0] (the KSP vessel's root part) and
//     Fixed joints for every other part (BG servos etc. are out of scope
//     for the spike — we treat every non-root joint as rigid).
//   - Maintain a fast Part → BodyId map for force-hook lookups.
//   - Place the Floating root's rootPose at the root part's current
//     world transform so the solver starts from a consistent pose.

using System.Collections.Generic;
using Longeron.Physics;
using UnityEngine;

namespace Longeron.Integration
{
    internal sealed class VesselScene
    {
        public Vessel Vessel { get; }
        public ArticulatedScene Scene { get; }
        public Part[] BodyToPart { get; private set; }
        public Dictionary<Part, BodyId> PartToBody { get; private set; }
        public bool RootVelocitySeeded { get; private set; }

        private VesselScene(Vessel vessel, ArticulatedScene scene, Part[] bodyToPart, Dictionary<Part, BodyId> partToBody)
        {
            Vessel = vessel;
            Scene = scene;
            BodyToPart = bodyToPart;
            PartToBody = partToBody;
            RootVelocitySeeded = false;
        }

        // Copy the root rigidbody's current velocity (world-frame) into the
        // Floating root's rootVelocity (body-frame), and mark the scene as
        // seeded so the driver doesn't overwrite it later.
        //
        // Called from the scene driver on the first FixedUpdate where the
        // root rb is available. VesselModule.OnGoOffRails fires *before*
        // Part.Unpack(), so rb may be null — or its velocity uninitialised
        // — at scene-build time. Deferring the seed here lets stock finish
        // its own off-rails setup (precalc.GoOffRails + Part.Unpack) first.
        public void TrySeedRootVelocity()
        {
            if (RootVelocitySeeded) return;
            if (BodyToPart.Length == 0) return;
            var rb = BodyToPart[0].rb;
            if (rb == null) return;

            // Spike milestone: start at rest in the Unity frame. Any residual
            // `rb.velocity` (e.g. the tiny Kerbin-rotation tangential on the pad)
            // would produce visible drift when combined with zero gravity.
            // Proper handling is `rb.velocity + Krakensbane.Instance.FrameVelocity`
            // and a Krakensbane-aware writeback — out of spike scope.
            Scene.Body.rootVelocity = SpatialMotion.zero;
            Scene.Body.rootPose = ToSpatialTransform(BodyToPart[0].transform);
            RootVelocitySeeded = true;
        }

        // Zero the external wrench accumulator across all bodies. Called at
        // the start of each FixedUpdate before stock/modded force producers
        // run; their AddForce calls re-populate fExt via the Harmony hooks.
        public void ResetExternalWrenches()
        {
            var b = Scene.Body;
            for (int i = 0; i < b.Count; i++)
                b.fExt[i] = SpatialForce.zero;
        }

        public void AddWorldForce(BodyId id, float3 worldForce)
        {
            var X = Scene.GetWorldTransform(id);
            float3 forceBody = math.mul(math.inverse(X.rotation), worldForce);
            var cur = Scene.Body.fExt[id.index];
            Scene.Body.fExt[id.index] = cur + new SpatialForce(float3.zero, forceBody);
        }

        public void AddWorldForceAtPosition(BodyId id, float3 worldForce, float3 worldPos)
        {
            var X = Scene.GetWorldTransform(id);
            var qInv = math.inverse(X.rotation);
            float3 forceBody = math.mul(qInv, worldForce);
            float3 rBody     = math.mul(qInv, worldPos - X.translation);
            float3 torqueBody = math.cross(rBody, forceBody);
            var cur = Scene.Body.fExt[id.index];
            Scene.Body.fExt[id.index] = cur + new SpatialForce(torqueBody, forceBody);
        }

        public void AddWorldTorque(BodyId id, float3 worldTorque)
        {
            var X = Scene.GetWorldTransform(id);
            float3 torqueBody = math.mul(math.inverse(X.rotation), worldTorque);
            var cur = Scene.Body.fExt[id.index];
            Scene.Body.fExt[id.index] = cur + new SpatialForce(torqueBody, float3.zero);
        }

        public static VesselScene Build(Vessel vessel)
        {
            var ordered = OrderPartsTopologically(vessel);
            var partToBody = new Dictionary<Part, BodyId>(ordered.Count);
            var bodyToPart = new Part[ordered.Count];
            var scene = new ArticulatedScene(ordered.Count);

            for (int i = 0; i < ordered.Count; i++)
            {
                Part p = ordered[i];
                var inertia = InertiaEstimator.Estimate(p);

                BodyId id;
                if (i == 0)
                {
                    // Root: Floating joint. Xtree is ignored for Floating; rootPose
                    // is seeded from the part's world transform. rootVelocity is
                    // deferred to the driver's first tick via TrySeedRootVelocity,
                    // since VesselModule.OnGoOffRails fires before Part.Unpack
                    // populates rb / rb.velocity.
                    id = scene.AddBody(BodyId.None, Physics.Joint.Floating(), inertia, SpatialTransform.identity);
                    scene.Body.rootPose = ToSpatialTransform(p.transform);
                    scene.Body.rootVelocity = SpatialMotion.zero;
                }
                else
                {
                    Part parent = p.parent;
                    BodyId parentId = partToBody[parent];
                    // Xtree: parent-body-frame → child-body-frame, constant over q.
                    // For Fixed joint this IS the full relative transform at rest.
                    var xtree = RelativeTransform(parent.transform, p.transform);
                    id = scene.AddBody(parentId, Physics.Joint.Fixed(), inertia, xtree);
                }

                partToBody[p] = id;
                bodyToPart[i] = p;
            }

            scene.Validate();
            return new VesselScene(vessel, scene, bodyToPart, partToBody);
        }

        // Orphaned parts (no vessel relationship) are skipped. The returned
        // list starts with vessel.rootPart and walks its descendants
        // breadth-first so that each part's parent precedes it.
        static List<Part> OrderPartsTopologically(Vessel vessel)
        {
            var result = new List<Part>(vessel.parts.Count);
            var queue = new Queue<Part>();
            if (vessel.rootPart == null) return result;
            queue.Enqueue(vessel.rootPart);
            while (queue.Count > 0)
            {
                var p = queue.Dequeue();
                result.Add(p);
                foreach (var child in p.children)
                    queue.Enqueue(child);
            }
            return result;
        }

        static SpatialTransform ToSpatialTransform(Transform t)
        {
            var rot = t.rotation;
            var pos = t.position;
            return new SpatialTransform(
                new Physics.quaternion(rot.x, rot.y, rot.z, rot.w),
                new Physics.float3(pos.x, pos.y, pos.z));
        }

        // World→B / World→A given, compute A→B: X_A→B = X_W→B ∘ X_A→W.
        static SpatialTransform RelativeTransform(Transform fromA, Transform toB)
        {
            var A = ToSpatialTransform(fromA);
            var B = ToSpatialTransform(toB);
            // A→B = (W→B) * (A→W) = B * A.Inverse()
            return B * A.Inverse();
        }
    }
}
