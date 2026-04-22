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

        // Last-tick Krakensbane FrameVelocity. Used to detect the ΔFV that
        // Krakensbane drains from rigidbodies each tick and apply the same
        // drain to rootVelocity, keeping our solver aligned with Unity's
        // translating frame.
        public float3 LastFrameVelocity;

        // Per-tick contact buffer. LongeronScenePreTick clears this at the
        // start of each FixedUpdate; ContactDiscovery.Discover fills it from
        // query-based physics probes; the late driver drains it into fExt via
        // ContactSolver.Apply before Scene.Step.
        public readonly List<ContactEntry> Contacts = new List<ContactEntry>(64);

        // Set of this vessel's own colliders. ContactDiscovery uses it to
        // filter intra-vessel hits out of the OverlapSphere results (OverlapSphere
        // doesn't honor Physics.IgnoreCollision pairs — it's a broadphase query).
        public readonly HashSet<Collider> OwnColliders = new HashSet<Collider>();

        private VesselScene(Vessel vessel, ArticulatedScene scene, Part[] bodyToPart, Dictionary<Part, BodyId> partToBody)
        {
            Vessel = vessel;
            Scene = scene;
            BodyToPart = bodyToPart;
            PartToBody = partToBody;
            RootVelocitySeeded = false;
            LastFrameVelocity = float3.zero;
        }

        public void ClearContacts() => Contacts.Clear();

        public void AddContact(Part part, ContactPoint cp)
        {
            if (!PartToBody.TryGetValue(part, out var bodyId)) return;
            Contacts.Add(new ContactEntry
            {
                bodyId = bodyId,
                point = new float3(cp.point.x, cp.point.y, cp.point.z),
                normal = new float3(cp.normal.x, cp.normal.y, cp.normal.z),
                separation = cp.separation,
            });
        }

        // Krakensbane bridge. Call once per FixedUpdate *before* Scene.Step.
        //   1. Subtract (thisFV - lastFV) from rootVelocity. Matches the drain
        //      Krakensbane just applied to rb.velocity via ChangeWorldVelocity.
        //   2. Subtract thisFV * dt from rootPose.translation. Compensates for
        //      FloatingOrigin's translating Unity's world by -FrameVel*dt each
        //      tick while Krakensbane is engaged.
        // Snap rootPose.translation to the root rigidbody's current Unity
        // position. This is how we reconcile with stock's absolute-position
        // authority — FloatingOrigin can teleport rb.position between
        // ticks, Krakensbane can translate it; we just follow.
        public void SyncRootPoseFromRigidbody()
        {
            if (BodyToPart.Length == 0) return;
            var rb = BodyToPart[0].rb;
            if (rb == null) return;
            var pos = rb.position;
            var rot = rb.rotation;
            Scene.Body.rootPose = new SpatialTransform(
                new Physics.quaternion(rot.x, rot.y, rot.z, rot.w),
                new float3(pos.x, pos.y, pos.z));
        }

        // Drain the Krakensbane ΔFV from rootVelocity — keeps our solver's
        // velocity consistent with what Krakensbane just took out of rb via
        // ChangeWorldVelocity. Position shifts are handled by
        // SyncRootPoseFromRigidbody above; this pass is velocity-only.
        public void ApplyKrakensbaneStep(float dt)
        {
            Vector3d fv = Krakensbane.GetFrameVelocity();
            float3 thisFV = new float3((float)fv.x, (float)fv.y, (float)fv.z);
            float3 deltaFV = thisFV - LastFrameVelocity;

            var qInv = math.inverse(Scene.Body.rootPose.rotation);
            float3 deltaBody = math.mul(qInv, deltaFV);
            Scene.Body.rootVelocity = new SpatialMotion(
                Scene.Body.rootVelocity.angular,
                Scene.Body.rootVelocity.linear - deltaBody);

            LastFrameVelocity = thisFV;
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

            // rootVelocity lives in "Unity's current frame" — same frame
            // Krakensbane's per-tick ΔFV operates in. Seed from rb.velocity
            // (which is what Krakensbane drains *from*). For a landed vessel
            // that's ~0 (Kerbin rotates beneath; rb frame rotates with it).
            // For high-speed / orbital vessels, Krakensbane has already
            // drained rb.velocity and parked the excess in FrameVelocity.
            var qInv = math.inverse(Scene.Body.rootPose.rotation);
            var worldV  = rb.velocity;
            var worldOm = rb.angularVelocity;
            float3 vBody  = math.mul(qInv, new float3(worldV.x, worldV.y, worldV.z));
            float3 omBody = math.mul(qInv, new float3(worldOm.x, worldOm.y, worldOm.z));
            Scene.Body.rootVelocity = new SpatialMotion(omBody, vBody);
            Scene.Body.rootPose = ToSpatialTransform(BodyToPart[0].transform);

            // Snapshot Krakensbane's current FrameVelocity as the baseline so
            // ApplyKrakensbaneStep's first ΔFV is zero (no spurious drain).
            Vector3d fv = Krakensbane.GetFrameVelocity();
            LastFrameVelocity = new float3((float)fv.x, (float)fv.y, (float)fv.z);

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
            var vs = new VesselScene(vessel, scene, bodyToPart, partToBody);

            // Collect every collider hanging off any managed part (including
            // sub-colliders on child transforms — ladders, attach nodes, etc.).
            // ContactDiscovery filters overlap hits against this set to skip
            // self-contacts.
            foreach (var p in ordered)
            {
                foreach (var col in p.GetComponentsInChildren<Collider>(includeInactive: true))
                    vs.OwnColliders.Add(col);
            }
            return vs;
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
