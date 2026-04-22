// Contact discovery — analytic narrowphase, not PhysX queries.
//
// Previous incarnation ran Physics.OverlapSphere / ComputePenetration / Raycast
// per part each tick. That worked but was fragile (concave mesh garbage, layer
// mask tuning, 66 Hz contact-set flicker) and produced point samples rather
// than shape-aware manifolds. The penalty solver that consumed it pumped
// energy, couldn't hold a rocket upright, and required five different
// stopgap knobs to even approximate stability.
//
// The replacement: walk each managed body's CapsuleShape (body-frame, built
// once at scene construction) through the solver's current world transform,
// then run analytic CapsuleVsAabb against every registered StaticWorld AABB.
// Each pair produces up to 4 ContactEntries — a proper support-polygon
// manifold for endpoint-penetrating capsules, which the PGS constraint
// solver converts into restoring-torque impulses.
//
// Intra-vessel pairs don't exist here — there are no capsule-vs-capsule
// checks yet. The launchpad and eventually other AABBs are the only
// contacts. When self-collision or part-vs-part is on the roadmap, this
// file grows a second pair-type loop (capsule-vs-capsule narrowphase).

using Longeron.Physics;
using Longeron.Physics.Contact;

namespace Longeron.Integration
{
    internal static class ContactDiscovery
    {
        public static void Discover(VesselScene scene)
        {
            if (scene.StaticWorld == null || scene.StaticWorld.Count == 0) return;
            if (scene.Shapes == null) return;

            int bodyCount = scene.BodyToPart.Length;
            var mf = new ContactManifold();

            for (int b = 0; b < bodyCount; b++)
            {
                var shape = scene.Shapes[b];
                if (shape.radius <= 0f) continue;

                var X = scene.Scene.GetWorldTransform(new BodyId(b));
                float3 segStartW = math.mul(X.rotation, shape.axisStart) + X.translation;
                float3 segEndW   = math.mul(X.rotation, shape.axisEnd)   + X.translation;

                for (int a = 0; a < scene.StaticWorld.Count; a++)
                {
                    Narrowphase.CapsuleVsAabb(segStartW, segEndW, shape.radius,
                                              scene.StaticWorld[a], ref mf);
                    for (int i = 0; i < mf.count; i++)
                    {
                        mf.Get(i, out var point, out var normal, out var depth);
                        scene.Contacts.Add(new ContactConstraint
                        {
                            bodyId = new BodyId(b),
                            point  = point,
                            normal = normal,
                            depth  = depth,
                        });
                    }
                }
            }
        }
    }
}
