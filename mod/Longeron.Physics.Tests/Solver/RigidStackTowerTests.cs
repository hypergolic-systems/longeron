using Microsoft.VisualStudio.TestTools.UnitTesting;
using Longeron.Physics;

namespace Longeron.Physics.Tests
{
    // The headline test: a 20-body tower of Fixed joints stacked on a
    // single Prismatic-x root DOF. Apply a lateral thrust at the tip. In
    // the real KSP / PhysX world this chain would flex and oscillate
    // ("wobble"). Featherstone on a Fixed-joined tree has *zero* internal
    // DOFs for the tower — it's mathematically a single rigid body. The
    // only DOF is the root prismatic, so the whole tower must accelerate
    // together at F / M_total.
    [TestClass]
    public class RigidStackTowerTests
    {
        const int N = 20;          // tower bodies above the prismatic root
        const float massPer = 1f;  // kg
        const float segLen = 0.5f; // m between bodies
        const float F = 10f;       // N, applied laterally at tip

        [TestMethod]
        public void TipForceAcceleratesWholeTowerAsRigidBody()
        {
            var scene = new ArticulatedScene();
            // No gravity — we're isolating the tip-force response.
            scene.SetGravity(float3.zero);

            // Root: massless prismatic along x.
            var root = scene.AddBody(BodyId.None, Joint.Prismatic(float3.unitX),
                new SpatialInertia(0f, float3.zero, float3x3.zero),
                SpatialTransform.identity);

            // N bodies stacked along +z, Fixed-joined. Each has mass massPer,
            // COM at its body frame origin, small diagonal inertia.
            var inertia = new float3x3(new float3(1e-3f, 0, 0), new float3(0, 1e-3f, 0), new float3(0, 0, 1e-3f));
            BodyId prev = root;
            var ids = new BodyId[N];
            for (int i = 0; i < N; i++)
            {
                // Xtree places this body `segLen` above its parent along z,
                // orientation unchanged.
                var xtree = new SpatialTransform(quaternion.identity, new float3(0f, 0f, segLen));
                ids[i] = scene.AddBody(prev, Joint.Fixed(),
                    new SpatialInertia(massPer, float3.zero, inertia),
                    xtree);
                prev = ids[i];
            }

            // Apply lateral force at tip. Body-frame = world-frame orientation
            // for this test (all Fixed joints with identity rotation).
            scene.SetExternalWrench(ids[N - 1], new SpatialForce(float3.zero, new float3(F, 0, 0)));

            float dt = 1e-3f;
            int steps = 1000;
            for (int i = 0; i < steps; i++) scene.Step(dt);

            // Total mass of the tower (root is massless).
            float Mtot = N * massPer;
            float aExpected = F / Mtot;

            // qdot(t) = a·t, q(t) ≈ ½·a·t² (semi-implicit Euler drift ~dt).
            float qdot = scene.GetJointVelocity(root);
            float q = scene.GetJointPosition(root);
            Assert.AreEqual(aExpected,        qdot, 0.02f, "qdot(1s) = F/M");
            Assert.AreEqual(0.5f * aExpected, q,    0.02f, "q(1s) = ½·F/M·t²");
        }

        [TestMethod]
        public void EveryBodyShareRootPrismaticTranslation()
        {
            // After applying the tip force, the x position of every body
            // in the tower must equal the root's x (no internal flex),
            // and each body's z must stay at its stacking height.
            var scene = new ArticulatedScene();
            scene.SetGravity(float3.zero);
            var root = scene.AddBody(BodyId.None, Joint.Prismatic(float3.unitX),
                new SpatialInertia(0f, float3.zero, float3x3.zero),
                SpatialTransform.identity);
            var inertia = new float3x3(new float3(1e-3f, 0, 0), new float3(0, 1e-3f, 0), new float3(0, 0, 1e-3f));
            BodyId prev = root;
            var ids = new BodyId[N];
            for (int i = 0; i < N; i++)
            {
                var xtree = new SpatialTransform(quaternion.identity, new float3(0f, 0f, segLen));
                ids[i] = scene.AddBody(prev, Joint.Fixed(),
                    new SpatialInertia(massPer, float3.zero, inertia),
                    xtree);
                prev = ids[i];
            }
            scene.SetExternalWrench(ids[N - 1], new SpatialForce(float3.zero, new float3(F, 0, 0)));

            for (int i = 0; i < 500; i++) scene.Step(1e-3f);

            // Query world transforms; root's x is its prismatic q. Every
            // tower body should report the same world x (zero flex) and
            // a world z that matches its stacking height.
            float rootX = scene.GetJointPosition(root);
            for (int i = 0; i < N; i++)
            {
                var X = scene.GetWorldTransform(ids[i]);
                Assert.AreEqual(rootX,            X.translation.x, 1e-4f, $"body {i} x");
                Assert.AreEqual(0f,               X.translation.y, 1e-4f, $"body {i} y");
                Assert.AreEqual(segLen * (i + 1), X.translation.z, 1e-4f, $"body {i} z");
            }
        }
    }
}
