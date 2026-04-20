using Microsoft.VisualStudio.TestTools.UnitTesting;
using Longeron.Physics;

namespace Longeron.Physics.Tests
{
    // Simplest forward-dynamics test: Fixed root + one Prismatic joint
    // along gravity. Body should fall at g (modulo integrator error).
    // Also the first test that verifies the gravity trick works end-to-end.
    [TestClass]
    public class PrismaticSlideTests
    {
        [TestMethod]
        public void BodyFallsAtGravityAlongPrismaticAxis()
        {
            var scene = new ArticulatedScene();
            scene.SetGravity(new float3(0, 0, -9.81f));

            // Massless Fixed root anchor.
            var root = scene.AddBody(BodyId.None, Joint.Fixed(),
                new SpatialInertia(0f, float3.zero, float3x3.zero),
                SpatialTransform.identity);
            // 1 kg body on a prismatic joint along +z.
            var slider = scene.AddBody(root,
                Joint.Prismatic(float3.unitZ),
                new SpatialInertia(1f, float3.zero,
                    new float3x3(new float3(0.1f, 0, 0), new float3(0, 0.1f, 0), new float3(0, 0, 0.1f))),
                SpatialTransform.identity);

            float dt = 1e-3f;
            int steps = 1000;
            for (int i = 0; i < steps; i++) scene.Step(dt);

            // Analytical: q(t) = 0.5·g·t² with g in -z, axis in +z → q negative.
            // Semi-implicit Euler drifts by ~(dt/t)·q_analytical ≈ 0.1% per t=1s / dt=1e-3.
            float q = scene.GetJointPosition(slider);
            float qdot = scene.GetJointVelocity(slider);
            Assert.AreEqual(-4.905f, q, 0.02f,    "q after 1s of free fall");
            Assert.AreEqual(-9.81f,  qdot, 0.02f, "qdot after 1s of free fall");
        }

        [TestMethod]
        public void RestStateStableWithNoForces()
        {
            var scene = new ArticulatedScene();
            // No gravity, no external forces; qdot=0.
            var root = scene.AddBody(BodyId.None, Joint.Fixed(),
                new SpatialInertia(0f, float3.zero, float3x3.zero),
                SpatialTransform.identity);
            var slider = scene.AddBody(root,
                Joint.Prismatic(float3.unitZ),
                new SpatialInertia(1f, float3.zero,
                    new float3x3(new float3(0.1f, 0, 0), new float3(0, 0.1f, 0), new float3(0, 0, 0.1f))),
                SpatialTransform.identity);

            for (int i = 0; i < 10_000; i++) scene.Step(1e-3f);

            Assert.AreEqual(0f, scene.GetJointPosition(slider), 1e-4f);
            Assert.AreEqual(0f, scene.GetJointVelocity(slider), 1e-4f);
        }
    }
}
