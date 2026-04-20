using Microsoft.VisualStudio.TestTools.UnitTesting;
using Longeron.Physics;

namespace Longeron.Physics.Tests
{
    // Simple pendulum: Fixed root at pivot, bob on a Revolute joint about
    // the y-axis. COM offset = (0, 0, -L) in bob's body frame (bob hangs
    // below pivot when q=0). Gravity in -z.
    //
    // Small-angle period T = 2π·√(L/g). This test validates that the
    // gravity trick, Coriolis handling, and joint subspace all compose
    // into the correct coupled dynamics.
    [TestClass]
    public class SimplePendulumTests
    {
        const float g = 9.81f;
        const float L = 1.0f;

        static ArticulatedScene BuildPendulum(float initialAngle)
        {
            var scene = new ArticulatedScene();
            scene.SetGravity(new float3(0, 0, -g));
            var root = scene.AddBody(BodyId.None, Joint.Fixed(),
                new SpatialInertia(0f, float3.zero, float3x3.zero),
                SpatialTransform.identity);
            // Point-mass bob at (0,0,-L) in body frame, tiny inertia about COM.
            var bob = scene.AddBody(root, Joint.Revolute(float3.unitY),
                new SpatialInertia(1f, new float3(0f, 0f, -L),
                    new float3x3(new float3(1e-4f, 0, 0), new float3(0, 1e-4f, 0), new float3(0, 0, 1e-4f))),
                SpatialTransform.identity);
            scene.SetJointPosition(bob, initialAngle);
            return scene;
        }

        [TestMethod]
        public void SmallAnglePeriodMatchesAnalytic()
        {
            // Tiny initial angle so the anharmonic correction is negligible
            // (θ₀²/16 ≈ 6e-5 for θ₀=0.03).
            float theta0 = 0.03f;
            var scene = BuildPendulum(theta0);

            float Texpected = 2f * math.PI * math.sqrt(L / g);
            float dt = 1e-4f;
            int N = (int)(Texpected / dt + 0.5f);

            var bob = new BodyId(1);
            for (int i = 0; i < N; i++) scene.Step(dt);

            // After one period, angle should return close to θ₀ and qdot near 0.
            float q = scene.GetJointPosition(bob);
            float qdot = scene.GetJointVelocity(bob);
            Assert.AreEqual(theta0, q, 1e-3f, "angle after one period");
            Assert.AreEqual(0f,     qdot, 5e-2f, "angular velocity after one period");
        }

        [TestMethod]
        public void HangingStraightDownIsEquilibrium()
        {
            // q=0, qdot=0: bob hangs vertically below pivot. Gravity torque
            // is zero there; system should stay put.
            var scene = BuildPendulum(0f);
            var bob = new BodyId(1);

            for (int i = 0; i < 5_000; i++) scene.Step(1e-3f);

            Assert.AreEqual(0f, scene.GetJointPosition(bob), 1e-4f);
            Assert.AreEqual(0f, scene.GetJointVelocity(bob), 1e-4f);
        }

        [TestMethod]
        public void SwingingPendulumConservesEnergyApproximately()
        {
            // Small oscillation: total energy E = 0.5·m·L²·qdot² + m·g·L·(1-cos q)
            // Semi-implicit Euler is symplectic → energy drift is bounded, not
            // monotonic. Check drift stays within a small band over many steps.
            float theta0 = 0.05f;
            var scene = BuildPendulum(theta0);
            var bob = new BodyId(1);
            const float m = 1f;

            float E0 = EnergyOf(scene, bob, m);
            float minE = E0, maxE = E0;
            for (int i = 0; i < 20_000; i++)
            {
                scene.Step(1e-4f);
                float E = EnergyOf(scene, bob, m);
                if (E < minE) minE = E;
                if (E > maxE) maxE = E;
            }
            float drift = (maxE - minE) / E0;
            Assert.IsTrue(drift < 0.05f, $"energy drift {drift:P} exceeded 5% band");
        }

        static float EnergyOf(ArticulatedScene scene, BodyId bob, float m)
        {
            float q = scene.GetJointPosition(bob);
            float qdot = scene.GetJointVelocity(bob);
            float kinetic = 0.5f * m * L * L * qdot * qdot;
            float potential = m * g * L * (1f - math.cos(q));
            return kinetic + potential;
        }
    }
}
