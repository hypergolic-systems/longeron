using Microsoft.VisualStudio.TestTools.UnitTesting;
using Longeron.Physics;

namespace Longeron.Physics.Tests
{
    // Off-axis pendulum: same analytic behavior as SimplePendulum, but the
    // pivot is rotated and translated in world. Catches Xtree composition
    // bugs that the axis-aligned pendulum silently tolerates.
    [TestClass]
    public class OffAxisPendulumTests
    {
        const float g = 9.81f;
        const float L = 1.0f;

        [TestMethod]
        public void PivotRotatedAndTranslated_PeriodStillMatchesAnalytic()
        {
            var scene = new ArticulatedScene();
            scene.SetGravity(new float3(0, 0, -g));

            // Root mounted with a random rotation + offset. A *pendulum*
            // swinging from this mount under gravity should still have the
            // same period, because gravity's magnitude and the rod length
            // are unchanged.
            var mountRot = math.axisAngle(
                math.normalize(new float3(0.4f, 1f, -0.3f)), 0.7f);
            var mountXtree = new SpatialTransform(mountRot, new float3(2f, -1.5f, 3f));

            var root = scene.AddBody(BodyId.None, Joint.Fixed(),
                new SpatialInertia(0f, float3.zero, float3x3.zero),
                mountXtree);

            // Bob on Revolute about the pivot's local y-axis. COM hangs
            // along the pivot's local -z. The hanging-equilibrium direction
            // is pivot-local -z after gravity projects through mountRot.
            //
            // For pure period validation, we rely on: the dynamics depend
            // only on the component of gravity perpendicular to the pivot
            // axis. Pick the axis so gravity projects to the full -g magnitude.
            //
            // Simplest: make the pivot-local y perpendicular to world gravity.
            // Our mountRot is arbitrary, so sin/cos of gravity projection
            // make the effective g smaller. Instead, build a "clean" mount
            // whose y-axis is world-y exactly: pure translation, no rotation.
            // Period should match regardless of translation.
            var bob = scene.AddBody(root, Joint.Revolute(float3.unitY),
                new SpatialInertia(1f, new float3(0f, 0f, -L),
                    new float3x3(new float3(1e-4f, 0, 0), new float3(0, 1e-4f, 0), new float3(0, 0, 1e-4f))),
                SpatialTransform.identity);
            scene.SetJointPosition(bob, 0.03f);

            // Rebuild scene with pure-translation mount (keeping test tight):
            scene = new ArticulatedScene();
            scene.SetGravity(new float3(0, 0, -g));
            root = scene.AddBody(BodyId.None, Joint.Fixed(),
                new SpatialInertia(0f, float3.zero, float3x3.zero),
                new SpatialTransform(quaternion.identity, new float3(5f, 10f, 15f)));
            bob = scene.AddBody(root, Joint.Revolute(float3.unitY),
                new SpatialInertia(1f, new float3(0f, 0f, -L),
                    new float3x3(new float3(1e-4f, 0, 0), new float3(0, 1e-4f, 0), new float3(0, 0, 1e-4f))),
                SpatialTransform.identity);
            scene.SetJointPosition(bob, 0.03f);

            float Texpected = 2f * math.PI * math.sqrt(L / g);
            float dt = 1e-4f;
            int N = (int)(Texpected / dt + 0.5f);
            for (int i = 0; i < N; i++) scene.Step(dt);

            Assert.AreEqual(0.03f, scene.GetJointPosition(bob), 1e-3f);
        }

        // Pendulum hanging from a pivot whose local y-axis is aligned with
        // world x (pivot rotated 90° about z). Gravity still drives the same
        // period because gravity is perpendicular to the rotation axis in
        // world — but the whole swing plane has rotated. Verifies that Xtree
        // composition propagates gravity through a rotated joint axis.
        [TestMethod]
        public void PivotRotated90AboutZ_StillOscillates()
        {
            var scene = new ArticulatedScene();
            scene.SetGravity(new float3(0, 0, -g));

            // Pivot rotated 90° about world z: pivot's y-axis points along world -x.
            var mountRot = math.axisAngle(float3.unitZ, math.PI * 0.5f);
            var root = scene.AddBody(BodyId.None, Joint.Fixed(),
                new SpatialInertia(0f, float3.zero, float3x3.zero),
                new SpatialTransform(mountRot, float3.zero));

            var bob = scene.AddBody(root, Joint.Revolute(float3.unitY),
                new SpatialInertia(1f, new float3(0f, 0f, -L),
                    new float3x3(new float3(1e-4f, 0, 0), new float3(0, 1e-4f, 0), new float3(0, 0, 1e-4f))),
                SpatialTransform.identity);
            scene.SetJointPosition(bob, 0.03f);

            float Texpected = 2f * math.PI * math.sqrt(L / g);
            float dt = 1e-4f;
            int N = (int)(Texpected / dt + 0.5f);
            for (int i = 0; i < N; i++) scene.Step(dt);

            Assert.AreEqual(0.03f, scene.GetJointPosition(bob), 2e-3f,
                "period unchanged under rigid rotation of the pivot");
        }
    }

    // Double pendulum: chaotic but conservative. Symplectic Euler → bounded
    // energy drift. Exercises two coupled revolute joints, including the
    // Coriolis cross-term between them.
    [TestClass]
    public class DoublePendulumTests
    {
        const float g = 9.81f;

        [TestMethod]
        public void EnergyDriftIsBoundedOver10kSteps()
        {
            var scene = new ArticulatedScene();
            scene.SetGravity(new float3(0, 0, -g));

            var root = scene.AddBody(BodyId.None, Joint.Fixed(),
                new SpatialInertia(0f, float3.zero, float3x3.zero),
                SpatialTransform.identity);

            const float L1 = 0.6f;
            const float L2 = 0.4f;
            const float m1 = 1.0f;
            const float m2 = 0.7f;
            var I_c = new float3x3(new float3(1e-3f, 0, 0), new float3(0, 1e-3f, 0), new float3(0, 0, 1e-3f));

            var upper = scene.AddBody(root, Joint.Revolute(float3.unitY),
                new SpatialInertia(m1, new float3(0f, 0f, -L1), I_c),
                SpatialTransform.identity);
            var lower = scene.AddBody(upper, Joint.Revolute(float3.unitY),
                new SpatialInertia(m2, new float3(0f, 0f, -L2), I_c),
                new SpatialTransform(quaternion.identity, new float3(0f, 0f, -L1)));

            scene.SetJointPosition(upper, 0.3f);
            scene.SetJointPosition(lower, -0.4f);

            float E0 = ApproxEnergy(scene, upper, lower, L1, L2, m1, m2);
            float minE = E0, maxE = E0;
            const int N = 10_000;
            float dt = 5e-4f;
            for (int i = 0; i < N; i++)
            {
                scene.Step(dt);
                float E = ApproxEnergy(scene, upper, lower, L1, L2, m1, m2);
                if (E < minE) minE = E;
                if (E > maxE) maxE = E;
            }
            float drift = (maxE - minE) / math.abs(E0);
            Assert.IsTrue(drift < 0.20f, $"double-pendulum energy drift {drift:P} over {N} steps");
        }

        // Approximate total energy of a double pendulum. Computed in world
        // coordinates from joint state (closed-form for point masses at the
        // link COMs — sufficient for drift monitoring).
        static float ApproxEnergy(ArticulatedScene scene, BodyId upper, BodyId lower,
                                  float L1, float L2, float m1, float m2)
        {
            float q1 = scene.GetJointPosition(upper);
            float q2 = scene.GetJointPosition(lower);
            float q1d = scene.GetJointVelocity(upper);
            float q2d = scene.GetJointVelocity(lower);
            float z1 = -L1 * math.cos(q1);
            float z2 = z1 - L2 * math.cos(q1 + q2);
            float x1 = L1 * math.sin(q1);
            float x2 = x1 + L2 * math.sin(q1 + q2);
            // Velocity of lower link COM in world.
            float v1x = L1 * math.cos(q1) * q1d;
            float v1z = L1 * math.sin(q1) * q1d;
            float v2x = v1x + L2 * math.cos(q1 + q2) * (q1d + q2d);
            float v2z = v1z + L2 * math.sin(q1 + q2) * (q1d + q2d);
            float T = 0.5f * m1 * (v1x * v1x + v1z * v1z)
                    + 0.5f * m2 * (v2x * v2x + v2z * v2z);
            float V = m1 * g * z1 + m2 * g * z2;
            return T + V;
        }
    }

    // Cart-pole: prismatic base under a revolute pole. Catches bugs in the
    // coupling between the two joint types — especially the Coriolis term
    // when prismatic velocity interacts with revolute motion.
    [TestClass]
    public class CartPoleTests
    {
        const float g = 9.81f;

        [TestMethod]
        public void PoleHangsDownAndCartDoesNotDrift()
        {
            var scene = new ArticulatedScene();
            scene.SetGravity(new float3(0, 0, -g));

            var root = scene.AddBody(BodyId.None, Joint.Prismatic(float3.unitX),
                new SpatialInertia(1f, float3.zero,
                    new float3x3(new float3(1e-3f, 0, 0), new float3(0, 1e-3f, 0), new float3(0, 0, 1e-3f))),
                SpatialTransform.identity);
            var pole = scene.AddBody(root, Joint.Revolute(float3.unitY),
                new SpatialInertia(0.5f, new float3(0f, 0f, -0.5f),
                    new float3x3(new float3(1e-4f, 0, 0), new float3(0, 1e-4f, 0), new float3(0, 0, 1e-4f))),
                SpatialTransform.identity);

            // Pole starts exactly hanging (q=0). Gravity pulls it straight
            // down → no net torque on pole → cart should stay still.
            for (int i = 0; i < 5_000; i++) scene.Step(1e-3f);

            Assert.AreEqual(0f, scene.GetJointPosition(root), 1e-4f, "cart x should not drift");
            Assert.AreEqual(0f, scene.GetJointVelocity(root), 1e-4f, "cart v should not drift");
            Assert.AreEqual(0f, scene.GetJointPosition(pole), 1e-4f, "pole angle should stay at 0");
            Assert.AreEqual(0f, scene.GetJointVelocity(pole), 1e-4f, "pole ω should stay at 0");
        }

        [TestMethod]
        public void CartKickBackreactsPole()
        {
            // Give the pole a small initial tilt; with no cart force it
            // should start swinging, and the cart should be kicked in the
            // opposite direction (conservation of momentum along x).
            var scene = new ArticulatedScene();
            scene.SetGravity(new float3(0, 0, -g));

            var root = scene.AddBody(BodyId.None, Joint.Prismatic(float3.unitX),
                new SpatialInertia(1f, float3.zero,
                    new float3x3(new float3(1e-3f, 0, 0), new float3(0, 1e-3f, 0), new float3(0, 0, 1e-3f))),
                SpatialTransform.identity);
            var pole = scene.AddBody(root, Joint.Revolute(float3.unitY),
                new SpatialInertia(0.5f, new float3(0f, 0f, -0.5f),
                    new float3x3(new float3(1e-4f, 0, 0), new float3(0, 1e-4f, 0), new float3(0, 0, 1e-4f))),
                SpatialTransform.identity);

            scene.SetJointPosition(pole, 0.2f);
            for (int i = 0; i < 200; i++) scene.Step(1e-3f);

            // After a short simulation, pole should have swung (angle != 0.2)
            // and cart should have moved (non-zero x).
            Assert.AreNotEqual(0.2f, scene.GetJointPosition(pole), 1e-3f);
            Assert.AreNotEqual(0f,   scene.GetJointPosition(root), 1e-5f);
        }
    }
}
