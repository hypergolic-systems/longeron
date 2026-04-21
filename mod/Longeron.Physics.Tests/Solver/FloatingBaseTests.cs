using Microsoft.VisualStudio.TestTools.UnitTesting;
using Longeron.Physics;

namespace Longeron.Physics.Tests
{
    // Floating-root validation: a single free rigid body under various
    // conditions. Catches bugs in the 6×6 joint-reduction (Pass 3 Solve6),
    // the SE(3) integrator, and the rootPose <-> GetWorldTransform bridge.
    [TestClass]
    public class FloatingBaseTests
    {
        const float g = 9.81f;

        static ArticulatedScene BuildFloatingBody(float mass, float3x3 I_c)
        {
            var scene = new ArticulatedScene();
            scene.AddBody(BodyId.None, Joint.Floating(),
                new SpatialInertia(mass, float3.zero, I_c),
                SpatialTransform.identity);
            return scene;
        }

        [TestMethod]
        public void SingleBodyFreeFall()
        {
            var I_c = new float3x3(
                new float3(0.1f, 0, 0),
                new float3(0, 0.1f, 0),
                new float3(0, 0, 0.1f));
            var scene = BuildFloatingBody(1f, I_c);
            scene.SetGravity(new float3(0, 0, -g));

            float dt = 1e-3f;
            int steps = 1000;
            for (int i = 0; i < steps; i++) scene.Step(dt);

            var X = scene.GetWorldTransform(new BodyId(0));
            // Semi-implicit Euler: analytical is -4.905; drift ~0.5·g·dt ≈ 5e-3
            // after 1 s with dt = 1e-3.
            Assert.AreEqual(0f,      X.translation.x, 1e-4f);
            Assert.AreEqual(0f,      X.translation.y, 1e-4f);
            Assert.AreEqual(-4.905f, X.translation.z, 0.02f, "z(1s) ≈ -½·g·t²");
        }

        [TestMethod]
        public void RestingBodyStaysAtRestInZeroGravity()
        {
            var I_c = new float3x3(
                new float3(0.5f, 0, 0),
                new float3(0, 0.7f, 0),
                new float3(0, 0, 0.3f));
            var scene = BuildFloatingBody(2f, I_c);
            scene.SetGravity(float3.zero);

            for (int i = 0; i < 10_000; i++) scene.Step(1e-3f);

            var X = scene.GetWorldTransform(new BodyId(0));
            Assert.AreEqual(0f, X.translation.x, 1e-4f);
            Assert.AreEqual(0f, X.translation.y, 1e-4f);
            Assert.AreEqual(0f, X.translation.z, 1e-4f);
            Assert.AreEqual(0f, scene.GetSpatialVelocity(new BodyId(0)).angular.x, 1e-4f);
        }

        // Spinning body in free space with diagonal inertia: angular velocity
        // stays constant, angular momentum conserved exactly. Catches v×Iv bugs.
        [TestMethod]
        public void SpinningDiagonalBodyConservesAngularVelocity()
        {
            // Symmetric (isotropic) inertia so no precession: ω̇ = 0 about any
            // axis. A general test of the Coriolis term not introducing drift.
            float I = 0.2f;
            var I_c = new float3x3(
                new float3(I, 0, 0),
                new float3(0, I, 0),
                new float3(0, 0, I));
            var scene = BuildFloatingBody(1f, I_c);
            scene.SetGravity(float3.zero);

            // Put the root in a spin state before simulating.
            scene.Body.rootVelocity = new SpatialMotion(new float3(0.3f, 0.5f, -0.2f), float3.zero);

            var v0 = scene.Body.rootVelocity;
            for (int i = 0; i < 5_000; i++) scene.Step(1e-3f);
            var vEnd = scene.Body.rootVelocity;

            float3Tests.AssertClose(v0.angular, vEnd.angular, 1e-3f);
            float3Tests.AssertClose(v0.linear,  vEnd.linear,  1e-3f);
        }

        // Tumbling asymmetric body: rotation about the intermediate axis is
        // unstable (tennis racket theorem). Euler's equations predict energy
        // and angular momentum are both conserved; we just verify |L_world|
        // magnitude is preserved over time (the direction changes as the body
        // tumbles but the magnitude in world does not change).
        [TestMethod]
        public void AsymmetricBodyPreservesAngularMomentumMagnitude()
        {
            // Principal inertias I1 > I2 > I3.
            var I_c = new float3x3(
                new float3(3f, 0, 0),
                new float3(0, 2f, 0),
                new float3(0, 0, 1f));
            var scene = BuildFloatingBody(1f, I_c);
            scene.SetGravity(float3.zero);

            // Spin principally about the intermediate (y) axis with a tiny
            // perturbation about z — classic tennis-racket instability setup.
            scene.Body.rootVelocity = new SpatialMotion(new float3(0f, 1f, 0.02f), float3.zero);

            float3 L0 = LBodyToWorld(scene);
            float magL0 = math.length(L0);

            float magMin = magL0, magMax = magL0;
            for (int i = 0; i < 5_000; i++)
            {
                scene.Step(5e-4f);
                float3 L = LBodyToWorld(scene);
                float mag = math.length(L);
                if (mag < magMin) magMin = mag;
                if (mag > magMax) magMax = mag;
            }
            float drift = (magMax - magMin) / magL0;
            Assert.IsTrue(drift < 0.05f, $"|L| drift {drift:P} exceeded 5%");
        }

        static float3 LBodyToWorld(ArticulatedScene scene)
        {
            // Angular momentum in body frame: L_body = I_c · ω  (COM at origin
            // for our test setups). Rotate into world: L_world = q · L_body.
            var body = scene.Body;
            var I_c = body.I[0].I_c;
            var omegaBody = body.rootVelocity.angular;
            float3 LBody = math.mul(I_c, omegaBody);
            return math.mul(body.rootPose.rotation, LBody);
        }
    }
}
