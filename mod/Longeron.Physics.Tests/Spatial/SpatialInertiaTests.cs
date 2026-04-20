using Microsoft.VisualStudio.TestTools.UnitTesting;
using Longeron.Physics;

namespace Longeron.Physics.Tests
{
    [TestClass]
    public class SpatialInertiaTests
    {
        const float Tol = 1e-4f;

        [TestMethod]
        public void PointMassAtOriginProducesPureLinearMomentum()
        {
            // Point mass (I_c = 0) at origin, pure translation: p = m·v, n = 0.
            var I = new SpatialInertia(2f, float3.zero, float3x3.zero);
            var m = new SpatialMotion(float3.zero, new float3(3f, 4f, 5f));
            var h = I.Mul(m);
            float3Tests.AssertClose(float3.zero,              h.angular, Tol);
            float3Tests.AssertClose(new float3(6f, 8f, 10f),  h.linear,  Tol);
        }

        [TestMethod]
        public void PureRotationProducesICOmega()
        {
            // Rotational inertia about origin, body at origin (COM = 0):
            // n = I_c · ω, p = 0.
            var I_c = new float3x3(
                new float3(2f, 0f, 0f),
                new float3(0f, 3f, 0f),
                new float3(0f, 0f, 4f));
            var I = new SpatialInertia(1f, float3.zero, I_c);
            var m = new SpatialMotion(new float3(1f, 1f, 1f), float3.zero);
            var h = I.Mul(m);
            float3Tests.AssertClose(new float3(2f, 3f, 4f), h.angular, Tol);
            float3Tests.AssertClose(float3.zero,            h.linear,  Tol);
        }

        [TestMethod]
        public void OffsetCOMCouplesAngularAndLinear()
        {
            // Body with mass offset from body frame origin: pure translation
            // at the origin corresponds to angular momentum about the origin.
            // For ω=0, v=(1,0,0), COM at (0,1,0), m=1:
            //   p = m·v = (1,0,0)
            //   n = I_c·ω + c × p = 0 + (0,1,0)×(1,0,0) = (0,0,-1)
            var I = new SpatialInertia(1f, new float3(0f, 1f, 0f), float3x3.zero);
            var m = new SpatialMotion(float3.zero, new float3(1f, 0f, 0f));
            var h = I.Mul(m);
            float3Tests.AssertClose(new float3(0f, 0f, -1f), h.angular, Tol);
            float3Tests.AssertClose(new float3(1f, 0f, 0f),  h.linear,  Tol);
        }

        [TestMethod]
        public void TransformInverseRoundTrip()
        {
            var I_c = new float3x3(
                new float3(2.5f, 0.1f, 0f),
                new float3(0.1f, 3f, 0.2f),
                new float3(0f, 0.2f, 1.5f));
            var I = new SpatialInertia(4f, new float3(0.3f, -0.4f, 0.5f), I_c);
            var X = new SpatialTransform(
                math.axisAngle(math.normalize(new float3(1f, 2f, -1f)), 0.9f),
                new float3(0.7f, -1.2f, 0.5f));
            var round = I.Transform(X).InverseTransform(X);
            Assert.AreEqual(I.mass, round.mass, Tol);
            float3Tests.AssertClose(I.com, round.com, Tol);
            float3x3Tests.AssertMatrixClose(I.I_c, round.I_c, Tol);
        }

        // Power invariance through an inertia: (I·m) · m is frame-invariant.
        // (I·m)·m is twice the kinetic energy; it should not depend on frame.
        [TestMethod]
        public void KineticEnergyFrameInvariant()
        {
            var I = new SpatialInertia(2f, new float3(0.3f, 0f, 0.1f),
                new float3x3(new float3(1.5f, 0, 0), new float3(0, 2f, 0), new float3(0, 0, 2.5f)));
            var m = new SpatialMotion(new float3(0.4f, -0.3f, 0.2f), new float3(1f, 0.5f, -0.7f));
            var X = new SpatialTransform(
                math.axisAngle(math.normalize(new float3(0.5f, 1f, 0.3f)), 0.7f),
                new float3(0.2f, -0.3f, 0.5f));
            float ke_A = SpatialCross.Dot(I.Mul(m), m);
            var I_B = I.Transform(X);
            var m_B = X.TransformMotion(m);
            float ke_B = SpatialCross.Dot(I_B.Mul(m_B), m_B);
            Assert.AreEqual(ke_A, ke_B, 1e-3f);
        }

        // Adding two inertias at the same location should increase mass linearly.
        [TestMethod]
        public void AdditionIsMassConservative()
        {
            var I_a = new SpatialInertia(3f, new float3(1f, 0f, 0f), float3x3.identity);
            var I_b = new SpatialInertia(2f, new float3(-1f, 0f, 0f), float3x3.identity);
            var sum = I_a + I_b;
            Assert.AreEqual(5f, sum.mass, Tol);
            // Weighted COM: (3·1 + 2·(-1))/5 = 1/5 = 0.2
            float3Tests.AssertClose(new float3(0.2f, 0f, 0f), sum.com, Tol);
        }
    }
}
