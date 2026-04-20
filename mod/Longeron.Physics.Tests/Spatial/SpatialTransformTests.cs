using Microsoft.VisualStudio.TestTools.UnitTesting;
using Longeron.Physics;

namespace Longeron.Physics.Tests
{
    [TestClass]
    public class SpatialTransformTests
    {
        const float Tol = 1e-4f;

        [TestMethod]
        public void IdentityLeavesMotionUnchanged()
        {
            var m = new SpatialMotion(new float3(0.5f, -0.3f, 1.1f), new float3(2f, 1f, -0.7f));
            var result = SpatialTransform.identity.TransformMotion(m);
            float3Tests.AssertClose(m.angular, result.angular, Tol);
            float3Tests.AssertClose(m.linear,  result.linear,  Tol);
        }

        [TestMethod]
        public void IdentityLeavesForceUnchanged()
        {
            var f = new SpatialForce(new float3(0.5f, -0.3f, 1.1f), new float3(2f, 1f, -0.7f));
            var result = SpatialTransform.identity.TransformForce(f);
            float3Tests.AssertClose(f.angular, result.angular, Tol);
            float3Tests.AssertClose(f.linear,  result.linear,  Tol);
        }

        [TestMethod]
        public void InverseRoundTripMotion()
        {
            var X = new SpatialTransform(
                math.axisAngle(math.normalize(new float3(0.3f, 0.7f, -0.5f)), 0.8f),
                new float3(1f, 2f, 3f));
            var m = new SpatialMotion(new float3(0.5f, -0.3f, 1.1f), new float3(2f, 1f, -0.7f));
            var transformed = X.TransformMotion(m);
            var back = X.InverseTransformMotion(transformed);
            float3Tests.AssertClose(m.angular, back.angular, Tol);
            float3Tests.AssertClose(m.linear,  back.linear,  Tol);
        }

        [TestMethod]
        public void InverseRoundTripForce()
        {
            var X = new SpatialTransform(
                math.axisAngle(math.normalize(new float3(1f, -1f, 0.2f)), 1.2f),
                new float3(-0.5f, 1.5f, 2.5f));
            var f = new SpatialForce(new float3(1f, 2f, 3f), new float3(4f, 5f, 6f));
            var back = X.InverseTransformForce(X.TransformForce(f));
            float3Tests.AssertClose(f.angular, back.angular, Tol);
            float3Tests.AssertClose(f.linear,  back.linear,  Tol);
        }

        // (X2 ∘ X1)(m) == X2(X1(m)) for both motion and force.
        [TestMethod]
        public void CompositionAgreesWithSequentialApplication()
        {
            var X1 = new SpatialTransform(
                math.axisAngle(float3.unitZ, 0.7f),
                new float3(1f, 0f, 0f));
            var X2 = new SpatialTransform(
                math.axisAngle(float3.unitX, 0.5f),
                new float3(0f, 2f, 0f));
            var m = new SpatialMotion(new float3(0.3f, -0.5f, 0.7f), new float3(1f, 2f, 3f));
            var composed = (X2 * X1).TransformMotion(m);
            var sequential = X2.TransformMotion(X1.TransformMotion(m));
            float3Tests.AssertClose(sequential.angular, composed.angular, Tol);
            float3Tests.AssertClose(sequential.linear,  composed.linear,  Tol);
        }

        // Duality across a transform: applying the force transform preserves
        // the pairing   f · m.  (Power is frame-invariant.)
        [TestMethod]
        public void ForceMotionPairingIsInvariant()
        {
            var X = new SpatialTransform(
                math.axisAngle(math.normalize(new float3(1f, 2f, -1f)), 0.6f),
                new float3(0.3f, -1f, 0.5f));
            var m = new SpatialMotion(new float3(0.5f, -0.3f, 1.1f), new float3(2f, 1f, -0.7f));
            var f = new SpatialForce(new float3(1f, -1f, 2f), new float3(-0.5f, 0.5f, 0.3f));
            float before = SpatialCross.Dot(f, m);
            float after  = SpatialCross.Dot(X.TransformForce(f), X.TransformMotion(m));
            Assert.AreEqual(before, after, Tol);
        }
    }
}
