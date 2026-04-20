using Microsoft.VisualStudio.TestTools.UnitTesting;
using Longeron.Physics;

namespace Longeron.Physics.Tests
{
    // THE duality gate. If this fails, every downstream ABA test is
    // meaningless — the cross products are wrong, and the Coriolis /
    // bias-force computations are silently polluted.
    [TestClass]
    public class SpatialCrossTests
    {
        const float Tol = 1e-5f;

        [TestMethod]
        public void ZeroCrossMotion()
        {
            var v = new SpatialMotion(new float3(1, 2, 3), new float3(4, 5, 6));
            var result = SpatialCross.CrossMotion(SpatialMotion.zero, v);
            Assert.AreEqual(float3.zero, result.angular);
            Assert.AreEqual(float3.zero, result.linear);
        }

        [TestMethod]
        public void ZeroCrossForceDual()
        {
            var f = new SpatialForce(new float3(1, 2, 3), new float3(4, 5, 6));
            var result = SpatialCross.CrossForceDual(SpatialMotion.zero, f);
            Assert.AreEqual(float3.zero, result.angular);
            Assert.AreEqual(float3.zero, result.linear);
        }

        // Featherstone duality: (v ×* f) · m + f · (v × m) = 0
        // for any motion v, force f, motion m. Rules out all sign/ordering
        // errors in the cross products simultaneously.
        [TestMethod]
        public void DualityHoldsForRandomVectors()
        {
            var rng = new System.Random(42);
            for (int trial = 0; trial < 50; trial++)
            {
                var v = new SpatialMotion(RandFloat3(rng), RandFloat3(rng));
                var f = new SpatialForce(RandFloat3(rng), RandFloat3(rng));
                var m = new SpatialMotion(RandFloat3(rng), RandFloat3(rng));

                float lhs = SpatialCross.Dot(SpatialCross.CrossForceDual(v, f), m);
                float rhs = SpatialCross.Dot(f, SpatialCross.CrossMotion(v, m));
                Assert.AreEqual(0f, lhs + rhs, Tol,
                    $"duality residual on trial {trial}: lhs={lhs}, rhs={rhs}");
            }
        }

        // Motion × motion is anti-symmetric: v × m = -(m × v).
        [TestMethod]
        public void CrossMotionAntiSymmetric()
        {
            var rng = new System.Random(7);
            for (int trial = 0; trial < 20; trial++)
            {
                var v = new SpatialMotion(RandFloat3(rng), RandFloat3(rng));
                var m = new SpatialMotion(RandFloat3(rng), RandFloat3(rng));
                var vxm = SpatialCross.CrossMotion(v, m);
                var mxv = SpatialCross.CrossMotion(m, v);
                float3Tests.AssertClose(vxm.angular, -mxv.angular, Tol);
                float3Tests.AssertClose(vxm.linear,  -mxv.linear,  Tol);
            }
        }

        static float3 RandFloat3(System.Random rng) => new float3(
            (float)(rng.NextDouble() * 2 - 1),
            (float)(rng.NextDouble() * 2 - 1),
            (float)(rng.NextDouble() * 2 - 1));
    }
}
