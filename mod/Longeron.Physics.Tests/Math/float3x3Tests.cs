using Microsoft.VisualStudio.TestTools.UnitTesting;
using Longeron.Physics;

namespace Longeron.Physics.Tests
{
    [TestClass]
    public class float3x3Tests
    {
        const float Tol = 1e-5f;

        [TestMethod]
        public void IdentityTimesVectorIsVector()
        {
            var v = new float3(1.1f, 2.2f, 3.3f);
            float3Tests.AssertClose(v, math.mul(float3x3.identity, v), Tol);
        }

        [TestMethod]
        public void SkewCrossAgreesWithVectorCross()
        {
            var a = new float3(1.2f, -3.4f, 5.6f);
            var b = new float3(-0.7f, 2.1f, 0.3f);
            var viaSkew = math.mul(math.skew(a), b);
            var viaCross = math.cross(a, b);
            float3Tests.AssertClose(viaCross, viaSkew, Tol);
        }

        [TestMethod]
        public void TransposeInvolution()
        {
            var m = new float3x3(
                new float3(1, 2, 3),
                new float3(4, 5, 6),
                new float3(7, 8, 9));
            Assert.AreEqual(m, math.transpose(math.transpose(m)));
        }

        [TestMethod]
        public void InverseRoundTrip()
        {
            // Non-singular matrix.
            var m = new float3x3(
                new float3(1, 2, 0),
                new float3(3, 1, 1),
                new float3(0, 2, 1));
            var inv = math.inverse(m);
            var product = math.mul(m, inv);
            AssertMatrixClose(float3x3.identity, product, Tol);
        }

        [TestMethod]
        public void MultiplicationIsAssociative()
        {
            var a = new float3x3(new float3(1, 2, 0), new float3(0, 1, 3), new float3(4, 0, 1));
            var b = new float3x3(new float3(2, 0, 1), new float3(1, 3, 0), new float3(0, 2, 4));
            var c = new float3x3(new float3(0, 1, 2), new float3(3, 1, 0), new float3(1, 0, 2));
            var v = new float3(1.5f, -2.5f, 0.5f);
            var via1 = math.mul(math.mul(a, b), math.mul(c, float3x3.identity));
            var via2 = math.mul(a, math.mul(b, c));
            float3Tests.AssertClose(math.mul(via1, v), math.mul(via2, v), Tol);
        }

        internal static void AssertMatrixClose(float3x3 expected, float3x3 actual, float tol)
        {
            float3Tests.AssertClose(expected.c0, actual.c0, tol);
            float3Tests.AssertClose(expected.c1, actual.c1, tol);
            float3Tests.AssertClose(expected.c2, actual.c2, tol);
        }
    }
}
