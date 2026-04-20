using Microsoft.VisualStudio.TestTools.UnitTesting;
using Longeron.Physics;

namespace Longeron.Physics.Tests
{
    [TestClass]
    public class float3Tests
    {
        const float Tol = 1e-6f;

        [TestMethod]
        public void Arithmetic()
        {
            var a = new float3(1, 2, 3);
            var b = new float3(4, 5, 6);
            Assert.AreEqual(new float3(5, 7, 9), a + b);
            Assert.AreEqual(new float3(-3, -3, -3), a - b);
            Assert.AreEqual(new float3(2, 4, 6), a * 2f);
            Assert.AreEqual(new float3(2, 4, 6), 2f * a);
            Assert.AreEqual(new float3(0.5f, 1f, 1.5f), a / 2f);
            Assert.AreEqual(new float3(-1, -2, -3), -a);
        }

        [TestMethod]
        public void DotAndCross()
        {
            var a = new float3(1, 0, 0);
            var b = new float3(0, 1, 0);
            Assert.AreEqual(0f, math.dot(a, b), Tol);
            Assert.AreEqual(1f, math.dot(a, a), Tol);
            // x × y = z (right-handed)
            Assert.AreEqual(new float3(0, 0, 1), math.cross(a, b));
            // y × z = x
            Assert.AreEqual(new float3(1, 0, 0), math.cross(b, new float3(0, 0, 1)));
        }

        [TestMethod]
        public void LengthAndNormalize()
        {
            var v = new float3(3, 0, 4);
            Assert.AreEqual(25f, math.lengthsq(v), Tol);
            Assert.AreEqual(5f, math.length(v), Tol);
            var n = math.normalize(v);
            Assert.AreEqual(1f, math.length(n), Tol);
            Assert.AreEqual(new float3(0.6f, 0f, 0.8f), n);
        }

        [TestMethod]
        public void CrossAntiCommutes()
        {
            var a = new float3(1.2f, -3.4f, 5.6f);
            var b = new float3(-0.7f, 2.1f, 0.3f);
            var ab = math.cross(a, b);
            var ba = math.cross(b, a);
            AssertClose(ab, -ba, Tol);
        }

        internal static void AssertClose(float3 expected, float3 actual, float tol)
        {
            Assert.AreEqual(expected.x, actual.x, tol, "x");
            Assert.AreEqual(expected.y, actual.y, tol, "y");
            Assert.AreEqual(expected.z, actual.z, tol, "z");
        }
    }
}
