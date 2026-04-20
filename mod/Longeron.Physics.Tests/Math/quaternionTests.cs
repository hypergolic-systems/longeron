using Microsoft.VisualStudio.TestTools.UnitTesting;
using Longeron.Physics;

namespace Longeron.Physics.Tests
{
    [TestClass]
    public class quaternionTests
    {
        const float Tol = 1e-5f;

        [TestMethod]
        public void IdentityRotatesToItself()
        {
            var v = new float3(1.2f, -3.4f, 5.6f);
            float3Tests.AssertClose(v, math.mul(quaternion.identity, v), Tol);
        }

        [TestMethod]
        public void AxisAngle90DegreesAboutZ()
        {
            var q = math.axisAngle(float3.unitZ, math.PI * 0.5f);
            // x axis rotates to y.
            float3Tests.AssertClose(float3.unitY, math.mul(q, float3.unitX), Tol);
            // y axis rotates to -x.
            float3Tests.AssertClose(-float3.unitX, math.mul(q, float3.unitY), Tol);
            // z axis stays fixed.
            float3Tests.AssertClose(float3.unitZ, math.mul(q, float3.unitZ), Tol);
        }

        [TestMethod]
        public void CompositionIsApplyBThenA()
        {
            var qx = math.axisAngle(float3.unitX, math.PI * 0.5f);
            var qy = math.axisAngle(float3.unitY, math.PI * 0.5f);
            // Apply qy to v, then qx: expect composed quaternion qx*qy to match.
            var v = new float3(1f, 2f, 3f);
            var sequential = math.mul(qx, math.mul(qy, v));
            var composed = math.mul(math.mul(qx, qy), v);
            float3Tests.AssertClose(sequential, composed, Tol);
        }

        [TestMethod]
        public void InverseRoundTrip()
        {
            var q = math.axisAngle(math.normalize(new float3(1, 2, 3)), 1.2345f);
            var v = new float3(0.3f, -0.7f, 1.1f);
            var roundTrip = math.mul(math.inverse(q), math.mul(q, v));
            float3Tests.AssertClose(v, roundTrip, Tol);
        }

        [TestMethod]
        public void ToMatrixAgreesWithDirectRotation()
        {
            var q = math.axisAngle(math.normalize(new float3(0.3f, 0.7f, -0.5f)), 0.8f);
            var v = new float3(1.1f, -0.4f, 2.3f);
            var viaQuat = math.mul(q, v);
            var viaMatrix = math.mul(math.toMatrix(q), v);
            float3Tests.AssertClose(viaQuat, viaMatrix, Tol);
        }

        [TestMethod]
        public void UnitQuaternionMatrixIsOrthonormal()
        {
            var q = math.axisAngle(math.normalize(new float3(1, 1, 1)), 0.7f);
            var R = math.toMatrix(q);
            // R^T R = I for orthonormal rotations.
            var product = math.mul(math.transpose(R), R);
            float3x3Tests.AssertMatrixClose(float3x3.identity, product, Tol);
        }
    }
}
