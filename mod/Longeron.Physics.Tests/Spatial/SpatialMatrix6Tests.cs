using Microsoft.VisualStudio.TestTools.UnitTesting;
using Longeron.Physics;

namespace Longeron.Physics.Tests
{
    [TestClass]
    public class SpatialMatrix6Tests
    {
        const float Tol = 1e-4f;

        // Round-trip: M · (M.Solve6(f)) == f for a non-singular rigid-body
        // inertia. Validates the 6×6 Gaussian elimination against the forward
        // Mul.
        [TestMethod]
        public void SolveRoundTripOnRigidBodyInertia()
        {
            var rng = new System.Random(101);
            for (int trial = 0; trial < 20; trial++)
            {
                float m = 0.2f + (float)rng.NextDouble() * 5f;
                float3 com = new float3(
                    (float)(rng.NextDouble() * 2 - 1) * 0.3f,
                    (float)(rng.NextDouble() * 2 - 1) * 0.3f,
                    (float)(rng.NextDouble() * 2 - 1) * 0.3f);
                float d = 0.1f + (float)rng.NextDouble();
                var I = new SpatialInertia(m, com,
                    new float3x3(new float3(d, 0, 0), new float3(0, d * 1.3f, 0), new float3(0, 0, d * 0.7f)));
                var M = SpatialMatrix6.FromInertia(I);

                var f = new SpatialForce(
                    new float3(
                        (float)(rng.NextDouble() * 2 - 1) * 3f,
                        (float)(rng.NextDouble() * 2 - 1) * 3f,
                        (float)(rng.NextDouble() * 2 - 1) * 3f),
                    new float3(
                        (float)(rng.NextDouble() * 2 - 1) * 5f,
                        (float)(rng.NextDouble() * 2 - 1) * 5f,
                        (float)(rng.NextDouble() * 2 - 1) * 5f));

                var x = M.Solve6(f);
                var fPrime = M.Mul(x);

                float3Tests.AssertClose(f.angular, fPrime.angular, Tol);
                float3Tests.AssertClose(f.linear,  fPrime.linear,  Tol);
            }
        }

        [TestMethod]
        public void SolveIdentityReturnsForceAsMotion()
        {
            // Build an "inertia" with mass 1, no COM offset, identity I_c.
            // Resulting spatial matrix = [[I, 0], [0, I]], so M.Solve6((a,b))
            // should yield SpatialMotion(a, b).
            var I = new SpatialInertia(1f, float3.zero, float3x3.identity);
            var M = SpatialMatrix6.FromInertia(I);

            var f = new SpatialForce(new float3(1, 2, 3), new float3(4, 5, 6));
            var x = M.Solve6(f);
            float3Tests.AssertClose(new float3(1, 2, 3), x.angular, Tol);
            float3Tests.AssertClose(new float3(4, 5, 6), x.linear,  Tol);
        }
    }
}
