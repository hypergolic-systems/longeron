using Microsoft.VisualStudio.TestTools.UnitTesting;
using Longeron.Physics;
using Longeron.Physics.Contact;

namespace Longeron.Physics.Tests
{
    // Capsule-vs-AABB narrowphase gate. The launchpad test vessel will stand
    // or fall (literally) on how this math behaves at rest and under small
    // perturbation — these tests pin down the invariants before any solver
    // interaction.
    [TestClass]
    public class NarrowphaseTests
    {
        // A 10×10×2 launchpad-shaped AABB centered at origin on the XZ plane,
        // top face at y = 1.
        static readonly AabbShape Pad = new AabbShape(
            new float3(-5f, -1f, -5f),
            new float3( 5f,  1f,  5f));

        const float Radius = 0.5f;

        [TestMethod]
        public void CapsuleWellAboveAabb_NoContact()
        {
            // Vertical capsule, bottom endpoint 2 m above pad top.
            var a = new float3(0f, 3f, 0f);
            var b = new float3(0f, 5f, 0f);

            var mf = new ContactManifold();
            Narrowphase.CapsuleVsAabb(a, b, Radius, Pad, ref mf);

            Assert.AreEqual(0, mf.count);
        }

        [TestMethod]
        public void CapsuleTouchingTopFace_SingleContact()
        {
            // Vertical capsule, bottom endpoint exactly at pad top + radius
            // → just kissing the face, no penetration. External-distance path.
            var a = new float3(0f, 1f + Radius, 0f);
            var b = new float3(0f, 5f, 0f);

            var mf = new ContactManifold();
            Narrowphase.CapsuleVsAabb(a, b, Radius, Pad, ref mf);

            Assert.AreEqual(1, mf.count);
            Assert.AreEqual(1f, mf.point0.y, 1e-4f, "contact on pad top face");
            Assert.AreEqual(1f, mf.normal0.y, 1e-4f, "normal points up");
            Assert.AreEqual(0f, mf.depth0, 1e-3f, "depth ≈ 0 when just touching");
        }

        [TestMethod]
        public void CapsuleEndpointPenetratingTop_FourPointManifold()
        {
            // Vertical capsule, bottom endpoint 0.1 m below pad top — hemisphere
            // is fully embedded. Expect a 4-point manifold (endpoint center +
            // 3 hemisphere samples), all with +Y normal, positive depth.
            var a = new float3(0f, 0.9f, 0f);   // inside AABB (y=0.9 < top=1)
            var b = new float3(0f, 4.9f, 0f);

            var mf = new ContactManifold();
            Narrowphase.CapsuleVsAabb(a, b, Radius, Pad, ref mf);

            Assert.AreEqual(4, mf.count, "primary + 3 hemisphere samples");

            // All normals should point up (endpoint is far from pad side faces,
            // top face is nearest for every sample).
            AssertNormalUp(mf.normal0);
            AssertNormalUp(mf.normal1);
            AssertNormalUp(mf.normal2);
            AssertNormalUp(mf.normal3);

            // Primary contact: endpoint (0, 0.9, 0), nearest face at y=1 →
            // distToFace = 0.1 m, depth = 0.1 + radius = 0.6.
            Assert.AreEqual(0.6f, mf.depth0, 1e-4f, "primary depth = distToFace + radius");
            Assert.AreEqual(1f, mf.point0.y, 1e-4f, "primary contact on top face");

            // Hemisphere samples: capsule axis is +Y, so sample plane is XZ.
            // Samples are at (± r·cos θ, 0.9, ± r·sin θ) — same y as endpoint.
            // Their distToFace is also 0.1 → depth 0.1 (NO radius add, because
            // the sample itself is already on the hemisphere surface).
            Assert.AreEqual(0.1f, mf.depth1, 1e-4f);
            Assert.AreEqual(0.1f, mf.depth2, 1e-4f);
            Assert.AreEqual(0.1f, mf.depth3, 1e-4f);

            // Sample points should be spread in XZ (distinct positions).
            var d12 = mf.point1 - mf.point2;
            Assert.IsTrue(math.length(d12) > 0.1f, "samples are distinct");
        }

        [TestMethod]
        public void CapsuleTiltedInward_AsymmetricDepths()
        {
            // Tilt the capsule 10° so one hemisphere sample sinks deeper and
            // another is shallower — produces the restoring-moment differential
            // the PGS solver needs. Capsule axis rotated about Z by 10°:
            //   bottom endpoint moves slightly in +X
            //   axis direction is (sin10, cos10, 0)
            float tilt = 10f * math.PI / 180f;
            float s = math.sin(tilt), c = math.cos(tilt);
            var a = new float3(0.1f, 0.9f, 0f);          // endpoint still inside
            var b = a + new float3(s, c, 0f) * 4f;

            var mf = new ContactManifold();
            Narrowphase.CapsuleVsAabb(a, b, Radius, Pad, ref mf);

            Assert.AreEqual(4, mf.count);

            // Depths should NOT all be equal — tilt should produce a differential.
            float maxD = math.max(math.max(mf.depth0, mf.depth1), math.max(mf.depth2, mf.depth3));
            float minD = math.min(math.min(mf.depth0, mf.depth1), math.min(mf.depth2, mf.depth3));
            Assert.IsTrue(maxD - minD > 1e-3f,
                $"tilted capsule should have asymmetric sample depths, got spread={maxD - minD:F4}");
        }

        [TestMethod]
        public void CapsuleHorizontalAboveAabb_SingleContact()
        {
            // Horizontal capsule with its body just touching pad top. One
            // external contact; proper line-contact enrichment is deferred.
            var a = new float3(-2f, 1f + Radius * 0.9f, 0f);
            var b = new float3( 2f, 1f + Radius * 0.9f, 0f);

            var mf = new ContactManifold();
            Narrowphase.CapsuleVsAabb(a, b, Radius, Pad, ref mf);

            Assert.AreEqual(1, mf.count, "one external contact (line-contact enrichment is a followup)");
            Assert.AreEqual(1f, mf.normal0.y, 1e-3f, "normal roughly up");
        }

        [TestMethod]
        public void CapsuleFarAway_NoContact()
        {
            // Capsule well clear of AABB on the side.
            var a = new float3(20f, 10f, 20f);
            var b = new float3(20f, 15f, 20f);

            var mf = new ContactManifold();
            Narrowphase.CapsuleVsAabb(a, b, Radius, Pad, ref mf);

            Assert.AreEqual(0, mf.count);
        }

        static void AssertNormalUp(float3 n)
        {
            Assert.AreEqual(0f, n.x, 1e-3f);
            Assert.AreEqual(1f, n.y, 1e-3f);
            Assert.AreEqual(0f, n.z, 1e-3f);
        }
    }
}
