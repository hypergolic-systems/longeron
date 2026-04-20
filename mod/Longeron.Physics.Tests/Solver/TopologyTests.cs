using Microsoft.VisualStudio.TestTools.UnitTesting;
using Longeron.Physics;

namespace Longeron.Physics.Tests
{
    [TestClass]
    public class TopologyTests
    {
        static readonly SpatialInertia unitInertia =
            new SpatialInertia(1f, float3.zero, float3x3.identity);

        [TestMethod]
        public void RootHasParentNone()
        {
            var body = new ArticulatedBody();
            var root = body.AddBody(BodyId.None, Joint.Fixed(), unitInertia, SpatialTransform.identity);
            Assert.AreEqual(0, root.index);
            Assert.AreEqual(-1, body.parent[root.index]);
            Assert.AreEqual(1, body.Count);
            body.Validate();
        }

        [TestMethod]
        public void ChildrenMustComeAfterParents()
        {
            var body = new ArticulatedBody();
            var root = body.AddBody(BodyId.None, Joint.Fixed(), unitInertia, SpatialTransform.identity);
            var c1 = body.AddBody(root, Joint.Revolute(float3.unitZ), unitInertia, SpatialTransform.identity);
            var c2 = body.AddBody(c1, Joint.Prismatic(float3.unitX), unitInertia, SpatialTransform.identity);
            Assert.AreEqual(3, body.Count);
            Assert.IsTrue(body.parent[c1.index] < c1.index);
            Assert.IsTrue(body.parent[c2.index] < c2.index);
            body.Validate();
        }

        [TestMethod]
        [ExpectedException(typeof(System.InvalidOperationException))]
        public void ForwardReferenceRejected()
        {
            var body = new ArticulatedBody();
            body.AddBody(new BodyId(5), Joint.Fixed(), unitInertia, SpatialTransform.identity);
        }

        [TestMethod]
        [ExpectedException(typeof(System.InvalidOperationException))]
        public void SelfParentRejected()
        {
            var body = new ArticulatedBody();
            // Adding body 0 whose parent is also 0 violates parent < index.
            body.AddBody(new BodyId(0), Joint.Fixed(), unitInertia, SpatialTransform.identity);
        }

        [TestMethod]
        public void CapacityGrowsOnInsert()
        {
            var body = new ArticulatedBody(initialCapacity: 2);
            var root = body.AddBody(BodyId.None, Joint.Fixed(), unitInertia, SpatialTransform.identity);
            var prev = root;
            for (int i = 0; i < 20; i++)
                prev = body.AddBody(prev, Joint.Fixed(), unitInertia, SpatialTransform.identity);
            Assert.AreEqual(21, body.Count);
            Assert.IsTrue(body.Capacity >= 21);
            body.Validate();
        }
    }

    [TestClass]
    public class JointTests
    {
        const float Tol = 1e-5f;

        [TestMethod]
        public void FixedJointHasZeroDof()
        {
            var j = Joint.Fixed();
            Assert.AreEqual(0, j.Dof);
            Assert.AreEqual(SpatialMotion.zero.angular, j.MotionSubspace().angular);
            Assert.AreEqual(SpatialMotion.zero.linear,  j.MotionSubspace().linear);
        }

        [TestMethod]
        public void RevoluteMotionSubspaceIsAngularAxis()
        {
            var j = Joint.Revolute(float3.unitZ);
            Assert.AreEqual(1, j.Dof);
            Assert.AreEqual(float3.unitZ, j.MotionSubspace().angular);
            Assert.AreEqual(float3.zero,  j.MotionSubspace().linear);
        }

        [TestMethod]
        public void PrismaticMotionSubspaceIsLinearAxis()
        {
            var j = Joint.Prismatic(float3.unitY);
            Assert.AreEqual(1, j.Dof);
            Assert.AreEqual(float3.zero,  j.MotionSubspace().angular);
            Assert.AreEqual(float3.unitY, j.MotionSubspace().linear);
        }

        [TestMethod]
        public void FixedJointTransformIsIdentity()
        {
            var X = Joint.Fixed().JointTransform(1.234f);
            Assert.AreEqual(quaternion.identity, X.rotation);
            Assert.AreEqual(float3.zero, X.translation);
        }

        [TestMethod]
        public void RevoluteJointTransformRotatesAboutAxis()
        {
            var j = Joint.Revolute(float3.unitZ);
            var X = j.JointTransform(math.PI * 0.5f);
            // Rotation about z by 90° maps x to y.
            float3Tests.AssertClose(float3.unitY, math.mul(X.rotation, float3.unitX), Tol);
            Assert.AreEqual(float3.zero, X.translation);
        }

        [TestMethod]
        public void PrismaticJointTransformTranslatesAlongAxis()
        {
            var j = Joint.Prismatic(float3.unitX);
            var X = j.JointTransform(2.5f);
            Assert.AreEqual(quaternion.identity, X.rotation);
            Assert.AreEqual(new float3(2.5f, 0f, 0f), X.translation);
        }
    }
}
