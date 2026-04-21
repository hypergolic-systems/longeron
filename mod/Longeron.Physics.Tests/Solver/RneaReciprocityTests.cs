using Microsoft.VisualStudio.TestTools.UnitTesting;
using Longeron.Physics;

namespace Longeron.Physics.Tests
{
    // The correctness gate: a random tree, random (q, qdot, fExt) state,
    // run ABA to get qddot (which assumes τ = 0), then run RNEA on that
    // qddot. RNEA should recover τ ≈ 0 across every joint, to float
    // precision. This cross-validates ABA against an independent inverse
    // dynamics algorithm — if any sign, frame, or Coriolis term is wrong
    // in either algorithm, residual will explode.
    [TestClass]
    public class RneaReciprocityTests
    {
        const float Tol = 1e-3f;        // float precision; mild numerical slack
        const int Trials = 50;          // number of random trees

        [TestMethod]
        public void AbaRneaResidualIsZeroOnRandomTrees()
        {
            var rng = new System.Random(20260420);
            for (int trial = 0; trial < Trials; trial++)
            {
                var body = BuildRandomTree(rng, minBodies: 3, maxBodies: 12);
                ApplyRandomState(body, rng);

                var abaScratch  = ABA.AllocateScratch(body.Count);
                var rneaScratch = Rnea.AllocateScratch(body.Count);

                var gravity = new float3(
                    (float)(rng.NextDouble() * 2 - 1),
                    (float)(rng.NextDouble() * 2 - 1),
                    (float)(rng.NextDouble() * 2 - 1));

                ABA.Solve(body, gravity, ref abaScratch);
                Rnea.Solve(body, gravity, abaScratch.qddot, rneaScratch, abaScratch.rootAccel);

                for (int i = 0; i < body.Count; i++)
                {
                    if (body.joint[i].kind == JointKind.Fixed) continue;
                    Assert.AreEqual(0f, rneaScratch.tau[i], Tol,
                        $"trial {trial} body {i} ({body.joint[i].kind}): ABA→RNEA residual too large");
                }
            }
        }

        // --- helpers ---

        static ArticulatedBody BuildRandomTree(System.Random rng, int minBodies, int maxBodies)
        {
            int N = rng.Next(minBodies, maxBodies + 1);
            var body = new ArticulatedBody(N);

            for (int i = 0; i < N; i++)
            {
                BodyId parent = i == 0 ? BodyId.None : new BodyId(rng.Next(0, i));
                Joint j = i == 0 ? RandomRootJoint(rng) : RandomInternalJoint(rng);

                float m = 0.1f + (float)rng.NextDouble() * 5f;
                float3 com = RandomFloat3(rng, 0.3f);
                float iDiag = 0.05f + (float)rng.NextDouble() * 0.5f;
                var inertia = new SpatialInertia(m, com,
                    new float3x3(new float3(iDiag, 0, 0), new float3(0, iDiag, 0), new float3(0, 0, iDiag)));

                var axis = math.normalize(RandomFloat3(rng, 1f) + float3.unitX * 0.01f);
                var rot = math.axisAngle(axis, (float)(rng.NextDouble() * 2f - 1f));
                var trans = RandomFloat3(rng, 0.8f);
                var xtree = new SpatialTransform(rot, trans);

                body.AddBody(parent, j, inertia, xtree);
            }
            body.Validate();
            return body;
        }

        // Root can be any of the four kinds — including Floating, which is only
        // supported at the root.
        static Joint RandomRootJoint(System.Random rng)
        {
            int pick = rng.Next(0, 4);
            switch (pick)
            {
                case 0: return Joint.Fixed();
                case 1: return Joint.Floating();
                default:
                    var axis = math.normalize(RandomFloat3(rng, 1f) + float3.unitX * 0.01f);
                    return pick == 2 ? Joint.Revolute(axis) : Joint.Prismatic(axis);
            }
        }

        static Joint RandomInternalJoint(System.Random rng)
        {
            int pick = rng.Next(0, 3);
            if (pick == 0) return Joint.Fixed();
            var axis = math.normalize(RandomFloat3(rng, 1f) + float3.unitX * 0.01f);
            return pick == 1 ? Joint.Revolute(axis) : Joint.Prismatic(axis);
        }

        static void ApplyRandomState(ArticulatedBody body, System.Random rng)
        {
            // For Floating root, populate rootPose + rootVelocity.
            if (body.joint[0].kind == JointKind.Floating)
            {
                var axis = math.normalize(RandomFloat3(rng, 1f) + float3.unitX * 0.01f);
                body.rootPose = new SpatialTransform(
                    math.axisAngle(axis, (float)(rng.NextDouble() * 2f - 1f)),
                    RandomFloat3(rng, 1f));
                body.rootVelocity = new SpatialMotion(RandomFloat3(rng, 0.5f), RandomFloat3(rng, 0.5f));
            }

            for (int i = 0; i < body.Count; i++)
            {
                var kind = body.joint[i].kind;
                if (kind == JointKind.Revolute || kind == JointKind.Prismatic)
                {
                    body.q[i]    = (float)(rng.NextDouble() * 2 - 1);
                    body.qdot[i] = (float)(rng.NextDouble() * 2 - 1);
                }
                body.fExt[i] = new SpatialForce(RandomFloat3(rng, 2f), RandomFloat3(rng, 5f));
            }
        }

        static float3 RandomFloat3(System.Random rng, float scale) => new float3(
            (float)(rng.NextDouble() * 2 - 1) * scale,
            (float)(rng.NextDouble() * 2 - 1) * scale,
            (float)(rng.NextDouble() * 2 - 1) * scale);
    }
}
