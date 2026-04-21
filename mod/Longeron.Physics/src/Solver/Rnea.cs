// Recursive Newton-Euler Algorithm — inverse dynamics.
//
// FRAME CONVENTION — Featherstone RBDA (2008), link-fixed body frame.
//
// Given (q, qdot, qddot) and external wrenches, compute the joint torque
// required at each 1-DOF joint to produce the given motion. Uses the same
// spatial algebra, frame conventions, and gravity trick as ABA.
//
// Two passes:
//
//   Pass 1 (base → tip): compute v[i], a[i], and net body wrench
//     f[i] = I[i]·a[i] + v[i] ×* (I[i]·v[i]) - fExt[i].
//
//   Pass 2 (tip → base): tau[i] = S[i]^T · f[i] (1-DOF), then transform
//     the remaining wrench into the parent frame and add it to f[parent].
//
// This implementation exists primarily to back the RneaReciprocityTests —
// an independent cross-check on ABA. ABA assumes τ_i = 0 and computes
// qddot; RNEA takes that qddot and recovers τ — which should come back at
// ~0 for an unactuated system.

namespace Longeron.Physics
{
    public static class Rnea
    {
        public readonly struct Scratch
        {
            public readonly SpatialMotion[]    v;
            public readonly SpatialMotion[]    a;
            public readonly SpatialForce[]     f;
            public readonly SpatialTransform[] Xup;
            public readonly float[]            tau;

            public Scratch(int capacity)
            {
                v   = new SpatialMotion[capacity];
                a   = new SpatialMotion[capacity];
                f   = new SpatialForce[capacity];
                Xup = new SpatialTransform[capacity];
                tau = new float[capacity];
            }
        }

        public static Scratch AllocateScratch(int capacity) => new Scratch(capacity);

        // Populate s.tau[] given the body's current (q, qdot) and supplied
        // joint accelerations. `qddot` carries scalars per 1-DOF joint (0 for
        // Fixed); `rootAccel` is the spatial acceleration of a Floating root
        // (ignored if root is not Floating).
        public static void Solve(
            ArticulatedBody body,
            float3 gravity,
            float[] qddot,
            Scratch s,
            SpatialMotion rootAccel = default)
        {
            int N = body.Count;

            // Pass 1: base → tip. Base acceleration encodes gravity (gravity trick).
            SpatialMotion aWorld = new SpatialMotion(float3.zero, -gravity);

            for (int i = 0; i < N; i++)
            {
                Joint j = body.joint[i];

                if (j.kind == JointKind.Floating)
                {
                    s.Xup[i] = body.rootPose;
                }
                else
                {
                    s.Xup[i] = j.JointTransform(body.q[i]) * body.Xtree[i];
                }

                SpatialMotion vParent = body.parent[i] == -1
                    ? SpatialMotion.zero
                    : s.v[body.parent[i]];
                SpatialMotion aParent = body.parent[i] == -1
                    ? aWorld
                    : s.a[body.parent[i]];

                SpatialMotion vParentInBody = s.Xup[i].TransformMotion(vParent);
                SpatialMotion aParentInBody = s.Xup[i].TransformMotion(aParent);

                switch (j.kind)
                {
                    case JointKind.Fixed:
                        s.v[i] = vParentInBody;
                        s.a[i] = aParentInBody;
                        break;

                    case JointKind.Floating:
                        // Root: v comes from stored rootVelocity; a is supplied
                        // rootAccel plus aParent (world aWorld transformed in);
                        // c (Coriolis) is 0 for v × v.
                        s.v[i] = body.rootVelocity;
                        s.a[i] = aParentInBody + rootAccel;
                        break;

                    default:
                    {
                        SpatialMotion S = j.MotionSubspace();
                        SpatialMotion vJ = S * body.qdot[i];
                        s.v[i] = vParentInBody + vJ;
                        s.a[i] = aParentInBody + S * qddot[i] + SpatialCross.CrossMotion(s.v[i], vJ);
                        break;
                    }
                }

                SpatialForce Iv = body.I[i].Mul(s.v[i]);
                SpatialForce Ia = body.I[i].Mul(s.a[i]);
                s.f[i] = Ia + SpatialCross.CrossForceDual(s.v[i], Iv) - body.fExt[i];
            }

            // Pass 2: tip → base. Extract joint torques and propagate wrench up.
            for (int i = N - 1; i >= 0; i--)
            {
                Joint j = body.joint[i];
                switch (j.kind)
                {
                    case JointKind.Fixed:
                    case JointKind.Floating:
                        // Fixed: no DOF. Floating: 6-vector residual wrench; we
                        // don't expose it per tau[] — the reciprocity test
                        // asserts 1-DOF joints only. Leave tau[i] at 0.
                        s.tau[i] = 0f;
                        break;
                    default:
                        s.tau[i] = SpatialCross.Dot(s.f[i], j.MotionSubspace());
                        break;
                }

                if (body.parent[i] != -1)
                    s.f[body.parent[i]] = s.f[body.parent[i]] + s.Xup[i].InverseTransformForce(s.f[i]);
            }
        }
    }
}
