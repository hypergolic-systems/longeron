// Featherstone Articulated Body Algorithm — forward dynamics.
//
// FRAME CONVENTION — Featherstone RBDA (2008), link-fixed body frame.
//
// Three passes:
//
//   Pass 1 (base → tip): propagate velocity through joints, compute
//     Coriolis term c[i] and velocity-dependent bias p[i].
//
//   Pass 2 (tip → base): accumulate articulated-body inertia IA[i] and
//     bias force pA[i] up the tree. For 1-DOF joints this includes the
//     rank-1 "joint reduction" that eliminates the child's joint DOF
//     before the quantities cross into the parent's frame.
//
//   Pass 3 (base → tip): compute joint accelerations qddot[i] and body
//     spatial accelerations a[i]. For Fixed joints, a[i] just follows
//     the propagated parent acceleration plus c[i] (no DOF to solve).
//
// The root's "parent acceleration" in Pass 3 is set to (0, -g) in world
// coords — Featherstone's gravity trick, avoiding per-body gravity
// wrenches. Valid for fixed-root setups (this PoC).

namespace Longeron.Physics
{
    public static class ABA
    {
        public readonly struct Scratch
        {
            public readonly SpatialMotion[]     v;
            public readonly SpatialMotion[]     c;
            public readonly SpatialMatrix6[]    IA;
            public readonly SpatialForce[]      pA;
            public readonly SpatialForce[]      U;
            public readonly float[]             D;
            public readonly float[]             u;
            public readonly SpatialMotion[]     a;
            public readonly float[]             qddot;
            public readonly SpatialTransform[]  Xup;    // parent body frame → this body frame

            public Scratch(int capacity)
            {
                v     = new SpatialMotion[capacity];
                c     = new SpatialMotion[capacity];
                IA    = new SpatialMatrix6[capacity];
                pA    = new SpatialForce[capacity];
                U     = new SpatialForce[capacity];
                D     = new float[capacity];
                u     = new float[capacity];
                a     = new SpatialMotion[capacity];
                qddot = new float[capacity];
                Xup   = new SpatialTransform[capacity];
            }
        }

        public static Scratch AllocateScratch(int capacity) => new Scratch(capacity);

        // Run all three passes. Populates scratch.qddot[] with joint accelerations.
        // gravity is in world coordinates (parent-of-root frame).
        public static void Solve(ArticulatedBody body, float3 gravity, Scratch s)
        {
            int N = body.Count;

            // --- Pass 1: base → tip. Velocity, Coriolis, velocity-dependent bias. ---
            for (int i = 0; i < N; i++)
            {
                Joint j = body.joint[i];
                SpatialTransform Xj = j.JointTransform(body.q[i]);
                SpatialTransform Xup = Xj * body.Xtree[i];
                s.Xup[i] = Xup;

                SpatialMotion vParentInBodyFrame = body.parent[i] == -1
                    ? SpatialMotion.zero
                    : Xup.TransformMotion(s.v[body.parent[i]]);

                SpatialMotion vJoint;
                if (j.kind == JointKind.Fixed)
                {
                    vJoint = SpatialMotion.zero;
                    s.v[i] = vParentInBodyFrame;
                    s.c[i] = SpatialMotion.zero;
                }
                else
                {
                    SpatialMotion S = j.MotionSubspace();
                    vJoint = S * body.qdot[i];
                    s.v[i] = vParentInBodyFrame + vJoint;
                    // c[i] = v[i] × (S·qdot)  (spatial motion cross product)
                    s.c[i] = SpatialCross.CrossMotion(s.v[i], vJoint);
                }

                // Body inertia as 6×6 (needed for rank-1 updates in pass 2).
                s.IA[i] = SpatialMatrix6.FromInertia(body.I[i]);

                // Velocity-dependent bias: v ×* (I·v) minus external wrench.
                SpatialForce Iv = body.I[i].Mul(s.v[i]);
                s.pA[i] = SpatialCross.CrossForceDual(s.v[i], Iv) - body.fExt[i];
            }

            // --- Pass 2: tip → base. Articulate inertia and bias into parents. ---
            //
            // Per body:
            //   (a) if 1-DOF, compute joint reduction scalars U[i], D[i], u[i]
            //       (needed in Pass 3 whether or not we accumulate into a parent);
            //   (b) if has parent, transform the reduced IA/pA into parent's frame
            //       and accumulate.
            // The root executes (a) but skips (b).
            for (int i = N - 1; i >= 0; i--)
            {
                Joint j = body.joint[i];
                SpatialMatrix6 IA_a;
                SpatialForce pA_a;

                if (j.kind == JointKind.Fixed)
                {
                    // No DOF — propagate the articulated quantities rigidly.
                    IA_a = s.IA[i];
                    pA_a = s.pA[i];
                }
                else
                {
                    SpatialMotion S = j.MotionSubspace();
                    SpatialForce U_i = s.IA[i].Mul(S);
                    float D_i = SpatialCross.Dot(U_i, S);
                    // Unactuated joint: τ_i = 0, so u_i = -S^T·pA[i].
                    float u_i = -SpatialCross.Dot(s.pA[i], S);
                    s.U[i] = U_i;
                    s.D[i] = D_i;
                    s.u[i] = u_i;

                    // IA_a = IA[i] - (1/D_i) U_i U_i^T
                    IA_a = s.IA[i].SubRankOne(U_i, D_i);
                    // pA_a = pA[i] + IA_a·c[i] + U_i·(u_i / D_i)
                    pA_a = s.pA[i] + IA_a.Mul(s.c[i]) + U_i * (u_i / D_i);
                }

                int p = body.parent[i];
                if (p == -1) continue;   // root has no parent to accumulate into
                s.IA[p] = s.IA[p] + IA_a.InverseTransform(s.Xup[i]);
                s.pA[p] = s.pA[p] + s.Xup[i].InverseTransformForce(pA_a);
            }

            // --- Pass 3: base → tip. Compute joint and body accelerations. ---
            SpatialMotion aWorld = new SpatialMotion(float3.zero, -gravity);   // Featherstone's gravity trick
            for (int i = 0; i < N; i++)
            {
                Joint j = body.joint[i];
                SpatialMotion aParent = body.parent[i] == -1
                    ? aWorld
                    : s.a[body.parent[i]];

                SpatialMotion aProp = s.Xup[i].TransformMotion(aParent) + s.c[i];

                if (j.kind == JointKind.Fixed)
                {
                    s.a[i] = aProp;
                    s.qddot[i] = 0f;
                }
                else
                {
                    // qddot_i = (u_i - U_i^T · aProp) / D_i
                    float qddot_i = (s.u[i] - SpatialCross.Dot(s.U[i], aProp)) / s.D[i];
                    s.qddot[i] = qddot_i;
                    SpatialMotion S = j.MotionSubspace();
                    s.a[i] = aProp + S * qddot_i;
                }
            }
        }
    }
}
