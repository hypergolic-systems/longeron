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
        public struct Scratch
        {
            public SpatialMotion[]     v;
            public SpatialMotion[]     c;
            public SpatialMatrix6[]    IA;
            public SpatialForce[]      pA;
            public SpatialForce[]      U;
            public float[]             D;
            public float[]             u;
            public SpatialMotion[]     a;
            public float[]             qddot;
            public SpatialTransform[]  Xup;    // parent body frame → this body frame

            // Spatial acceleration of the Floating root (6-DOF qddot). Only
            // meaningful when joint[0].kind == Floating. Integrated into
            // rootVelocity each Step.
            public SpatialMotion       rootAccel;

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
                rootAccel = SpatialMotion.zero;
            }
        }

        public static Scratch AllocateScratch(int capacity) => new Scratch(capacity);

        // Run all three passes. Populates scratch.qddot[] with joint accelerations
        // and scratch.rootAccel (when root is Floating). gravity is in world
        // coordinates (parent-of-root frame). Scratch is passed by ref because
        // rootAccel is a value field — pass-by-value would drop the write.
        public static void Solve(ArticulatedBody body, float3 gravity, ref Scratch s)
        {
            int N = body.Count;

            // --- Pass 1: base → tip. Velocity, Coriolis, velocity-dependent bias. ---
            for (int i = 0; i < N; i++)
            {
                Joint j = body.joint[i];

                SpatialTransform Xup;
                if (j.kind == JointKind.Floating)
                {
                    // Floating root: the "joint" transform IS the stored rootPose
                    // (world → body 0). Xtree[0] is ignored.
                    Xup = body.rootPose;
                }
                else
                {
                    Xup = j.JointTransform(body.q[i]) * body.Xtree[i];
                }
                s.Xup[i] = Xup;

                SpatialMotion vParentInBodyFrame = body.parent[i] == -1
                    ? SpatialMotion.zero
                    : Xup.TransformMotion(s.v[body.parent[i]]);

                SpatialMotion vJoint;
                switch (j.kind)
                {
                    case JointKind.Fixed:
                        vJoint = SpatialMotion.zero;
                        s.v[i] = vParentInBodyFrame;
                        s.c[i] = SpatialMotion.zero;
                        break;

                    case JointKind.Floating:
                        // Root has no parent velocity to add; state lives in rootVelocity.
                        // c[0] = v × (S·qdot) = v × v = 0 (motion cross with itself).
                        s.v[i] = body.rootVelocity;
                        s.c[i] = SpatialMotion.zero;
                        vJoint = body.rootVelocity;
                        break;

                    default: // Revolute / Prismatic
                        SpatialMotion S = j.MotionSubspace();
                        vJoint = S * body.qdot[i];
                        s.v[i] = vParentInBodyFrame + vJoint;
                        s.c[i] = SpatialCross.CrossMotion(s.v[i], vJoint);
                        break;
                }

                // Body inertia as 6×6 (needed for rank-1 updates / 6×6 solve).
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
            // Floating root: neither (a) nor (b) — its reduction is the full 6×6
            // solve in Pass 3, and it has no parent.
            for (int i = N - 1; i >= 0; i--)
            {
                Joint j = body.joint[i];
                SpatialMatrix6 IA_a;
                SpatialForce pA_a;

                if (j.kind == JointKind.Fixed || j.kind == JointKind.Floating)
                {
                    // No rank-1 reduction: Fixed has no DOF, Floating's
                    // reduction is 6×6 and handled in Pass 3.
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

                switch (j.kind)
                {
                    case JointKind.Fixed:
                        s.a[i] = aProp;
                        s.qddot[i] = 0f;
                        break;

                    case JointKind.Floating:
                    {
                        // 6-DOF joint reduction: solve IA · qddot = -pA - IA·aProp.
                        // Then a[0] = aProp + qddot = -IA^{-1}·pA.
                        SpatialForce Ia = s.IA[i].Mul(aProp);
                        SpatialForce rhs = -s.pA[i] - Ia;
                        SpatialMotion qddot = s.IA[i].Solve6(rhs);
                        s.rootAccel = qddot;
                        s.a[i] = aProp + qddot;
                        s.qddot[i] = 0f;
                        break;
                    }

                    default: // Revolute / Prismatic
                    {
                        float qddot_i = (s.u[i] - SpatialCross.Dot(s.U[i], aProp)) / s.D[i];
                        s.qddot[i] = qddot_i;
                        SpatialMotion S = j.MotionSubspace();
                        s.a[i] = aProp + S * qddot_i;
                        break;
                    }
                }
            }
        }
    }
}
