// FRAME CONVENTION — Featherstone RBDA (2008), link-fixed body frame.
//
// Compact rigid-body spatial inertia in its own body frame. Stored as:
//   mass m (scalar)
//   com  c (float3) — position of center of mass, in the body frame origin's coords
//   I_c  (float3x3) — rotational inertia tensor about the COM, expressed in the body frame basis
//
// The equivalent 6×6 spatial inertia (with body frame origin as reference point):
//   I_sp = [[ I_c - m·ĉ²,   m·ĉ    ],
//           [ -m·ĉ,          m·I_3  ]]
//   where ĉ = skew(c).
//
// Never materialized; we operate on the compact form directly.

namespace Longeron.Physics
{
    public readonly struct SpatialInertia
    {
        public readonly float mass;
        public readonly float3 com;         // COM in body frame
        public readonly float3x3 I_c;       // rotational inertia about COM, in body basis

        public SpatialInertia(float mass, float3 com, float3x3 I_c)
        {
            this.mass = mass;
            this.com = com;
            this.I_c = I_c;
        }

        public static readonly SpatialInertia zero =
            new SpatialInertia(0f, float3.zero, float3x3.zero);

        // Apply spatial inertia to a motion vector, producing spatial momentum (force units).
        //
        //   p (linear)  = m·(v - c×ω)
        //   n (angular) = I_c·ω + c × p
        public SpatialForce Mul(SpatialMotion m)
        {
            float3 cxo = math.cross(com, m.angular);      // c × ω
            float3 p   = mass * (m.linear - cxo);         // linear momentum
            float3 n   = math.mul(I_c, m.angular) + math.cross(com, p);
            return new SpatialForce(n, p);
        }

        // Transform this inertia from frame A into frame B via X_A→B = (q, t).
        //   c_B = q⁻¹ · (c - t)
        //   I_c_B = E · I_c · E^T   where E = R(q)^T = R(q⁻¹) is the passive
        //                           rotation matrix for basis change.
        public SpatialInertia Transform(SpatialTransform X)
        {
            quaternion qInv = math.inverse(X.rotation);
            float3 comB = math.mul(qInv, com - X.translation);
            float3x3 E = math.toMatrix(qInv);
            float3x3 I_cB = math.mul(math.mul(E, I_c), math.transpose(E));
            return new SpatialInertia(mass, comB, I_cB);
        }

        // Inverse: given inertia expressed in B, recover it in A.
        //   c_A = q · c + t
        //   I_c_A = E^T · I_c · E   where E = R(q⁻¹).
        public SpatialInertia InverseTransform(SpatialTransform X)
        {
            quaternion qInv = math.inverse(X.rotation);
            float3 comA = math.mul(X.rotation, com) + X.translation;
            float3x3 E = math.toMatrix(qInv);
            float3x3 Et = math.transpose(E);
            float3x3 I_cA = math.mul(math.mul(Et, I_c), E);
            return new SpatialInertia(mass, comA, I_cA);
        }

        // Sum two inertias expressed in the same frame.
        // Combined COM is the mass-weighted average; combined I_c uses the
        // parallel-axis theorem twice (shift each body's I_c to the new COM).
        public static SpatialInertia operator +(SpatialInertia a, SpatialInertia b)
        {
            float mSum = a.mass + b.mass;
            if (mSum <= 0f) return zero;
            float3 cSum = (a.mass * a.com + b.mass * b.com) * (1f / mSum);
            float3 da = a.com - cSum;
            float3 db = b.com - cSum;
            // Parallel-axis shift: I_about_new = I_com + m · (|d|²·I - d·d^T)
            float3x3 I_sum = a.I_c + ParallelShift(a.mass, da) + b.I_c + ParallelShift(b.mass, db);
            return new SpatialInertia(mSum, cSum, I_sum);
        }

        private static float3x3 ParallelShift(float m, float3 d)
        {
            // m · (|d|²·I - d·d^T) = -m · skew(d)²
            float3x3 s = math.skew(d);
            float3x3 s2 = math.mul(s, s);
            return (-m) * s2;
        }
    }
}
