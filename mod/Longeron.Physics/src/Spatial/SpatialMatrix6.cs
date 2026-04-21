// FRAME CONVENTION — Featherstone RBDA (2008), link-fixed body frame.
//
// General 6×6 symmetric spatial matrix, stored as four 3×3 blocks:
//
//   M = [[a, b],
//        [c, d]]
//
// Rigid-body spatial inertia has b = -c^T, a and d symmetric; ABA rank-1
// updates preserve symmetry but break the rigid-body pattern, so we need
// the general form for IA[] during pass 2. Body inertia I[] stays in the
// compact SpatialInertia form.

namespace Longeron.Physics
{
    public readonly struct SpatialMatrix6
    {
        public readonly float3x3 a, b, c, d;

        public SpatialMatrix6(float3x3 a, float3x3 b, float3x3 c, float3x3 d)
        {
            this.a = a; this.b = b; this.c = c; this.d = d;
        }

        public static readonly SpatialMatrix6 zero = new SpatialMatrix6(
            float3x3.zero, float3x3.zero, float3x3.zero, float3x3.zero);

        // Construct from a rigid-body inertia (m, c_com, I_c):
        //   M = [[I_c - m·ĉ²,   m·ĉ   ],
        //        [-m·ĉ,          m·I_3 ]]
        public static SpatialMatrix6 FromInertia(SpatialInertia I)
        {
            float3x3 ch = math.skew(I.com);
            float3x3 ch2 = math.mul(ch, ch);
            float3x3 top = I.I_c + (-I.mass) * ch2;          // I_c - m·ĉ²
            float3x3 off = I.mass * ch;                       // m·ĉ
            float3x3 diag = I.mass * float3x3.identity;       // m·I
            return new SpatialMatrix6(top, off, -off, diag);
        }

        // M · motion  = force.  Block-wise:
        //   n = a·ω + b·v
        //   f = c·ω + d·v
        public SpatialForce Mul(SpatialMotion m)
        {
            float3 n = math.mul(a, m.angular) + math.mul(b, m.linear);
            float3 f = math.mul(c, m.angular) + math.mul(d, m.linear);
            return new SpatialForce(n, f);
        }

        // Rank-1 update:  M - (1/s) · u u^T  where u is a spatial force.
        // Used in ABA pass 2 for the joint-reduction step.
        public SpatialMatrix6 SubRankOne(SpatialForce u, float s)
        {
            float inv = 1f / s;
            float3x3 newA = a + (-inv) * Outer(u.angular, u.angular);
            float3x3 newB = b + (-inv) * Outer(u.angular, u.linear);
            float3x3 newC = c + (-inv) * Outer(u.linear,  u.angular);
            float3x3 newD = d + (-inv) * Outer(u.linear,  u.linear);
            return new SpatialMatrix6(newA, newB, newC, newD);
        }

        // Outer product u·v^T as a 3×3 matrix (column-major: column j = u·v_j).
        static float3x3 Outer(float3 u, float3 v) =>
            new float3x3(u * v.x, u * v.y, u * v.z);

        public static SpatialMatrix6 operator +(SpatialMatrix6 x, SpatialMatrix6 y) =>
            new SpatialMatrix6(x.a + y.a, x.b + y.b, x.c + y.c, x.d + y.d);

        // Transform a spatial matrix from frame A to frame B given X_A→B = (q, p).
        //   M_B = X_f · M_A · X_m^{-1}
        // with X_f = [[E, -E·p̂], [0, E]], X_m^{-1} = [[E^T, 0], [p̂·E^T, E^T]],
        // where E = R(q⁻¹) is the *passive* rotation matrix for basis change.
        public SpatialMatrix6 Transform(SpatialTransform X)
        {
            float3x3 E  = math.toMatrix(math.inverse(X.rotation));
            float3x3 Et = math.transpose(E);
            float3x3 ph = math.skew(X.translation);
            float3x3 Eph = math.mul(E, ph);
            float3x3 phEt = math.mul(ph, Et);

            // temp = M_A · X_m^{-1}
            float3x3 t_aa = math.mul(a, Et) + math.mul(b, phEt);
            float3x3 t_ab = math.mul(b, Et);
            float3x3 t_ba = math.mul(c, Et) + math.mul(d, phEt);
            float3x3 t_bb = math.mul(d, Et);

            // result = X_f · temp
            float3x3 r_aa = math.mul(E, t_aa) + (-1f) * math.mul(Eph, t_ba);
            float3x3 r_ab = math.mul(E, t_ab) + (-1f) * math.mul(Eph, t_bb);
            float3x3 r_ba = math.mul(E, t_ba);
            float3x3 r_bb = math.mul(E, t_bb);

            return new SpatialMatrix6(r_aa, r_ab, r_ba, r_bb);
        }

        // Inverse-transform: if M is in B, apply inverse of X_A→B to get M in A.
        public SpatialMatrix6 InverseTransform(SpatialTransform X) =>
            Transform(X.Inverse());

        // Solve M · x = rhs for x, where M is treated as 6×6 with the angular
        // block indexing 0..2 and linear block indexing 3..5 (matching
        // SpatialMotion / SpatialForce field layout).
        //
        // Gaussian elimination with partial pivoting on a flat row-major
        // augmented matrix. Allocates two small arrays per call — acceptable
        // at the rate this fires (once per Floating root per tick). Can be
        // stackalloc'd later for Burst.
        public SpatialMotion Solve6(SpatialForce rhs)
        {
            var A = new float[42]; // 6 rows × (6 + 1) cols

            // Fill 6×6 M (block form → flat row-major).
            for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                A[i * 7 + j]             = a[i, j];
                A[i * 7 + (j + 3)]       = b[i, j];
                A[(i + 3) * 7 + j]       = c[i, j];
                A[(i + 3) * 7 + (j + 3)] = d[i, j];
            }

            // rhs column: angular then linear.
            A[0 * 7 + 6] = rhs.angular.x;
            A[1 * 7 + 6] = rhs.angular.y;
            A[2 * 7 + 6] = rhs.angular.z;
            A[3 * 7 + 6] = rhs.linear.x;
            A[4 * 7 + 6] = rhs.linear.y;
            A[5 * 7 + 6] = rhs.linear.z;

            for (int col = 0; col < 6; col++)
            {
                // Partial-pivot: find the row with max |A[row, col]| in [col, 6).
                int pivot = col;
                float maxAbs = math.abs(A[col * 7 + col]);
                for (int row = col + 1; row < 6; row++)
                {
                    float val = math.abs(A[row * 7 + col]);
                    if (val > maxAbs) { maxAbs = val; pivot = row; }
                }
                if (pivot != col)
                {
                    for (int k = col; k < 7; k++)
                    {
                        float tmp = A[col * 7 + k];
                        A[col * 7 + k] = A[pivot * 7 + k];
                        A[pivot * 7 + k] = tmp;
                    }
                }

                float pivotVal = A[col * 7 + col];
                for (int row = col + 1; row < 6; row++)
                {
                    float factor = A[row * 7 + col] / pivotVal;
                    for (int k = col; k < 7; k++)
                        A[row * 7 + k] -= factor * A[col * 7 + k];
                }
            }

            // Back-substitute.
            var x = new float[6];
            for (int i = 5; i >= 0; i--)
            {
                float sum = A[i * 7 + 6];
                for (int j = i + 1; j < 6; j++) sum -= A[i * 7 + j] * x[j];
                x[i] = sum / A[i * 7 + i];
            }

            return new SpatialMotion(
                new float3(x[0], x[1], x[2]),
                new float3(x[3], x[4], x[5]));
        }
    }
}
