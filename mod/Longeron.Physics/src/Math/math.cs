// Free-function math helpers. Matches Unity.Mathematics naming
// (`math.cross`, `math.dot`, `math.mul`, etc.) so migration later is a
// namespace swap.

using System;

namespace Longeron.Physics
{
    public static class math
    {
        public const float PI = 3.14159265358979323846f;
        public const float EPSILON = 1e-6f;

        // --- scalar ---

        public static float abs(float a)   => a < 0f ? -a : a;
        public static float min(float a, float b) => a < b ? a : b;
        public static float max(float a, float b) => a > b ? a : b;
        public static float sqrt(float a)  => (float)Math.Sqrt(a);
        public static float sin(float a)   => (float)Math.Sin(a);
        public static float cos(float a)   => (float)Math.Cos(a);
        public static float atan2(float y, float x) => (float)Math.Atan2(y, x);
        public static float asin(float a)  => (float)Math.Asin(a);
        public static float acos(float a)  => (float)Math.Acos(a);

        // --- float3 ---

        public static float dot(float3 a, float3 b)    => a.x * b.x + a.y * b.y + a.z * b.z;
        public static float3 cross(float3 a, float3 b) => new float3(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x);
        public static float lengthsq(float3 v) => dot(v, v);
        public static float length(float3 v)   => sqrt(dot(v, v));
        public static float3 normalize(float3 v)
        {
            float len = length(v);
            return len > 0f ? v / len : float3.zero;
        }
        public static float3 normalizesafe(float3 v, float3 fallback)
        {
            float lsq = lengthsq(v);
            return lsq > EPSILON * EPSILON ? v / sqrt(lsq) : fallback;
        }

        // --- float3x3 ---

        public static float3 mul(float3x3 m, float3 v) =>
            m.c0 * v.x + m.c1 * v.y + m.c2 * v.z;

        public static float3x3 mul(float3x3 a, float3x3 b) => new float3x3(
            mul(a, b.c0), mul(a, b.c1), mul(a, b.c2));

        public static float3x3 transpose(float3x3 m) => new float3x3(
            new float3(m.c0.x, m.c1.x, m.c2.x),
            new float3(m.c0.y, m.c1.y, m.c2.y),
            new float3(m.c0.z, m.c1.z, m.c2.z));

        // Cross-product matrix (skew-symmetric). math.mul(skew(a), b) == math.cross(a, b).
        public static float3x3 skew(float3 v) => new float3x3(
            new float3( 0f,  v.z, -v.y),
            new float3(-v.z, 0f,   v.x),
            new float3( v.y, -v.x, 0f));

        public static float3x3 inverse(float3x3 m)
        {
            float3 a = m.c0, b = m.c1, c = m.c2;
            float3 bxc = cross(b, c);
            float3 cxa = cross(c, a);
            float3 axb = cross(a, b);
            float det = dot(a, bxc);
            float invDet = 1f / det;
            return new float3x3(
                new float3(bxc.x, cxa.x, axb.x) * invDet,
                new float3(bxc.y, cxa.y, axb.y) * invDet,
                new float3(bxc.z, cxa.z, axb.z) * invDet);
        }

        public static float determinant(float3x3 m) => dot(m.c0, cross(m.c1, m.c2));

        // --- quaternion ---

        public static quaternion mul(quaternion a, quaternion b) => a * b;

        public static quaternion axisAngle(float3 axis, float angle)
        {
            float3 n = normalize(axis);
            float half = angle * 0.5f;
            float s = sin(half);
            return new quaternion(n.x * s, n.y * s, n.z * s, cos(half));
        }

        public static quaternion conjugate(quaternion q) => new quaternion(-q.x, -q.y, -q.z, q.w);

        // Inverse of a unit quaternion is the conjugate. We don't normalize
        // here; callers ensure unit norm via `normalize(q)` when needed.
        public static quaternion inverse(quaternion q) => conjugate(q);

        public static float lengthsq(quaternion q) => q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        public static float length(quaternion q)   => sqrt(lengthsq(q));
        public static quaternion normalize(quaternion q)
        {
            float len = length(q);
            return len > 0f
                ? new quaternion(q.x / len, q.y / len, q.z / len, q.w / len)
                : quaternion.identity;
        }

        // Rotate a vector v by quaternion q. Derived from q * v̂ * q⁻¹ reduced
        // to a closed form (Rodrigues-like). Unit quaternion assumed.
        public static float3 mul(quaternion q, float3 v)
        {
            float3 qv = new float3(q.x, q.y, q.z);
            float3 t  = 2f * cross(qv, v);
            return v + q.w * t + cross(qv, t);
        }

        // Rotation matrix from a unit quaternion.
        public static float3x3 toMatrix(quaternion q)
        {
            float x = q.x, y = q.y, z = q.z, w = q.w;
            float xx = x * x, yy = y * y, zz = z * z;
            float xy = x * y, xz = x * z, yz = y * z;
            float wx = w * x, wy = w * y, wz = w * z;
            return new float3x3(
                new float3(1f - 2f * (yy + zz), 2f * (xy + wz),       2f * (xz - wy)),
                new float3(2f * (xy - wz),      1f - 2f * (xx + zz),  2f * (yz + wx)),
                new float3(2f * (xz + wy),      2f * (yz - wx),       1f - 2f * (xx + yy)));
        }
    }
}
