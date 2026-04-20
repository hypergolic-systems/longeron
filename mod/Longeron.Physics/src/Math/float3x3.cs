// 3x3 float matrix, column-major storage (matches Unity.Mathematics).
//
// Columns c0, c1, c2 are float3 vectors. Element (row r, col c) = c[c][r].

using System;

namespace Longeron.Physics
{
    public readonly struct float3x3 : IEquatable<float3x3>
    {
        public readonly float3 c0, c1, c2;

        public float3x3(float3 c0, float3 c1, float3 c2) { this.c0 = c0; this.c1 = c1; this.c2 = c2; }

        public float3x3(float m00, float m01, float m02,
                        float m10, float m11, float m12,
                        float m20, float m21, float m22)
        {
            c0 = new float3(m00, m10, m20);
            c1 = new float3(m01, m11, m21);
            c2 = new float3(m02, m12, m22);
        }

        public static readonly float3x3 zero = new float3x3(float3.zero, float3.zero, float3.zero);
        public static readonly float3x3 identity =
            new float3x3(float3.unitX, float3.unitY, float3.unitZ);

        // M[row, col]
        public float this[int row, int col]
        {
            get
            {
                float3 c = col == 0 ? c0 : col == 1 ? c1 : c2;
                return c[row];
            }
        }

        public static float3x3 operator +(float3x3 a, float3x3 b) =>
            new float3x3(a.c0 + b.c0, a.c1 + b.c1, a.c2 + b.c2);
        public static float3x3 operator -(float3x3 a, float3x3 b) =>
            new float3x3(a.c0 - b.c0, a.c1 - b.c1, a.c2 - b.c2);
        public static float3x3 operator -(float3x3 a) =>
            new float3x3(-a.c0, -a.c1, -a.c2);
        public static float3x3 operator *(float3x3 a, float s) =>
            new float3x3(a.c0 * s, a.c1 * s, a.c2 * s);
        public static float3x3 operator *(float s, float3x3 a) => a * s;

        public bool Equals(float3x3 o) => c0.Equals(o.c0) && c1.Equals(o.c1) && c2.Equals(o.c2);
        public override bool Equals(object o) => o is float3x3 m && Equals(m);
        public override int GetHashCode() => (c0.GetHashCode() * 397) ^ (c1.GetHashCode() * 17) ^ c2.GetHashCode();
        public static bool operator ==(float3x3 a, float3x3 b) => a.Equals(b);
        public static bool operator !=(float3x3 a, float3x3 b) => !a.Equals(b);

        public override string ToString() =>
            $"float3x3(({c0.x}, {c1.x}, {c2.x}) ({c0.y}, {c1.y}, {c2.y}) ({c0.z}, {c1.z}, {c2.z}))";
    }
}
