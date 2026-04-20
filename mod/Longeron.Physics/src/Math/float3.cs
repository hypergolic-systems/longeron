// 3D float vector. Matches Unity.Mathematics shape (lowercase struct name,
// free functions on `math` class) so migration to KSPBurst-hosted
// Unity.Mathematics is a namespace swap, not a port.

using System;

namespace Longeron.Physics
{
    public readonly struct float3 : IEquatable<float3>
    {
        public readonly float x, y, z;

        public float3(float x, float y, float z) { this.x = x; this.y = y; this.z = z; }
        public float3(float s) { x = s; y = s; z = s; }

        public static readonly float3 zero  = new float3(0f, 0f, 0f);
        public static readonly float3 one   = new float3(1f, 1f, 1f);
        public static readonly float3 unitX = new float3(1f, 0f, 0f);
        public static readonly float3 unitY = new float3(0f, 1f, 0f);
        public static readonly float3 unitZ = new float3(0f, 0f, 1f);

        public float this[int i] => i == 0 ? x : i == 1 ? y : z;

        public static float3 operator +(float3 a, float3 b) => new float3(a.x + b.x, a.y + b.y, a.z + b.z);
        public static float3 operator -(float3 a, float3 b) => new float3(a.x - b.x, a.y - b.y, a.z - b.z);
        public static float3 operator -(float3 a)           => new float3(-a.x, -a.y, -a.z);
        public static float3 operator *(float3 a, float s)  => new float3(a.x * s, a.y * s, a.z * s);
        public static float3 operator *(float s, float3 a)  => new float3(a.x * s, a.y * s, a.z * s);
        public static float3 operator *(float3 a, float3 b) => new float3(a.x * b.x, a.y * b.y, a.z * b.z);
        public static float3 operator /(float3 a, float s)  => new float3(a.x / s, a.y / s, a.z / s);

        public bool Equals(float3 o) => x == o.x && y == o.y && z == o.z;
        public override bool Equals(object o) => o is float3 v && Equals(v);
        public override int GetHashCode() => (x.GetHashCode() * 397) ^ (y.GetHashCode() * 17) ^ z.GetHashCode();
        public static bool operator ==(float3 a, float3 b) => a.Equals(b);
        public static bool operator !=(float3 a, float3 b) => !a.Equals(b);

        public override string ToString() => $"float3({x}, {y}, {z})";
    }
}
