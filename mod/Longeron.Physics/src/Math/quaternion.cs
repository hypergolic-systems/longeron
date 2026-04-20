// Unit quaternion for rotations. Stored as (x, y, z, w) with w being the
// scalar part — matches Unity.Mathematics layout.
//
// Unit-norm is *not* enforced by the type; callers must normalize
// accumulated rotations periodically (integrators drift).

using System;

namespace Longeron.Physics
{
    public readonly struct quaternion : IEquatable<quaternion>
    {
        public readonly float x, y, z, w;

        public quaternion(float x, float y, float z, float w)
        {
            this.x = x; this.y = y; this.z = z; this.w = w;
        }

        public static readonly quaternion identity = new quaternion(0f, 0f, 0f, 1f);

        // Hamilton product: a * b represents "apply b, then a".
        public static quaternion operator *(quaternion a, quaternion b) => new quaternion(
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);

        public bool Equals(quaternion o) => x == o.x && y == o.y && z == o.z && w == o.w;
        public override bool Equals(object o) => o is quaternion q && Equals(q);
        public override int GetHashCode() =>
            (x.GetHashCode() * 397) ^ (y.GetHashCode() * 17) ^ (z.GetHashCode() * 31) ^ w.GetHashCode();
        public static bool operator ==(quaternion a, quaternion b) => a.Equals(b);
        public static bool operator !=(quaternion a, quaternion b) => !a.Equals(b);

        public override string ToString() => $"quaternion({x}, {y}, {z}, {w})";
    }
}
