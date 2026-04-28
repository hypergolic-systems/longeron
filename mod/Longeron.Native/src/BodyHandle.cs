using System;

namespace Longeron.Native
{
    /// <summary>
    /// Opaque integer body identifier. Stable across the lifetime of a
    /// body within a single <see cref="World"/>. C# never allocates body
    /// IDs directly; the native side returns them on body-create
    /// records.
    /// </summary>
    public readonly struct BodyHandle : IEquatable<BodyHandle>
    {
        public readonly uint Id;

        public BodyHandle(uint id) { Id = id; }

        public static readonly BodyHandle Invalid = new BodyHandle(0);

        public bool IsValid => Id != 0;

        public bool   Equals(BodyHandle other)         => Id == other.Id;
        public override bool Equals(object obj)        => obj is BodyHandle b && Equals(b);
        public override int  GetHashCode()             => (int)Id;
        public override string ToString()              => $"Body({Id})";

        public static bool operator ==(BodyHandle a, BodyHandle b) => a.Id == b.Id;
        public static bool operator !=(BodyHandle a, BodyHandle b) => a.Id != b.Id;
    }
}
