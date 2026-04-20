// Articulated body — SoA storage for a tree of rigid bodies.
//
// FRAME CONVENTION — Featherstone RBDA (2008), link-fixed body frame.
//
// Storage invariant: bodies are stored in topological order, i.e.,
// `parent[i] < i` for every body (parent = -1 marks the root's parent;
// the root body is index 0). This invariant is asserted on AddBody and
// verified by Validate(); the entire solver relies on it.
//
// Plain T[] arrays for now. Wrapping in a NativeArray-shaped view for
// Burst migration is mechanical and deferred.

using System;

namespace Longeron.Physics
{
    public readonly struct BodyId
    {
        public readonly int index;
        public BodyId(int index) { this.index = index; }
        public static readonly BodyId None = new BodyId(-1);
        public static bool operator ==(BodyId a, BodyId b) => a.index == b.index;
        public static bool operator !=(BodyId a, BodyId b) => a.index != b.index;
        public override bool Equals(object o) => o is BodyId b && b.index == index;
        public override int GetHashCode() => index;
    }

    public sealed class ArticulatedBody
    {
        public int Count { get; private set; }
        public int Capacity { get; private set; }

        // Topology + constants.
        public int[] parent;
        public Joint[] joint;
        public SpatialTransform[] Xtree;
        public SpatialInertia[] I;

        // Configuration + inputs (1 DOF max per joint for this PoC).
        public float[] q;
        public float[] qdot;
        public SpatialForce[] fExt;

        public ArticulatedBody(int initialCapacity = 16)
        {
            Capacity = initialCapacity < 1 ? 1 : initialCapacity;
            parent = new int[Capacity];
            joint  = new Joint[Capacity];
            Xtree  = new SpatialTransform[Capacity];
            I      = new SpatialInertia[Capacity];
            q      = new float[Capacity];
            qdot   = new float[Capacity];
            fExt   = new SpatialForce[Capacity];
            Count  = 0;
        }

        public BodyId AddBody(BodyId parentId, Joint j, SpatialInertia inertia, SpatialTransform xtree)
        {
            int parentIdx = parentId.index;
            int idx = Count;
            if (parentIdx >= idx)
                throw new InvalidOperationException(
                    $"parent index ({parentIdx}) must be less than new body index ({idx}); topological order violated");
            if (parentIdx < -1)
                throw new InvalidOperationException($"invalid parent index {parentIdx}");

            EnsureCapacity(idx + 1);
            parent[idx] = parentIdx;
            joint[idx]  = j;
            Xtree[idx]  = xtree;
            I[idx]      = inertia;
            q[idx]      = 0f;
            qdot[idx]   = 0f;
            fExt[idx]   = SpatialForce.zero;
            Count = idx + 1;
            return new BodyId(idx);
        }

        public void SetExternalWrench(BodyId b, SpatialForce f) => fExt[b.index] = f;

        public void SetJointPosition(BodyId b, float value) => q[b.index] = value;
        public void SetJointVelocity(BodyId b, float value) => qdot[b.index] = value;
        public float GetJointPosition(BodyId b)  => q[b.index];
        public float GetJointVelocity(BodyId b)  => qdot[b.index];

        // Scans every stored body and re-verifies the topological-order
        // invariant plus basic tree structure (exactly one root, no cycles
        // possible given parent[i] < i).
        public void Validate()
        {
            int roots = 0;
            for (int i = 0; i < Count; i++)
            {
                int p = parent[i];
                if (p >= i)
                    throw new InvalidOperationException(
                        $"body {i} has parent {p}; expected parent < i");
                if (p < -1)
                    throw new InvalidOperationException(
                        $"body {i} has invalid parent {p}");
                if (p == -1) roots++;
            }
            if (Count > 0 && roots != 1)
                throw new InvalidOperationException(
                    $"expected exactly one root (parent = -1); found {roots}");
        }

        void EnsureCapacity(int needed)
        {
            if (needed <= Capacity) return;
            int newCap = Capacity;
            while (newCap < needed) newCap *= 2;
            Array.Resize(ref parent, newCap);
            Array.Resize(ref joint,  newCap);
            Array.Resize(ref Xtree,  newCap);
            Array.Resize(ref I,      newCap);
            Array.Resize(ref q,      newCap);
            Array.Resize(ref qdot,   newCap);
            Array.Resize(ref fExt,   newCap);
            Capacity = newCap;
        }
    }
}
