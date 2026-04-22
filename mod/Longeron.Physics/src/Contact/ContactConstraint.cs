// Single-point contact constraint between a body in the articulated system
// and the static world. Produced by the narrowphase, consumed by the
// constraint solver.
//
// Fields:
//   bodyId    — which body this contact is on.
//   point     — world-space contact position (on the static side).
//   normal    — unit direction pointing OUT of the static collider into the
//               dynamic body. Sign convention: a positive normal impulse
//               pushes the dynamic body away from the static one.
//   depth     — penetration depth, ≥ 0 while penetrating. Used for
//               position-level correction (Baumgarte term) in the PGS solver.
//
// Lives in Longeron.Physics.Contact because the solver depends on it; the
// Integration layer populates it but doesn't define it.

namespace Longeron.Physics.Contact
{
    public struct ContactConstraint
    {
        public BodyId bodyId;
        public float3 point;
        public float3 normal;
        public float  depth;
    }
}
