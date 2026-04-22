// One contact point captured from Unity's OnCollisionStay, normalized into
// solver-friendly types. World-frame point + normal; lever arms and body-frame
// forces are computed later by ContactSolver.
//
// `separation` is negative while penetrating (Unity convention); we compute
// penetration depth as max(0, -separation) in the solver.

using Longeron.Physics;

namespace Longeron.Integration
{
    internal struct ContactEntry
    {
        public BodyId bodyId;
        public float3 point;
        public float3 normal;
        public float  separation;
    }
}
