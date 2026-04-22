// Contact manifold — up to 4 contact points per pair. Fixed-size to stay
// blittable (Burst-friendly, no heap allocation per query). The narrowphase
// fills mf.count with the number of valid entries.
//
// Each contact:
//   point  — world-space contact position on the static side (ON the AABB
//            surface for our one pair type).
//   normal — unit world-space direction, pointing OUT of the static collider
//            into the dynamic one. Sign convention matches our penalty-phase
//            ContactEntry for drop-in replacement downstream.
//   depth  — penetration depth ≥ 0. Zero means "just touching" — the PGS
//            constraint will still pin velocity, just with no position
//            correction bias. Negative depths are not contacts.
//
// Why 4 points: a vertical capsule-end pressed into a flat face produces a
// disc-shaped real-world contact patch. Four sampled points around the disc
// form a proper support polygon — enough to resist tipping torque without
// hitting pathological "too many redundant points" degeneracy in the PGS
// solver.

namespace Longeron.Physics.Contact
{
    public struct ContactManifold
    {
        public int count;

        public float3 point0, normal0; public float depth0;
        public float3 point1, normal1; public float depth1;
        public float3 point2, normal2; public float depth2;
        public float3 point3, normal3; public float depth3;

        public const int Capacity = 4;

        public void Clear() => count = 0;

        public bool Add(float3 point, float3 normal, float depth)
        {
            switch (count)
            {
                case 0: point0 = point; normal0 = normal; depth0 = depth; count = 1; return true;
                case 1: point1 = point; normal1 = normal; depth1 = depth; count = 2; return true;
                case 2: point2 = point; normal2 = normal; depth2 = depth; count = 3; return true;
                case 3: point3 = point; normal3 = normal; depth3 = depth; count = 4; return true;
                default: return false;
            }
        }

        public void Get(int i, out float3 point, out float3 normal, out float depth)
        {
            switch (i)
            {
                case 0: point = point0; normal = normal0; depth = depth0; return;
                case 1: point = point1; normal = normal1; depth = depth1; return;
                case 2: point = point2; normal = normal2; depth = depth2; return;
                case 3: point = point3; normal = normal3; depth = depth3; return;
                default: point = float3.zero; normal = float3.zero; depth = 0f; return;
            }
        }
    }
}
