// Axis-aligned bounding box in world coordinates — the static-world primitive
// for Longeron's demo-scope contact geometry. One AabbShape for the launchpad
// is enough to make a rocket stand; terrain / buildings / cross-vessel pairs
// are deliberately out of scope at this stage.

namespace Longeron.Physics.Contact
{
    public readonly struct AabbShape
    {
        public readonly float3 min;
        public readonly float3 max;

        public AabbShape(float3 min, float3 max)
        {
            this.min = min;
            this.max = max;
        }

        public float3 Center => (min + max) * 0.5f;
        public float3 Extents => (max - min) * 0.5f;

        public bool Contains(float3 p) =>
            p.x >= min.x && p.x <= max.x &&
            p.y >= min.y && p.y <= max.y &&
            p.z >= min.z && p.z <= max.z;

        // Closest point on the AABB surface or interior to p. If p is inside,
        // returns p unchanged (no projection to surface). For a surface-only
        // projection, use ClosestExteriorPoint.
        public float3 ClosestPoint(float3 p)
        {
            float x = p.x < min.x ? min.x : p.x > max.x ? max.x : p.x;
            float y = p.y < min.y ? min.y : p.y > max.y ? max.y : p.y;
            float z = p.z < min.z ? min.z : p.z > max.z ? max.z : p.z;
            return new float3(x, y, z);
        }
    }
}
