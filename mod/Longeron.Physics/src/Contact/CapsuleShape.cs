// Capsule collision primitive — body-frame segment endpoints + radius.
//
// Capsule = swept sphere along segment [axisStart, axisEnd]. Natural fit for
// cylindrical parts (fuel tanks, engines, pods) and rounds out the worst-case
// sharp-edge behavior that box primitives would introduce.
//
// Stored in body-local coords; world-space transform happens per tick via
// ArticulatedScene.GetWorldTransform(bodyId).

namespace Longeron.Physics.Contact
{
    public readonly struct CapsuleShape
    {
        public readonly float3 axisStart;
        public readonly float3 axisEnd;
        public readonly float  radius;

        public CapsuleShape(float3 axisStart, float3 axisEnd, float radius)
        {
            this.axisStart = axisStart;
            this.axisEnd = axisEnd;
            this.radius = radius;
        }
    }
}
