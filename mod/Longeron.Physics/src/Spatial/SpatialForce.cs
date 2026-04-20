// FRAME CONVENTION — Featherstone RBDA (2008), link-fixed body frame.
//
// Spatial force vector ordering: (angular, linear) = (n_O, f),
// where n_O is the moment about the *origin of the body frame* and f is
// the linear force. Angular first, matching SpatialMotion.

namespace Longeron.Physics
{
    public readonly struct SpatialForce
    {
        public readonly float3 angular;   // moment n about body origin
        public readonly float3 linear;    // force f

        public SpatialForce(float3 angular, float3 linear)
        {
            this.angular = angular;
            this.linear = linear;
        }

        public static readonly SpatialForce zero = new SpatialForce(float3.zero, float3.zero);

        public static SpatialForce operator +(SpatialForce a, SpatialForce b) =>
            new SpatialForce(a.angular + b.angular, a.linear + b.linear);
        public static SpatialForce operator -(SpatialForce a, SpatialForce b) =>
            new SpatialForce(a.angular - b.angular, a.linear - b.linear);
        public static SpatialForce operator -(SpatialForce a) =>
            new SpatialForce(-a.angular, -a.linear);
        public static SpatialForce operator *(SpatialForce a, float s) =>
            new SpatialForce(a.angular * s, a.linear * s);
        public static SpatialForce operator *(float s, SpatialForce a) => a * s;
    }
}
