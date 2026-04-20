// FRAME CONVENTION — Featherstone RBDA (2008), link-fixed body frame.
// Every spatial quantity lives in the body frame of the link it's attached to.
//
// Spatial motion vector ordering: (angular, linear) = (ω, v_O),
// where v_O is the linear velocity of the *point at the body frame's origin*
// (not the COM). Angular first, throughout.

namespace Longeron.Physics
{
    public readonly struct SpatialMotion
    {
        public readonly float3 angular;   // ω
        public readonly float3 linear;    // v at origin of body frame

        public SpatialMotion(float3 angular, float3 linear)
        {
            this.angular = angular;
            this.linear = linear;
        }

        public static readonly SpatialMotion zero = new SpatialMotion(float3.zero, float3.zero);

        public static SpatialMotion operator +(SpatialMotion a, SpatialMotion b) =>
            new SpatialMotion(a.angular + b.angular, a.linear + b.linear);
        public static SpatialMotion operator -(SpatialMotion a, SpatialMotion b) =>
            new SpatialMotion(a.angular - b.angular, a.linear - b.linear);
        public static SpatialMotion operator -(SpatialMotion a) =>
            new SpatialMotion(-a.angular, -a.linear);
        public static SpatialMotion operator *(SpatialMotion a, float s) =>
            new SpatialMotion(a.angular * s, a.linear * s);
        public static SpatialMotion operator *(float s, SpatialMotion a) => a * s;
    }
}
