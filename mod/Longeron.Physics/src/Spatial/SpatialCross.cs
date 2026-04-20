// FRAME CONVENTION — Featherstone RBDA (2008), link-fixed body frame.
//
// Two spatial cross-products, as distinct named methods. Mixing them up is
// the #1 silent bug in multibody implementations.
//
//   CrossMotion(v, m)    — motion × motion → motion.   "crm" in RBDA.
//   CrossForceDual(v, f) — motion × force  → force.    "crf" in RBDA.
//
// Featherstone's convention (RBDA eq. 2.13, 2.14), with v = (ω_v, v_v),
// m = (ω_m, v_m), f = (n_f, f_f):
//
//   v × m  = ( ω_v × ω_m,
//              ω_v × v_m + v_v × ω_m )
//
//   v ×* f = ( ω_v × n_f + v_v × f_f,
//              ω_v × f_f )
//
// Duality (RBDA eq. 2.17): crf(v) = -crm(v)^T. For any motion m and force f:
//
//   (v ×* f) · m  +  f · (v × m)  =  0
//
// This identity is enforced as a unit test and is the single most effective
// sign/convention gate in the solver. It rules out whole families of errors.

namespace Longeron.Physics
{
    public static class SpatialCross
    {
        public static SpatialMotion CrossMotion(SpatialMotion v, SpatialMotion m) =>
            new SpatialMotion(
                math.cross(v.angular, m.angular),
                math.cross(v.angular, m.linear) + math.cross(v.linear, m.angular));

        public static SpatialForce CrossForceDual(SpatialMotion v, SpatialForce f) =>
            new SpatialForce(
                math.cross(v.angular, f.angular) + math.cross(v.linear, f.linear),
                math.cross(v.angular, f.linear));

        // Scalar pairing: force · motion = n · ω + f · v.
        public static float Dot(SpatialForce f, SpatialMotion m) =>
            math.dot(f.angular, m.angular) + math.dot(f.linear, m.linear);

        public static float Dot(SpatialMotion m, SpatialForce f) => Dot(f, m);
    }
}
