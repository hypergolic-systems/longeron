// C ABI dispatcher: thin shim that forwards to longeron::LongeronWorld.

#include "bridge.h"
#include "spatial.h"
#include "world.h"

#include <cmath>
#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>
#include <Jolt/Math/Quat.h>

namespace {

constexpr const char* kVersionString =
#if defined(LONGERON_HAS_JOLT)
    #if defined(JPH_DOUBLE_PRECISION)
        "longeron_native 0.0.1 (jolt: enabled, double-precision)";
    #else
        "longeron_native 0.0.1 (jolt: enabled, single-precision)";
    #endif
#else
    "longeron_native 0.0.1 (jolt: stub)";
#endif

} // namespace

// LongeronWorld is the opaque struct exposed to C. Internally it
// inherits from longeron::LongeronWorld.
struct LongeronWorld : public longeron::LongeronWorld {
    using longeron::LongeronWorld::LongeronWorld;
};

extern "C" {

LONGERON_EXPORT LongeronWorld* longeron_world_create(const LongeronConfig* cfg) {
    if (cfg == nullptr) return nullptr;
    try {
        return new LongeronWorld(*cfg);
    } catch (...) {
        return nullptr;
    }
}

LONGERON_EXPORT void longeron_world_destroy(LongeronWorld* w) {
    delete w;
}

LONGERON_EXPORT int32_t longeron_step(
    LongeronWorld* w,
    const uint8_t* input,
    size_t         input_len,
    uint8_t*       output,
    size_t         output_cap,
    size_t*        output_len,
    float          dt)
{
    if (w == nullptr || output_len == nullptr) return -1;
    return w->Step(input, input_len, output, output_cap, output_len, dt);
}

LONGERON_EXPORT const char* longeron_version(void) {
    return kVersionString;
}

// In-process sanity checks on spatial.h primitives. Returns 0 on pass
// or the 1-indexed number of the first failing check.
LONGERON_EXPORT int32_t longeron_spatial_selftest(void) {
    using namespace longeron;

    auto vec_close = [](JPH::Vec3 a, JPH::Vec3 b, float tol) {
        return (a - b).Length() < tol;
    };
    auto float_close = [](float a, float b, float tol) {
        return std::fabs(a - b) < tol;
    };

    // --- 1. Identity transform leaves motion / force unchanged. ----
    {
        SpatialTransform X = SpatialTransform::Identity();
        SpatialMotion m{JPH::Vec3(1, 2, 3), JPH::Vec3(4, 5, 6)};
        SpatialForce  f{JPH::Vec3(7, 8, 9), JPH::Vec3(10, 11, 12)};
        SpatialMotion m2 = X.TransformMotion(m);
        SpatialForce  f2 = X.TransformForce(f);
        if (!vec_close(m.angular, m2.angular, 1e-5f)) return 1;
        if (!vec_close(m.linear,  m2.linear,  1e-5f)) return 1;
        if (!vec_close(f.angular, f2.angular, 1e-5f)) return 1;
        if (!vec_close(f.linear,  f2.linear,  1e-5f)) return 1;
    }

    // --- 2. Spatial cross duality (RBDA eq. 2.17): -----------------
    //   (v ×̂* f) · m + f · (v ×̂ m) = 0
    {
        SpatialMotion v{JPH::Vec3(0.3f, -0.5f, 0.7f), JPH::Vec3(1.1f, 2.2f, -1.3f)};
        SpatialMotion m{JPH::Vec3(-0.2f, 0.4f, 0.1f), JPH::Vec3(0.6f, -0.7f, 0.8f)};
        SpatialForce  f{JPH::Vec3(2.0f, -3.0f, 1.5f), JPH::Vec3(-0.5f, 1.2f, 2.7f)};
        float lhs = Dot(CrossForceDual(v, f), m) + Dot(f, CrossMotion(v, m));
        if (!float_close(lhs, 0.0f, 1e-4f)) return 2;
    }

    // --- 3. X · X⁻¹ ≡ identity for both motion and force. ---------
    {
        JPH::Quat q = JPH::Quat::sRotation(JPH::Vec3(1, 1, 1).Normalized(), 0.7f);
        SpatialTransform X{q, JPH::Vec3(0.5f, -1.2f, 0.3f)};
        SpatialMotion m{JPH::Vec3(1, 2, 3), JPH::Vec3(4, 5, 6)};
        SpatialForce  f{JPH::Vec3(7, 8, 9), JPH::Vec3(10, 11, 12)};
        SpatialMotion mB = X.TransformMotion(m);
        SpatialMotion mA = X.InverseTransformMotion(mB);
        SpatialForce  fB = X.TransformForce(f);
        SpatialForce  fA = X.InverseTransformForce(fB);
        if (!vec_close(m.angular, mA.angular, 1e-4f)) return 3;
        if (!vec_close(m.linear,  mA.linear,  1e-4f)) return 3;
        if (!vec_close(f.angular, fA.angular, 1e-4f)) return 3;
        if (!vec_close(f.linear,  fA.linear,  1e-4f)) return 3;
    }

    // --- 4. SpatialInertia.Mul matches the rigid-body formula. ----
    //   p_lin = m·(v − c×ω);  n_ang = I_c·ω + c × p_lin
    {
        float mass = 2.5f;
        JPH::Vec3 com    = JPH::Vec3(0.1f, 0.3f, -0.2f);
        JPH::Vec3 I_diag = JPH::Vec3(0.5f, 0.7f, 0.9f);
        SpatialInertia I = SpatialInertia::FromDiagonal(mass, com, I_diag);
        SpatialMotion mvec{JPH::Vec3(0.4f, -0.2f, 0.6f),
                            JPH::Vec3(0.7f, 0.1f, -0.3f)};
        SpatialForce out = I.Mul(mvec);
        JPH::Vec3 cxo = com.Cross(mvec.angular);
        JPH::Vec3 p_lin = mass * (mvec.linear - cxo);
        JPH::Vec3 I_ang(I_diag.GetX() * mvec.angular.GetX(),
                        I_diag.GetY() * mvec.angular.GetY(),
                        I_diag.GetZ() * mvec.angular.GetZ());
        JPH::Vec3 n_ang = I_ang + com.Cross(p_lin);
        if (!vec_close(out.angular, n_ang, 1e-4f)) return 4;
        if (!vec_close(out.linear,  p_lin, 1e-4f)) return 4;
    }

    // --- 5. SpatialMatrix6.FromInertia agrees with SpatialInertia.Mul. -
    {
        float mass = 2.5f;
        JPH::Vec3 com    = JPH::Vec3(0.1f, 0.3f, -0.2f);
        JPH::Vec3 I_diag = JPH::Vec3(0.5f, 0.7f, 0.9f);
        SpatialInertia I = SpatialInertia::FromDiagonal(mass, com, I_diag);
        SpatialMatrix6 M6 = SpatialMatrix6::FromInertia(I);
        SpatialMotion mvec{JPH::Vec3(0.4f, -0.2f, 0.6f),
                            JPH::Vec3(0.7f, 0.1f, -0.3f)};
        SpatialForce f1 = I.Mul(mvec);
        SpatialForce f2 = M6.Mul(mvec);
        if (!vec_close(f1.angular, f2.angular, 1e-4f)) return 5;
        if (!vec_close(f1.linear,  f2.linear,  1e-4f)) return 5;
    }

    // --- 6. Mat33 inverse: M · M⁻¹ ≡ identity. -------------------
    {
        Mat33 M{
            JPH::Vec3(2, 1, 0),
            JPH::Vec3(1, 3, 1),
            JPH::Vec3(0, 1, 2)};
        Mat33 Minv;
        if (!Inverse(M, &Minv)) return 6;
        Mat33 prod = M * Minv;
        Mat33 I = Mat33::Identity();
        if (!vec_close(prod.row0, I.row0, 1e-4f)) return 6;
        if (!vec_close(prod.row1, I.row1, 1e-4f)) return 6;
        if (!vec_close(prod.row2, I.row2, 1e-4f)) return 6;
    }

    return 0;
}

} // extern "C"
