// Featherstone spatial-vector primitives — 6-vec motion / force, 6×6
// matrices, spatial transform (Plücker), spatial cross product (`×̂`
// for motion×motion → motion, `×̂*` for motion×force → force).
//
// FRAME CONVENTION (Featherstone RBDA 2008, link-fixed body frame):
//   Spatial motion ordering: (angular, linear) = (ω, v_O), where v_O
//   is the linear velocity of the *body-frame origin* (not the CoM).
//   Spatial force ordering:  (angular, linear) = (n_O, f), n_O is the
//   moment about the body-frame origin.
//   Angular FIRST throughout — matching C# reference at
//   mod/Longeron.Physics/src/Spatial/* (read-only ref, unvalidated
//   in-game; this C++ side is canonical).
//
// SpatialTransform X_A→B = (q, t):
//   q is the *active* rotation putting frame B's basis in A
//     (basis_B[i] = q · basis_A[i]).
//   t is the position of B's origin, expressed in A.
//   Motion transform: ω_B = q⁻¹·ω_A; v_B = q⁻¹·(v_A − t × ω_A).
//   Force transform:  n_B = q⁻¹·(n_A − t × f_A); f_B = q⁻¹·f_A.
//
// Spatial cross duality (RBDA eq. 2.17):
//   (v ×̂* f) · m  +  f · (v ×̂ m)  =  0
// for any motion v, m and force f.  This is the load-bearing identity
// for spotting sign errors; M2 self-test exercises it.

#ifndef LONGERON_SPATIAL_H
#define LONGERON_SPATIAL_H

#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>
#include <Jolt/Math/Quat.h>

#include <cmath>

namespace longeron {

// ---- Mat33 ---------------------------------------------------------
// 3×3 matrix stored as three row vectors. Column-vector convention for
// multiplication: y = M × x means y_i = Σ_j M[i,j] × x_j, so each row
// dotted with x produces an output component. Sufficient for our
// inertia-rotation, skew, and 3×3 solve needs without pulling in a
// heavier matrix library.

struct Mat33 {
    JPH::Vec3 row0, row1, row2;

    static Mat33 Zero() {
        return Mat33{JPH::Vec3::sZero(), JPH::Vec3::sZero(), JPH::Vec3::sZero()};
    }
    static Mat33 Identity() {
        return Mat33{
            JPH::Vec3(1, 0, 0),
            JPH::Vec3(0, 1, 0),
            JPH::Vec3(0, 0, 1)};
    }

    // Column j of M as a Vec3.
    JPH::Vec3 Col(int j) const {
        return JPH::Vec3(row0[j], row1[j], row2[j]);
    }

    // M × v.
    JPH::Vec3 operator*(JPH::Vec3 v) const {
        return JPH::Vec3(row0.Dot(v), row1.Dot(v), row2.Dot(v));
    }

    Mat33 operator+(const Mat33& o) const {
        return Mat33{row0 + o.row0, row1 + o.row1, row2 + o.row2};
    }
    Mat33 operator-(const Mat33& o) const {
        return Mat33{row0 - o.row0, row1 - o.row1, row2 - o.row2};
    }

    Mat33 Transposed() const {
        return Mat33{
            JPH::Vec3(row0.GetX(), row1.GetX(), row2.GetX()),
            JPH::Vec3(row0.GetY(), row1.GetY(), row2.GetY()),
            JPH::Vec3(row0.GetZ(), row1.GetZ(), row2.GetZ())};
    }
};

inline Mat33 operator*(float s, const Mat33& m) {
    return Mat33{m.row0 * s, m.row1 * s, m.row2 * s};
}
inline Mat33 operator*(const Mat33& m, float s) { return s * m; }

// 3×3 × 3×3 — operator overload so callsites stay readable.
inline Mat33 operator*(const Mat33& a, const Mat33& b) {
    Mat33 bt = b.Transposed();
    return Mat33{
        JPH::Vec3(a.row0.Dot(bt.row0), a.row0.Dot(bt.row1), a.row0.Dot(bt.row2)),
        JPH::Vec3(a.row1.Dot(bt.row0), a.row1.Dot(bt.row1), a.row1.Dot(bt.row2)),
        JPH::Vec3(a.row2.Dot(bt.row0), a.row2.Dot(bt.row1), a.row2.Dot(bt.row2))};
}

// Outer product u·v^T as a 3×3 matrix. M[i,j] = u[i] × v[j], so
// row i = u[i] × v.
inline Mat33 OuterProduct(JPH::Vec3 u, JPH::Vec3 v) {
    return Mat33{
        v * u.GetX(),
        v * u.GetY(),
        v * u.GetZ()};
}

// Skew-symmetric matrix from a 3-vec. skew(a) × b == a × b for any b.
//   [  0  -a.z  a.y]
//   [ a.z   0  -a.x]
//   [-a.y  a.x   0 ]
inline Mat33 Skew(JPH::Vec3 a) {
    return Mat33{
        JPH::Vec3(0,         -a.GetZ(),  a.GetY()),
        JPH::Vec3(a.GetZ(),   0,        -a.GetX()),
        JPH::Vec3(-a.GetY(),  a.GetX(),  0)};
}

// Build the 3×3 rotation matrix that's active-equivalent to quaternion q,
// i.e., (QuatToMat33(q)) × v ≡ q · v for any v. Columns are q·basis_j.
inline Mat33 QuatToMat33(JPH::Quat q) {
    JPH::Vec3 c0 = q * JPH::Vec3(1, 0, 0);
    JPH::Vec3 c1 = q * JPH::Vec3(0, 1, 0);
    JPH::Vec3 c2 = q * JPH::Vec3(0, 0, 1);
    return Mat33{
        JPH::Vec3(c0.GetX(), c1.GetX(), c2.GetX()),
        JPH::Vec3(c0.GetY(), c1.GetY(), c2.GetY()),
        JPH::Vec3(c0.GetZ(), c1.GetZ(), c2.GetZ())};
}

// 3×3 → quaternion-rotated 3×3 (active basis change of a tensor):
//   M_rotated = R · M · R^T,  where R = QuatToMat33(q).
// Uses one R materialization + two 3×3 multiplies.
inline Mat33 RotateBasis(JPH::Quat q, const Mat33& m) {
    Mat33 R = QuatToMat33(q);
    return R * (m * R.Transposed());
}

// Determinant of a 3×3.
inline float Det(const Mat33& m) {
    JPH::Vec3 c0 = m.Col(0), c1 = m.Col(1), c2 = m.Col(2);
    return c0.Dot(c1.Cross(c2));
}

// Inverse of a 3×3 via adjugate / det. Returns true on success;
// returns Identity in *out_inv on degenerate matrices (det near zero).
inline bool Inverse(const Mat33& m, Mat33* out_inv) {
    JPH::Vec3 c0 = m.Col(0), c1 = m.Col(1), c2 = m.Col(2);
    JPH::Vec3 cof0 = c1.Cross(c2);
    JPH::Vec3 cof1 = c2.Cross(c0);
    JPH::Vec3 cof2 = c0.Cross(c1);
    float det = c0.Dot(cof0);
    if (std::fabs(det) < 1.0e-20f) {
        *out_inv = Mat33::Identity();
        return false;
    }
    float inv_det = 1.0f / det;
    // Adjugate (transpose of cofactor matrix); columns are the cofactors.
    *out_inv = Mat33{
        JPH::Vec3(cof0.GetX(), cof1.GetX(), cof2.GetX()) * inv_det,
        JPH::Vec3(cof0.GetY(), cof1.GetY(), cof2.GetY()) * inv_det,
        JPH::Vec3(cof0.GetZ(), cof1.GetZ(), cof2.GetZ()) * inv_det};
    return true;
}

// ---- Spatial vectors -----------------------------------------------

struct SpatialMotion {
    JPH::Vec3 angular;   // ω
    JPH::Vec3 linear;    // v at body-frame origin

    static SpatialMotion Zero() {
        return {JPH::Vec3::sZero(), JPH::Vec3::sZero()};
    }

    SpatialMotion operator+(SpatialMotion o) const { return {angular + o.angular, linear + o.linear}; }
    SpatialMotion operator-(SpatialMotion o) const { return {angular - o.angular, linear - o.linear}; }
    SpatialMotion operator-() const { return {-angular, -linear}; }
};

inline SpatialMotion operator*(SpatialMotion m, float s) { return {m.angular * s, m.linear * s}; }
inline SpatialMotion operator*(float s, SpatialMotion m) { return m * s; }

struct SpatialForce {
    JPH::Vec3 angular;   // moment about body-frame origin
    JPH::Vec3 linear;    // force

    static SpatialForce Zero() {
        return {JPH::Vec3::sZero(), JPH::Vec3::sZero()};
    }

    SpatialForce operator+(SpatialForce o) const { return {angular + o.angular, linear + o.linear}; }
    SpatialForce operator-(SpatialForce o) const { return {angular - o.angular, linear - o.linear}; }
    SpatialForce operator-() const { return {-angular, -linear}; }
};

inline SpatialForce operator*(SpatialForce f, float s) { return {f.angular * s, f.linear * s}; }
inline SpatialForce operator*(float s, SpatialForce f) { return f * s; }

// Scalar pairing: f · m = n · ω + f · v.
inline float Dot(SpatialForce f, SpatialMotion m) {
    return f.angular.Dot(m.angular) + f.linear.Dot(m.linear);
}
inline float Dot(SpatialMotion m, SpatialForce f) { return Dot(f, m); }

// ---- Spatial cross --------------------------------------------------
// Two distinct cross-products — mixing them is the #1 silent bug.
//
//   CrossMotion(v, m)     — motion × motion → motion. RBDA "crm".
//     v ×̂ m = (ω_v × ω_m,  ω_v × v_m + v_v × ω_m)
//
//   CrossForceDual(v, f)  — motion × force → force. RBDA "crf*".
//     v ×̂* f = (ω_v × n_f + v_v × f_f,  ω_v × f_f)

inline SpatialMotion CrossMotion(SpatialMotion v, SpatialMotion m) {
    return {
        v.angular.Cross(m.angular),
        v.angular.Cross(m.linear) + v.linear.Cross(m.angular)};
}

inline SpatialForce CrossForceDual(SpatialMotion v, SpatialForce f) {
    return {
        v.angular.Cross(f.angular) + v.linear.Cross(f.linear),
        v.angular.Cross(f.linear)};
}

// ---- Spatial transform (Plücker) -----------------------------------

struct SpatialTransform {
    JPH::Quat rotation;     // active rotation: B's basis in A
    JPH::Vec3 translation;  // origin of B, expressed in A

    static SpatialTransform Identity() {
        return {JPH::Quat::sIdentity(), JPH::Vec3::sZero()};
    }

    // m_B = X · m_A.
    //   ω_B = q⁻¹ · ω_A
    //   v_B = q⁻¹ · (v_A − t × ω_A)
    SpatialMotion TransformMotion(SpatialMotion m) const {
        JPH::Quat qInv = rotation.Conjugated();
        return {
            qInv * m.angular,
            qInv * (m.linear - translation.Cross(m.angular))};
    }

    // f_B = X^* · f_A.
    //   n_B = q⁻¹ · (n_A − t × f_A)
    //   f_B = q⁻¹ · f_A
    SpatialForce TransformForce(SpatialForce f) const {
        JPH::Quat qInv = rotation.Conjugated();
        return {
            qInv * (f.angular - translation.Cross(f.linear)),
            qInv * f.linear};
    }

    // m_A = X^{-1} · m_B.
    //   ω_A = q · ω_B
    //   v_A = q · v_B + t × ω_A
    SpatialMotion InverseTransformMotion(SpatialMotion m) const {
        JPH::Vec3 omega_a = rotation * m.angular;
        return {
            omega_a,
            rotation * m.linear + translation.Cross(omega_a)};
    }

    // f_A = X^{-*} · f_B.
    //   f_A = q · f_B
    //   n_A = q · n_B + t × f_A
    SpatialForce InverseTransformForce(SpatialForce f) const {
        JPH::Vec3 f_a = rotation * f.linear;
        return {
            rotation * f.angular + translation.Cross(f_a),
            f_a};
    }

    SpatialTransform Inverse() const {
        JPH::Quat qInv = rotation.Conjugated();
        return {qInv, -(qInv * translation)};
    }
};

// ---- Spatial inertia (compact form) --------------------------------
// Stored as (mass, com_in_body_frame, I_about_com_in_body_basis).
// Equivalent 6×6 inertia (about body-frame origin):
//   I_sp = [[ I_c − m·ĉ²,   m·ĉ   ],
//           [ −m·ĉ,          m·I_3 ]]
//
// Mul(motion) → spatial momentum:
//   p_lin = m · (v − c × ω)
//   n_ang = I_c · ω + c × p_lin

struct SpatialInertia {
    float     mass;
    JPH::Vec3 com;     // CoM in body frame
    Mat33     I_c;     // inertia about CoM, in body basis

    static SpatialInertia Zero() {
        return {0.0f, JPH::Vec3::sZero(), Mat33::Zero()};
    }

    // Convenience: build from a diagonal inertia tensor (principal axes
    // aligned with body frame).
    static SpatialInertia FromDiagonal(float mass, JPH::Vec3 com, JPH::Vec3 I_diag) {
        return {
            mass,
            com,
            Mat33{
                JPH::Vec3(I_diag.GetX(), 0, 0),
                JPH::Vec3(0, I_diag.GetY(), 0),
                JPH::Vec3(0, 0, I_diag.GetZ())}};
    }

    SpatialForce Mul(SpatialMotion m) const {
        JPH::Vec3 p_lin = mass * (m.linear - com.Cross(m.angular));
        JPH::Vec3 n_ang = I_c * m.angular + com.Cross(p_lin);
        return {n_ang, p_lin};
    }
};

// ---- Spatial 6×6 (general) -----------------------------------------
// Used for articulated-body inertia I^A in ABA pass 2 — rank-r updates
// preserve symmetry but break the rigid-body block pattern, so we
// store the general form.
//
//   M = [[a, b],   M · v_motion = (a·ω + b·v_lin, c·ω + d·v_lin)
//        [c, d]]

struct SpatialMatrix6 {
    Mat33 a, b, c, d;

    static SpatialMatrix6 Zero() {
        return {Mat33::Zero(), Mat33::Zero(), Mat33::Zero(), Mat33::Zero()};
    }

    // Build the 6×6 form from a compact rigid-body spatial inertia:
    //   [[I_c − m·ĉ²,    m·ĉ  ],
    //    [-m·ĉ,           m·I_3]]
    static SpatialMatrix6 FromInertia(const SpatialInertia& I) {
        Mat33 ch = Skew(I.com);
        Mat33 ch2 = ch * ch;
        Mat33 top = I.I_c + (-I.mass) * ch2;
        Mat33 off = I.mass * ch;
        Mat33 diag = I.mass * Mat33::Identity();
        return {top, off, (-1.0f) * off, diag};
    }

    // M · motion → force.
    SpatialForce Mul(SpatialMotion m) const {
        return {
            a * m.angular + b * m.linear,
            c * m.angular + d * m.linear};
    }

    // Subtract a rank-1 update: M ← M − (1/s) · u u^T (where u is a
    // spatial force, u u^T is a 6×6 outer product).
    SpatialMatrix6 SubRankOne(SpatialForce u, float s) const {
        float inv = 1.0f / s;
        return {
            a + (-inv) * OuterProduct(u.angular, u.angular),
            b + (-inv) * OuterProduct(u.angular, u.linear),
            c + (-inv) * OuterProduct(u.linear,  u.angular),
            d + (-inv) * OuterProduct(u.linear,  u.linear)};
    }

    // Transform the matrix from frame A to frame B (it represents a
    // spatial mapping; basis-changes both arguments). M_B = X_f · M_A · X_m^{-1}.
    // See Featherstone Eq 2.66 / RBDA Sec 2.10.
    //   X_f = [[E,  −E·p̂], [0,  E]]
    //   X_m^{-1} = [[E^T, 0], [p̂·E^T, E^T]]
    // E = R(q⁻¹) is the passive rotation (basis change). p̂ = skew(t).
    SpatialMatrix6 Transform(const SpatialTransform& X) const {
        Mat33 E = QuatToMat33(X.rotation.Conjugated());
        Mat33 Et = E.Transposed();
        Mat33 ph = Skew(X.translation);
        Mat33 Eph  = E * ph;
        Mat33 phEt = ph * Et;

        // temp = M_A · X_m^{-1}
        Mat33 t_aa = a * Et + b * phEt;
        Mat33 t_ab = b * Et;
        Mat33 t_ba = c * Et + d * phEt;
        Mat33 t_bb = d * Et;

        // result = X_f · temp
        Mat33 r_aa = E * t_aa + (-1.0f) * (Eph * t_ba);
        Mat33 r_ab = E * t_ab + (-1.0f) * (Eph * t_bb);
        Mat33 r_ba = E * t_ba;
        Mat33 r_bb = E * t_bb;

        return {r_aa, r_ab, r_ba, r_bb};
    }

    SpatialMatrix6 InverseTransform(const SpatialTransform& X) const {
        return Transform(X.Inverse());
    }

    SpatialMatrix6 operator+(const SpatialMatrix6& o) const {
        return {a + o.a, b + o.b, c + o.c, d + o.d};
    }
};

// ---- Spherical-joint reduction (M3) --------------------------------
// For a spherical joint, motion subspace S = [I_3; 0_3] (top 3 rows
// identity, bottom 3 zero). The rank-3 reduction collapses to:
//   U ≡ I^A · S = (I^A.a as columns; I^A.c as columns)  → SpatialForce per-column
//   D ≡ S^T · I^A · S = I^A.a (the angular-block 3×3)
//   u ≡ −S^T · pA + τ_spring = −pA.angular + τ_spring  (3-vec)
//   I^A_a = I^A − U · D⁻¹ · U^T  (rank-3 update)
//
// We expose helpers below; aba.cpp uses them in pass 2 / pass 3.

// Rank-3 update I^A − U · D⁻¹ · U^T where U is 6×3 stored as
// (U_top: a-block columns, U_bot: c-block columns) and D = a-block.
// For spherical joints: U_top = IA.a, U_bot = IA.c, D = IA.a.
inline SpatialMatrix6 SubRankSpherical(const SpatialMatrix6& IA) {
    Mat33 D = IA.a;  // S^T · IA · S = a-block
    Mat33 D_inv;
    if (!Inverse(D, &D_inv)) {
        // Degenerate (zero angular inertia in this block) — return as-is.
        return IA;
    }
    // U_top = IA.a, U_bot = IA.c.
    // U · D⁻¹ · U^T has blocks:
    //   [U_top · D⁻¹ · U_top^T,   U_top · D⁻¹ · U_bot^T]
    //   [U_bot · D⁻¹ · U_top^T,   U_bot · D⁻¹ · U_bot^T]
    Mat33 U_top = IA.a;
    Mat33 U_bot = IA.c;
    Mat33 U_top_Dinv = U_top * D_inv;
    Mat33 U_bot_Dinv = U_bot * D_inv;
    Mat33 R_aa = U_top_Dinv * U_top.Transposed();
    Mat33 R_ab = U_top_Dinv * U_bot.Transposed();
    Mat33 R_ba = U_bot_Dinv * U_top.Transposed();
    Mat33 R_bb = U_bot_Dinv * U_bot.Transposed();

    return {
        IA.a + (-1.0f) * R_aa,
        IA.b + (-1.0f) * R_ab,
        IA.c + (-1.0f) * R_ba,
        IA.d + (-1.0f) * R_bb};
}

} // namespace longeron

#endif // LONGERON_SPATIAL_H
