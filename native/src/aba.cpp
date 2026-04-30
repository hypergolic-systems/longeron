// Phase 5.1.2: canonical reduced-coord Featherstone ABA forward pass.
// Spherical joint at every non-root edge. Each link integrates its
// joint state (q_joint, ω_joint) via the three-pass recursion:
//
//   Pass 1 (root → leaf): outward kinematics. Spatial velocity v_i and
//     Coriolis bias c_i in each link's body frame, transformed through
//     X_up = (q_joint, t_up_const).
//
//   Pass 2 (leaf → root): inward articulated-inertia. I^A and p^A per
//     link, each with the rank-3 spherical reduction projecting out
//     the joint motion subspace before transforming + adding to parent.
//
//   Pass 3 (root → leaf): outward acceleration solve. qddot = D⁻¹ ·
//     (u − S^T·I^A·a_parent_propagated). Spatial acceleration a_i =
//     a_parent_propagated + S·qddot + c_i.
//
// Then integrate (semi-implicit Euler): ω_joint += dt·qddot,
// q_joint = integrate_quat_right(q_joint, ω_joint, dt).
//
// State storage maps to PartFlex as:
//   delta_rot[i] — vessel-frame cumulative rotation R_v[i] (output;
//     re-decomposed at entry to recover q_joint[i]).
//   delta_pos[i] — vessel-frame CoM deflection (output, derived from
//     joint chain).
//   ang_vel[i]   — ω_joint[i] in body i's frame (primary integrated
//     state across ticks).
//   lin_vel[i]   — unused (no linear DOF).
//
// Design note: ω_joint is stored in body i's frame so the joint
// motion subspace S = [I_3; 0_3] applies directly (S × ω_joint =
// SpatialMotion(ω_joint, 0) in body frame). Quaternion integration
// is right-multiply: q ← q · (1, ω·dt/2).
//
// External wrench (F_flex, T_flex) is computed once per call in
// vessel frame and rotated into each part's body frame at bias-force
// time using the per-substep-current R_v[i]. This realizes the
// `feedback_physics_visuals_align` invariant: forces act through
// each part's CURRENT flexed orientation.

#include "aba.h"
#include "spatial.h"

#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>
#include <Jolt/Math/Quat.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace longeron {

namespace {

// Convert a quaternion to its axis-angle 3-vec (axis × angle in rad).
// Handles double-cover: q and -q represent the same rotation.
JPH::Vec3 QuatToAxisAngle(JPH::Quat q) {
    if (q.GetW() < 0.0f) q = JPH::Quat(-q.GetX(), -q.GetY(), -q.GetZ(), -q.GetW());
    JPH::Vec3 v(q.GetX(), q.GetY(), q.GetZ());
    float w = q.GetW();
    float v_len = v.Length();
    if (v_len < 1.0e-6f) return JPH::Vec3::sZero();
    float angle = 2.0f * std::atan2(v_len, w);
    return v * (angle / v_len);
}

// Right-multiply quaternion integration: q_new = q · (1, ω_body·dt/2),
// renormalized. ω_body is angular velocity expressed in q's body frame.
JPH::Quat IntegrateRotationRight(JPH::Quat q, JPH::Vec3 omega_body, float dt) {
    JPH::Quat dq(0.5f * omega_body.GetX() * dt,
                 0.5f * omega_body.GetY() * dt,
                 0.5f * omega_body.GetZ() * dt,
                 0.0f);
    JPH::Quat q_dot = q * dq;
    JPH::Quat q_new(q.GetX() + q_dot.GetX(),
                    q.GetY() + q_dot.GetY(),
                    q.GetZ() + q_dot.GetZ(),
                    q.GetW() + q_dot.GetW());
    return q_new.Normalized();
}

// Component-wise inertia × vec — used only for the inertial-residual
// computation. Independent of frame: works on the diag values.
JPH::Vec3 InertiaTimes(JPH::Vec3 I_diag, JPH::Vec3 v) {
    return JPH::Vec3(I_diag.GetX() * v.GetX(),
                     I_diag.GetY() * v.GetY(),
                     I_diag.GetZ() * v.GetZ());
}

// Rotate a 3-vec by a quaternion's inverse (q⁻¹ · v) — used to take
// a vessel-frame vector into a part's body frame.
JPH::Vec3 RotateByInv(JPH::Quat q, JPH::Vec3 v) { return q.Conjugated() * v; }

} // namespace

void RunAbaForwardStep(
    VesselTree& tree,
    JPH::Vec3 v_body_unused,
    JPH::Vec3 omega,
    JPH::Vec3 a_body,
    JPH::Vec3 alpha,
    float fixed_dt,
    uint32_t body_id,
    std::vector<AbaPartDiagRecord>* diag_out)
{
    (void)v_body_unused;
    const size_t n = tree.nodes.size();
    if (n < 2) return;


    const float dt = fixed_dt / static_cast<float>(kAbaSubsteps);

    // ---- Per-call setup ------------------------------------------------
    //
    // Inertial-residual force / torque per part (vessel frame), held
    // constant across substeps (KSP only updates external forces at
    // 50 Hz). F_inertial accounts for the share of part motion that's
    // dragged along by Jolt's vessel-CoM integration; the residual
    // F_flex / T_flex is what actually drives flex.

    static thread_local std::vector<JPH::Vec3> F_flex, T_flex;
    F_flex.assign(n, JPH::Vec3::sZero());
    T_flex.assign(n, JPH::Vec3::sZero());

    for (size_t i = 0; i < n; ++i) {
        const PartNode& p = tree.nodes[i];
        const JPH::Vec3 r = p.com_local + tree.flex[i].delta_pos;
        const JPH::Vec3 a_part = a_body + alpha.Cross(r) + omega.Cross(omega.Cross(r));
        const JPH::Vec3 F_inertial = p.mass * a_part;
        const JPH::Vec3 T_inertial = InertiaTimes(p.inertia_diag, alpha)
                                     + omega.Cross(InertiaTimes(p.inertia_diag, omega));
        F_flex[i] = tree.ext_force[i]  - F_inertial;
        T_flex[i] = tree.ext_torque[i] - T_inertial;

        // Diag emit (first kAbaDiagWindow ticks per body, every part).
        if (diag_out != nullptr && tree.diag_tick < kAbaDiagWindow) {
            AbaPartDiagRecord rec;
            rec.body_id    = body_id;
            rec.tick       = static_cast<uint16_t>(tree.diag_tick);
            rec.part_idx   = static_cast<uint16_t>(i);
            rec.ext_force  = tree.ext_force[i];
            rec.f_inertial = F_inertial;
            rec.f_flex     = F_flex[i];
            rec.delta_pos  = tree.flex[i].delta_pos;
            JPH::Quat q = tree.flex[i].delta_rot;
            if (q.GetW() < 0.0f) q = JPH::Quat(-q.GetX(), -q.GetY(), -q.GetZ(), -q.GetW());
            float vmag = std::sqrt(q.GetX()*q.GetX() + q.GetY()*q.GetY() + q.GetZ()*q.GetZ());
            rec.delta_angle_rad = (vmag > 1.0e-6f)
                ? 2.0f * std::atan2(vmag, q.GetW())
                : 0.0f;
            diag_out->push_back(rec);
        }
    }

    // ---- Reconstruct joint state from PartFlex --------------------------
    //
    // delta_rot[i] from the previous tick is vessel-frame cumulative
    // rotation R_v[i]. Decompose into per-edge q_joint[i] = R_v[parent]⁻¹
    // · R_v[i]. ω_joint[i] is stored directly in PartFlex.ang_vel as
    // joint angular velocity in body i's frame (its primary persisted
    // form).

    static thread_local std::vector<JPH::Quat> q_joint;
    static thread_local std::vector<JPH::Vec3> omega_joint;
    q_joint.assign(n, JPH::Quat::sIdentity());
    omega_joint.assign(n, JPH::Vec3::sZero());

    for (size_t i = 1; i < n; ++i) {
        const uint16_t p = tree.nodes[i].parent_idx;
        if (p >= n) continue;
        // q_joint[i] = R_v[p]⁻¹ × R_v[i]
        q_joint[i] = tree.flex[p].delta_rot.Conjugated() * tree.flex[i].delta_rot;
        omega_joint[i] = tree.flex[i].ang_vel;
    }

    // Constant per-edge body-frame translation: t_up = attach_local[i]
    // − attach_local[parent], the offset of body i's frame origin (its
    // joint anchor) in parent's body frame.
    static thread_local std::vector<JPH::Vec3> t_up;
    t_up.assign(n, JPH::Vec3::sZero());
    for (size_t i = 1; i < n; ++i) {
        const uint16_t p = tree.nodes[i].parent_idx;
        if (p >= n) continue;
        t_up[i] = tree.nodes[i].attach_local - tree.nodes[p].attach_local;
    }

    // Per-call constant: each link's spatial inertia in its own body
    // frame. com_in_body = com_local[i] − attach_local[i]; I_c is
    // diagonal (principal-axis approximation).
    static thread_local std::vector<SpatialInertia> M_link;
    M_link.assign(n, SpatialInertia::Zero());
    for (size_t i = 0; i < n; ++i) {
        JPH::Vec3 com_in_body = tree.nodes[i].com_local - tree.nodes[i].attach_local;
        M_link[i] = SpatialInertia::FromDiagonal(
            tree.nodes[i].mass, com_in_body, tree.nodes[i].inertia_diag);
    }

    // Per-substep scratch.
    static thread_local std::vector<JPH::Quat>     R_v;
    static thread_local std::vector<JPH::Vec3>     pos_v;        // body i's frame origin in vessel frame
    static thread_local std::vector<SpatialMotion> v_body;
    static thread_local std::vector<SpatialMotion> c_body;
    static thread_local std::vector<SpatialMatrix6> IA;
    static thread_local std::vector<SpatialForce>   pA;
    static thread_local std::vector<JPH::Vec3>      tau_body;    // joint spring/damper torque in body frame
    static thread_local std::vector<SpatialMotion>  a_body_sp;   // spatial acceleration

    R_v.assign(n, JPH::Quat::sIdentity());
    pos_v.assign(n, JPH::Vec3::sZero());
    v_body.assign(n, SpatialMotion::Zero());
    c_body.assign(n, SpatialMotion::Zero());
    IA.assign(n, SpatialMatrix6::Zero());
    pA.assign(n, SpatialForce::Zero());
    tau_body.assign(n, JPH::Vec3::sZero());
    a_body_sp.assign(n, SpatialMotion::Zero());

    // ---- Substep loop ---------------------------------------------------

    for (int sub = 0; sub < kAbaSubsteps; ++sub) {

        // Pass 1: outward kinematics (root → leaf). Parents are always
        // before children in the topology order (parent_idx < i), so a
        // forward sweep is correct.
        R_v[0]   = JPH::Quat::sIdentity();
        pos_v[0] = tree.nodes[0].attach_local;       // = (0,0,0) by convention
        v_body[0] = SpatialMotion::Zero();           // fixed-base flex frame
        c_body[0] = SpatialMotion::Zero();

        for (size_t i = 1; i < n; ++i) {
            const uint16_t p = tree.nodes[i].parent_idx;
            if (p >= n) continue;

            // X_up: parent body frame → body i's frame.
            SpatialTransform X_up{q_joint[i], t_up[i]};

            // v_body[i] = X_up · v_body[p] + S × ω_joint[i]
            // For spherical, S × ω_joint = SpatialMotion(ω_joint_body, 0).
            SpatialMotion v_par_in_body = X_up.TransformMotion(v_body[p]);
            SpatialMotion v_joint{omega_joint[i], JPH::Vec3::sZero()};
            v_body[i] = v_par_in_body + v_joint;

            // c_i = v_body[i] ×̂ (S × ω_joint_body)
            c_body[i] = CrossMotion(v_body[i], v_joint);

            // Cumulative vessel-frame rotation + position.
            R_v[i]   = R_v[p] * q_joint[i];
            pos_v[i] = pos_v[p] + R_v[p] * t_up[i];
        }

        // Pass 1b: build articulated-inertia + bias force at each link
        // in body frame. Spring + damper joint torque computed here so
        // it's available in pass 2 and pass 3 reads.
        for (size_t i = 0; i < n; ++i) {
            // Initialize from rigid-body inertia.
            IA[i] = SpatialMatrix6::FromInertia(M_link[i]);

            // Bias: v ×̂* (M·v) − fext_body
            //
            // External wrench rotation:
            //   F_body         = R_v[i]⁻¹ · F_flex[i]   (vessel-frame F at part CoM)
            //   T_at_com_body  = R_v[i]⁻¹ · T_flex[i]   (vessel-frame T about part CoM)
            //   n_at_origin_body = T_at_com_body + com_in_body × F_body
            //                       (translate moment from CoM to body-frame origin)
            JPH::Vec3 com_in_body = tree.nodes[i].com_local - tree.nodes[i].attach_local;
            JPH::Vec3 F_body = RotateByInv(R_v[i], F_flex[i]);
            JPH::Vec3 T_at_com_body = RotateByInv(R_v[i], T_flex[i]);
            JPH::Vec3 n_at_origin_body = T_at_com_body + com_in_body.Cross(F_body);
            SpatialForce fext_body{n_at_origin_body, F_body};

            SpatialForce Iv = M_link[i].Mul(v_body[i]);
            SpatialForce gyro = CrossForceDual(v_body[i], Iv);
            pA[i] = gyro - fext_body;

            // Joint spring + damper torque (only for non-root parts).
            if (i > 0) {
                // axisAngle(q_joint) and ω_joint are both in body i's
                // frame (the axis of q_joint is invariant under q;
                // ω_joint stored in body frame by construction).
                JPH::Vec3 axis_angle = QuatToAxisAngle(q_joint[i]);
                const EdgeCompliance& c = tree.edge_compliance[i];
                tau_body[i] = -c.k_ang * axis_angle - c.c_ang * omega_joint[i];
            } else {
                tau_body[i] = JPH::Vec3::sZero();
            }
        }

        // Pass 2: inward articulated-inertia (leaf → root).
        // For each non-root link i with parent p:
        //   D     = IA.a (3×3 angular block)         — S^T · IA · S
        //   u     = -pA.angular + tau_body            — generalized force at joint
        //   IA_a  = IA - U · D⁻¹ · U^T (rank-3)       — joint-reduced articulated inertia
        //   pA_a  = pA + IA_a · c + U · D⁻¹ · u       — joint-reduced bias
        //   accumulate IA_a, pA_a into parent (transformed to parent's frame).
        //
        // Iterate from highest index down; parent_idx < i guarantees
        // parent is processed after this child.
        static thread_local std::vector<JPH::Vec3> u_joint;
        static thread_local std::vector<JPH::Vec3> Dinv_u_joint;
        static thread_local std::vector<Mat33>     D_inv_joint;
        u_joint.assign(n, JPH::Vec3::sZero());
        Dinv_u_joint.assign(n, JPH::Vec3::sZero());
        D_inv_joint.assign(n, Mat33::Identity());

        for (size_t ii = n; ii-- > 1; ) {
            size_t i = ii;
            const uint16_t p = tree.nodes[i].parent_idx;
            if (p >= n) continue;

            Mat33 D = IA[i].a;
            Mat33 D_inv;
            if (!Inverse(D, &D_inv)) {
                // Degenerate angular-inertia block — skip the joint
                // reduction; transmit IA / pA up the chain unchanged.
                IA[p] = IA[p] + IA[i].InverseTransform(SpatialTransform{q_joint[i], t_up[i]});
                pA[p] = pA[p] + SpatialTransform{q_joint[i], t_up[i]}.InverseTransformForce(pA[i]);
                continue;
            }
            D_inv_joint[i] = D_inv;

            // u = -S^T · pA + tau_body = -pA.angular + tau_body
            JPH::Vec3 u = -pA[i].angular + tau_body[i];
            u_joint[i] = u;
            JPH::Vec3 Dinv_u = D_inv * u;
            Dinv_u_joint[i] = Dinv_u;

            // Rank-3 reduction: IA_a = IA - U · D⁻¹ · U^T.
            SpatialMatrix6 IA_a = SubRankSpherical(IA[i]);

            // pA_a = pA + IA_a · c + U · D⁻¹ · u, where U·D⁻¹·u is a
            // SpatialForce with components (IA.a · D⁻¹·u, IA.c · D⁻¹·u).
            SpatialForce U_Dinv_u{IA[i].a * Dinv_u, IA[i].c * Dinv_u};
            SpatialForce pA_a = pA[i] + IA_a.Mul(c_body[i]) + U_Dinv_u;

            // Transform IA_a, pA_a from body i's frame to parent body
            // frame, accumulate.
            SpatialTransform X_up{q_joint[i], t_up[i]};
            IA[p] = IA[p] + IA_a.InverseTransform(X_up);
            pA[p] = pA[p] + X_up.InverseTransformForce(pA_a);
        }

        // Pass 3: outward acceleration solve (root → leaf).
        // Root is fixed-base in flex frame: a_body_sp[0] = 0.
        a_body_sp[0] = SpatialMotion::Zero();

        for (size_t i = 1; i < n; ++i) {
            const uint16_t p = tree.nodes[i].parent_idx;
            if (p >= n) continue;

            // aProp = X_up · a_parent + c_i  (in body i's frame).
            SpatialTransform X_up{q_joint[i], t_up[i]};
            SpatialMotion aProp = X_up.TransformMotion(a_body_sp[p]) + c_body[i];

            // qddot = D⁻¹ · (u - S^T · IA · aProp)
            //       = D⁻¹ · (u - (IA · aProp).angular)
            SpatialForce IA_aProp = IA[i].Mul(aProp);
            JPH::Vec3 rhs = u_joint[i] - IA_aProp.angular;
            JPH::Vec3 qddot_i = D_inv_joint[i] * rhs;

            // a_body[i] = aProp + S × qddot_i (= aProp with qddot added to angular).
            a_body_sp[i] = aProp + SpatialMotion{qddot_i, JPH::Vec3::sZero()};

            // Integrate joint state — semi-implicit Euler.
            //   ω_joint += dt × qddot
            //   q_joint  ← q_joint × (1, ω_joint·dt/2) (right-multiply)
            omega_joint[i] = omega_joint[i] + qddot_i * dt;
            q_joint[i] = IntegrateRotationRight(q_joint[i], omega_joint[i], dt);
        }
    } // substep loop

    // ---- Compose vessel-frame outputs (delta_rot, delta_pos) ----------
    // Walk root → leaf one final time using the post-substep joint
    // state. Write results directly into PartFlex for downstream
    // consumers (RouteContactForce, RNEA, ApplyFlexToBodies).

    R_v[0]   = JPH::Quat::sIdentity();
    pos_v[0] = tree.nodes[0].attach_local;     // (0,0,0) by convention

    tree.flex[0].delta_pos = JPH::Vec3::sZero();
    tree.flex[0].delta_rot = JPH::Quat::sIdentity();
    tree.flex[0].lin_vel   = JPH::Vec3::sZero();
    tree.flex[0].ang_vel   = JPH::Vec3::sZero();

    for (size_t i = 1; i < n; ++i) {
        const uint16_t p = tree.nodes[i].parent_idx;
        if (p >= n) continue;
        R_v[i]   = R_v[p] * q_joint[i];
        pos_v[i] = pos_v[p] + R_v[p] * t_up[i];

        // Vessel-frame CoM under current configuration:
        //   com_v[i] = pos_v[i] + R_v[i] · (com_local[i] - attach_local[i])
        JPH::Vec3 com_in_body = tree.nodes[i].com_local - tree.nodes[i].attach_local;
        JPH::Vec3 com_v = pos_v[i] + R_v[i] * com_in_body;

        tree.flex[i].delta_pos = com_v - tree.nodes[i].com_local;
        tree.flex[i].delta_rot = R_v[i];
        tree.flex[i].ang_vel   = omega_joint[i];          // body-frame ω_joint (state for next tick)
        tree.flex[i].lin_vel   = JPH::Vec3::sZero();      // unused under spherical
    }

    // ---- Safety clamp (unchanged in spirit) ---------------------------
    // Catch numerical blowups before they propagate to KSP's orbit
    // reset. Per-part state reset to rest if any field exceeds bounds.
    constexpr float kMaxLinearFlexM = 1.0f;
    constexpr float kMaxAngVelRad   = 50.0f;
    static int s_clamp_log_count = 0;
    for (size_t i = 1; i < n; ++i) {
        PartFlex& f = tree.flex[i];
        const float pos_mag = f.delta_pos.Length();
        const float av_mag  = f.ang_vel.Length();
        if (pos_mag > kMaxLinearFlexM || av_mag > kMaxAngVelRad
            || !std::isfinite(pos_mag) || !std::isfinite(av_mag))
        {
            if (s_clamp_log_count < 8) {
                s_clamp_log_count++;
                JPH::Trace("[aba/clamp] part_idx=%zu reset: |dp|=%.2f |av|=%.2f",
                           i, pos_mag, av_mag);
            }
            f.delta_pos = JPH::Vec3::sZero();
            f.delta_rot = JPH::Quat::sIdentity();
            f.lin_vel   = JPH::Vec3::sZero();
            f.ang_vel   = JPH::Vec3::sZero();
        }
    }
}

} // namespace longeron
