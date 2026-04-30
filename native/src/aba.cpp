#include "aba.h"

#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>
#include <Jolt/Math/Quat.h>

#include <algorithm>
#include <cmath>

namespace longeron {

namespace {

// Convert a small-angle rotation quaternion to its axis-angle vector
// (axis × angle in radians). For unit quaternion q = (xyz, w),
// axis-angle ≈ 2 × xyz / w for small angles. We clamp via the sign of
// w to handle the double-cover (q and -q represent the same rotation).
JPH::Vec3 QuatToAxisAngle(JPH::Quat q) {
    // Ensure scalar part is non-negative; otherwise -q is the smaller
    // rotation in the cover.
    if (q.GetW() < 0.0f) q = JPH::Quat(-q.GetX(), -q.GetY(), -q.GetZ(), -q.GetW());
    JPH::Vec3 v(q.GetX(), q.GetY(), q.GetZ());
    float w = q.GetW();
    // For w near 1, sin(θ/2) ≈ θ/2 → axis-angle ≈ 2 × v.
    // For larger angles, axis-angle = 2 × atan2(|v|, w) × v / |v|.
    float v_len = v.Length();
    if (v_len < 1.0e-6f) return JPH::Vec3::sZero();
    float angle = 2.0f * std::atan2(v_len, w);
    return v * (angle / v_len);
}

// Integrate a quaternion forward by the given angular velocity over dt.
// First-order: q_{n+1} = normalize(q_n + dt/2 · ω · q_n)
// where ω is treated as a pure quaternion (0, ωxyz).
JPH::Quat IntegrateRotation(JPH::Quat q, JPH::Vec3 omega, float dt) {
    JPH::Quat dq(0.5f * omega.GetX() * dt,
                 0.5f * omega.GetY() * dt,
                 0.5f * omega.GetZ() * dt,
                 0.0f);
    JPH::Quat q_dot = dq * q;
    JPH::Quat q_new(q.GetX() + q_dot.GetX(),
                    q.GetY() + q_dot.GetY(),
                    q.GetZ() + q_dot.GetZ(),
                    q.GetW() + q_dot.GetW());
    return q_new.Normalized();
}

// Multiply per-axis inertia (diagonal in vessel-frame approximation)
// by an angular vector, giving the angular momentum / inertia-velocity.
JPH::Vec3 InertiaTimes(JPH::Vec3 I_diag, JPH::Vec3 v) {
    return JPH::Vec3(I_diag.GetX() * v.GetX(),
                     I_diag.GetY() * v.GetY(),
                     I_diag.GetZ() * v.GetZ());
}

// Element-wise inertia inverse times vector (for the integration step).
// Guards against zero inertia by treating inverse as zero (no rotation
// from torque if inertia is degenerate — rare but possible for tiny
// massless parts that escape filtering on the C# side).
JPH::Vec3 InertiaInverseTimes(JPH::Vec3 I_diag, JPH::Vec3 v) {
    auto inv = [](float i) { return (i > 1.0e-9f) ? (1.0f / i) : 0.0f; };
    return JPH::Vec3(inv(I_diag.GetX()) * v.GetX(),
                     inv(I_diag.GetY()) * v.GetY(),
                     inv(I_diag.GetZ()) * v.GetZ());
}

} // namespace

void RunAbaForwardStep(
    VesselTree& tree,
    JPH::Vec3 v_body,
    JPH::Vec3 omega,
    JPH::Vec3 a_body,
    JPH::Vec3 alpha,
    float fixed_dt)
{
    (void)v_body; // not used for the flex residual — only a_body matters
    const size_t n = tree.nodes.size();
    if (n < 2) return; // single-part vessel: no flex possible

    // Substep dt — semi-implicit Euler is stable when dt < 2/ω_n. With
    // our default ~30× stock stiffness, ω_n is typically ≤ 100 rad/s
    // → dt < 20 ms ≈ one full Jolt tick. Substep 10× for headroom.
    const float dt = fixed_dt / static_cast<float>(kAbaSubsteps);

    // Per-part inertial residual force (vessel-body axes). This is the
    // share of each part's accumulated external force that's used by
    // Jolt's rigid-body integration of the vessel CoM motion. ABA only
    // sees what's left over.
    //
    //   F_inertial_i = m_i · (a_body + α × r_i + ω × (ω × r_i))
    //   T_inertial_i = I_i · α + ω × (I_i · ω)
    //   F_flex_i = F_ext_i - F_inertial_i
    //   T_flex_i = T_ext_i - T_inertial_i
    //
    // Held constant across substeps (KSP forces only update at 50 Hz).
    std::vector<JPH::Vec3> F_flex(n);
    std::vector<JPH::Vec3> T_flex(n);
    for (size_t i = 0; i < n; ++i) {
        const PartNode& p = tree.nodes[i];
        // Use the deflected r vector so the inertial residual matches
        // what Jolt actually integrated for this part this tick.
        const JPH::Vec3 r = p.com_local + tree.flex[i].delta_pos;

        const JPH::Vec3 a_part = a_body + alpha.Cross(r) + omega.Cross(omega.Cross(r));
        const JPH::Vec3 F_inertial = p.mass * a_part;
        const JPH::Vec3 T_inertial = InertiaTimes(p.inertia_diag, alpha)
                                     + omega.Cross(InertiaTimes(p.inertia_diag, omega));

        F_flex[i] = tree.ext_force[i]  - F_inertial;
        T_flex[i] = tree.ext_torque[i] - T_inertial;
    }

    // Substep loop — semi-implicit (symplectic) Euler:
    //   compute all forces + torques (joints + external residual)
    //   v_lin += dt · F / m
    //   ω     += dt · I⁻¹ · (T - ω × Iω)        // Newton-Euler torque
    //   x     += dt · v_lin (using the JUST-updated v_lin)
    //   q     += dt · 0.5 · ω · q (using the JUST-updated ω)
    //
    // For each part we accumulate joint contributions from BOTH sides
    // of the joint (i.e., child receives spring force, parent receives
    // -spring force). To avoid double-iterating, walk by edge once and
    // apply ±F to (child, parent).
    std::vector<JPH::Vec3> total_F(n);
    std::vector<JPH::Vec3> total_T(n);

    for (int sub = 0; sub < kAbaSubsteps; ++sub) {
        // Reset per-part accumulators with the constant flex residual.
        for (size_t i = 0; i < n; ++i) {
            total_F[i] = F_flex[i];
            total_T[i] = T_flex[i];
        }

        // Walk every non-root edge — accumulate joint spring/damper.
        for (size_t i = 1; i < n; ++i) {
            const uint16_t parent = tree.nodes[i].parent_idx;
            if (parent >= n) continue; // safety: malformed tree

            const PartNode& child_node  = tree.nodes[i];
            const PartNode& parent_node = tree.nodes[parent];
            const PartFlex& child_flex  = tree.flex[i];
            const PartFlex& parent_flex = tree.flex[parent];
            const EdgeCompliance& c     = tree.edge_compliance[i];

            // Joint anchor on child side, in vessel frame.
            // Rest position of anchor = attach_local (same point on
            // both child and parent at rest by construction). Child's
            // current pose: position = com_local + delta_pos,
            // orientation = delta_rot. The anchor sits at
            //   (attach_local - com_local) in child's local frame
            // → world = child_position + child_rotation × that offset.
            const JPH::Vec3 child_anchor_offset =
                child_flex.delta_rot * (child_node.attach_local - child_node.com_local);
            const JPH::Vec3 child_anchor =
                child_node.com_local + child_flex.delta_pos + child_anchor_offset;

            const JPH::Vec3 parent_anchor_offset =
                parent_flex.delta_rot * (child_node.attach_local - parent_node.com_local);
            const JPH::Vec3 parent_anchor =
                parent_node.com_local + parent_flex.delta_pos + parent_anchor_offset;

            // Linear displacement: where child's anchor IS minus where
            // parent says it SHOULD be. Spring pulls child back to the
            // parent-relative anchor position.
            const JPH::Vec3 disp = child_anchor - parent_anchor;

            // Linear velocity at the anchor (vessel frame):
            //   v_anchor_child  = v_child_com + ω_child × child_anchor_offset
            //   v_anchor_parent = v_parent_com + ω_parent × parent_anchor_offset
            const JPH::Vec3 v_anchor_child =
                child_flex.lin_vel + child_flex.ang_vel.Cross(child_anchor_offset);
            const JPH::Vec3 v_anchor_parent =
                parent_flex.lin_vel + parent_flex.ang_vel.Cross(parent_anchor_offset);
            const JPH::Vec3 v_rel = v_anchor_child - v_anchor_parent;

            // Linear spring + damper force on the child, applied at
            // child_anchor. Equal+opposite on the parent at parent_anchor.
            const JPH::Vec3 F_spring = -c.k_lin * disp - c.c_lin * v_rel;

            // Angular displacement: relative orientation of child WRT
            // parent. At rest both are identity → relative is identity.
            // Under flex: relative = parent_rot⁻¹ × child_rot.
            const JPH::Quat rel_rot = parent_flex.delta_rot.Conjugated() * child_flex.delta_rot;
            const JPH::Vec3 rot_err = QuatToAxisAngle(rel_rot);
            const JPH::Vec3 omega_rel = child_flex.ang_vel - parent_flex.ang_vel;
            // Note: we're approximating that K_ang is in vessel frame
            // and the relative rotation is "small enough" that the
            // axis-angle vector is the right thing to pull on. For
            // 5-15° flex this is fine; for larger we'd need the
            // exponential map proper.
            const JPH::Vec3 T_spring = -c.k_ang * rot_err - c.c_ang * omega_rel;

            // Apply to child:
            //   F_child = F_spring (linear, at child_anchor)
            //   T_child = T_spring (angular)
            //          + child_anchor_offset × F_spring   (lever arm of
            //                                               linear force
            //                                               about CoM)
            total_F[i] = total_F[i] + F_spring;
            total_T[i] = total_T[i] + T_spring + child_anchor_offset.Cross(F_spring);

            // And equal+opposite to parent:
            total_F[parent] = total_F[parent] - F_spring;
            total_T[parent] = total_T[parent] - T_spring - parent_anchor_offset.Cross(F_spring);
        }

        // Integrate per-part state. Root (i=0) stays at zero by
        // convention — it IS the vessel body's anchor, owned by Jolt.
        for (size_t i = 1; i < n; ++i) {
            const PartNode& p = tree.nodes[i];
            PartFlex& f = tree.flex[i];

            if (p.mass <= 1.0e-9f) continue; // massless part: skip integration

            // Linear: v_lin += dt · F / m, then x += dt · v_lin
            const JPH::Vec3 a_lin = total_F[i] * (1.0f / p.mass);
            f.lin_vel = f.lin_vel + a_lin * dt;
            f.delta_pos = f.delta_pos + f.lin_vel * dt;

            // Angular: τ_eff = T - ω × (I·ω); α = I⁻¹ · τ_eff
            const JPH::Vec3 I_omega = InertiaTimes(p.inertia_diag, f.ang_vel);
            const JPH::Vec3 tau_eff = total_T[i] - f.ang_vel.Cross(I_omega);
            const JPH::Vec3 alpha_part = InertiaInverseTimes(p.inertia_diag, tau_eff);
            f.ang_vel = f.ang_vel + alpha_part * dt;
            f.delta_rot = IntegrateRotation(f.delta_rot, f.ang_vel, dt);
        }
    }

    // Phase 5.0 safety clamp — if flex blew up beyond reasonable bounds
    // (joint anchor moved > 1 m, or angular velocity > 50 rad/s), log
    // once per vessel and reset that part's flex to rest. This catches
    // numerical instability before it propagates to KSP's orbit reset.
    // TODO (Phase 5.x): remove once implicit-Euler integration is in
    // place and we trust stability fully.
    constexpr float kMaxLinearFlexM   = 1.0f;
    constexpr float kMaxAngVelRad     = 50.0f;
    constexpr float kMaxLinVelMs      = 100.0f;
    static int s_clamp_log_count = 0;
    for (size_t i = 1; i < n; ++i) {
        PartFlex& f = tree.flex[i];
        const float pos_mag = f.delta_pos.Length();
        const float lv_mag  = f.lin_vel.Length();
        const float av_mag  = f.ang_vel.Length();
        if (pos_mag > kMaxLinearFlexM || lv_mag > kMaxLinVelMs || av_mag > kMaxAngVelRad
            || !std::isfinite(pos_mag) || !std::isfinite(lv_mag) || !std::isfinite(av_mag))
        {
            if (s_clamp_log_count < 8) {
                s_clamp_log_count++;
                JPH::Trace("[aba/clamp] part_idx=%zu reset: |dp|=%.2f |lv|=%.2f |av|=%.2f",
                           i, pos_mag, lv_mag, av_mag);
            }
            f.delta_pos = JPH::Vec3::sZero();
            f.delta_rot = JPH::Quat::sIdentity();
            f.lin_vel   = JPH::Vec3::sZero();
            f.ang_vel   = JPH::Vec3::sZero();
        }
    }
}

} // namespace longeron
