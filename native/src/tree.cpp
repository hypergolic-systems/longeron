#include "tree.h"
#include "records.h"

#include <Jolt/Jolt.h>
#include <Jolt/Core/Core.h>
#include <Jolt/Math/Vec3.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyLockInterface.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <algorithm>
#include <cmath>

namespace longeron {

void TreeRegistry::Upsert(uint32_t body_id, std::vector<PartNode>&& nodes) {
    if (nodes.empty()) {
        mTrees.erase(body_id);
        return;
    }
    VesselTree& t = mTrees[body_id];
    t.body_id = body_id;
    t.nodes   = std::move(nodes);
    // Re-size accumulator to match the new node count and zero it.
    // Topology change implies a fresh accounting window.
    t.ext_force.assign(t.nodes.size(), JPH::Vec3::sZero());
    t.ext_torque.assign(t.nodes.size(), JPH::Vec3::sZero());
}

void TreeRegistry::Erase(uint32_t body_id) {
    mTrees.erase(body_id);
}

void TreeRegistry::AccumulateExternalForce(
    uint32_t body_id, uint16_t part_idx,
    JPH::Vec3 force_world, JPH::RVec3 point_world,
    JPH::RVec3 body_com_world, JPH::Quat body_rot)
{
    auto it = mTrees.find(body_id);
    if (it == mTrees.end()) return;
    VesselTree& t = it->second;
    if (part_idx >= t.nodes.size()) return;

    // Transform the world force + application point into body-local
    // axes. Forces are float-precision; positions cross the double→
    // float boundary safely once we've subtracted the body CoM (the
    // result is small — meters within the vessel).
    JPH::Quat body_rot_inv = body_rot.Conjugated();
    JPH::Vec3 force_local  = body_rot_inv * force_world;
    JPH::RVec3 P_rel       = point_world - body_com_world;
    JPH::Vec3 P_rel_f(
        static_cast<float>(P_rel.GetX()),
        static_cast<float>(P_rel.GetY()),
        static_cast<float>(P_rel.GetZ()));
    JPH::Vec3 P_local = body_rot_inv * P_rel_f;

    // Lever from part CoM to the application point.
    JPH::Vec3 lever_local = P_local - t.nodes[part_idx].com_local;

    t.ext_force[part_idx]  = t.ext_force[part_idx]  + force_local;
    t.ext_torque[part_idx] = t.ext_torque[part_idx] + lever_local.Cross(force_local);
}

namespace {

// Compute the CoM-frame inertial wrench required to drag a part with
// given mass and inertia through the vessel's rigid motion.
//
// Inputs in vessel body axes:
//   r        — part CoM offset from vessel reference point
//   m        — part mass (tonnes)
//   I_diag   — principal inertia (diagonal approximation) at part CoM
//   v_body   — vessel linear velocity at reference point
//   omega    — vessel angular velocity
//   a_body   — vessel linear acceleration at reference point
//   alpha    — vessel angular acceleration
//
// Outputs (force/torque "absorbed" by this part — what its joints have
// to transmit if there are no external forces):
//   F_p = m * (a_body + alpha × r + omega × (omega × r))
//   τ_p = I·α + ω × (I·ω)        (computed at part CoM)
struct PartInertialWrench {
    JPH::Vec3 force;     // at part CoM, body axes
    JPH::Vec3 torque;    // at part CoM, body axes
};

PartInertialWrench InertialWrench(
    JPH::Vec3 r, float m, JPH::Vec3 I_diag,
    JPH::Vec3 v_body, JPH::Vec3 omega,
    JPH::Vec3 a_body, JPH::Vec3 alpha)
{
    (void)v_body; // velocities don't enter the rigid-body force balance directly
    JPH::Vec3 a_part = a_body + alpha.Cross(r) + omega.Cross(omega.Cross(r));
    PartInertialWrench w;
    w.force  = m * a_part;
    JPH::Vec3 I_omega(I_diag.GetX() * omega.GetX(),
                      I_diag.GetY() * omega.GetY(),
                      I_diag.GetZ() * omega.GetZ());
    JPH::Vec3 I_alpha(I_diag.GetX() * alpha.GetX(),
                      I_diag.GetY() * alpha.GetY(),
                      I_diag.GetZ() * alpha.GetZ());
    w.torque = I_alpha + omega.Cross(I_omega);
    return w;
}

// Translate a wrench (F, τ) from the part's CoM to the joint anchor
// (lever-arm parallel-axis term). Both points in body axes.
//   τ_at_attach = τ_at_CoM + (com - attach) × F
JPH::Vec3 TranslateTorque(JPH::Vec3 torque_at_com, JPH::Vec3 com, JPH::Vec3 attach, JPH::Vec3 force) {
    JPH::Vec3 lever = com - attach;
    return torque_at_com + lever.Cross(force);
}

} // namespace

bool TreeRegistry::RunAdvisoryPass(
    const JPH::PhysicsSystem& system,
    const std::unordered_map<uint32_t, JPH::BodyID>& registry,
    uint64_t step_count,
    float fixed_dt)
{
    if (mTrees.empty()) return false;

    // Rate-limit emission. KSP runs at 50 Hz; emit every ~1s = 50 steps.
    // Per-tick math still runs (cheap relative to Jolt) — well, actually
    // we currently only run when emitting; integrating per-tick to track
    // peak transient stress is Phase 4.1.
    bool should_emit = (step_count - mLastEmitStep) >= 50;
    if (!should_emit) return false;
    // Window between samples: number of ticks since the previous emit
    // × fixed_dt. The finite-difference acceleration must use this
    // window, not the per-tick fixed_dt — sampling once per ~50 ticks
    // and dividing by 0.02 s would give a 50× overestimate. On the
    // very first emit the window is the full step_count up to here.
    const uint64_t prev_emit_step = mLastEmitStep;
    const float window_dt = (step_count > prev_emit_step)
        ? static_cast<float>(step_count - prev_emit_step) * fixed_dt
        : fixed_dt;
    mLastEmitStep = step_count;
    mLastSummaries.clear();

    // Track previous-tick velocity per body to estimate acceleration via
    // finite difference. Persisting state across calls keeps Step()
    // signature unchanged. Map keyed by body_id; lazily populated.
    static std::unordered_map<uint32_t, JPH::Vec3> s_prev_lin_v;
    static std::unordered_map<uint32_t, JPH::Vec3> s_prev_ang_v;

    const JPH::BodyLockInterfaceNoLock& lock_iface = system.GetBodyLockInterfaceNoLock();
    const float inv_dt = (window_dt > 1e-6f) ? (1.0f / window_dt) : 0.0f;

    for (auto& [body_id, tree] : mTrees) {
        auto it = registry.find(body_id);
        if (it == registry.end()) continue;
        JPH::BodyID jph_id = it->second;
        JPH::BodyLockRead lock(lock_iface, jph_id);
        if (!lock.Succeeded()) continue;
        const JPH::Body& body = lock.GetBody();

        JPH::Vec3 v_body = body.GetLinearVelocity();
        JPH::Vec3 omega  = body.GetAngularVelocity();

        // Acceleration estimate from finite difference. Crude — for
        // advisory log it's fine; Phase 4.1 can read accumulated forces
        // directly to avoid the 1-tick lag.
        JPH::Vec3 a_body = JPH::Vec3::sZero();
        JPH::Vec3 alpha  = JPH::Vec3::sZero();
        auto pv = s_prev_lin_v.find(body_id);
        auto pa = s_prev_ang_v.find(body_id);
        if (pv != s_prev_lin_v.end() && pa != s_prev_ang_v.end()) {
            a_body = (v_body - pv->second) * inv_dt;
            alpha  = (omega  - pa->second) * inv_dt;
        }
        s_prev_lin_v[body_id] = v_body;
        s_prev_ang_v[body_id] = omega;

        if (tree.nodes.size() < 2) {
            // Single-part vessels have no joints; clear the accumulator
            // to avoid stale carry-over and skip.
            std::fill(tree.ext_force.begin(),  tree.ext_force.end(),  JPH::Vec3::sZero());
            std::fill(tree.ext_torque.begin(), tree.ext_torque.end(), JPH::Vec3::sZero());
            continue;
        }

        // Per-part NET wrench: inertial requirement minus the average
        // external wrench applied across the window. F_self[i] = m × a_part
        // − F_ext_avg(i). Hooke's law for the joint: this is what the
        // joints have to transmit to keep the part following the
        // observed motion given the externals we know about. With
        // gravity + thrust + drag all routed via Harmony into our
        // per-part accumulator, F_self should approach zero on a
        // cruising stable vessel and grow on a tumbling / impacting one.
        const uint64_t window_ticks = (step_count > prev_emit_step)
            ? (step_count - prev_emit_step) : 1;
        const float inv_ticks = 1.0f / static_cast<float>(window_ticks);

        std::vector<JPH::Vec3> F_self(tree.nodes.size(), JPH::Vec3::sZero());
        std::vector<JPH::Vec3> T_self(tree.nodes.size(), JPH::Vec3::sZero());
        for (size_t i = 0; i < tree.nodes.size(); ++i) {
            const PartNode& n = tree.nodes[i];
            PartInertialWrench w = InertialWrench(
                n.com_local, n.mass, n.inertia_diag,
                v_body, omega, a_body, alpha);
            // Average external wrench over the window:
            JPH::Vec3 ext_F_avg = tree.ext_force[i]  * inv_ticks;
            JPH::Vec3 ext_T_avg = tree.ext_torque[i] * inv_ticks;
            F_self[i] = w.force  - ext_F_avg;
            T_self[i] = w.torque - ext_T_avg;
        }
        // Drain accumulator for the next window.
        std::fill(tree.ext_force.begin(),  tree.ext_force.end(),  JPH::Vec3::sZero());
        std::fill(tree.ext_torque.begin(), tree.ext_torque.end(), JPH::Vec3::sZero());

        // Backward (leaf → root) accumulation. Joint wrench at part i =
        // sum of (force, torque-translated-to-parent's-attach) over the
        // subtree rooted at i. Iteration from highest index to lowest
        // works since parent_idx < i for all i.
        std::vector<JPH::Vec3> F_subtree = F_self;
        std::vector<JPH::Vec3> T_subtree(tree.nodes.size());
        for (size_t i = 0; i < tree.nodes.size(); ++i) {
            // Torque about the part's own attach point (joint location)
            T_subtree[i] = TranslateTorque(
                T_self[i], tree.nodes[i].com_local, tree.nodes[i].attach_local, F_self[i]);
        }
        for (size_t i = tree.nodes.size(); i-- > 0; ) {
            uint16_t parent = tree.nodes[i].parent_idx;
            if (parent == kInvalidPartIdx) continue;
            // Bubble this subtree's wrench up to its parent's attach
            // point. Translate torque from this part's attach to the
            // parent's attach via the lever (this.attach - parent.attach).
            JPH::Vec3 lever = tree.nodes[i].attach_local - tree.nodes[parent].attach_local;
            F_subtree[parent] = F_subtree[parent] + F_subtree[i];
            T_subtree[parent] = T_subtree[parent] + T_subtree[i] + lever.Cross(F_subtree[i]);
        }

        // Per-edge max + sum for a compact summary. Per-edge wrench is
        // too noisy for a 1Hz log; the C# side can probe specific
        // edges later if we want them.
        float max_F = 0.0f, max_T = 0.0f, sum_F = 0.0f, sum_T = 0.0f;
        uint16_t max_F_idx = 0, max_T_idx = 0;
        for (uint16_t i = 0; i < tree.nodes.size(); ++i) {
            if (tree.nodes[i].parent_idx == kInvalidPartIdx) continue;
            float Fmag = F_subtree[i].Length();
            float Tmag = T_subtree[i].Length();
            sum_F += Fmag;
            sum_T += Tmag;
            if (Fmag > max_F) { max_F = Fmag; max_F_idx = i; }
            if (Tmag > max_T) { max_T = Tmag; max_T_idx = i; }
        }

        RneaSummary s;
        s.body_id    = body_id;
        s.part_count = static_cast<uint16_t>(tree.nodes.size());
        s.max_F      = max_F;
        s.max_F_idx  = max_F_idx;
        s.max_T      = max_T;
        s.max_T_idx  = max_T_idx;
        s.sum_F      = sum_F;
        s.sum_T      = sum_T;
        s.accel_mag  = a_body.Length();
        s.alpha_mag  = alpha.Length();
        mLastSummaries.push_back(s);
    }
    return true;
}

} // namespace longeron
