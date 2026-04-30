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
#include <limits>

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

void TreeRegistry::RouteContactForce(
    uint32_t body_id,
    JPH::Vec3 force_world, JPH::RVec3 point_world,
    JPH::RVec3 body_com_world, JPH::Quat body_rot)
{
    auto it = mTrees.find(body_id);
    if (it == mTrees.end()) return;
    const VesselTree& t = it->second;
    if (t.nodes.empty()) return;

    // Project the contact point into body-local frame for the per-part
    // proximity scan.
    JPH::Quat body_rot_inv = body_rot.Conjugated();
    JPH::RVec3 P_rel = point_world - body_com_world;
    JPH::Vec3 P_local(
        static_cast<float>(P_rel.GetX()),
        static_cast<float>(P_rel.GetY()),
        static_cast<float>(P_rel.GetZ()));
    P_local = body_rot_inv * P_local;

    // Closest part by CoM. Linear scan; vessels typically have ≤30
    // parts so this is cheap. See header TODO for the O(1) replacement.
    uint16_t best_idx = 0;
    float best_dist2 = std::numeric_limits<float>::infinity();
    for (uint16_t i = 0; i < t.nodes.size(); ++i) {
        JPH::Vec3 d = P_local - t.nodes[i].com_local;
        float d2 = d.LengthSq();
        if (d2 < best_dist2) {
            best_dist2 = d2;
            best_idx = i;
        }
    }

    AccumulateExternalForce(body_id, best_idx, force_world, point_world,
                             body_com_world, body_rot);
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
    mLastEdgeWrenches.clear();
    if (mTrees.empty()) return false;

    // Per-edge wrenches go out every tick so PartModules see fresh
    // joint stresses without 1-second cadence lag. The compact
    // RneaSummary log is still throttled.
    const bool emit_summary = (step_count - mLastEmitStep) >= 50;
    if (emit_summary) {
        mLastEmitStep = step_count;
        mLastSummaries.clear();
    }

    // Track previous-tick velocity per body to estimate per-tick
    // acceleration via finite difference. Map keyed by body_id;
    // updated every tick now that RNEA runs every tick.
    static std::unordered_map<uint32_t, JPH::Vec3> s_prev_lin_v;
    static std::unordered_map<uint32_t, JPH::Vec3> s_prev_ang_v;

    const JPH::BodyLockInterfaceNoLock& lock_iface = system.GetBodyLockInterfaceNoLock();
    const float inv_dt = (fixed_dt > 1e-6f) ? (1.0f / fixed_dt) : 0.0f;

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

        // Per-part NET wrench this tick: inertial requirement minus the
        // external wrench applied this tick. F_self[i] = m × a_part −
        // F_ext(i). Hooke's law for the joint: this is what the joints
        // have to transmit to keep the part following the observed
        // motion given the externals we know about. With gravity +
        // thrust + drag + contact reactions all routed into our
        // per-part accumulator, F_self approaches zero on a cruising
        // stable vessel and grows on a tumbling / impacting / loaded
        // one. Per-tick (no window averaging) so PartModules see real
        // peak forces during impacts and decoupler fires.
        std::vector<JPH::Vec3> F_self(tree.nodes.size(), JPH::Vec3::sZero());
        std::vector<JPH::Vec3> T_self(tree.nodes.size(), JPH::Vec3::sZero());
        for (size_t i = 0; i < tree.nodes.size(); ++i) {
            const PartNode& n = tree.nodes[i];
            PartInertialWrench w = InertialWrench(
                n.com_local, n.mass, n.inertia_diag,
                v_body, omega, a_body, alpha);
            F_self[i] = w.force  - tree.ext_force[i];
            T_self[i] = w.torque - tree.ext_torque[i];
        }
        // Drain accumulator for next tick.
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

        // Per-edge decomposition in the joint's natural frame. Joint
        // axis = direction from parent CoM toward child's joint anchor
        // (in vessel-body local frame). For a stack joint this is along
        // the part's stacking axis; for a radial decoupler it points
        // sideways. With this:
        //   F · axis > 0 → compression (squeeze, doesn't break joints)
        //   F · axis < 0 → tension (pull-apart, this is what shears bolts)
        //   |F − (F·axis)·axis| → shear (perpendicular)
        //   T · axis → torsion (twist)
        //   |T − (T·axis)·axis| → bending (perpendicular moment)
        //
        // For breakage detection (Phase 4.x), the compression
        // component is benign; the rest are candidates for comparison
        // against joint.breakForce / breakTorque.
        float max_compression = 0.0f, max_tension = 0.0f, max_shear = 0.0f;
        float max_torsion = 0.0f, max_bending = 0.0f;
        uint16_t max_compression_idx = 0, max_tension_idx = 0, max_shear_idx = 0;
        uint16_t max_torsion_idx = 0, max_bending_idx = 0;

        for (uint16_t i = 0; i < tree.nodes.size(); ++i) {
            uint16_t parent = tree.nodes[i].parent_idx;
            if (parent == kInvalidPartIdx) continue;

            JPH::Vec3 axis_raw = tree.nodes[i].attach_local - tree.nodes[parent].com_local;
            float axis_len = axis_raw.Length();
            // Degenerate (joint anchor coincident with parent CoM): no
            // well-defined axis, skip the decomposition for this edge.
            // Phase 4.x will use stock attach-node orientation instead.
            if (axis_len < 1e-4f) continue;
            JPH::Vec3 e_x = axis_raw / axis_len;

            // Build a stable orthonormal basis (e_x, e_y, e_z). Pick
            // a reference perpendicular: vessel-body Y unless e_x is
            // already aligned with Y, in which case use X. Gram-Schmidt
            // gives e_y; cross product gives e_z. Stable across ticks
            // for a fixed joint topology.
            JPH::Vec3 ref = (std::fabs(e_x.GetY()) < 0.9f)
                ? JPH::Vec3(0.0f, 1.0f, 0.0f)
                : JPH::Vec3(1.0f, 0.0f, 0.0f);
            JPH::Vec3 e_y = (ref - e_x * ref.Dot(e_x)).Normalized();
            JPH::Vec3 e_z = e_x.Cross(e_y);

            const JPH::Vec3& F = F_subtree[i];
            const JPH::Vec3& T = T_subtree[i];

            // Project into joint frame.
            JPH::Vec3 F_joint(F.Dot(e_x), F.Dot(e_y), F.Dot(e_z));
            JPH::Vec3 T_joint(T.Dot(e_x), T.Dot(e_y), T.Dot(e_z));

            // Scalars used for the diag summary (RneaSummary). These are
            // derived from F_joint / T_joint; the per-edge record itself
            // carries the full vectors.
            float f_axial = F_joint.GetX();                 // signed
            float f_shear = std::sqrt(F_joint.GetY() * F_joint.GetY()
                                       + F_joint.GetZ() * F_joint.GetZ());
            float compression = (f_axial > 0.0f) ? f_axial : 0.0f;
            float tension     = (f_axial < 0.0f) ? -f_axial : 0.0f;
            float t_axial = T_joint.GetX();
            float t_torsion_abs = std::fabs(t_axial);
            float t_bending = std::sqrt(T_joint.GetY() * T_joint.GetY()
                                         + T_joint.GetZ() * T_joint.GetZ());

                // Per-edge wrench record — emitted every tick so PartModules
            // see fresh joint stress on the next FixedUpdate. Carries
            // full F / T vectors in joint frame; X = axial.
            EdgeWrenchRecord ew;
            ew.body_id  = body_id;
            ew.part_idx = i;
            ew.force    = F_joint;
            ew.torque   = T_joint;
            mLastEdgeWrenches.push_back(ew);

            if (compression  > max_compression) { max_compression = compression;  max_compression_idx = i; }
            if (tension      > max_tension)     { max_tension     = tension;      max_tension_idx     = i; }
            if (f_shear      > max_shear)       { max_shear       = f_shear;      max_shear_idx       = i; }
            if (t_torsion_abs > max_torsion)    { max_torsion     = t_torsion_abs; max_torsion_idx    = i; }
            if (t_bending    > max_bending)     { max_bending     = t_bending;    max_bending_idx     = i; }
        }

        if (emit_summary) {
            RneaSummary s;
            s.body_id            = body_id;
            s.part_count         = static_cast<uint16_t>(tree.nodes.size());
            s.max_compression    = max_compression;
            s.max_compression_idx = max_compression_idx;
            s.max_tension        = max_tension;
            s.max_tension_idx    = max_tension_idx;
            s.max_shear          = max_shear;
            s.max_shear_idx      = max_shear_idx;
            s.max_torsion        = max_torsion;
            s.max_torsion_idx    = max_torsion_idx;
            s.max_bending        = max_bending;
            s.max_bending_idx    = max_bending_idx;
            s.accel_mag          = a_body.Length();
            s.alpha_mag          = alpha.Length();
            mLastSummaries.push_back(s);
        }
    }
    return emit_summary;
}

} // namespace longeron
