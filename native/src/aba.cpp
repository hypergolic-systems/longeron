// Featherstone ABA forward dynamics — Pinocchio-backed (Phase 5.1.2).
//
// FRAME CONVENTION ALERT:
//   Pinocchio orders spatial vectors as (linear; angular). Our spatial.h /
//   C# Longeron.Physics use (angular; linear) — Featherstone's RBDA 2008
//   convention. The boundary code in this file (VesselWrenchToJointFrame,
//   the q/v packing) is the only place that translates between them. If
//   you're reading this file expecting the angular-first ordering used in
//   the rest of the codebase, you will see swaps. They're correct.
//
// Reduced-coord spherical-joint flex over a vessel's part tree, fixed-base
// (the vessel's CoM motion is owned by Jolt; ABA owns only the residual).
// Per substep:
//   1. forwardKinematics (root→leaf) populates data.oMi[i] = R_v[i] (joint
//      placement in world frame, which for fixed-base = vessel frame).
//   2. tau[i] = -K·axisAngle(q_joint[i]) - C·ω_joint[i]   (spring + damper).
//   3. fext[jid[i]] = vessel-frame F_flex / T_flex rotated into the joint's
//      body frame, with moment transported from CoM to joint origin.
//   4. pinocchio::aba(...) → data.ddq.
//   5. Semi-implicit Euler: v += dt·ddq, q = pinocchio::integrate(q, v·dt).
// Substepping (kAbaSubsteps=20, dt=fixed_dt/20) preserved from the prior
// custom ABA — Pinocchio's aba is a single forward-dynamics evaluation;
// time-marching is ours.
//
// Pinocchio Model is built on first call (or when tree.topology_version
// changes), held in a file-private side map keyed by body_id. No
// allocations after the first call for a stable topology.

#include "aba.h"
#include "tree.h"

#include <Jolt/Jolt.h>
#include <Jolt/Core/Core.h>
#include <Jolt/Math/Vec3.h>
#include <Jolt/Math/Quat.h>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/joint/joint-collection.hpp>
#include <pinocchio/spatial/inertia.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/force.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

namespace longeron {

namespace {

inline Eigen::Vector3d JoltToEigen(JPH::Vec3 v) {
    return Eigen::Vector3d(static_cast<double>(v.GetX()),
                           static_cast<double>(v.GetY()),
                           static_cast<double>(v.GetZ()));
}
inline Eigen::Quaterniond JoltToEigen(JPH::Quat q) {
    // Eigen::Quaterniond ctor is (w, x, y, z).
    return Eigen::Quaterniond(static_cast<double>(q.GetW()),
                              static_cast<double>(q.GetX()),
                              static_cast<double>(q.GetY()),
                              static_cast<double>(q.GetZ()));
}
inline JPH::Vec3 EigenToJolt(const Eigen::Vector3d& v) {
    return JPH::Vec3(static_cast<float>(v.x()),
                     static_cast<float>(v.y()),
                     static_cast<float>(v.z()));
}
inline JPH::Quat EigenToJolt(const Eigen::Quaterniond& q) {
    // JPH::Quat ctor is (x, y, z, w).
    return JPH::Quat(static_cast<float>(q.x()),
                     static_cast<float>(q.y()),
                     static_cast<float>(q.z()),
                     static_cast<float>(q.w()));
}

// Per-vessel Pinocchio state. Built once per topology revision; reused
// across ticks. q/v/tau/fext are sized to model.nq / model.nv / model.njoints.
struct PinState {
    pinocchio::Model model;
    pinocchio::Data  data;
    Eigen::VectorXd  q, v, tau;
    pinocchio::container::aligned_vector<pinocchio::Force> fext;

    // VesselTree part_idx → Pinocchio joint id. Root maps to 0 (universe).
    std::vector<pinocchio::JointIndex> part_to_jid;
    std::vector<int>             idx_q;        // per-part offset into q (-1 for root)
    std::vector<int>             idx_v;        // per-part offset into v (-1 for root)
    std::vector<Eigen::Vector3d> com_in_body;  // CoM offset from joint origin (joint frame)

    uint64_t topology_version = 0;
    bool     valid            = false;

    PinState() : data(model) {}
};

// File-private state map. Capacity grows monotonically with body count.
static std::unordered_map<uint32_t, PinState> g_states;

inline Eigen::Vector3d QuatToAxisAngleEigen(double qx, double qy, double qz, double qw) {
    if (qw < 0.0) { qx = -qx; qy = -qy; qz = -qz; qw = -qw; }
    double vmag = std::sqrt(qx*qx + qy*qy + qz*qz);
    if (vmag < 1.0e-9) return Eigen::Vector3d::Zero();
    return Eigen::Vector3d(qx, qy, qz) * (2.0 * std::atan2(vmag, qw) / vmag);
}

inline JPH::Vec3 InertiaTimes(JPH::Vec3 I_diag, JPH::Vec3 v) {
    return JPH::Vec3(I_diag.GetX() * v.GetX(),
                     I_diag.GetY() * v.GetY(),
                     I_diag.GetZ() * v.GetZ());
}

void RebuildModel(PinState& ps, const VesselTree& tree) {
    ps.model = pinocchio::Model();
    const size_t n = tree.nodes.size();
    ps.part_to_jid.assign(n, 0);
    ps.idx_q.assign(n, -1);
    ps.idx_v.assign(n, -1);
    ps.com_in_body.assign(n, Eigen::Vector3d::Zero());

    for (size_t i = 1; i < n; ++i) {
        const PartNode& node   = tree.nodes[i];
        const uint16_t  parent = node.parent_idx;
        if (parent >= n) continue;

        // Joint placement: parent joint frame → this joint frame, at rest.
        // Identity rotation (rest frames share vessel-body axes); translation
        // = this part's attach_local minus the parent's attach_local. The
        // root part's "joint origin" is the vessel origin (Vec3::Zero).
        Eigen::Vector3d parent_attach = (parent == 0)
            ? Eigen::Vector3d::Zero()
            : JoltToEigen(tree.nodes[parent].attach_local);
        Eigen::Vector3d this_attach   = JoltToEigen(node.attach_local);
        pinocchio::SE3 placement = pinocchio::SE3::Identity();
        placement.translation() = this_attach - parent_attach;

        const pinocchio::JointIndex jid = ps.model.addJoint(
            ps.part_to_jid[parent],
            pinocchio::JointModelSpherical(),
            placement,
            std::string("p") + std::to_string(i));
        ps.part_to_jid[i] = jid;
        ps.idx_q[i] = ps.model.joints[jid].idx_q();
        ps.idx_v[i] = ps.model.joints[jid].idx_v();

        // Body inertia. Pinocchio takes (mass, lever, I_about_com) all in
        // joint frame. lever = CoM offset from joint origin.
        Eigen::Vector3d com_offset = JoltToEigen(node.com_local) - this_attach;
        ps.com_in_body[i] = com_offset;

        Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
        I.diagonal() = JoltToEigen(node.inertia_diag);
        pinocchio::Inertia bodyI(static_cast<double>(node.mass), com_offset, I);
        ps.model.appendBodyToJoint(jid, bodyI, pinocchio::SE3::Identity());
    }

    // Disable Pinocchio's built-in gravity. Longeron handles gravity
    // upstream — gravitational acceleration is part of the Jolt body's
    // CoM motion (the a_body finite-diff feeds the F_inertial subtraction
    // above), and per-part lift / drag / thrust come in via
    // tree.ext_force. If we left model.gravity at its non-zero default,
    // Pinocchio would add a SECOND gravity term on top, double-counting.
    // Verified empirically: ~0.098 rad of ghost deflection per body in
    // M3.4 traces directly to (com_in_body × default_gravity) / K_ang.
    ps.model.gravity.setZero();

    ps.data = pinocchio::Data(ps.model);
    ps.q   = pinocchio::neutral(ps.model);
    ps.v   = Eigen::VectorXd::Zero(ps.model.nv);
    ps.tau = Eigen::VectorXd::Zero(ps.model.nv);
    ps.fext.resize(static_cast<std::size_t>(ps.model.njoints));
    for (auto& f : ps.fext) f.setZero();

    ps.topology_version = tree.topology_version;
    ps.valid            = (ps.model.njoints >= 2);
}

// Ensure the Pinocchio cache for this body matches the current tree. Returns
// nullptr if the tree is too small to flex (single-part vessel).
PinState* EnsureState(const VesselTree& tree, uint32_t body_id) {
    PinState& ps = g_states[body_id];
    if (!ps.valid || ps.topology_version != tree.topology_version) {
        RebuildModel(ps, tree);
    }
    return ps.valid ? &ps : nullptr;
}

// Vessel-frame wrench at part CoM → Pinocchio joint-frame Force.
//   Pinocchio Force = (linear, angular at joint origin), in joint body frame.
inline pinocchio::Force VesselWrenchToJointFrame(
    JPH::Vec3 F_vessel,
    JPH::Vec3 T_at_com_vessel,
    const Eigen::Quaterniond& R_v_i,
    const Eigen::Vector3d&    com_in_body)
{
    Eigen::Vector3d F_body        = R_v_i.conjugate() * JoltToEigen(F_vessel);
    Eigen::Vector3d T_at_com_body = R_v_i.conjugate() * JoltToEigen(T_at_com_vessel);
    Eigen::Vector3d T_at_origin   = T_at_com_body + com_in_body.cross(F_body);
    return pinocchio::Force(F_body, T_at_origin);   // (linear, angular)
}

}  // namespace

void RunAbaForwardStep(
    VesselTree& tree,
    JPH::Vec3 v_body,
    JPH::Vec3 omega,
    JPH::Vec3 a_body,
    JPH::Vec3 alpha,
    float fixed_dt,
    uint32_t body_id,
    std::vector<AbaPartDiagRecord>* diag_out)
{
    (void)v_body;
    const size_t n = tree.nodes.size();
    if (n < 2) return;

    PinState* psp = EnsureState(tree, body_id);
    if (psp == nullptr) return;
    PinState& ps = *psp;

    const float dt = fixed_dt / static_cast<float>(kAbaSubsteps);

    // -- Per-part inertial-residual decomposition (vessel frame) -------------
    // Held constant across substeps. Mirrors the prior custom-ABA code:
    //   F_flex = ext_force - m·(a_body + α×r + ω×(ω×r))
    //   T_flex = ext_torque - I·α - ω×(I·ω)
    // where r = com_local + delta_pos (the deflected lever to the part CoM).
    std::vector<JPH::Vec3> F_flex(n);
    std::vector<JPH::Vec3> T_flex(n);
    for (size_t i = 0; i < n; ++i) {
        const PartNode& p = tree.nodes[i];
        const JPH::Vec3 r = p.com_local + tree.flex[i].delta_pos;

        const JPH::Vec3 a_part     = a_body + alpha.Cross(r) + omega.Cross(omega.Cross(r));
        const JPH::Vec3 F_inertial = p.mass * a_part;
        const JPH::Vec3 T_inertial = InertiaTimes(p.inertia_diag, alpha)
                                     + omega.Cross(InertiaTimes(p.inertia_diag, omega));

        F_flex[i] = tree.ext_force[i]  - F_inertial;
        T_flex[i] = tree.ext_torque[i] - T_inertial;

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

    // -- Pack q from current flex state -------------------------------------
    // delta_rot[i] is R_v[i] (cumulative vessel-frame rotation); we recover
    // the per-edge joint rotation as q_joint[i] = R_v[parent].conj() · R_v[i].
    // Pinocchio JointModelSpherical packs q as (x, y, z, w).
    for (size_t i = 1; i < n; ++i) {
        const uint16_t parent = tree.nodes[i].parent_idx;
        if (parent >= n || ps.idx_q[i] < 0) continue;
        const Eigen::Quaterniond R_parent = JoltToEigen(tree.flex[parent].delta_rot);
        const Eigen::Quaterniond R_child  = JoltToEigen(tree.flex[i].delta_rot);
        Eigen::Quaterniond q_joint = R_parent.conjugate() * R_child;
        q_joint.normalize();
        ps.q[ps.idx_q[i] + 0] = q_joint.x();
        ps.q[ps.idx_q[i] + 1] = q_joint.y();
        ps.q[ps.idx_q[i] + 2] = q_joint.z();
        ps.q[ps.idx_q[i] + 3] = q_joint.w();
    }

    // -- Pack v from PartFlex.ang_vel ---------------------------------------
    // PartFlex.ang_vel stores joint-relative ω in the joint's frame; same
    // convention as Pinocchio's v segment for spherical joints.
    for (size_t i = 1; i < n; ++i) {
        if (ps.idx_v[i] < 0) continue;
        ps.v.segment<3>(ps.idx_v[i]) = JoltToEigen(tree.flex[i].ang_vel);
    }

    // -- Substep loop -------------------------------------------------------
    for (int sub = 0; sub < kAbaSubsteps; ++sub) {
        // Forward kinematics — populates data.oMi[jid] = R_v[i] in joint
        // placement world frame (= vessel frame for fixed base).
        pinocchio::forwardKinematics(ps.model, ps.data, ps.q);

        for (size_t i = 1; i < n; ++i) {
            if (ps.idx_v[i] < 0) continue;
            const EdgeCompliance& c = tree.edge_compliance[i];

            const Eigen::Vector3d aa = QuatToAxisAngleEigen(
                ps.q[ps.idx_q[i] + 0],
                ps.q[ps.idx_q[i] + 1],
                ps.q[ps.idx_q[i] + 2],
                ps.q[ps.idx_q[i] + 3]);

            const Eigen::Vector3d w_joint = ps.v.segment<3>(ps.idx_v[i]);
            ps.tau.segment<3>(ps.idx_v[i]) =
                -static_cast<double>(c.k_ang) * aa
                - static_cast<double>(c.c_ang) * w_joint;

            const pinocchio::JointIndex jid = ps.part_to_jid[i];
            const Eigen::Quaterniond R_v_i(ps.data.oMi[jid].rotation());
            ps.fext[jid] = VesselWrenchToJointFrame(
                F_flex[i], T_flex[i], R_v_i, ps.com_in_body[i]);
        }
        ps.fext[0].setZero();   // universe / base joint

        pinocchio::aba(ps.model, ps.data, ps.q, ps.v, ps.tau, ps.fext);

        // Semi-implicit Euler integration on the joint manifold.
        ps.v.noalias() += static_cast<double>(dt) * ps.data.ddq;
        ps.q = pinocchio::integrate(ps.model, ps.q, ps.v * static_cast<double>(dt));
    }

    // Final forward kinematics so the writeback below sees fresh oMi.
    pinocchio::forwardKinematics(ps.model, ps.data, ps.q);

    // -- Writeback: tree.flex[i] in vessel-body frame -----------------------
    tree.flex[0].delta_pos = JPH::Vec3::sZero();
    tree.flex[0].delta_rot = JPH::Quat::sIdentity();
    tree.flex[0].lin_vel   = JPH::Vec3::sZero();
    tree.flex[0].ang_vel   = JPH::Vec3::sZero();

    for (size_t i = 1; i < n; ++i) {
        if (ps.idx_v[i] < 0) continue;
        const pinocchio::JointIndex jid = ps.part_to_jid[i];

        // For spherical joints with rest placement (Identity, t_up), the
        // joint origin DOES NOT translate when q changes — only rotates
        // through the chain. So the part CoM in vessel frame is:
        //   pos_v[i] = oMi[jid].translation() + R_v[i] · com_in_body[i]
        // and our delta_pos (deflection from rest CoM) is:
        //   delta_pos[i] = pos_v[i] - com_local[i]
        const Eigen::Quaterniond R_v_i(ps.data.oMi[jid].rotation());
        const Eigen::Vector3d pos_v   = ps.data.oMi[jid].translation()
                                        + R_v_i * ps.com_in_body[i];
        const Eigen::Vector3d com_v0  = JoltToEigen(tree.nodes[i].com_local);
        const Eigen::Vector3d delta_p = pos_v - com_v0;

        tree.flex[i].delta_pos = EigenToJolt(delta_p);
        tree.flex[i].delta_rot = EigenToJolt(R_v_i);
        tree.flex[i].lin_vel   = JPH::Vec3::sZero();
        tree.flex[i].ang_vel   = EigenToJolt(Eigen::Vector3d(ps.v.segment<3>(ps.idx_v[i])));
    }

    // -- Safety clamp -------------------------------------------------------
    constexpr float kMaxLinearFlexM = 1.0f;
    constexpr float kMaxAngVelRad   = 50.0f;
    constexpr float kMaxLinVelMs    = 100.0f;
    static int s_clamp_log_count = 0;
    bool any_clamp = false;
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
            any_clamp = true;
        }
    }
    if (any_clamp) {
        // Cached q/v are out of sync with the (now-zeroed) flex state.
        // Force a fresh repack on the next tick.
        ps.valid = false;
    }
}

}  // namespace longeron
