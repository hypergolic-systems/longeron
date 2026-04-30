// Per-vessel spanning tree + Featherstone RNEA pass + ABA forward pass.
//
// Phase 5.1: every vessel runs a fixed-base reduced-coordinate
// Featherstone ABA forward dynamics pass over its part tree. Each
// non-root part connects to its parent via a **spherical joint** —
// 3 angular DOF about the joint anchor, with K_ang spring + C_ang
// damper; translation between the two anchors is rigidly pinned.
// Linear flex (delta_pos in vessel frame) is a *consequence* of
// angular flex through lever arms — never integrated, never sprung.
//
// Jolt owns the vessel's CoM motion (rigid-body integration under
// contacts + summed external wrenches). ABA owns the per-part flex
// in reduced coords; the per-edge joint state is (q_i, ω_i) where
// q_i is the rotation of child relative to parent at the anchor.
//
// Per tick:
//  1. C# accumulates per-part forces (gravity, thrust, drag, contacts)
//     into ext_force / ext_torque, vessel-body axes.
//  2. Jolt steps the vessel body forward dt under summed wrench +
//     contact response.
//  3. ABA forward pass — fixed-base reduced-coord recursion (Ch. 7):
//     outward kinematics (root → leaf) propagates spatial velocity;
//     inward articulated-inertia pass (leaf → root) builds I^A and
//     bias force at each link, with external wrench rotated into
//     each part's CURRENT frame; outward acceleration solve produces
//     qddot; semi-implicit Euler integrates joint state.
//  4. The cumulative joint chain is unrolled into vessel-frame
//     delta_pos / delta_rot per part, written to PartFlex for
//     downstream consumers.
//  5. ModifyShapes on the vessel's MutableCompoundShape from the new
//     per-part offsets so collision geometry tracks visible flex.
//  6. RNEA backward pass on the deflected geometry for break detection.
//
// Frame: PartFlex.delta_pos / delta_rot are in vessel-body axes
// (cumulative result of the joint chain), so RouteContactForce,
// RNEA, and ApplyFlexToBodies see the same semantics they did under
// Phase 5.0. Joint-state integration internally uses parent-frame
// angular velocity ω_i, stored in PartFlex.ang_vel.

#ifndef LONGERON_TREE_H
#define LONGERON_TREE_H

#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace longeron {

// Per-part data carried in a VesselTree. Body-frame quantities
// (relative to the vessel's Jolt body anchor / CoM).
struct PartNode {
    uint16_t  parent_idx;          // kInvalidPartIdx for the root part
    float     mass;                // tonnes (KSP convention)
    JPH::Vec3 com_local;           // CoM offset from vessel root, body axes
    JPH::Vec3 inertia_diag;        // principal inertia (diagonal approx), tonnes·m²
    JPH::Vec3 attach_local;        // joint-to-parent anchor, body axes
};

// Per-edge spring-damper config for the joint connecting a part to its
// parent. Phase 5.1: spherical joint — child anchor rigidly pinned to
// parent anchor (3 translation DOF eliminated, no linear compliance).
// Child has 3 angular DOF about that anchor, with isotropic spring +
// damper:
//   T_joint = -K_ang · rotation_error - C_ang · angular_velocity_diff
// applied at the joint anchor (attach_local), equal+opposite on parent.
//
// Anisotropic per-axis stiffness (torsion vs. bending) is a Phase 5.x
// extension — would need joint-frame basis vectors and per-channel K/C.
// For v1, isotropic in the parent's current frame.
//
// Units: K_ang in kN·m/rad, C_ang in kN·m·s/rad
// (consistent with vessel mass in tonnes, force in kN.)
struct EdgeCompliance {
    float k_ang;
    float c_ang;
};

// Per-part flex state. Phase 5.1 spherical-joint reduced coords:
//   ang_vel   — joint-relative angular velocity ω_i in parent frame
//               (the only true integrated DOF per edge).
//   delta_rot — derived: cumulative rotation child-relative-to-vessel,
//               composed from joint chain each substep.
//   delta_pos — derived: child CoM deflection from rest in vessel frame,
//               from joint-chain forward kinematics + lever arms.
//   lin_vel   — unused (no linear DOF under spherical); kept zero.
//
// Rest pose (delta_pos = 0, delta_rot = identity, ang_vel = 0): part is
// where it would be if the vessel were rigid.
//
// Current pose (after each ABA tick):
//   position_in_vessel_frame = com_local + delta_pos
//   rotation_in_vessel_frame = delta_rot       (identity at rest)
//
// Layout retained from Phase 5.0 to keep RouteContactForce, RNEA, and
// ApplyFlexToBodies unchanged. A defensible follow-up is to split a
// per-edge JointState struct out of PartFlex.
struct PartFlex {
    JPH::Vec3 delta_pos;     // derived: vessel-frame CoM deflection
    JPH::Quat delta_rot;     // derived: vessel-frame cumulative rotation
    JPH::Vec3 lin_vel;       // unused under spherical joints
    JPH::Vec3 ang_vel;       // joint-relative ω in parent frame
};

// One vessel's spanning tree. Parts are stored in topology order: parent
// indices always strictly less than the child's index (root at 0 unless
// out-of-order send, in which case we sort on receive).
struct VesselTree {
    uint32_t              body_id;     // user_id of the Jolt vessel body
    std::vector<PartNode> nodes;

    // Per-part external-wrench accumulator. Summed by
    // AccumulateExternalForce as input ForceAtPosition records arrive,
    // drained + cleared at each emit by RunAdvisoryPass.
    //   ext_force[i]  — sum of force vectors applied to part i this
    //                   window, in vessel-body local axes (kN units,
    //                   stock convention).
    //   ext_torque[i] — sum of (P_local - com_local) × F_local, the
    //                   component of torque about each part's CoM
    //                   contributed by lever-arm forces.
    // Tick count over the window is tracked by TreeRegistry so we can
    // average force values back from impulse-style sums if needed.
    std::vector<JPH::Vec3> ext_force;
    std::vector<JPH::Vec3> ext_torque;

    // Per-edge joint compliance. Index i corresponds to the edge from
    // part i to nodes[i].parent_idx (root has trivial / unused entry).
    // Set on Upsert from the C# side; constant across the tree's life
    // (rebuilt with the topology on any change).
    std::vector<EdgeCompliance> edge_compliance;

    // Per-part flex state. Index 0 (root) stays zero — by convention
    // the root is glued to the vessel body's anchor frame and ABA only
    // computes flex for parts 1..n-1. Updated every tick by the ABA
    // forward pass; consumed by RNEA / ModifyShapes / PartPose emit.
    std::vector<PartFlex> flex;

    // SubShape rest positions in the body's CoM frame (after Jolt's
    // MutableCompoundShape::AdjustCenterOfMass auto-shifts at create
    // time). Lazily populated on first ApplyFlexToBodies pass and then
    // used as the immutable rest reference for ModifyShapes:
    //   subshape_pos[i] = rest_subshape_pos[i] + delta_pos[part_for_subshape[i]]
    //   subshape_rot[i] = delta_rot[part_for_subshape[i]] · rest_subshape_rot[i]
    // Without this, reading the compound's CURRENT SubShape position
    // each tick (which equals the prior tick's modified position)
    // would feed the flex offset into rest, accumulating drift.
    bool                   rest_captured = false;
    std::vector<JPH::Vec3> rest_subshape_pos;
    std::vector<JPH::Quat> rest_subshape_rot;

    // SubShape index → part_idx mapping. A part may have multiple
    // colliders; each becomes its own SubShape, but they all share the
    // same flex transform. Sent by C# via SubShapeMap right after
    // BodyCreate. If empty, ApplyFlexToBodies treats each SubShape as
    // its own part (legacy / ABA disabled fallback).
    std::vector<uint16_t> subshape_to_part;

    // Last tick's body-frame kinematic state. Filled by RunAbaPass
    // (finite-diff vs previous tick); reused by RunAdvisoryPass for
    // the inertial wrench decomposition. Avoids a second finite-diff.
    JPH::Vec3 last_v_body  = JPH::Vec3::sZero();
    JPH::Vec3 last_omega   = JPH::Vec3::sZero();
    JPH::Vec3 last_a_body  = JPH::Vec3::sZero();
    JPH::Vec3 last_alpha   = JPH::Vec3::sZero();

    // Phase 5 diag: tick count since this tree was Upserted. ABA emits
    // per-part diag records for the first kAbaDiagWindow ticks so we
    // can correlate post-decouple instability with input/output forces.
    int diag_tick = 0;

    // First-tick guard for finite-diff acceleration: false until the
    // tree has seen one full RunAbaPass call. While false, RunAbaPass
    // skips the (v - prev_v)/dt computation (no prev_v to diff against)
    // and sets last_a_body / last_alpha to zero. Reset on Upsert so a
    // fresh topology rebuild can't inherit a stale velocity from
    // an unrelated previous body that happened to share the same
    // body_id.
    bool has_prev_velocity = false;
};

// Per-part diag record output by ABA each tick during the first
// kAbaDiagWindow ticks of a body's life. Streams to C# log for
// post-decouple instability investigation.
struct AbaPartDiagRecord {
    uint32_t body_id;
    uint16_t tick;
    uint16_t part_idx;
    JPH::Vec3 ext_force;
    JPH::Vec3 f_inertial;
    JPH::Vec3 f_flex;
    JPH::Vec3 delta_pos;
    float     delta_angle_rad;
};

// Per-vessel RNEA pass output. Filled by RunAdvisoryPass each tick;
// world.cpp drains into the output record stream (RneaSummary).
//
// Decomposition convention: per-edge joint axis points from parent
// CoM toward the child's joint anchor (vessel-body frame). For each
// edge i:
//   F_axial = F_subtree[i] · axis    (signed)
//     > 0 → compression: parts squeezed together (gravity on stack,
//            ground reaction). Doesn't break joints.
//     < 0 → tension:    parts pulled apart. THIS is what shears bolts
//                       / cracks welds — break-detection lives here.
//   F_shear = ‖F_subtree[i] − F_axial · axis‖   (transverse)
//   T_torsion = T_subtree[i] · axis              (signed; twisting)
//   T_bending = ‖T_subtree[i] − T_torsion · axis‖ (perpendicular;
//             bending moment, e.g. radial decoupler holding a heavy
//             booster against gravity).
struct RneaSummary {
    uint32_t body_id;
    uint16_t part_count;

    // Per-vessel maxima across edges (each with the part index where
    // it peaks, joint-to-parent).
    float    max_compression;       uint16_t max_compression_idx;
    float    max_tension;           uint16_t max_tension_idx;
    float    max_shear;             uint16_t max_shear_idx;
    float    max_torsion;           uint16_t max_torsion_idx;
    float    max_bending;           uint16_t max_bending_idx;

    float    accel_mag;   // |a_body| this tick (finite-diff)
    float    alpha_mag;   // |α_body| this tick
};

// Per-edge wrench in the joint's reference frame, emitted every tick.
// PartModules read these (via JoltPart) on the next tick's
// OnFixedUpdate to decide whether to break.
//
// Frame convention (orthonormal, right-handed):
//   e_x = joint axial direction (parent CoM → child joint anchor,
//         normalized in vessel-body axes).
//   e_y, e_z = perpendicular pair built from a stable Gram-Schmidt
//         against vessel-body Y (or X if Y is parallel to e_x).
//
// Force vector semantics:
//   force.x  → signed axial: +compression (squeeze, benign) /
//              -tension (pull-apart, breaks bolts).
//   force.y, force.z → shear (perpendicular to axis); magnitude is
//                       physically meaningful, individual y/z split
//                       is implementation-defined but stable across
//                       ticks for a given joint topology.
//
// Torque vector semantics:
//   torque.x → signed torsion (twist around the axis).
//   torque.y, torque.z → bending moment (perpendicular to axis);
//                        magnitude is physically meaningful.
struct EdgeWrenchRecord {
    uint32_t  body_id;
    uint16_t  part_idx;
    JPH::Vec3 force;       // joint frame: X = axial, YZ = shear
    JPH::Vec3 torque;      // joint frame: X = torsion, YZ = bending
    JPH::Vec3 ext_force;   // diag: per-part external force this tick,
                            //   body-local axes (gravity + thrust + drag
                            //   + contact lambdas summed). Lets the C#
                            //   side see whether contact is routing.
};

class TreeRegistry {
public:
    // Replace (or insert) the tree for a given vessel. Called from the
    // VesselTreeUpdate input handler. compliance.size() must equal
    // nodes.size() — index i is the joint compliance from part i to
    // its parent (root's entry is unused).
    void Upsert(uint32_t body_id,
                std::vector<PartNode>&& nodes,
                std::vector<EdgeCompliance>&& compliance);

    // Compute the real mass-weighted CoM offset (in vessel-root frame)
    // from PartNodes for a given vessel. Returns zero if the body has
    // no tree or zero total mass. Used by Phase 5 diagnostics to compare
    // against Jolt's auto-CoM (which is volume × density weighted).
    JPH::Vec3 ComputeRealCom(uint32_t body_id) const;

    // Drop a vessel's tree (called on BodyDestroy).
    void Erase(uint32_t body_id);

    // Set the SubShape→part_idx mapping for a body's compound shape.
    // Called when SubShapeMap input arrives. The body may not yet have
    // a tree (the VesselTreeUpdate may come later); we lazily create
    // a tree-only entry to hold the mapping.
    void SetSubShapeMap(uint32_t body_id, std::vector<uint16_t>&& subshape_to_part);

    // Add an external force record into the per-part accumulator for
    // RNEA subtraction. World-frame force + point are converted to
    // body-local axes using the body's current CoM position + rotation.
    // No-op when the vessel has no tree or part_idx is out of range.
    void AccumulateExternalForce(
        uint32_t body_id, uint16_t part_idx,
        JPH::Vec3 force_world, JPH::RVec3 point_world,
        JPH::RVec3 body_com_world, JPH::Quat body_rot);

    // Route a contact-resolved force into the per-part accumulator.
    // The application point is in CB-frame world coords; the per-part
    // attribution is by closest body-local CoM (linear scan over the
    // vessel's parts). No-op when the body has no tree.
    //
    // TODO (Phase 4.x): replace the O(n_parts) closest-CoM scan with
    // an O(1) sub-shape→part_idx lookup precomputed at BodyCreate /
    // VesselTreeUpdate time. Capture (body_pair → sub_shape_pair) in
    // ContactListener::OnContactPersisted so the constraint→part
    // mapping is exact instead of position-based.
    void RouteContactForce(
        uint32_t body_id,
        JPH::Vec3 force_world, JPH::RVec3 point_world,
        JPH::RVec3 body_com_world, JPH::Quat body_rot);

    // Run the ABA forward dynamics pass over every vessel tree. Reads
    // each vessel body's velocity/acceleration (finite-diff against
    // last tick) from Jolt, runs the substepped semi-implicit Euler
    // integration of per-part flex state (delta_pos, delta_rot, vels)
    // under spring-damper joint forces. Caches the body-frame
    // velocities + accelerations on the tree for RunAdvisoryPass to
    // reuse without a second finite-diff.
    //
    // Called once per Step, BEFORE ApplyFlexToBodies + RunAdvisoryPass.
    void RunAbaPass(const JPH::PhysicsSystem& system,
                    const std::unordered_map<uint32_t, JPH::BodyID>& registry,
                    float fixed_dt);

    // Apply each tree's flex state to the corresponding Jolt body's
    // MutableCompoundShape. Walks per-part flex, computes the SubShape
    // local transform (com_local + delta_pos, delta_rot), batches via
    // ModifyShapes, then emits NotifyShapeChanged so broadphase +
    // narrow-phase caches refresh on the next tick.
    //
    // Called once per Step, AFTER RunAbaPass and BEFORE RunAdvisoryPass
    // (so RNEA sees the deflected geometry).
    void ApplyFlexToBodies(JPH::PhysicsSystem& system,
                           const std::unordered_map<uint32_t, JPH::BodyID>& registry);

    // Run RNEA for every tree using current Jolt state. Always computes
    // per-edge wrench decomposition (mLastEdgeWrenches) so PartModules
    // can read fresh joint forces every tick. The compact RneaSummary
    // (mLastSummaries) is still gated by an emit cadence so the diag
    // log doesn't drown KSP.log.
    //
    // Returns true if a new RneaSummary was produced this tick; false
    // if it was skipped (per-edge data was still computed). Called once
    // per Step after RunAbaPass + ApplyFlexToBodies.
    bool RunAdvisoryPass(const JPH::PhysicsSystem& system,
                         const std::unordered_map<uint32_t, JPH::BodyID>& registry,
                         uint64_t step_count, float fixed_dt);

    const std::vector<RneaSummary>& GetLastSummaries() const { return mLastSummaries; }
    const std::vector<EdgeWrenchRecord>& GetLastEdgeWrenches() const { return mLastEdgeWrenches; }
    const std::vector<AbaPartDiagRecord>& GetLastAbaDiags() const { return mLastAbaDiags; }
    const std::unordered_map<uint32_t, VesselTree>& GetTrees() const { return mTrees; }

private:
    std::unordered_map<uint32_t, VesselTree> mTrees;

    // Latest pass output. mLastEdgeWrenches is refilled every tick
    // (PartModules need fresh data); mLastSummaries is refilled only
    // on cadence ticks (1 Hz log smoothing). Both vectors are cleared
    // at the start of each respective fill.
    std::vector<RneaSummary>        mLastSummaries;
    std::vector<EdgeWrenchRecord>   mLastEdgeWrenches;
    std::vector<AbaPartDiagRecord>  mLastAbaDiags;

    // Last step at which we emitted a summary. Throttle to keep
    // KSP.log readable while per-edge wrenches still flow every tick.
    uint64_t mLastEmitStep = 0;
};

} // namespace longeron

#endif // LONGERON_TREE_H
