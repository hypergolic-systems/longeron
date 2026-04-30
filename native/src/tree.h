// Per-vessel spanning tree + Featherstone RNEA pass.
//
// Phase 4 advisory mode: read-only on Jolt's simulation. We don't drive
// vessel motion (Jolt's job) — we decompose it into per-joint transmitted
// wrench for breakage detection (eventually) and currently for diagnostic
// logging.
//
// Single-body model: every part shares the vessel body's ω, α, and
// rigid-body kinematics give each part's CoM acceleration as
//   a_p = a_body + α × r_p + ω × (ω × r_p)
// where r_p is the part's CoM offset from the vessel body's reference
// point (CoM in Jolt's auto-shifted convention).
//
// RNEA backward pass aggregates leaf → root: the wrench transmitted
// across a part's joint-to-parent equals the inertial wrench of that
// part's entire subtree minus the external wrenches on the subtree.
// For Phase 4.0 we ignore external wrenches entirely (just the inertial
// component) — a sanity baseline that's zero when the vessel cruises and
// nonzero in maneuvers.

#ifndef LONGERON_TREE_H
#define LONGERON_TREE_H

#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>
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
    // VesselTreeUpdate input handler.
    void Upsert(uint32_t body_id, std::vector<PartNode>&& nodes);

    // Drop a vessel's tree (called on BodyDestroy).
    void Erase(uint32_t body_id);

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

    // Run RNEA for every tree using current Jolt state. Always computes
    // per-edge wrench decomposition (mLastEdgeWrenches) so PartModules
    // can read fresh joint forces every tick. The compact RneaSummary
    // (mLastSummaries) is still gated by an emit cadence so the diag
    // log doesn't drown KSP.log.
    //
    // Returns true if a new RneaSummary was produced this tick; false
    // if it was skipped (per-edge data was still computed). Called once
    // per Step after PhysicsSystem::Update + ResolveContacts.
    bool RunAdvisoryPass(const JPH::PhysicsSystem& system,
                         const std::unordered_map<uint32_t, JPH::BodyID>& registry,
                         uint64_t step_count, float fixed_dt);

    const std::vector<RneaSummary>& GetLastSummaries() const { return mLastSummaries; }
    const std::vector<EdgeWrenchRecord>& GetLastEdgeWrenches() const { return mLastEdgeWrenches; }

private:
    std::unordered_map<uint32_t, VesselTree> mTrees;

    // Latest pass output. mLastEdgeWrenches is refilled every tick
    // (PartModules need fresh data); mLastSummaries is refilled only
    // on cadence ticks (1 Hz log smoothing). Both vectors are cleared
    // at the start of each respective fill.
    std::vector<RneaSummary>        mLastSummaries;
    std::vector<EdgeWrenchRecord>   mLastEdgeWrenches;

    // Last step at which we emitted a summary. Throttle to keep
    // KSP.log readable while per-edge wrenches still flow every tick.
    uint64_t mLastEmitStep = 0;
};

} // namespace longeron

#endif // LONGERON_TREE_H
