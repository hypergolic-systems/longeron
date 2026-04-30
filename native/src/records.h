// Shared schema for the input/output record streams. C# side mirrors
// this in mod/Longeron.Native/src/InputBuffer.cs (RecordType enum and
// per-record byte layouts).
//
// Record stream is a sequence of typed records. Each record starts with
// a 1-byte type tag. Layouts are stable (additive only); the schema
// version in LongeronConfig::schema_version gates compatibility.

#ifndef LONGERON_RECORDS_H
#define LONGERON_RECORDS_H

#include <stdint.h>

namespace longeron {

enum class RecordType : uint8_t {
    None              = 0,

    // ---- Input records (C# → native) ----------------------------------
    ForceDelta        = 1,   // u32 user_id, double3 force, double3 torque (world-frame)
    MassUpdate        = 2,   // u32 user_id, float mass, float[6] inertia (Phase 2+)
    BodyCreate        = 3,   // u32 user_id, u8 body_type, u8 shape_kind, shape params,
                             // double3 pos, float4 rot, float mass, u8 layer
    BodyDestroy       = 4,   // u32 user_id
    ConstraintCreate  = 5,   // Phase 2+
    ConstraintDestroy = 6,   // Phase 2+
    SetGravity        = 7,   // double3 gravity
    SetKinematicPose  = 8,   // u32 user_id, double3 pos, float4 rot, float3 lin_vel, float3 ang_vel
    ShiftWorld        = 9,   // double3 delta — translate every body in absolute coords by delta
    SetBodyGroup      = 10,  // u32 user_id, u32 group_id — change collision filter group post-create
                             //   (used when a part migrates between vessels via decouple/dock)
    ConstraintCreateFixedAt = 11,  // u32 cid, u8 kind, u32 body_a, u32 body_b, double3 anchor_world
                                   //   FixedConstraint with EConstraintSpace::WorldSpace; explicit
                                   //   anchor (KSP attach-node position in CB-frame coords) — better
                                   //   PGS conditioning than mAutoDetectPoint=true. (Phase 2 multi-
                                   //   body experiment; left here for inter-vessel future use.)
    ForceAtPosition   = 12,  // u32 user_id, double3 force, double3 point_world, u16 part_idx
                             //   Apply force at a specific CB-frame world point on the body. Native
                             //   side computes (point - body_CoM) × force as the implicit torque.
                             //   Used by the single-body-per-vessel model: every part's AddForce
                             //   redirects to the vessel body at the part's CoM-or-attach-point.
                             //   part_idx tags the force with the originating part's tree index
                             //   (0xFFFF = unattributed) so Phase 4 RNEA can subtract the per-part
                             //   external wrench from the inertial wrench.
    VesselTreeUpdate  = 13,  // u32 vessel_body_id, u16 part_count,
                             //   part_count × {
                             //     u16 parent_idx (0xFFFF = root), float mass,
                             //     float3 com_local,        // CoM in vessel-root frame
                             //     float3 inertia_diag,     // diagonal inertia in vessel-root axes
                             //     float3 attach_local      // joint-to-parent attach point
                             //   }
                             //   Sent on every vessel reconcile (after BodyCreate). Native runs
                             //   an RNEA pass each tick using Jolt's vessel motion to compute
                             //   per-edge transmitted wrench — Phase 4 advisory; logs only for now.

    // ---- Output records (native → C#) ---------------------------------
    BodyPose          = 64,  // u32 user_id, double3 pos, float4 rot, float3 lin_vel, float3 ang_vel
    ContactReport     = 65,  // u32 user_id_a, u32 user_id_b, double3 point, float3 normal, float depth, float impulse
    RneaSummary       = 66,  // u32 vessel_body_id, u16 part_count,
                             //   {float max_compression, u16 idx},
                             //   {float max_tension,     u16 idx},
                             //   {float max_shear,       u16 idx},
                             //   {float max_torsion,     u16 idx},
                             //   {float max_bending,     u16 idx},
                             //   float accel_mag, float alpha_mag
                             //   Per-vessel RNEA summary, decomposed in each joint's reference
                             //   frame: F_subtree projected onto the joint axis (parent-to-child
                             //   direction) gives compression / tension; perpendicular gives
                             //   shear. T_subtree gives torsion / bending the same way. Only
                             //   tension + shear + torsion + bending are candidates for
                             //   joint.breakForce / breakTorque comparison; compression is benign.
                             //   Emitted at ~1 Hz cadence for log smoothing.
    JointWrench       = 67,  // u32 vessel_body_id, u16 part_idx,
                             //   float3 force_joint  (X=axial: +compression / -tension; YZ=shear),
                             //   float3 torque_joint (X=torsion signed; YZ=bending),
                             //   float3 ext_force    (per-part external force this tick, body axes;
                             //                       diag-only — gravity + thrust + drag + contact).
                             //   Per-edge wrench in joint reference frame. e_x = parent_CoM →
                             //   child_attach (normalized). e_y, e_z form a stable Gram-Schmidt
                             //   orthonormal pair. Emitted every tick; C# stashes on JoltPart so
                             //   PartModules can read on the next tick's OnFixedUpdate to make
                             //   break decisions.
    PartPose          = 68,  // u32 vessel_body_id, u16 part_idx,
                             //   float3 delta_pos (vessel-body axes — flex offset from rest CoM),
                             //   float4 delta_rot (vessel-body axes — flex rotation from rest,
                             //                     identity at no-flex).
                             //   Per-part flex pose from the ABA forward pass. SceneDriver applies
                             //   to each part's Unity rb as
                             //     rb.position = vesselPos + vesselRot · (PartLocalPos + delta_pos)
                             //     rb.rotation = vesselRot · delta_rot · PartLocalRot
                             //   Emitted every tick for every part with delta != identity (root
                             //   skipped — always at rest).
};

// Phase 4: cap on parts per vessel for the RNEA pass. Stock KSP allows
// vessels of hundreds of parts; 1024 is comfortable headroom for stock
// + heavy mods. Wire format uses u16 part indices (0xFFFF reserved for
// "root, no parent").
inline constexpr uint32_t kMaxPartsPerVessel = 1024;
inline constexpr uint16_t kInvalidPartIdx    = 0xFFFFu;

// Body motion types. Mirrors JPH::EMotionType. C# side uses the same
// values. Phase 1 Longeron only uses Static and Kinematic; Dynamic is
// reserved for future use.
enum class BodyType : uint8_t {
    Static    = 0,
    Kinematic = 1,
    Dynamic   = 2,
};

// Shape kinds. Phase 1.5: Box, Sphere, ConvexHull. Phase 2+ extends
// with Capsule, Cylinder, MeshShape (non-convex).
//
// Encoding within a BodyCreate record's sub-shape list:
//   Box        — float3 half_extents (12 bytes)
//   Sphere     — float radius        (4 bytes)
//   ConvexHull — u32 vertex_count + vertex_count × float3
//                (4 + 12·N bytes; N capped at kMaxConvexHullVertices)
enum class ShapeKind : uint8_t {
    Box          = 0,
    Sphere       = 1,
    ConvexHull   = 2,
    TriangleMesh = 3,   // Phase 3b: streaming PQS terrain quads — static-only
};

// Cap on vertices passed to JPH::ConvexHullShapeSettings. Jolt itself
// will simplify if the hull is degenerate; we cap on the wire to
// bound input-buffer growth and to surface "absurd mesh" cases as
// truncation warnings rather than silently massive payloads.
inline constexpr uint32_t kMaxConvexHullVertices = 256;

// Cap on triangle counts for TriangleMesh sub-shapes. Default PQS
// quads ship 392 triangles (15×15 vertex grid → 2 × 14² triangles)
// so 4096 is a comfortable ~10× headroom; bumps if mods change PQS
// resolution (Parallax / Kopernicus stretch this).
inline constexpr uint32_t kMaxMeshTriangles = 4096;

// Constraint kinds carried in ConstraintCreate records.
//   Fixed   — JPH::FixedConstraint with mAutoDetectPoint=true. Rigidly
//             freezes the relative pose of two bodies. Phase 2.2.
//   SixDOF  — Phase 2.x compliant joint (per-axis spring/damper). Future.
enum class ConstraintKind : uint8_t {
    Fixed = 0,
};

// Object layer assignments (Phase 1). Phase 2 will extend with
// per-vessel layers for self-collision filtering.
enum class Layer : uint8_t {
    Static    = 0,
    Kinematic = 1,
    NumLayers = 2,
};

} // namespace longeron

#endif // LONGERON_RECORDS_H
