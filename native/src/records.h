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

    // ---- Output records (native → C#) ---------------------------------
    BodyPose          = 64,  // u32 user_id, double3 pos, float4 rot, float3 lin_vel, float3 ang_vel
    ContactReport     = 65,  // u32 user_id_a, u32 user_id_b, double3 point, float3 normal, float depth, float impulse
};

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
    Box        = 0,
    Sphere     = 1,
    ConvexHull = 2,
};

// Cap on vertices passed to JPH::ConvexHullShapeSettings. Jolt itself
// will simplify if the hull is degenerate; we cap on the wire to
// bound input-buffer growth and to surface "absurd mesh" cases as
// truncation warnings rather than silently massive payloads.
inline constexpr uint32_t kMaxConvexHullVertices = 256;

// Object layer assignments (Phase 1). Phase 2 will extend with
// per-vessel layers for self-collision filtering.
enum class Layer : uint8_t {
    Static    = 0,
    Kinematic = 1,
    NumLayers = 2,
};

} // namespace longeron

#endif // LONGERON_RECORDS_H
