// LongeronWorld — Jolt PhysicsSystem owner + per-tick step driver.
//
// One instance per flight scene. Owns the body registry (user_id ↔
// JPH::BodyID), the temp allocator + job system, and the contact
// listener that emits ContactReport records into the per-tick output
// stream.

#ifndef LONGERON_WORLD_H
#define LONGERON_WORLD_H

#include <Jolt/Jolt.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Constraints/Constraint.h>

#include "bridge.h"  // LongeronConfig
#include "layers.h"
#include "records.h"

#include <unordered_map>
#include <memory>
#include <cstdint>

namespace longeron {

class ContactListenerImpl;

class LongeronWorld {
public:
    explicit LongeronWorld(const ::LongeronConfig& cfg);
    ~LongeronWorld();

    LongeronWorld(const LongeronWorld&) = delete;
    LongeronWorld& operator=(const LongeronWorld&) = delete;

    // Per-tick: parse the input record stream, advance Jolt by dt,
    // emit poses + contacts into the output buffer. Returns the number
    // of bytes written; if output_cap is too small returns the
    // required size and writes nothing (caller can detect).
    int32_t Step(const uint8_t* input, size_t input_len,
                 uint8_t* output, size_t output_cap, size_t* output_len,
                 float dt);

    // Used by ContactListenerImpl to look up the user_id stored in a
    // body's UserData. Returns 0 (Invalid) if the body was destroyed
    // mid-step or is unknown.
    uint32_t GetUserIdForBody(const JPH::Body& body) const;

    // Output buffer accessors used by the contact listener while
    // PhysicsSystem::Update is running. The contact listener fires from
    // worker threads, so output writes are protected by mOutputMutex.
    void AppendContactReport(uint32_t user_id_a, uint32_t user_id_b,
                             JPH::RVec3 point, JPH::Vec3 normal,
                             float depth, float impulse);

private:
    // -- Input handlers -----------------------------------------------
    void HandleBodyCreate(const uint8_t*& cur, const uint8_t* end);
    void HandleBodyDestroy(const uint8_t*& cur, const uint8_t* end);
    void HandleSetGravity(const uint8_t*& cur, const uint8_t* end);
    void HandleSetKinematicPose(const uint8_t*& cur, const uint8_t* end);
    void HandleForceDelta(const uint8_t*& cur, const uint8_t* end);
    void HandleConstraintCreate(const uint8_t*& cur, const uint8_t* end);
    void HandleConstraintDestroy(const uint8_t*& cur, const uint8_t* end);
    void HandleShiftWorld(const uint8_t*& cur, const uint8_t* end);
    void HandleSetBodyGroup(const uint8_t*& cur, const uint8_t* end);
    void HandleMassUpdate(const uint8_t*& cur, const uint8_t* end);

    // -- Output emission ----------------------------------------------
    void EmitBodyPoses();

    // -- Output buffer cursor -----------------------------------------
    bool ReserveOutput(size_t bytes);   // returns false if cap exceeded

private:
    // Jolt's TempAllocatorImpl and JobSystemThreadPool both call into
    // Jolt globals (allocator, factory) at construction time. Those
    // globals are initialized lazily once per process via
    // EnsureJoltGlobals(); to make sure that runs *before* the rest of
    // the member-initializer list executes, declare this sentinel
    // first — C++ runs base/member initializers in declaration order.
    struct JoltGlobalsSentinel { JoltGlobalsSentinel(); };
    JoltGlobalsSentinel          mJoltGlobalsSentinel;

    // Jolt
    JPH::TempAllocatorImpl       mTempAllocator;
    JPH::JobSystemThreadPool     mJobSystem;
    BPLayerInterfaceImpl                  mBPLayerInterface;
    ObjectVsBroadPhaseLayerFilterImpl     mObjectVsBPFilter;
    ObjectLayerPairFilterImpl             mObjectVsObjectFilter;
    JPH::Ref<JPH::GroupFilterTable>       mGroupFilter;
    JPH::PhysicsSystem           mPhysicsSystem;
    std::unique_ptr<ContactListenerImpl>  mContactListener;

    // Body registry: user_id → JPH::BodyID. Body's mUserData stores
    // user_id (uint64) so reverse lookup uses Body::GetUserData() and
    // doesn't require this map.
    std::unordered_map<uint32_t, JPH::BodyID> mBodyRegistry;

    // Constraint registry: caller-assigned constraint_id → Jolt
    // Constraint ref. Phase 2.2 only adds FixedConstraint; future
    // kinds extend the same map.
    std::unordered_map<uint32_t, JPH::Ref<JPH::Constraint>> mConstraintRegistry;

    // Per-tick output buffer cursor (only valid during Step).
    uint8_t* mOutputBuffer = nullptr;
    size_t   mOutputCap    = 0;
    size_t   mOutputLen    = 0;
    bool     mOutputOverflow = false;

    uint64_t mStepCount = 0;
    bool     mNeedsBroadphaseOptimize = false;
};

} // namespace longeron

#endif // LONGERON_WORLD_H
