// Jolt object/broadphase layer configuration for Longeron.
//
// Phase 1 has two layers: STATIC (terrain, KSC, plane) and KINEMATIC
// (vessel parts). Phase 2 will likely partition KINEMATIC into per-
// vessel layers for self-collision filtering.

#ifndef LONGERON_LAYERS_H
#define LONGERON_LAYERS_H

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/CollisionGroup.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>

namespace longeron {

namespace ObjLayers {
    static constexpr JPH::ObjectLayer STATIC      = 0;
    static constexpr JPH::ObjectLayer KINEMATIC   = 1;
    static constexpr JPH::uint        NUM_LAYERS  = 2;
}

namespace BPLayers {
    static constexpr JPH::BroadPhaseLayer STATIC(0);
    static constexpr JPH::BroadPhaseLayer MOVING(1);
    static constexpr JPH::uint NUM_LAYERS = 2;
}

class ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer a, JPH::ObjectLayer b) const override {
        // STATIC ↔ STATIC: never collide (no impulse to compute either way).
        // STATIC ↔ KINEMATIC: yes — Longeron consumes contact reports.
        // KINEMATIC ↔ KINEMATIC: yes — inter-vessel contacts.
        if (a == ObjLayers::STATIC && b == ObjLayers::STATIC) return false;
        return true;
    }
};

class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface {
public:
    BPLayerInterfaceImpl() {
        mObjectToBroadPhase[ObjLayers::STATIC]    = BPLayers::STATIC;
        mObjectToBroadPhase[ObjLayers::KINEMATIC] = BPLayers::MOVING;
    }

    JPH::uint GetNumBroadPhaseLayers() const override { return BPLayers::NUM_LAYERS; }

    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override {
        JPH_ASSERT(layer < ObjLayers::NUM_LAYERS);
        return mObjectToBroadPhase[layer];
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const override {
        switch ((JPH::BroadPhaseLayer::Type)layer) {
        case (JPH::BroadPhaseLayer::Type)BPLayers::STATIC: return "STATIC";
        case (JPH::BroadPhaseLayer::Type)BPLayers::MOVING: return "MOVING";
        default: JPH_ASSERT(false); return "INVALID";
        }
    }
#endif

private:
    JPH::BroadPhaseLayer mObjectToBroadPhase[ObjLayers::NUM_LAYERS];
};

class ObjectVsBroadPhaseLayerFilterImpl final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer obj, JPH::BroadPhaseLayer bp) const override {
        // STATIC objects only collide with the MOVING broadphase tree
        // (kinematic + future dynamic bodies). Skipping STATIC ↔ STATIC
        // saves the broadphase work of intersecting two static trees.
        if (obj == ObjLayers::STATIC) {
            return bp == BPLayers::MOVING;
        }
        // KINEMATIC objects collide with everything.
        return true;
    }
};

// Per-vessel collision filtering uses Jolt's stock GroupFilterTable
// rather than a custom subclass — Jolt is statically linked with
// JPH_EXPORT compiling to nothing, which combined with our hidden
// default visibility leaves base class typeinfo unexported, breaking
// derived-class subtyping at link time. GroupFilterTable's built-in
// CanCollide rules already match what we need:
//   - One body has cInvalidGroup → collide (terrain / synthetic
//     ground bypasses the filter).
//   - Different group IDs → collide (different vessels).
//   - Same group ID + same SubGroupID → don't collide.
// We assign one group ID per vessel and a constant SubGroupID = 0
// for every part, so any two parts in the same vessel skip
// collision.

} // namespace longeron

#endif // LONGERON_LAYERS_H
