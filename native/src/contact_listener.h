// JPH::ContactListener implementation that emits ContactReport records
// into the per-tick output buffer. Runs from Jolt's worker threads
// during PhysicsSystem::Update — output appends are serialized via the
// world's internal mutex.

#ifndef LONGERON_CONTACT_LISTENER_H
#define LONGERON_CONTACT_LISTENER_H

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/ContactListener.h>

namespace longeron {

class LongeronWorld;

class ContactListenerImpl final : public JPH::ContactListener {
public:
    explicit ContactListenerImpl(LongeronWorld* world) : mWorld(world) {}

    // Phase 1: emit a contact record on add and persist. Validate is
    // accept-everything; remove is currently ignored (Phase 2+ may
    // surface lift-off events).

    JPH::ValidateResult OnContactValidate(
        const JPH::Body& body1,
        const JPH::Body& body2,
        JPH::RVec3Arg base_offset,
        const JPH::CollideShapeResult& result) override;

    void OnContactAdded(
        const JPH::Body& body1,
        const JPH::Body& body2,
        const JPH::ContactManifold& manifold,
        JPH::ContactSettings& settings) override;

    void OnContactPersisted(
        const JPH::Body& body1,
        const JPH::Body& body2,
        const JPH::ContactManifold& manifold,
        JPH::ContactSettings& settings) override;

    void OnContactRemoved(const JPH::SubShapeIDPair& pair) override;

private:
    void Emit(const JPH::Body& body1,
              const JPH::Body& body2,
              const JPH::ContactManifold& manifold);

    LongeronWorld* mWorld;
};

} // namespace longeron

#endif // LONGERON_CONTACT_LISTENER_H
