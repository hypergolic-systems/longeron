#include "contact_listener.h"
#include "world.h"

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Collision/ContactListener.h>

namespace longeron {

JPH::ValidateResult ContactListenerImpl::OnContactValidate(
    const JPH::Body& /*body1*/,
    const JPH::Body& /*body2*/,
    JPH::RVec3Arg /*base_offset*/,
    const JPH::CollideShapeResult& /*result*/)
{
    return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
}

void ContactListenerImpl::OnContactAdded(
    const JPH::Body& body1,
    const JPH::Body& body2,
    const JPH::ContactManifold& manifold,
    JPH::ContactSettings& /*settings*/)
{
    Emit(body1, body2, manifold);
}

void ContactListenerImpl::OnContactPersisted(
    const JPH::Body& body1,
    const JPH::Body& body2,
    const JPH::ContactManifold& manifold,
    JPH::ContactSettings& /*settings*/)
{
    Emit(body1, body2, manifold);
}

void ContactListenerImpl::OnContactRemoved(const JPH::SubShapeIDPair& /*pair*/)
{
    // Phase 1: silent. Phase 2+ may surface lift-off as a separate
    // record so ABA can release any stuck-contact state.
}

void ContactListenerImpl::Emit(
    const JPH::Body& body1,
    const JPH::Body& body2,
    const JPH::ContactManifold& manifold)
{
    const uint32_t a = mWorld->GetUserIdForBody(body1);
    const uint32_t b = mWorld->GetUserIdForBody(body2);
    if (a == 0 || b == 0) return;  // body unknown / destroyed mid-step

    // Use the deepest contact point as the representative for Phase 1.
    // Phase 2+ may emit per-contact-point records; for now one report
    // per body pair per tick is enough.
    const JPH::RVec3 point = manifold.GetWorldSpaceContactPointOn1(0);
    const JPH::Vec3  normal = manifold.mWorldSpaceNormal;
    const float      depth  = manifold.mPenetrationDepth;
    const float      impulse = 0.0f;  // Phase 1: not propagated

    mWorld->AppendContactReport(a, b, point, normal, depth, impulse);
}

} // namespace longeron
