#include "contact_listener.h"
#include "world.h"

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>

namespace longeron {

// Decode a CompoundShape SubShapeID to its sub-shape index.
// Returns 0xFFFF if the shape isn't a compound or decoding fails.
// Used for diagnostic contact reports — tells us *which* part on the
// vessel made each contact pair.
static uint16_t DecodeSubShapeIndex(const JPH::Shape* shape, JPH::SubShapeID id) {
    if (shape == nullptr) return 0xFFFFu;
    auto sub = shape->GetSubType();
    const JPH::CompoundShape* compound = nullptr;
    if (sub == JPH::EShapeSubType::StaticCompound
        || sub == JPH::EShapeSubType::MutableCompound)
    {
        compound = static_cast<const JPH::CompoundShape*>(shape);
    }
    if (compound == nullptr) return 0xFFFFu;
    JPH::SubShapeID remainder;
    uint32_t idx = compound->GetSubShapeIndexFromID(id, remainder);
    return idx > 0xFFFEu ? 0xFFFEu : static_cast<uint16_t>(idx);
}

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
    // Sub-shape index on body B (the dynamic vessel). Tells C# which
    // part collider this contact is for via the vessel's SubShapeMap.
    // 0xFFFF means non-compound or decode failure (treat as unknown).
    const uint16_t subB = DecodeSubShapeIndex(body2.GetShape(), manifold.mSubShapeID2);

    mWorld->AppendContactReport(a, b, point, normal, depth, impulse, subB);
}

} // namespace longeron
