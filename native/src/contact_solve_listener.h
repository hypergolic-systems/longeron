// Phase 4.2 — listen for the post-solve callback we patched into Jolt
// (see Jolt/Physics/Constraints/ContactConstraintManager.h —
// ContactSolveListener) and route each constraint's resolved impulse
// (normal + 2 friction lambdas) into the per-part external-wrench
// accumulator. Each KSP part is its own sub-shape inside the vessel's
// compound, so Jolt creates one ContactConstraint per (sub-shape,
// sub-shape) pair — three landing legs touching the ground = three
// independent constraints with their own lambdas. This gives us
// physically accurate per-part contact reactions, not aggregates.

#ifndef LONGERON_CONTACT_SOLVE_LISTENER_H
#define LONGERON_CONTACT_SOLVE_LISTENER_H

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Constraints/ContactConstraintManager.h>

namespace longeron {

class LongeronWorld;

class ContactSolveListenerImpl final : public JPH::ContactSolveListener {
public:
    explicit ContactSolveListenerImpl(LongeronWorld* world) : mWorld(world) {}

    void OnContactsSolved(const JPH::ContactConstraintManager& mgr) override;

private:
    LongeronWorld* mWorld;
};

} // namespace longeron

#endif // LONGERON_CONTACT_SOLVE_LISTENER_H
