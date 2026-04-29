#include "contact_solve_listener.h"
#include "world.h"
#include "tree.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Constraints/ContactConstraintManager.h>
#include <Jolt/Physics/Constraints/ConstraintPart/AxisConstraintPart.h>

namespace longeron {

void ContactSolveListenerImpl::OnContactsSolved(
    const JPH::ContactConstraintManager& mgr)
{
    if (mWorld == nullptr) return;
    const float dt = mWorld->GetCurrentDt();
    if (dt <= 1e-6f) return;
    const float inv_dt = 1.0f / dt;

    auto& tree = mWorld->GetTreeRegistry();

    const uint32_t num = mgr.GetNumConstraints();
    for (uint32_t ci = 0; ci < num; ++ci) {
        const auto& c = mgr.GetConstraint(ci);

        const JPH::Vec3 normal = c.GetWorldSpaceNormal();
        JPH::Vec3 t1, t2;
        c.GetTangents(t1, t2);

        // Sum lambdas across this constraint's contact points (up to 4
        // per manifold). Each constraint represents a single
        // sub-shape pair = single (KSP-part, KSP-part-or-static) pair,
        // so summing within is the right granularity.
        float lambda_n  = 0.0f;
        float lambda_t1 = 0.0f;
        float lambda_t2 = 0.0f;
        JPH::RVec3 sum_point = JPH::RVec3::sZero();
        int point_count = 0;

        const JPH::RMat44 xform1 = c.mBody1->GetCenterOfMassTransform();

        for (const auto& wcp : c.mContactPoints) {
            lambda_n  += wcp.mNonPenetrationConstraint.GetTotalLambda();
            lambda_t1 += wcp.mFrictionConstraint1.GetTotalLambda();
            lambda_t2 += wcp.mFrictionConstraint2.GetTotalLambda();

            // Body-local position on body 1 → world via body1's CoM
            // transform. Float3 → Vec3 via sLoadFloat3Unsafe (same
            // pattern Jolt itself uses; the surrounding allocation
            // pads safely past the 12 bytes).
            JPH::Vec3 local1 = JPH::Vec3::sLoadFloat3Unsafe(
                wcp.mContactPoint->mPosition1);
            sum_point += xform1 * local1;
            ++point_count;
        }
        if (point_count == 0) continue;

        const JPH::RVec3 avg_point = sum_point / static_cast<float>(point_count);

        // Total impulse vector this tick, world axes.
        const JPH::Vec3 J_world =
            lambda_n  * normal +
            lambda_t1 * t1 +
            lambda_t2 * t2;
        const JPH::Vec3 F_world = J_world * inv_dt;

        // Jolt convention: mWorldSpaceNormal points "along which to
        // move body 2 out of collision" (Jolt/Physics/Collision/
        // ContactListener.h:30). The non-penetration impulse pushes
        // body 2 along +normal and body 1 along -normal (Newton's 3rd).
        // For a vessel resting on ground: ground = body 1, vessel = body
        // 2 (or vice versa) — whichever way Jolt orders them, the side
        // that gets pushed AWAY from the other is the one receiving
        // +F_world; the other gets -F_world.
        const uint32_t user1 = mWorld->GetUserIdForBody(*c.mBody1);
        const uint32_t user2 = mWorld->GetUserIdForBody(*c.mBody2);

        if (user1 != 0) {
            tree.RouteContactForce(
                user1, -F_world, avg_point,
                c.mBody1->GetCenterOfMassPosition(),
                c.mBody1->GetRotation());
        }
        if (user2 != 0) {
            tree.RouteContactForce(
                user2, F_world, avg_point,
                c.mBody2->GetCenterOfMassPosition(),
                c.mBody2->GetRotation());
        }
    }
}

} // namespace longeron
