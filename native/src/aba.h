// Featherstone-style ABA forward dynamics for vessel internal flex.
//
// Architecture (Phase 5.0): each non-root part is a free 6-DOF body in
// vessel-body frame, coupled to its parent by a 6-DOF compliant
// spring-damper joint at the joint anchor (attach_local). Jolt owns
// the vessel-CoM motion (rigid-body integration under summed external
// wrench + contacts). ABA owns the flex residual — per-part state
// (delta_pos, delta_rot, lin_vel, ang_vel) tracked in vessel-body axes,
// integrated by semi-implicit (symplectic) Euler with substepping.
//
// Why "ABA-style" but maximal coords: with 6-DOF compliant joints,
// reduced-coordinate Featherstone ABA collapses to the same equations
// as maximal-coords spring-coupled bodies. The reduced formulation's
// advantage is for hard constraints (revolute, prismatic) — for pure
// soft compliance, the maximal form is equivalent and simpler.
//
// Stability: semi-implicit Euler is stable for harmonic oscillators
// when dt < 2/ω_n. With our default 30× stock stiffness on size-1
// stack joints (K_ang ≈ 1.8 MN·m/rad, I ≈ 0.5 t·m²) → ω_n ≈ 60 rad/s
// → stable dt < 33 ms. KSP runs at 50 Hz (dt = 20 ms) — borderline.
// We substep at 10× (500 Hz internal) for headroom.
//
// TODO (Phase 5.x): swap to implicit-Euler joint integration so we
// can run at native 50 Hz without substep cost, and handle stiffer
// joints stably. Featherstone Ch. 9 covers the formulation.

#ifndef LONGERON_ABA_H
#define LONGERON_ABA_H

#include "tree.h"

#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>
#include <Jolt/Math/Quat.h>

namespace longeron {

// Run one ABA forward step over a vessel tree.
//
// Inputs (in vessel-body frame):
//   v_body        — vessel body's linear velocity at its CoM (body axes)
//   omega         — vessel body's angular velocity
//   a_body        — vessel body's linear acceleration this tick
//   alpha         — vessel body's angular acceleration this tick
//   fixed_dt      — KSP physics tick (currently 0.02 s @ 50 Hz)
//
// State on `tree`:
//   tree.flex       — read+write: previous tick's flex state in / new
//                     tick's flex state out (semi-implicit Euler step).
//   tree.ext_force  — read: per-part external force this tick (vessel axes).
//   tree.ext_torque — read: per-part external torque about part CoM
//                     this tick (vessel axes).
//   tree.edge_compliance — read: per-edge K/C values.
//   tree.nodes      — read: mass / inertia / topology / rest geometry.
//
// Substepping: the function divides fixed_dt into kSubsteps internal
// steps and integrates flex state through each. External wrenches are
// held constant across all substeps (KSP only updates them at 50 Hz).
void RunAbaForwardStep(
    VesselTree& tree,
    JPH::Vec3 v_body,
    JPH::Vec3 omega,
    JPH::Vec3 a_body,
    JPH::Vec3 alpha,
    float fixed_dt,
    uint32_t body_id,
    std::vector<AbaPartDiagRecord>* diag_out);

// Number of internal substeps per Jolt physics tick. With Phase 5.0
// stiffness (kStiffnessScale=5×, kLinAngRatio=5) the highest natural
// freq is around ω_n ≈ 100 rad/s; semi-implicit Euler is stable when
// dt × ω < 2, so dt < 20 ms is fine. We substep at 20× → 1 ms substep
// → comfortable 5× margin.
inline constexpr int kAbaSubsteps = 20;

// Number of post-Upsert ticks during which ABA emits per-part diag
// records (ext_force, F_inertial, F_flex, delta_pos, delta_angle).
// 500 ticks ≈ 10 s at 50 Hz — long enough to capture the full
// transient from decouple through visible disintegration. Each
// record is ~60 bytes, ~10000 records over the window per vessel
// → ~600 kB of KSP.log noise per vessel; manageable for debugging.
inline constexpr int kAbaDiagWindow = 500;

} // namespace longeron

#endif // LONGERON_ABA_H
