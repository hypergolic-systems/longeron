// Featherstone ABA forward dynamics for vessel internal flex.
//
// Architecture (Phase 5.1): fixed-base reduced-coordinate Featherstone
// ABA over the vessel's part tree, with **spherical joints** (3 angular
// DOF, anchors rigidly pinned). Jolt owns vessel-CoM motion (contacts,
// gravity, summed external wrench → rigid-body integration). ABA owns
// the per-edge joint state — quaternion q_i (child-relative-to-parent
// rotation about anchor) and ω_i (joint-relative angular velocity in
// parent frame), stored in PartFlex.delta_rot (cumulative, vessel-frame)
// and PartFlex.ang_vel respectively.
//
// Per substep:
//  1. Outward kinematics (root → leaf): propagate parent's spatial
//     velocity through joint motion subspace S = [I_3; 0_3]; cache
//     each part's vessel-frame delta_rot / delta_pos for downstream
//     consumers (RouteContactForce, RNEA, ApplyFlexToBodies).
//  2. Inward articulated-inertia pass (leaf → root): build I^A and
//     bias p^A at each link, with external wrench rotated into the
//     part's CURRENT frame (preserving feedback_physics_visuals_align —
//     forces act through visible flexed geometry, never rest).
//  3. Outward acceleration solve: 3×3 angular block solve per joint
//     yields qddot_i; spatial accel propagates outward.
//  4. Semi-implicit Euler integrate: ω += dt × qddot, then
//     q ← integrate_quaternion(q, ω, dt).
//
// Why reduced coords: hard translation constraint between anchors
// (the structural reality) is closed-form in the joint motion
// subspace — no PGS, no Lagrange, no penalty spring. Linear flex
// falls out from the joint chain through lever arms, never integrated.
//
// Stability: semi-implicit Euler is stable for harmonic oscillators
// when dt < 2/ω_n. With our default 30× stock stiffness on size-1
// stack joints (K_ang ≈ 1.8 MN·m/rad, I ≈ 0.5 t·m²) → ω_n ≈ 60 rad/s
// → stable dt < 33 ms. KSP runs at 50 Hz (dt = 20 ms) — borderline.
// We substep at 20× (1 ms substep) for ~30× margin. With linear DOF
// gone (no K_lin frequency contributing), the worst-case ω is
// strictly the angular mode.
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

// Number of internal substeps per Jolt physics tick. With Phase 5.1
// (spherical joints only — no linear stiffness contributing to ω_n)
// the highest natural freq under default 30× stock stiffness is the
// angular mode, ω_n ≈ 60 rad/s. Semi-implicit Euler is stable when
// dt × ω < 2, so dt < 33 ms is fine on its own. We substep at 20×
// → 1 ms substep → ample headroom for spikes (heavy parts on light
// joints).
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
