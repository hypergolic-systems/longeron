# Phase 5.1.2 — Canonical Featherstone ABA, test-driven

## Context

Phase 5.1 v1 shipped a tree-coupled flex integrator that the code
honestly calls "ABA-style" but is actually **per-link Newton-Euler in
vessel frame + per-substep XPBD-style anchor projection**. It has the
right structural shape (no PGS, anchors rigidly pinned, angular spring
at joint) but the wrong *state representation*: each link's rotation
is stored in vessel frame (`PartFlex.delta_rot`) and the joint angle
is derived as `q_joint = parent.Conj() × child` each substep.

The consequence is that **rigid rotations of the parent are not
kinematically propagated to the child** — child's vessel-frame
rotation is a free integrand whose only coupling to the parent is
the angular spring. When SAS / gimbal commands cause the vessel to
rotate, parent and child rotate at different rates within a substep,
the spring fires to drag the child along, and you get a phase-lag
oscillation that the spring + damper can't remove because the spring
itself is what's creating the lag.

Ground-truth symptom (in-game test, 2026-04-30): a stack rocket
with central engine and side tanks is stable under axial loads but
gimbaling and SAS torque produce oscillatory engine-position
displacement and uncontrolled vessel roll. The screenshot shows the
engine separated from its parent by a visible gap — not "anchor
constraint slipping" (anchors *are* pinned each substep) but
extreme angular flex during oscillation, displacing the engine CoM
from its rest position.

This plan replaces the v1 forward step with **canonical reduced-coord
Featherstone ABA** per Featherstone *Rigid Body Dynamics Algorithms*
Ch. 7. The state representation flips: joint state is per-edge
(`q_joint`, `ω_joint`) and the child's vessel-frame rotation is
*derived* by composition with the parent. Rigid rotations of the
parent propagate kinematically; the spring fires only on actual
relative deformation.

## Working principle (unchanged from prior plan)

| Layer | Owner |
|---|---|
| Vessel-CoM motion | **Jolt** rigid body |
| Per-part SubShape transforms (visible flex) | Driven by ABA via `ModifyShapes` |
| Forward dynamics of internal flex | **Reduced-coord Featherstone ABA, fixed-base** (this plan) |
| Per-edge stress for break detection | **RNEA backward pass** (already in place, unchanged) |
| Joint type | **Spherical** for all structural attachments (3-DOF angular) |

## Approach — test-driven, milestone-gated

Each algorithm component lands behind a unit-test gate that runs
against the C ABI via the existing `Longeron.Native.Probe` console
harness. KSP integration is the last gate, not the first — we don't
re-deploy until the math is validated headlessly.

### Why test-driven this round

- Last round, KSP iteration was the verification loop. Every cycle
  was 5+ minutes of build → install → reset save → fly → observe.
  Numerical bugs hid behind game noise (visual artifacts, contact
  forces, gimbal commands). Symptoms got mistuned as parameters.
- The native bridge is already exercised headlessly by
  `Longeron.Native.Probe`: a console app that loads
  `liblongeron_native.dylib`, instantiates `Longeron.Native.World`,
  steps it, and asserts on output records. Runs via `dotnet run` /
  `mono` in seconds.
- We have a working canonical Featherstone implementation in C# at
  `mod/Longeron.Physics/src/Solver/ABA.cs` (the "vestigial" project
  per CLAUDE.md, but the math is intact and validated by
  `Longeron.Physics.Tests/Solver/{SimplePendulumTests,
  RigidStackTowerTests, RneaReciprocityTests}.cs`). It is the
  reference implementation we port to C++ guided by, *and* the
  oracle for differential testing.

### Test infrastructure (M1, lands first)

- Extend `Longeron.Native.Probe`. Same harness shape (`Run(name,
  action)` with try/catch, exit non-zero on failure). Group ABA
  scenarios under their own section.
- Add `mod/Longeron.Native.Probe/src/AbaTestRig.cs` — a fluent
  helper:
  - `Vessel rocket = rig.Vessel().BodyCreate(...).VesselTreeUpdate(parts, compliance)`
  - `rig.Step(dt)` — drains output records into per-part snapshots
  - `rig.PartPose(idx)` / `rig.JointWrench(idx)` — read latest
  - `rig.ApplyForceAtPart(idx, force, point)` — input convenience
- Add `mod/Longeron.Native.Probe/src/AbaScenarios.cs` — collection
  of scenario builders + assertions (one method per scenario).

The vessel body is `Dynamic` in Jolt for our scenarios (since ABA
needs `last_a_body`, `last_alpha` finite-diff). For pure-flex tests
where we want vessel CoM stationary, we apply a counter-gravity at
the root or set linear/angular velocity targets to zero between
ticks. Either is fine; the test rig provides both.

## Files to create / modify

### New (this plan)

- `native/src/spatial.h` — spatial-vector primitives (M2):
  `SpatialMotion` (6-vec [angular; linear]), `SpatialForce`,
  `SpatialInertia` (6×6 with shorthand for diagonal-at-CoM case),
  `SpatialTransform` (Plücker), spatial cross product `^×̂`.
- `mod/Longeron.Native.Probe/src/AbaTestRig.cs` — test rig helper
  (M1).
- `mod/Longeron.Native.Probe/src/AbaScenarios.cs` — scenario
  definitions (M1, M3-M6).

### Modified

- `native/src/aba.h` — minor: header comment update to reflect the
  canonical recursion shape.
- `native/src/aba.cpp` — full rewrite of `RunAbaForwardStep` (M3).
- `mod/Longeron.Native.Probe/src/Program.cs` — add `Run(...)` calls
  for new scenarios.

### Reference (read-only)

- `mod/Longeron.Physics/src/Solver/ABA.cs` — port from this. C#
  works in the same general formulation: outward kinematics →
  inward articulated-inertia → outward acceleration → integrate.
- `mod/Longeron.Physics/src/Spatial/*.cs` — port primitives from
  here.

### Unchanged

- C ABI surface (`bridge.{h,cpp}`).
- Wire format (`records.h`, `InputBuffer.cs`, `OutputBuffer.cs`).
- `World.cs` C# wrapper.
- `TopologyReconciler.cs` joint-compliance extraction.
- `LongeronSceneDriver.cs` pose composition.
- `tree.{h,cpp}` aside from per-tick consumers reading `delta_pos` /
  `delta_rot` (still vessel-frame derived output).
- RNEA backward pass.
- The `feedback_physics_visuals_align.md` invariant.

## Milestones

Each milestone has gates: don't move to the next until the gates
pass. Native build + Probe execution should complete in seconds.

### M1 — Test rig (no algorithm change)

**Adds:** `AbaTestRig` + smoke scenarios in `AbaScenarios.cs`.

**Tests** (run against current XPBD-style implementation; this round
just proves the harness):

1. *Single-part vessel, no forces, no flex.* `PartPose` not emitted
   for root; vessel pose tracks ballistically.
2. *Two-part vessel, no forces.* Both parts have zero flex.
3. *Two-part vessel, gravity, root pinned.* Child develops a steady
   flex (the v1 implementation will produce *some* answer — not yet
   asserting what — proves the bridge round-trip works for a flexing
   vessel).

**Gate:** all three tests pass against current code; baseline numbers
recorded for future comparison.

### M2 — Spatial-vector primitives in C++

**Adds:** `native/src/spatial.h`. Mirrors `Longeron.Physics/Spatial/*`:

- `SpatialMotion v = (ω, v_lin)` — 6-vec, top is angular.
- `SpatialForce f = (τ, f_lin)` — 6-vec, top is moment.
- `SpatialInertia` — block form for inertia-at-CoM (mass × I_3 in
  bottom-right, full I_local rotated to vessel frame in top-left,
  zero off-diagonal). Shorthand for the diagonal-inertia case we
  use.
- `SpatialTransform` (Plücker) — rotation R + translation r;
  applies as `^iX_p × v` and `^iX_p^T × f`.
- Spatial cross `v ×̂ x` and `v ×̂* f` (the latter for force-vector
  cross, dual operator).

**Tests** (M2, in Probe `Spatial` section):

1. *Identity transform is identity.*
2. *Cross product satisfies Jacobi-style identities* (compare against
   3-vec cross composed correctly).
3. *Inverse transform composes to identity.*
4. *Inertia-at-anchor parallel-axis matches direct formula.*

**Gate:** all four pass.

### M3 — Canonical Featherstone for spherical joints (the rewrite)

**Replaces:** `RunAbaForwardStep` body. Joint state is per-edge:
`q_joint[i]`, `ω_joint[i]` stored in `PartFlex.delta_rot` /
`PartFlex.ang_vel` *but reinterpreted* as joint-relative (child wrt
parent in parent's frame). `PartFlex.delta_pos` and `lin_vel` become
**output-only**, written at the end of each substep from joint chain.

**Per substep:**

1. **Outward kinematics** (root → leaf, parent-first order):
   - Root: `R_v[0] = I`, `pos_v[0] = com_local[0]`,
     `ω_v[0] = 0`, `v_v[0] = 0`, `a_v[0] = 0`.
     (Fixed-base flex frame; vessel CoM motion already subtracted
     into `F_flex`.)
   - For each non-root `i` with parent `p`:
     - `R_v[i] = R_v[p] × q_joint[i]`
     - `r_p_to_anchor = R_v[p] × (attach_local - com_local[p])`
     - `anchor_pos[i] = pos_v[p] + r_p_to_anchor`
     - `r_anchor_to_c = R_v[i] × (com_local[i] - attach_local)`
     - `pos_v[i] = anchor_pos[i] + r_anchor_to_c`
     - `ω_world_joint = R_v[p] × ω_joint[i]` (joint-rate in vessel)
     - `ω_v[i] = ω_v[p] + ω_world_joint`
     - `v_v[i] = v_v[p] + ω_v[p] × r_p_to_anchor + ω_v[i] × r_anchor_to_c`
     - Coriolis bias `c_i` (Featherstone Eq 7.30 specialized):
       `c_angular = ω_v[p] × ω_world_joint`
       `c_linear = ω_v[p] × (ω_v[p] × r_p_to_anchor) + ω_v[i] × (ω_v[i] × r_anchor_to_c)`
            `+ 2 × ω_world_joint × (ω_v[p] × r_anchor_to_c) ...` (port from C# ref)
   - Cache `delta_pos[i] = pos_v[i] - com_local[i]` and
     `delta_rot[i] = R_v[i]` for downstream consumers + diagnostics.

2. **Inward articulated-inertia pass** (leaf → root):
   - Each link initialized: `I^A_i = M_i` (spatial inertia at link
     CoM, vessel frame), `p^A_i = bias`. Bias includes:
     - Gyroscopic: `ω_v[i] ×̂* (M_i × v_v[i])`
     - External wrench from `F_flex_i` / `T_flex_i` (vessel frame at
       part CoM), expressed as a spatial force at link CoM, sign-
       flipped (Featherstone convention).
     - Spring + damper joint torque at the parent's anchor:
       `T_spring = -k_ang × axisAngle(q_joint[i]) - c_ang × ω_joint[i]`
       (in parent frame). Acts on child at the joint anchor (in
       child's frame after transformation). Equal-opposite on
       parent.
   - Aggregate child contributions to parent:
     `I^A_p += ^iX_p^T × (I^A_i - I^A_i S (S^T I^A_i S)⁻¹ S^T I^A_i) × ^iX_p`
     `p^A_p += ^iX_p^T × (p^A_i + I^A_i × c_i + I^A_i S × (S^T I^A_i S)⁻¹ × (-S^T p^A_i + τ_i))`
     For spherical, `S = [I_3; 0]` and `S^T I^A_i S` is the upper-
     left 3×3 block of `I^A_i`. Invertible 3×3 per joint.

3. **Outward acceleration solve** (root → leaf):
   - Root: `a_root = 0` (fixed-base flex frame).
   - For each non-root child:
     `qddot_i = (S^T I^A_i S)⁻¹ × (-S^T p^A_i + τ_spring_i - S^T I^A_i × ^iX_p × a_p)`
     `a_i = ^iX_p × a_p + S × qddot_i + c_i`

4. **Integrate joint state** (semi-implicit Euler):
   - `ω_joint[i] += dt × qddot_i`
   - `q_joint[i] = integrate_quaternion(q_joint[i], ω_joint[i], dt)`

5. **Compose vessel-frame outputs:** already done in pass 1
   (`delta_pos[i]`, `delta_rot[i]` written into PartFlex).

**Tests** (in Probe `Aba` section):

1. *Single-link spherical pendulum at rest, no gravity.*
   Initial `q_joint = identity`. Expect zero motion forever.

2. *Spherical pendulum equilibrium under gravity.*
   1-link vessel: root + 1 child with offset CoM. Apply uniform
   gravity. Steady-state angular deflection `θ_ss = M·g·L / K_ang`.
   Settle for 1 simulated second; assert `|θ_observed - θ_ss| /
   θ_ss < 0.05`.

3. *Spherical pendulum oscillation period* (with damping disabled).
   Small initial deflection, no damping. Period `T = 2π × √(I /
   K_ang)`. Run for 10 periods; assert phase drift < 5%.

4. *Spherical pendulum critical damping decay.*
   Initial deflection, critical damping. Settles to within 1% of
   equilibrium in `~3 × τ` where `τ = 1/(ζ ω_n)`.

**Gate:** scenarios 1-4 pass. M3 ships only if all pass.

**Note on the C# reference:** `mod/Longeron.Physics/src/Solver/ABA.cs`
is read-only reference only — it was never validated in-game and is
not an oracle. We port guided by its structure but the C++ ground
truth is the analytic gates (1-4 above), not parity with the C#.

### M4 — Multi-link stack

**Tests:**

1. *3-part stack at rest, no forces.* All three parts at rest pose;
   joint wrenches near zero.
2. *3-part stack, axial gravity.* Bottom joint bears total weight;
   middle joint bears top + middle weight; RNEA tension matches
   `m_above × g` to 1%.
3. *3-part stack, lateral gravity (offset 90°).* Bending case; each
   joint deflects to `M_below_lever × g_lat / K_ang_joint`. Match
   to 5%.
4. *3-part stack, transient release.* Start with a configuration
   bent 5° at the middle joint; release with no external force;
   verify the chain damps to rest within expected time.

**Gate:** all four pass.

### M5 — Booster + side masses (the symptom test)

The exact configuration that exposed the v1 bug.

**Setup:** central 3-part stack with two side masses (~½ central
mass each) attached at radial decouplers on the middle part.

**Tests:**

1. *Configuration at rest, axial gravity.* All parts at rest within
   tolerance; side masses don't oscillate; joint stresses match
   analytic.
2. *SAS-style root torque.* Apply 5 kN·m torque on the root for
   500 ms; verify side masses follow the rigid rotation (their
   `q_joint` stays within 0.5° of identity throughout). **This is
   the test the v1 implementation fails.**
3. *Engine-style offset thrust.* Apply 50 kN axial force on the
   bottom of the central stack with a 1° offset (simulating
   gimbal). Verify: (a) bend angle steady-state within 0.5° of
   analytic; (b) no oscillation; (c) thrust direction follows the
   engine's actual flexed orientation.

**Gate:** all three pass. Especially test 2 — this is the regression
test for what we're fixing.

### M6 — Long-duration stability

**Tests:**

1. *5000-tick free cruise.* M5 booster, no inputs. Track max angular
   flex; assert bounded < 1°. Assert clamp never fires.
2. *5000-tick sinusoidal SAS excitation.* Sine wave torque at 1 Hz
   amplitude 5 kN·m for 50 ticks, then zero. Verify response
   bounded throughout, decays after excitation ends, no NaN, no
   drift.

**Gate:** both pass.

### M7 — KSP integration test (the real-world gate)

Re-deploy and re-run the gimbal/SAS scenario from the screenshot.

**Acceptance:**
- No visible engine displacement under gimbal commands.
- Vessel rotates as commanded under SAS without overshoot or roll.
- No clamp firings in the diag stream over a 60-second flight.

If M7 fails despite M1-M6 passing, the next debugging step is the
real-vs-simulated-vessel inertia mismatch (Jolt's auto-CoM versus
real mass distribution — flagged in the BodyMassDiag output already).

## Out of scope

- **Implicit Euler** (Featherstone Ch. 9): swap-in later if M6
  long-duration tests show stability margin issues. Same wire
  format / same joint state.
- **Revolute / prismatic joints**: Phase 5.2 / 5.3. Reduced-coord
  framework supports them by joint motion subspace `S`; adding is
  additive.
- **Loop closures** (KAS / docking-ring cycles inside a vessel):
  out per the user's scope.
- **Wheels / BG**: Phase 5.3+.
- **`PartFlex` layout cleanup** (split per-edge `JointState`
  struct): defensible follow-up; out this round.

## Risk / scope estimate

- M1: ~100 LOC C# (test rig + 3 scenarios). 0.5 day.
- M2: ~300 LOC C++ (spatial primitives + 4 tests). 1 day.
- M3: ~500 LOC C++ (full forward-pass rewrite + 4-5 tests). 2-3
  days, gated on M3 tests passing — extra time if differential
  test against C# uncovers porting bugs.
- M4: ~150 LOC C# scenarios. 0.5 day.
- M5: ~200 LOC C# scenarios. 0.5 day.
- M6: ~100 LOC C# scenarios. 0.25 day.
- M7: in-game test session. Whatever it takes.

**Total ~5-7 days of focused work.** All steps are reversible —
each milestone leaves the codebase buildable.

## Decision points

None blocking; the reference C# ABA already settles the
formulation question. If during M3 the C# port reveals a
numerical issue we want to fix before the C++ side, that's
where we'd pause.
