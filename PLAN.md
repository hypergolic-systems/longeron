# Phase 5.1.2 — Pinocchio-backed reduced-coord ABA

## Context

Phase 5.1 v1 shipped a tree-coupled flex integrator that the code
honestly called "ABA-style" but was actually **per-link Newton-Euler
in vessel frame + per-substep XPBD-style anchor projection**. It had
the right structural shape (no PGS, anchors rigidly pinned, angular
spring at joint) but the wrong *state representation*: each link's
rotation was stored in vessel frame and the joint angle was derived
as `q_joint = parent.Conj() × child` each substep.

The consequence was that **rigid rotations of the parent were not
kinematically propagated to the child** — child's vessel-frame
rotation was a free integrand whose only coupling to the parent was
the angular spring. When SAS / gimbal commands caused the vessel to
rotate, parent and child rotated at different rates within a
substep, the spring fired to drag the child along, and we got a
phase-lag oscillation that the spring + damper couldn't remove
because the spring itself was creating the lag.

Ground-truth symptom (in-game test, 2026-04-30): a stack rocket
with central engine and side tanks was stable under axial loads but
gimbaling and SAS torque produced oscillatory engine-position
displacement and uncontrolled vessel roll. The screenshot showed the
engine separated from its parent by a visible gap — extreme angular
flex during oscillation, displacing the engine CoM from rest.

This plan replaced the v1 forward step with **canonical reduced-coord
Featherstone ABA** (Featherstone *Rigid Body Dynamics Algorithms*
Ch. 7). Joint state flips to per-edge (`q_joint`, `ω_joint`); the
child's vessel-frame rotation is *derived* by composition with the
parent. Rigid rotations of the parent propagate kinematically; the
spring fires only on actual relative deformation.

A test-driven mid-pivot evaluated [Pinocchio](https://stack-of-tasks.github.io/pinocchio/)
(`stack-of-tasks/pinocchio` v3.9.0) as the ABA implementation
instead of writing it in tree. **The spike succeeded:** M1–M3 gates
(single-pendulum equilibrium / period / critical damping) pass on
first compile. Pinocchio is now the forward-dynamics backend for
Phase 5.1.2.

## Working principle

| Layer | Owner |
|---|---|
| Vessel-CoM motion | **Jolt** rigid body |
| Per-part SubShape transforms (visible flex) | Driven by ABA via `ModifyShapes` |
| Forward dynamics of internal flex | **Pinocchio** reduced-coord ABA, fixed-base |
| Per-edge stress for break detection | **RNEA backward pass** in `tree.cpp:RunAdvisoryPass`, unchanged |
| Joint type | **Spherical** for all structural attachments (3-DOF angular) |
| Per-vessel `pinocchio::Model` lifecycle | Built once per topology change, keyed by `VesselTree.topology_version` |
| Substepping | 20× substeps @ 1 ms per Jolt 50 Hz tick, in our wrapper |
| Frame conversion (Pinocchio `linear;angular` ↔ our `angular;linear`) | Two helpers in `aba.cpp` |

Why Pinocchio rather than custom: the M3 gates (analytic single-
pendulum) pass on first compile with a battle-tested implementation,
and the deployment cost — after a round of optimization (header-only
Pinocchio + LTO + `-dead_strip_dylibs` + `strip -x`) — collapses to
a 1.5 MB self-contained dylib (smaller than the 4.84 MB
custom-baseline) with no new runtime deps. Build time at `-j 4` is
~32 s clean (~2× baseline).

The custom canonical Featherstone implementation (in
`mod/Longeron.Physics/src/Solver/ABA.cs` and the spatial primitives
under `Longeron.Physics/src/Spatial/`) remains as a read-only
reference. The C++ `native/src/spatial.h` is no longer the
canonical forward-dynamics math but is still used by the RNEA
backward pass.

## Current status

- **M1 (test rig):** ✅ landed —
  `mod/Longeron.Native.Probe/src/{AbaTestRig.cs,AbaScenarios.cs}`,
  3 smoke scenarios.
- **M2 (spatial primitives self-test):** ✅ passes via
  `World.SpatialSelfTest()`. `spatial.h` retained for RNEA.
- **M3 (single-pendulum analytic gates):**
  ✅ all four pass with Pinocchio backing —
  - M3.1 *Pendulum at rest, no motion*
  - M3.2 *Equilibrium = F·L/K_ang* (±5%)
  - M3.3 *Undamped period = 2π√(I/K)* (±5%)
  - M3.4 *Critical-damped decay to rest* (~3·τ)

Remaining work tracked below.

## Approach — test-driven, milestone-gated

Each milestone runs against the C ABI via the existing
`Longeron.Native.Probe` console harness. KSP integration is the last
gate, not the first — we don't re-deploy until the math is validated
headlessly.

The vessel body is `Dynamic` in Jolt for our scenarios (since ABA
needs `last_a_body`, `last_alpha` finite-diff). For pure-flex tests
where we want vessel CoM stationary, we apply counter-gravity at the
root or set linear/angular velocity targets to zero between ticks.
Either is fine; the test rig provides both.

**Asymmetry of trust** (per `feedback_battle_tested_priors.md`):
Pinocchio's solver is battle-tested; our scenarios are
hand-derivations. When a scenario disagrees with Pinocchio,
investigate both — Pinocchio gets the priors, but the scenario's
analytic target / tolerance / setup is not above suspicion. Track
scenario edits in commit messages.

## Native bridge architecture

Wire-format contract (locked, must not change):

- Input record `VesselTreeUpdate` carries per-part `parent_idx,
  mass, com_local, inertia_diag, attach_local, K_ang, C_ang`.
- Input record `ForceAtPosition` (world-frame force + world point +
  `part_idx` tag) is routed by `TreeRegistry::AccumulateExternalForce`
  into `tree.ext_force[i]` / `tree.ext_torque[i]` (vessel-body axes).
- Output record `PartPose` carries per non-root part: `delta_pos`
  (vessel-frame CoM deflection from rest) + `delta_rot` (vessel-frame
  cumulative rotation). C# composes Unity rb pose:
  ```
  rb.position = vesselPos + vesselRot · (PartLocalPos + delta_pos)
  rb.rotation = vesselRot · delta_rot · PartLocalRot
  ```
- Output record `JointWrench` carries per-edge force/torque in
  joint-frame (X = axial signed, YZ = shear; X = torsion signed,
  YZ = bending). Produced by RNEA backward pass on the deflected
  geometry (`tree.cpp:RunAdvisoryPass`).

Per-tick step (in `native/src/aba.cpp:RunAbaForwardStep`):

1. Look up cached `PinocchioState` for `body_id`. Rebuild
   `pinocchio::Model + Data + q/v/tau/fext` if
   `tree.topology_version` changed since last build.
2. Compute `F_inertial / T_inertial / F_flex / T_flex` per part
   (vessel frame); held constant across substeps.
3. Pack `q` from `tree.flex[i].delta_rot` (decompose to per-edge
   joint quat); pack `v` from `tree.flex[i].ang_vel`.
4. Substep loop (kAbaSubsteps = 20, dt = fixed_dt / 20):
   - `forwardKinematics(model, data, q)` → `data.oMi[jid]` =
     vessel-frame joint placement.
   - For each non-root part: `tau = -K_ang·axisAngle(q_joint) -
     C_ang·ω_joint`; `fext[jid] = VesselWrenchToJointFrame(F_flex,
     T_flex, R_v_i, com_in_body)` (translates angular;linear →
     linear;angular).
   - `pinocchio::aba(model, data, q, v, tau, fext)` → `data.ddq`.
   - Semi-implicit Euler: `v += dt·ddq`, `q = pinocchio::integrate(q,
     v·dt)` (manifold integration on unit-quat Lie group).
5. Write back `tree.flex[i]`: `delta_pos = oMi[jid].translation() +
   R_v_i · com_in_body - com_local`, `delta_rot = R_v_i`,
   `ang_vel = v.segment<3>(idx_v[i])`.
6. Safety clamp; if any clamp fired, invalidate the Pinocchio cache
   (the next tick repacks from the zeroed flex state).

## Milestones

Each milestone has gates: don't move to the next until the gates
pass. Native build + Probe execution should complete in seconds at
`-j 4` (default, capped to avoid OOM on the 24 GB / 14-core dev box).

### M4 — Multi-link stack

**Tests** (~150 LOC of C# in `AbaScenarios.cs`):

1. *3-part stack at rest, no forces.* All three parts at rest pose;
   joint wrenches near zero.
2. *3-part stack, axial gravity.* Bottom joint bears total weight
   (RNEA tension matches `m_above × g` to 1%); middle joint bears
   top + middle weight.
3. *3-part stack, lateral gravity (offset 90°).* Bending case; each
   joint deflects to `M_below_lever × g_lat / K_ang_joint`. Match
   to 5%.
4. *3-part stack, transient release.* Start with a 5° bend at the
   middle joint; release; chain damps to rest within `~3·τ` of the
   slowest mode.

**Gate:** all four pass.

### M5 — Booster + side masses (the symptom test)

The exact configuration that exposed the v1 bug.

**Setup:** central 3-part stack with two side masses (~½ central
mass each) at radial decouplers on the middle part.

**Tests:**

1. *Configuration at rest, axial gravity.* All parts at rest within
   tolerance; side masses don't oscillate; joint stresses match
   analytic.
2. *SAS-style root torque.* 5 kN·m torque on root for 500 ms; side
   masses' `q_joint` stays within 0.5° of identity throughout.
   **The v1 regression test** — Pinocchio's reduced-coord forward
   kinematics handles this by construction.
3. *Engine-style offset thrust.* 50 kN axial force on the bottom of
   the central stack with a 1° offset (gimbal). Verify (a) bend
   angle steady-state within 0.5° of analytic, (b) no oscillation,
   (c) thrust direction follows the engine's actual flexed
   orientation (`feedback_physics_visuals_align`).

**Gate:** all three pass.

### M6 — Long-duration stability

**Tests:**

1. *5000-tick free cruise.* M5 booster, no inputs. Track max
   angular flex; assert bounded < 1°. Assert clamp never fires.
2. *5000-tick sinusoidal SAS excitation.* Sine wave torque at 1 Hz,
   amplitude 5 kN·m for 50 ticks, then zero. Verify response
   bounded throughout, decays after excitation, no NaN, no drift.

**Gate:** both pass. Fallback if drift surfaces: implicit-Euler
joint integration via finer substep granularity through
`pinocchio::integrate`.

### M7 — KSP integration test (the real-world gate)

Re-deploy and re-run the gimbal/SAS scenario from the original
screenshot.

**Acceptance:**
- No visible engine displacement under gimbal commands.
- Vessel rotates as commanded under SAS without overshoot or roll.
- No clamp firings in the diag stream over a 60-second flight.

If M7 fails despite M4–M6 passing, the next debugging step is the
real-vs-simulated-vessel inertia mismatch (Jolt's auto-CoM versus
real mass distribution — flagged in the `BodyMassDiag` output
already; see `project_aba_branch_status.md`).

## Files to modify

### Modified (M4–M6)

- `mod/Longeron.Native.Probe/src/AbaScenarios.cs` — 9 new scenarios
  (4 M4 + 3 M5 + 2 M6).
- `mod/Longeron.Native.Probe/src/AbaTestRig.cs` — minor builders for
  3-part stacks and side-mass configs (`Child(...)` likely covers).
- `mod/Longeron.Native.Probe/src/Program.cs` — register the new
  scenarios.

### Unchanged (Pinocchio backend in place)

- C ABI surface (`bridge.{h,cpp}`).
- Wire format (`records.h`, `InputBuffer.cs`, `OutputBuffer.cs`).
- `World.cs` C# wrapper.
- `TopologyReconciler.cs` joint-compliance extraction.
- `LongeronSceneDriver.cs` pose composition.
- `tree.{h,cpp}` aside from `topology_version` field.
- RNEA backward pass.
- The `feedback_physics_visuals_align.md` invariant.

### Reference (read-only)

- `mod/Longeron.Physics/src/Solver/ABA.cs` — C# canonical
  Featherstone, retained as ground-truth differential reference.
- `mod/Longeron.Physics.Tests/Solver/{SimplePendulumTests,
  RigidStackTowerTests, RneaReciprocityTests}.cs` — useful for
  cross-checking M4 multi-link analytic targets.

## Cross-platform deployment (separate plan)

Out of scope here, tracked separately:

- Build-time Boost on Windows (`vcpkg`) and Linux (`apt`).
- macOS universal arm64+x86_64 (build twice + `lipo`).
- CI matrix (no `.github/workflows/` exists today).
- `just dist` recipe updates.

The 1.5 MB native artifact is *self-contained* at runtime
(`-dead_strip_dylibs` removed the phantom Boost runtime link). Boost
is only required at compile time.

## Out of scope

- **Implicit Euler** (Featherstone Ch. 9): swap-in later if M6
  long-duration tests show stability margin issues. Same wire format
  / same joint state.
- **Revolute / prismatic joints**: Phase 5.2 / 5.3. Pinocchio's
  joint zoo supports them via `JointModelRevoluteUnaligned` /
  `JointModelPrismaticUnaligned`; adding is one line per type.
- **Loop closures** (KAS / docking-ring cycles inside a vessel): out
  per CLAUDE.md's vessel-as-rigid-body framing; inter-vessel docking
  is a force-pair, not a constraint needing KKT projection.
- **Wheels / BG**: Phase 5.3+.
- **`PartFlex` layout cleanup** (split per-edge `JointState`
  struct): defensible follow-up; out this round.

## Risk / scope estimate

Remaining (M4–M7):

- **M4** (~150 LOC C# scenarios): 0.5 day.
- **M5** (~200 LOC C# scenarios): 0.5 day.
- **M6** (~100 LOC C# scenarios): 0.25 day.
- **M7**: in-game test session. Whatever it takes.

**Total ~1.5 days of focused work + M7 in-game.**

## Decision points

- **If M5 test 2 (SAS rigid rotation) fails:** investigate joint
  velocity packing in `aba.cpp`. Pinocchio expects v in joint frame,
  not parent frame; sign of frame is the most likely miscompose. M3
  scenarios don't stress this because a single pendulum has only one
  joint.
- **If M6 long-duration drifts unboundedly:** swap the semi-implicit
  Euler step in the substep loop for an implicit-Euler step.
  Pinocchio supports it via finer substep granularity through
  `pinocchio::integrate`; a knob on `kAbaSubsteps` may suffice
  without algorithm change.
- **If M7 fails despite M4–M6 passing:** Jolt-auto-CoM mismatch.
  Plumb `mOverrideMassProperties` from C#-computed real inertia per
  `project_aba_branch_status.md`. New plan; out of scope here.
