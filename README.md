# Longeron

A physics rewrite for Kerbal Space Program 1.12.5. Replaces Unity's
PhysX with two purpose-built solvers wired together: **[Jolt
Physics](https://github.com/jrouwe/JoltPhysics)** for vessel-world
interactions (rigid-body motion, contacts, terrain, breakable joints
between vessels) and **[Pinocchio](https://stack-of-tasks.github.io/pinocchio/)**
for inter-part articulated-body dynamics — Featherstone ABA in
reduced coordinates, with stiff spring-damper joints integrated by
implicit Euler. No PGS over a chain of compliant `ConfigurableJoint`s,
no autostrut.

> Don't fly your career save with it; save first, expect crashes,
> NaN'd vessels, and stranded Kerbals. See [`CLAUDE.md`](CLAUDE.md)
> for the full architecture write-up.

The name is the [aerospace structural member](https://en.wikipedia.org/wiki/Longeron)
that runs the length of a fuselage and keeps it rigid. That is
approximately what this mod does.

## Why this exists

Stock KSP rockets wobble. The wobble is not a PhysX-tuning bug —
it's a structural consequence of how the game models a vessel.
Stock chose that model for a reason, and we should be honest about
that reason before claiming to do better.

### What stock actually does, and why

Each `Part` in stock is a separate `Rigidbody`. Adjacent parts are
tied together by a 6-DOF `ConfigurableJoint` whose drives and limits
Squad sets to "stiff but not infinite" — effectively a six-axis
spring-damper. PhysX integrates the resulting maximal-coordinate
system with an iterative Projected Gauss–Seidel constraint solver.

The reason the joints are *compliant* (springs, not rigid welds) is
**structural failure**. KSP wants engines to rip off pylons,
fairings to shatter under aero loads, and tall stacks to snap under
hard burns — and those mechanics need a force on each joint to
compare against each part's `breakForce`. The cleanest way to get
that force out of a black-box maximal-coordinate solver is to make
the joint compliant in the first place: stiffness × deflection =
force, applied right at the boundary, no inverse-dynamics pass
required. Compliance was a *means*, and breakage was the *end*.

The cost is the wobble. Iterative PGS converges slowly on long
kinematic chains with high mass ratios between adjacent links — the
residual on each constraint depends on its neighbours' residuals,
and information propagates one link per iteration. A 30-part stack
with a heavy upper stage and a light lower stage is exactly the
regime where PGS with a fixed iteration budget visibly fails to
converge. The error shows up as oscillation along the chain —
that's what you can see flying. Worse, the convergence rate per
iteration scales like `1 − 2/(1 + κ)` where κ is roughly the
stack's mass-ratio condition number; a heavy booster sandwiched
between a light decoupler and a heavy parent can leave a single
PGS iteration only ≈ 1% closer to the weld it's nominally
enforcing. Hundreds of iterations later, the residual is still
measurable, and the residual *is* the wobble.

The existing mitigations work by avoiding this regime, not by
improving it:

* **Autostrut** and the [Kerbal Joint Reinforcement](https://forum.kerbalspaceprogram.com/topic/55657-)
  family short-circuit the chain — they add direct constraints
  between distant parts, so PGS has fewer steps to propagate
  residuals across.
* Swapping the underlying physics engine for KSP's PhysX-3-vintage
  solver helps at the margins (Jolt's PGS is meaningfully better
  than PhysX 3's, and Jolt is multithreaded), but as long as the
  formulation stays maximal-coordinate with a chain of compliant
  joints, the wobble stays too. We measured this: at 50 Hz physics
  rate, even Jolt with 256 velocity iterations and 128 position
  iterations leaves visible residual on a side-booster decoupler
  (peak transient ~6° at spawn, settled to ~0.3° at rest, 8× the
  CPU cost of stock).

### Longeron's reframe: split vessel-world from inter-part

Stock conflates two physical processes onto the same primitive
(a tree of compliant 6-DOF constraints): the vessel's macroscopic
motion through the world, and the small structural deformations
between parts. They have different time constants, different
stability requirements, and different solvers do them well. Mixing
them is what creates the convergence pathology.

Longeron splits the two:

1. **Vessel-world (Jolt).** Each loaded vessel is **one rigid
   body** in Jolt — compound shape made from every part's
   colliders, body mass = total vessel mass, body inertia =
   compound aggregate. Jolt owns broadphase, contact detection,
   contact solving, integration, and gravity. No constraints
   between parts within a vessel, so no PGS-iterating-over-a-chain.
   Inter-vessel coupling that genuinely keeps two vessels separate
   (KAS struts spanning two distinct vessels, vessel-on-vessel
   contact) becomes ordinary Jolt constraints between two
   separately-integrated bodies, where PGS has just one link to
   converge. Docking is *not* inter-vessel — KSP merges the two
   vessels at port engagement, and the docking port becomes another
   articulation in the merged tree below.
2. **Inter-part (Pinocchio).** Within a vessel, per-part
   articulation is integrated in reduced coordinates by Pinocchio's
   Featherstone ABA. Each non-root part connects to its parent via
   a 3-DOF spherical joint at the attach node, with spring-damper
   torsion `τ = −K·θ − C·ω`. Docking ports, when engaged, are just
   another edge of the same kind. The joint state is integrated by
   implicit Euler at 50 Hz, *unconditionally* stable for any
   stiffness (the K-vs-Δt instability that doomed the maximal-
   coordinate compliant approach doesn't arise). The output —
   per-part deflection in the vessel-body frame — gets written
   back into the Jolt body's compound shape via `MutableCompoundShape::
   ModifyShape`, so the visible collider geometry tracks the flex
   and contacts hit the deflected surfaces.
3. **Stress accounting (RNEA).** Once Jolt and Pinocchio have
   advanced state, an O(n) Recursive Newton-Euler backward pass
   over the same part tree decomposes the per-edge wrench given
   the vessel's acceleration plus per-part external wrenches.
   The result feeds breakage — compare against stock's `breakForce`
   / `breakTorque`, fire `Part.decouple()` when exceeded.

The neighbour-residual coupling that breaks PGS on long chains
doesn't arise: there are no per-tick constraint residuals to
converge in either solver. Pinocchio computes the chain dynamics
directly from joint inertia, joint torques, and the propagated
spatial-velocity recurrence — O(n), exact, no iteration. We get
a vessel that integrates as one rigid body (no PGS wobble), per-
part flex driven by real Featherstone dynamics (not a residual
artifact), and per-edge stress accounting (breakage), all from
solvers chosen for what they're good at.

## Why Jolt, not PhysX

Earlier iterations tried to keep Unity-PhysX for collision and layer
Longeron's solver on top in C#. Both directions of that bargain
failed on the same structural problem:

* *Kinematic Unity rigidbodies* — clean integration ownership, but
  PhysX silences contact callbacks for kinematic-vs-static pairs and
  reverts position writes against a freeze anchor.
* *Non-kinematic Unity rigidbodies* — contacts come back, but
  computing a meaningful corrective wrench requires Longeron's
  solver to *predict* what PhysX's contact solver is about to do.
  Two contact solvers, neither authoritative.

Diagnosis: Unity will not let the rigidbody state be shared cleanly
with PhysX. Either PhysX owns integration (and Longeron fights
position writes), or PhysX owns contacts (and Longeron runs a worse
contact solver to stay in sync). There is no middle ground inside
Unity's physics ownership model.

Jolt sidesteps the problem by replacing PhysX entirely. The
architectural win is *single ownership*: Jolt owns broadphase,
contacts, and integration; Pinocchio owns inter-part dynamics; the
tree-based RNEA owns stress accounting; Unity rigidbodies are
kinematic proxies driven by Jolt each tick for the benefit of stock
+ modded code that reads `rb.velocity`, `rb.position`, etc. Nothing
fights anything.

Jolt is built with `JPH_DOUBLE_PRECISION=ON`. World coordinates are
doubles, and we anchor Jolt to the active mainBody's body-fixed
rotating frame, which keeps the active vessel near origin in
physics-space and makes terrain (PQS quads) stationary by
construction — no per-tick re-mirroring as the planet rotates.
[Krakensbane](https://wiki.kerbalspaceprogram.com/wiki/Krakensbane)
is patched out entirely; Jolt's double precision is the replacement.
[FloatingOrigin](https://en.wikipedia.org/wiki/Floating_origin) keeps
working for Unity rendering, but doesn't touch Jolt.

## Approach

Per-`FixedUpdate`, the runtime pipeline is:

1. **Accumulate inputs** in C#: per-vessel force/torque records
   (from Harmony-intercepted `Rigidbody.AddForce*` calls, applied at
   the originating part's world-space CoM on the vessel body),
   mass aggregates, topology mutations, gravity vectors, vessel
   trees with per-edge stiffness from stock `breakForce` /
   `breakTorque`.
2. **Single P/Invoke** into the C++ bridge: `longeron_step(world,
   inputs, outputs, dt)`. The bridge mutates the Jolt world per
   the input records, runs `JPH::PhysicsSystem::Update`, then for
   every vessel runs Pinocchio's ABA forward step (substepped
   internally), writes the resulting per-part flex back into the
   vessel's `MutableCompoundShape`, and runs an in-tree RNEA
   backward pass for break-detection wrenches. Output buffer fills
   with per-vessel poses, per-part flex, per-edge wrenches,
   contact records.
3. **Read outputs** in C#: drain pose updates, flex, contacts, and
   joint wrenches.
4. **Propagate vessel pose to parts.** Each part's Unity rigidbody
   gets `rb.position = vesselAnchor + vesselRot · (partOffset +
   flexOffset)` with rotation composed similarly. Per-part poses
   are bookkeeping for stock + modded code that reads them; the
   simulation depends on the vessel pose plus the per-part flex
   from Pinocchio.

External wrenches (thrust, aero, RCS, reaction wheels, gravity)
are captured at the engine boundary by patching `Rigidbody.AddForce*`
to redirect into Longeron's per-vessel force record stream — so
stock and modded force-producers (FAR, RealFuels, RealPlume, KER)
work without any per-mod special handling. A force on a per-part
rigidbody applies to the vessel body at that part's world-space
CoM (so off-CoM thrust and offset aero still produce the right
torque about the vessel CoM), *and* gets routed into the per-part
external-wrench accumulator that Pinocchio reads (so the same
force enters the inter-part flex computation at the right node).

Topology mutations (decouple, dock, joint break, fairing eject)
trigger a vessel-body rebuild: destroy the old body, walk the
new part list, build a new compound shape and inertia, emit
`BodyCreate` + `VesselTreeUpdate`. Pinocchio's per-vessel `Model`
is invalidated by the topology version bump and rebuilt on the
next tick. Velocity inheritance comes from each new vessel
reading its starting state from the old body's pose; stock
decoupler `AddForce` impulses redirect normally.

Terrain comes from Unity's PQS — colliders streamed in/out as the
camera moves are mirrored into Jolt as static `MeshShape` bodies
in the active mainBody's CB-frame, where they don't drift. KSC
buildings, runway, water plane mirror the same way at scene load.

## References

The canonical reference for the solver math:

> Featherstone, Roy (2008). *Rigid Body Dynamics Algorithms.* Springer
> US. [doi:10.1007/978-1-4899-7560-7](https://doi.org/10.1007/978-1-4899-7560-7).
> ISBN 978-0-387-74314-1.

Specifically:

* **Chapter 5 — Recursive Newton-Euler.** Inverse dynamics on a
  tree; the workhorse for the per-edge stress accounting and
  breakage path.
* **Chapter 7 — Forward Dynamics (ABA).** The articulated-body
  algorithm proper, the spatial-vector machinery, and the joint-
  space inertia matrix; this is what Pinocchio implements.
* **Chapter 9 — Joint Modeling.** Spring-damper joint formulation
  and the implicit-Euler integrator that makes stiff joints stable
  at fixed timestep.

Adjacent reading on why the choice of formulation matters even
when the solver is fine:

* Erleben, Kenny (2007). "[Velocity-based shock propagation for
  multibody dynamics animation.](https://dl.acm.org/doi/10.1145/1243980.1243986)"
  *ACM Transactions on Graphics*, 26 (2). Why PGS converges slowly
  on long chains and what you can do about it without changing
  formulation.
* Mirtich, Brian (1996). *[Impulse-based Dynamic Simulation of
  Rigid Body Systems](https://people.eecs.berkeley.edu/~jfc/mirtich/thesis/mirtichThesis.pdf)*
  (Ph.D. thesis, UC Berkeley). The other way to dodge the
  convergence problem, which KSP didn't take.

The two physics engines:

* [Jolt Physics](https://github.com/jrouwe/JoltPhysics) —
  open-source C++ rigid-body engine, Apache-2.0. The
  [architecture document](https://jrouwe.github.io/JoltPhysics/)
  walks through the broadphase, contact manager, constraint
  solver, and determinism guarantees.
* [Pinocchio](https://stack-of-tasks.github.io/pinocchio/) —
  open-source C++ rigid-body library specialized for robotics,
  BSD-2-Clause. Header-only build is what we vendor; Carpentier
  et al.'s [SII 2019 paper](https://hal.science/hal-01866228)
  documents the algorithmic choices.

## License

MIT. See [LICENSE](LICENSE).
