# Longeron

A physics rewrite for Kerbal Space Program 1.12.5. Replaces Unity's
PhysX with [Jolt Physics](https://github.com/jrouwe/JoltPhysics) and
re-shapes the vessel model: each loaded vessel is a single rigid body
in Jolt, and joint forces — used for structural-failure detection —
fall out of an O(n) inverse-dynamics pass over the part tree. No
chain-of-compliant-joints, no PGS wobble, no autostrut.

> **Status: work in progress.** Phase 0–3b on `main`: native Jolt
> bridge, kinematic contact callbacks, force redirect, mass updates,
> CB-frame reference model, PQS terrain mirroring, and the single-body
> vessel pivot are all in. Phase 4 — Featherstone-based joint forces
> and stock breakage parity — is next. Don't fly your career save with
> it; save first, expect crashes, NaN'd vessels, and stranded Kerbals.
> See [`CLAUDE.md`](CLAUDE.md) for architecture and
> [`PLAN.md`](PLAN.md) for the phased build plan.

The name is the [aerospace structural member](https://en.wikipedia.org/wiki/Longeron)
that runs the length of a fuselage and keeps it rigid. That is
approximately what this mod does.

## Why this exists

Stock KSP rockets wobble. The wobble is not a PhysX-tuning bug — it's
a structural consequence of how the game models a vessel. Stock chose
that model for a reason, and we should be honest about that reason
before claiming to do better.

### What stock actually does, and why

Each `Part` in stock is a separate `Rigidbody`. Adjacent parts are
tied together by a 6-DOF `ConfigurableJoint` whose drives and limits
Squad sets to "stiff but not infinite" — effectively a six-axis
spring-damper. PhysX integrates the resulting maximal-coordinate
system with an iterative Projected Gauss–Seidel constraint solver.

The reason the joints are *compliant* (springs, not rigid welds) is
**structural failure**. KSP wants engines to rip off pylons, fairings
to shatter under aero loads, and tall stacks to snap under hard burns
— and those mechanics need a force on each joint to compare against
each part's `breakForce`. The cleanest way to get that force out of a
black-box maximal-coordinate solver is to make the joint compliant in
the first place: stiffness × deflection = force, applied right at the
boundary, no inverse-dynamics pass required. Compliance was a
*means*, and breakage was the *end*.

The cost is the wobble. Iterative PGS converges slowly on long
kinematic chains with high mass ratios between adjacent links — the
residual on each constraint depends on its neighbours' residuals, and
information propagates one link per iteration. A 30-part stack with a
heavy upper stage and a light lower stage is exactly the regime where
PGS with a fixed iteration budget visibly fails to converge. The
error shows up as oscillation along the chain — that's what you can
see flying. Worse, the convergence rate per iteration scales like
`1 − 2/(1 + κ)` where κ is roughly the stack's mass-ratio condition
number; a heavy booster sandwiched between a light decoupler and a
heavy parent can leave a single PGS iteration only ≈ 1% closer to the
weld it's nominally enforcing. Hundreds of iterations later, the
residual is still measurable, and the residual *is* the wobble.

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

### Longeron's reframe: separate motion from stress

If breakage is the actual end and compliance is an instrumental
means, we don't have to keep the means. Longeron's vessel model:

1. **Vessel motion**: each loaded vessel is **one rigid body** in
   Jolt. Compound shape made from every part's colliders. Body mass =
   total vessel mass. No internal degrees of freedom. No constraints.
   No PGS-iterating-over-a-chain. Vessel pose comes out of Jolt's
   single-body integrator each tick; per-part Unity transforms ride
   along by composing the vessel pose with each part's frozen
   root-relative offset.
2. **Joint forces (Phase 4)**: once we have the vessel's overall
   acceleration from Jolt, we run **Recursive Newton-Euler** (RNEA)
   over the part tree. RNEA is the inverse-dynamics partner of the
   Featherstone ABA family — a leaf-to-root then root-to-leaf O(n)
   pass that, given the kinematic state and the inertia of every
   subtree, computes the wrench transmitted across every joint.
   Compare against stock's `breakForce` / `breakTorque`, fire
   `Part.decouple()` when exceeded — exactly what the stock joint-break
   path does, just driven by an inverse-dynamics computation instead
   of a PGS-residual reading.

The neighbour-residual coupling that breaks PGS on long chains
doesn't arise: there are no constraint residuals to converge. We get
a perfectly rigid vessel (no wobble) *and* per-edge stress
accounting (breakage), because we stopped trying to do both with the
same primitive.

The cost is that we lose the visible *flex* — the small amount of
give that lets a stock stack visibly sag under a hard burn. Some
players value that. Longeron treats vessels as black boxes between
break events; if you want the noodle, fly stock.

## Why Jolt, not PhysX

Earlier iterations tried to keep Unity-PhysX for collision and layer
Longeron's solver on top in C#. Both directions of that bargain
failed on the same structural problem:

* *Kinematic Unity rigidbodies* — clean integration ownership, but
  PhysX silences contact callbacks for kinematic-vs-static pairs and
  reverts position writes against a freeze anchor.
* *Non-kinematic Unity rigidbodies* — contacts come back, but
  computing a meaningful corrective wrench requires Longeron's solver
  to *predict* what PhysX's contact solver is about to do. Two
  contact solvers, neither authoritative.

Diagnosis: Unity will not let the rigidbody state be shared cleanly
with PhysX. Either PhysX owns integration (and Longeron fights
position writes), or PhysX owns contacts (and Longeron runs a worse
contact solver to stay in sync). There is no middle ground inside
Unity's physics ownership model.

Jolt sidesteps the problem by replacing PhysX entirely. The
architectural win is *single ownership*: Jolt owns broadphase,
contacts, and integration; Longeron owns external-wrench aggregation
and (in Phase 4) joint-force decomposition; Unity rigidbodies are
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

Longeron's runtime pipeline, per `FixedUpdate`:

1. **Accumulate inputs** in C#: per-vessel force/torque records (from
   Harmony-intercepted `Rigidbody.AddForce*` calls, applied at the
   originating part's world-space CoM on the vessel body), mass
   aggregates, topology mutations, gravity vectors.
2. **Single P/Invoke** into the C++ bridge: `longeron_step(world,
   inputs, outputs, dt)`. The bridge mutates the Jolt world per the
   input records, runs `JPH::PhysicsSystem::Update`, and writes
   per-vessel poses + contact records into the output buffer.
3. **Read outputs** in C#: drain pose updates and contacts.
4. **Propagate vessel pose to parts.** Each part's Unity rigidbody
   gets `rb.position = vesselPose ∘ partOffset`, with offsets frozen
   at body-create time. Velocity and angular velocity propagate via
   `v(r) = v_anchor + ω × r`. Per-part poses are bookkeeping for
   stock + modded code that reads them; the simulation depends on the
   vessel pose only.

External wrenches (thrust, aero, RCS, reaction wheels, gravity) are
captured at the engine boundary by patching `Rigidbody.AddForce*` to
redirect into Longeron's per-vessel force record stream — so stock and
modded force-producers (FAR, RealFuels, RealPlume, KER) work without
any per-mod special handling. A force on a per-part rigidbody applies
to the vessel body at that part's world-space CoM, so off-CoM thrust
and offset aero still produce the right torque about the vessel CoM.

Topology mutations (decouple, dock, joint break, fairing eject)
trigger a vessel-body rebuild: destroy the old body, walk the new
part list, build a new compound shape and inertia, emit `BodyCreate`.
Same code path for every event class. Velocity inheritance comes from
each new vessel reading its starting state from the old body's pose
plus stock decoupler `AddForce` impulses, which redirect normally.

Phases 4+ adds:

* **RNEA-based joint forces.** Per-tick inverse dynamics over the
  part tree using the vessel's acceleration and external wrenches as
  inputs; output is the wrench across every spanning-tree edge.
  Compare against `breakForce` / `breakTorque`; trigger
  `Part.decouple()` when exceeded. Loop closures (docking rings, KAS
  cycles) treated as additional non-tree edges contributing force
  pairs from kinematic state — same primitive, same inverse-dynamics
  pass, no KKT projection.
* **Compliant Featherstone option.** If a future Longeron user
  *wants* visible flex (e.g., space-station ring sag, soft landing
  legs), the same data structures support a forward-dynamics ABA pass
  with finite-stiffness joints in reduced coordinates. The implicit
  ABA integrator (Featherstone Ch. 9 spring-damper formulation) is
  unconditionally stable at 50 Hz without substepping, sidestepping
  the K-vs-Δt instability that doomed the maximal-coordinate
  compliant approach. This is opt-in, off by default.

Terrain comes from Unity's PQS — colliders streamed in/out as the
camera moves are mirrored into Jolt as static `MeshShape` bodies in
the active mainBody's CB-frame, where they don't drift. KSC buildings,
runway, water plane mirror the same way at scene load.

## References

The canonical reference for the solver math:

> Featherstone, Roy (2008). *Rigid Body Dynamics Algorithms.* Springer
> US. [doi:10.1007/978-1-4899-7560-7](https://doi.org/10.1007/978-1-4899-7560-7).
> ISBN 978-0-387-74314-1.

Specifically:

* **Chapter 5 — Recursive Newton-Euler.** Inverse dynamics on a tree;
  the workhorse for Phase 4's stress accounting and breakage.
* **Chapter 7 — Forward Dynamics.** ABA proper, the spatial-vector
  machinery, and the joint-space inertia matrix; relevant if we ever
  enable the optional compliant-flex mode.
* **Chapter 9 — Joint Modeling.** Spring-damper joint formulation;
  basis for the implicit ABA integrator the optional flex mode would
  use for stiff compliant joints.

Adjacent reading on why the choice of formulation matters even when
the solver is fine:

* Erleben, Kenny (2007). "[Velocity-based shock propagation for
  multibody dynamics animation.](https://dl.acm.org/doi/10.1145/1243980.1243986)"
  *ACM Transactions on Graphics*, 26 (2). Why PGS converges slowly on
  long chains and what you can do about it without changing
  formulation.
* Mirtich, Brian (1996). *[Impulse-based Dynamic Simulation of Rigid
  Body Systems](https://people.eecs.berkeley.edu/~jfc/mirtich/thesis/mirtichThesis.pdf)*
  (Ph.D. thesis, UC Berkeley). The other way to dodge the convergence
  problem, which KSP didn't take.

The Jolt engine itself:

* [Jolt Physics](https://github.com/jrouwe/JoltPhysics) — open-source
  C++ rigid-body engine, Apache-2.0. The
  [architecture document](https://jrouwe.github.io/JoltPhysics/) walks
  through the broadphase, contact manager, constraint solver, and
  determinism guarantees.

## License

MIT. See [LICENSE](LICENSE).
