# Longeron

A physics rewrite for Kerbal Space Program 1.12.5. Replaces Unity's
PhysX with [Jolt Physics](https://github.com/jrouwe/JoltPhysics) for
vessel simulation, and layers a Featherstone articulated-body solver on
top of Jolt to model joint compliance — eliminating the wobble of stock
KSP without losing the structural flex Squad's joint model intentionally
provides.

> **Status: work in progress.** The Jolt-bridge pivot is underway.
> Phase 0 (native bridge scaffold), Phase 1 (kinematic contact-callback
> verification, all twelve probe checks green out-of-game), and
> Phase 1.5 (in-game smoke test: real collider mirroring, one-part
> vessel managed with kinematic-vs-static contacts firing every tick
> on the launchpad) all reached on `main`. Phase 2 — force redirect,
> mass updates, pose readback Jolt → Unity, multi-part joints — is
> next. Don't fly your career save with it. Save first; expect
> crashes, NaN'd vessels, and stranded Kerbals. See [`CLAUDE.md`](CLAUDE.md)
> for architecture and [`PLAN.md`](PLAN.md) for the phased build plan.

The name is the [aerospace structural member](https://en.wikipedia.org/wiki/Longeron)
that runs the length of a fuselage and keeps it rigid. That is approximately
what this mod does.

## Why this exists

KSP's "noodle rocket" wobble is not a PhysX bug. It is a direct, predictable
consequence of how the game models a vessel:

* Each `Part` is a separate `Rigidbody`.
* Adjacent parts are tied together by a 6-DOF `ConfigurableJoint` whose
  drives and limits Squad sets to "stiff but not infinite."
* PhysX integrates the resulting maximal-coordinate system with an iterative
  Projected Gauss–Seidel constraint solver.

Iterative PGS converges slowly on long kinematic chains with high mass ratios
between adjacent links — the residual on each constraint depends on its
neighbours' residuals, and information propagates one link per iteration.
A 30-part stack with a heavy upper stage and a light lower stage is exactly
the regime where PGS with a fixed iteration budget visibly fails to converge.
The error shows up as oscillation along the chain — the wobble you can see.

Existing mitigations work by avoiding the regime, not by improving it:

* **Autostrut** and the [Kerbal Joint Reinforcement](https://forum.kerbalspaceprogram.com/topic/55657-)
  family short-circuit the chain — they add direct constraints between
  distant parts, so PhysX has fewer steps to propagate residuals across.
* Swapping the underlying physics engine (Jolt, PhysX 5) for KSP's
  PhysX-3-vintage solver would help at the margins because the inner
  solver is faster or stabler — but if the formulation stays maximal-
  coordinate, the wobble stays too.

Note that compliance is not the bug. Squad could have used Unity's
`FixedJoint` everywhere and avoided wobble outright; they didn't,
because they wanted real structural flex — the small amount of give
that lets a tall stack survive a hard burn rather than shattering when
any internal stress overshoots a hard limit. That's the right design
choice. The bug is that PhysX's PGS, applied to a long chain of
compliant constraints in maximal coordinates, diverges into wobble
instead of converging into flex.

Longeron changes both the *engine* and the *formulation*. Jolt — a
modern, deterministic, multithread-friendly C++ rigid-body engine —
replaces PhysX for collision detection, contact solving, and
integration. On top of Jolt, Featherstone's Articulated-Body Algorithm
runs the spanning tree once per tick in O(n) leaf-to-root then
root-to-leaf, computing joint accelerations from the current state and
applied wrenches without any constraint-residual loop. Joint compliance
enters as a local spring-damper term in joint space — applied at each
joint individually — rather than as a constraint to be projected
globally. The neighbour-residual coupling that breaks PGS on long
chains doesn't arise: there are no constraint residuals to converge.
We keep the compliance Squad wanted; we lose the wobble that PhysX's
PGS was producing.

## Why Jolt, not PhysX

Earlier iterations tried to keep Unity-PhysX for collision and layer
Longeron's solver on top in C#. Both directions of that bargain failed
on the same structural problem:

* *Kinematic Unity rigidbodies* — clean integration ownership, but
  PhysX silences contact callbacks for kinematic-vs-static pairs and
  reverts position writes against a freeze anchor.
* *Non-kinematic Unity rigidbodies* — contacts come back, but
  computing a meaningful corrective wrench requires Longeron's solver
  to *predict* what PhysX's contact solver is about to do. Two contact
  solvers, neither authoritative.

Diagnosis: Unity will not let the rigidbody state be shared cleanly
with PhysX. Either PhysX owns integration (and Longeron fights
position writes), or PhysX owns contacts (and Longeron runs a worse
contact solver to stay in sync). There is no middle ground inside
Unity's physics ownership model.

Jolt sidesteps the problem by replacing PhysX entirely. Jolt is not
*better at solving the same maximal-coordinate problem* — though it
also is, and the Phase 2 build (Jolt's native `SixDOFConstraint` with
stiff drives, no Featherstone yet) will already feel measurably less
wobbly than stock — but the architectural win is *single ownership*:
Jolt owns broadphase, contacts, and integration; Featherstone owns
joint forces; Unity rigidbodies are kinematic proxies driven by Jolt
each tick for the benefit of stock + modded code that reads
`rb.velocity`, `rb.position`, etc. Nothing fights anything.

## Approach

Longeron's runtime pipeline, per `FixedUpdate`:

1. **Accumulate inputs** in C#: per-body force/torque deltas (from
   Harmony-intercepted `Rigidbody.AddForce*` calls), mass-update
   records, topology mutations, gravity vectors.
2. **Single P/Invoke** into the C++ bridge: `longeron_step(world,
   inputs, outputs, dt)`. The bridge mutates the Jolt world per the
   input records, runs `JPH::PhysicsSystem::Update`, and writes
   per-body poses + contact records into the output buffer.
3. **Read outputs** in C#: drain pose updates and contacts.
4. **Write Unity rigidbody state** from output. Kinematic rigidbodies
   take Jolt's pose so stock and modded code see consistent state.

External wrenches (thrust, aero, RCS, reaction wheels, gravity) are
captured at the engine boundary by patching `Rigidbody.AddForce*` to
redirect into Longeron's per-body force accumulator — so stock and
modded force-producers (FAR, RealFuels, RealPlume, KER) work without
any per-mod special handling. Unity rigidbodies are kinematic, so the
original `AddForce` is a no-op; the Harmony prefix replaces it cleanly.

Inside Jolt, each loaded vessel is N kinematic bodies (one per part)
in a vessel-scoped `ObjectLayer` that filters out same-vessel
self-collision. Joints between parts are handled differently in
Phase 2 vs Phase 4:

* **Phase 2 (Jolt-native joints).** Adjacent parts are coupled by Jolt
  `SixDOFConstraint`s with stiff per-axis spring/damper drives mirroring
  stock's `ConfigurableJoint`. Inter-vessel coupling (docking, KAS) is
  a Jolt constraint between bodies in different `ObjectLayer`s. PGS
  in Jolt is meaningfully better than PhysX's at long chains and high
  mass ratios, so this build already reduces wobble — but it is still
  maximal-coordinate.
* **Phase 4 (Featherstone).** All joints collapse to a single
  `Compliant6DOF` primitive parameterized by per-axis stiffness,
  damping, and motor force. Per-axis parameters select between what
  stock would call fixed (high K, all axes), revolute (high K on 5,
  motor on 1), prismatic (high K on 5, motor on 1), free (zero K),
  or soft (medium K with high damping) — not separate joint types.
  ABA computes joint forces in reduced coordinates and writes per-part
  poses to Jolt each tick; Jolt sees no inter-body constraints, only
  per-tick kinematic pose updates. **Loop closures collapse out of the
  math**: cycle-closing graph edges (docking rings, KAS struts forming
  cycles) are additional `Compliant6DOF` records contributing force
  pairs from kinematic state, not constraints needing KKT projection.
  Stiff joints use implicit ABA integration for unconditional
  stability at KSP's 50 Hz `FixedUpdate`.

Terrain comes from Unity's PQS — colliders streamed in/out as the
camera moves are mirrored into Jolt as static `MeshShape` bodies
(Phase 3). KSC buildings, runway, water plane mirror the same way at
scene load. Jolt is built with `JPH_DOUBLE_PRECISION=ON` so its world
coordinates are doubles, eliminating floating-origin sync on the
physics side; Krakensbane keeps shifting Unity's render origin.

## References

The canonical reference for the solver math:

> Featherstone, Roy (2008). *Rigid Body Dynamics Algorithms.* Springer
> US. [doi:10.1007/978-1-4899-7560-7](https://doi.org/10.1007/978-1-4899-7560-7).
> ISBN 978-0-387-74314-1.

Specifically:

* **Chapter 7 — Forward Dynamics.** ABA, recursive form, joint-space
  inertia matrix, the spatial-vector machinery.
* **Chapter 9 — Joint Modeling.** Spring-damper joint formulation;
  basis for the implicit ABA integrator that Phase 4 uses for stiff
  compliant joints.

Adjacent reading on why the choice of formulation matters even when the
solver is fine:

* Erleben, Kenny (2007). "[Velocity-based shock propagation for
  multibody dynamics animation.](https://dl.acm.org/doi/10.1145/1243980.1243986)"
  *ACM Transactions on Graphics*, 26 (2). Why PGS converges slowly on
  long chains and what you can do about it without changing formulation.
* Mirtich, Brian (1996). *[Impulse-based Dynamic Simulation of Rigid Body
  Systems](https://people.eecs.berkeley.edu/~jfc/mirtich/thesis/mirtichThesis.pdf)*
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
