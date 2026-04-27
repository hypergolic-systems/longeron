# Longeron

A physics rewrite for Kerbal Space Program 1.12.5. Replaces the maximal-coordinate
chain of `ConfigurableJoint`s that Squad uses to tie a vessel together with a
single reduced-coordinate articulated body, integrated by Featherstone's
Articulated-Body Algorithm.

> **Status: work in progress.** Longeron is under active development. The
> articulated-body solver runs and a 3-part rocket sits stably on the launch
> pad with PhysX-driven contact handoff, but everything past that —
> liftoff, control authority, BG robotics, docking, vessel breakup, KAS/KIS
> compatibility — is not implemented or not yet validated. Don't fly your
> career save with it. Save first; expect crashes, NaN'd vessels, and
> stranded Kerbals.

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
* Swapping the underlying physics engine (Jolt, PhysX 5) helps at the
  margins because the inner solver gets faster or stabler, but the
  formulation is the same.

Longeron changes the *formulation*. A rigid stack is, mathematically, zero
degrees of freedom. In a reduced-coordinate articulated-body formulation,
that zero-DOF subtree integrates exactly, in closed form, in O(n) time —
because there is nothing to integrate. The wobble doesn't get suppressed;
it never enters the simulation in the first place.

## Approach

Longeron treats each loaded vessel as a single articulated body in
reduced coordinates:

* One spanning tree per vessel, rooted at `vessel.rootPart`. Every other
  part is a child connected by a typed joint:
  * **Fixed** (zero DOF) — most stack and surface attachments. The
    interesting case: this is where the wobble was.
  * **Revolute / Prismatic** (one DOF) — Breaking Ground hinges and pistons.
  * **Free6** (six DOF) — pre-lock docking ports, until they latch.
* **Forward dynamics** via Featherstone's [**ABA**](https://en.wikipedia.org/wiki/Featherstone%27s_algorithm) —
  one O(n) recursive pass per tick, computing joint accelerations from
  the current state and applied wrenches.
* **Loop closures** (docked rings, struts that close a cycle in the
  vessel graph, BG servos that bridge two existing branches) handled
  on top of the spanning-tree pass via Lagrange multipliers in a KKT
  system. The multipliers double as constraint reaction forces — break
  detection (`|λ| > breakForce`) is direct, not heuristic.
* **Contact** with the world (terrain, KSC structures, other vessels) via
  PhysX's existing collider geometry and broadphase — Unity owns
  collision detection; we own everything that comes after.
* **Integration** via semi-implicit Euler at KSP's existing
  `FixedUpdate` rate (50 Hz). No substepping.

External wrenches (thrust, aero, control surfaces, RCS, reaction wheels,
gravity) are inferred at the engine boundary by reading the velocity delta
PhysX would have applied to each `Rigidbody` and converting back to a net
force per tick — so stock and modded force-producers (FAR, RealFuels,
RealPlume, KER) work without explicit integration.

PhysX's Unity wrapper is kept in the loop deliberately. Unity still handles:

* Collider geometry, broadphase, and triangle-soup terrain raycasts.
* Inter-vessel contact detection and collision events.
* Wheel, ladder, and airlock-trigger semantics.

Longeron's ownership stops at *constraint solving and integration*. Both
sides need each other and neither tries to do the other's job.

## References

The canonical reference for everything above:

> Featherstone, Roy (2008). *Rigid Body Dynamics Algorithms.* Springer
> US. [doi:10.1007/978-1-4899-7560-7](https://doi.org/10.1007/978-1-4899-7560-7).
> ISBN 978-0-387-74314-1.

Specifically:

* **Chapter 7 — Forward Dynamics.** ABA, recursive form, joint-space
  inertia matrix, the spatial-vector machinery.
* **Chapter 8 — Closed-Loop Systems.** KKT formulation for loop closure
  via Lagrange multipliers; baseline for `Longeron.Physics.Solver.LoopClosure`.

Adjacent reading that explains why PhysX's choice of formulation matters
even though its solver is fine:

* Erleben, Kenny (2007). "[Velocity-based shock propagation for
  multibody dynamics animation.](https://dl.acm.org/doi/10.1145/1243980.1243986)"
  *ACM Transactions on Graphics*, 26 (2). Why PGS converges slowly on
  long chains and what you can do about it without changing formulation.
* Mirtich, Brian (1996). *[Impulse-based Dynamic Simulation of Rigid Body
  Systems](https://www.cs.cmu.edu/~baraff/sigcourse/notesg.pdf)* (Ph.D.
  thesis, UC Berkeley). The other way to dodge the convergence problem,
  which KSP didn't take.

## License

MIT. See [LICENSE](LICENSE).
