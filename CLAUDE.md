# Longeron

A KSP 1.12.5 mod that replaces Unity's PhysX with [Jolt
Physics](https://github.com/jrouwe/JoltPhysics) and reshapes the
vessel model: each loaded vessel is one rigid body in Jolt, and joint
forces (used for structural-failure detection) come from a Recursive
Newton-Euler inverse-dynamics pass over the part tree.

The name is the aerospace structural member that keeps a fuselage
rigid along its length. That is approximately what this mod does.

## Why this exists

KSP's "noodle rocket" wobble is structural, not a PhysX-tuning bug.
Stock chose its formulation deliberately, and the wobble is the cost
of that choice — but the choice itself was instrumental, and we're
not bound to it.

### What stock does, and what it's actually for

Each `Part` in stock is a separate `Rigidbody`. Adjacent parts are
tied together by a 6-DOF `ConfigurableJoint` whose drives and limits
Squad sets to "stiff but not infinite" — effectively a six-axis
spring-damper. PhysX integrates the resulting maximal-coordinate
system with an iterative Projected Gauss-Seidel constraint solver.

Why are the joints *compliant* (springs, not rigid welds) in the
first place? **Structural failure.** KSP wants engines to rip off
pylons and fairings to shatter under aero loads, so the game needs a
force on each joint to compare against `breakForce` / `breakTorque`.
The cleanest way to extract that force from a black-box maximal-
coordinate solver is to make the joint compliant: stiffness ×
deflection = force, applied at the boundary, no inverse-dynamics
pass required. **Compliance was the means; breakage was the end.**

The cost of that choice is the wobble. PGS on a chain of compliant
constraints converges slowly when:

- mass ratios between adjacent links are extreme (heavy boosters on
  light decouplers — convergence rate ≈ `1 - 2/(1+κ)` where κ is the
  condition number);
- the chain is long (PGS propagates one link of correction per
  iteration);
- the timestep is fixed (`50 Hz` in KSP), so iterations-per-tick is
  bounded.

The leftover residual is observable as oscillation along the chain.
Autostrut and KJR mitigate by adding direct constraints between
distant parts, shortening the propagation distance — they don't fix
the convergence pathology, they just route around it. Switching the
solver (Jolt's PGS is real-world better than PhysX 3's, plus
multithreaded) helps at the margins, but as long as the model is a
chain of compliant joints in maximal coordinates, the wobble stays.
We measured: at 50 Hz, even Jolt with 256 velocity / 128 position
iterations leaves visible residual on a side-booster decoupler
(~6° spawn transient, ~0.3° steady, 8× the CPU cost of stock for
no functional gain).

### Longeron's reframe

If breakage is the actual end, we don't need to keep the means. We
separate vessel motion from stress accounting:

1. **Vessel motion = single Jolt rigid body.** Compound shape from
   every part's colliders, all transformed into the vessel-root
   frame. Body mass = total. No internal DOF, no constraints, no
   chain. Rigid by construction, no PGS-iterating-over-residuals.
2. **Joint forces = inverse dynamics on the part tree (Phase 4).**
   Once Jolt has integrated the vessel's overall motion, run RNEA
   leaf-to-root then root-to-leaf, computing the wrench across every
   spanning-tree edge. Compare against stock's `breakForce` /
   `breakTorque`, fire `Part.decouple()` on excess. Loop-closing
   non-tree edges (docking rings, KAS cycles) become additional force
   pairs from kinematic state, not constraints needing KKT
   projection.

We get a perfectly rigid vessel (no wobble) **and** per-edge stress
accounting (breakage), because we stopped trying to do both with the
same primitive. The trade-off: no visible structural flex. Stacks
don't sag under hard burns. Players who valued that should fly stock.

### Ownership history

Two earlier branches (`main` pre-pivot and `non-kinematic`) tried to
keep Unity-PhysX for collision and layer Longeron's solver on top in
C#. Both hit the same wall:

- *Kinematic Unity rigidbodies* — clean integration ownership, but
  PhysX silences contact reports for kinematic-vs-static pairs and
  silently reverts position writes against a freeze anchor.
- *Non-kinematic Unity rigidbodies* — contacts come back, but
  computing a corrective wrench requires our solver to *predict* what
  PhysX's contact solver is about to do. Two contact solvers, neither
  authoritative.

Diagnosis: Unity won't let the rigidbody state be shared cleanly with
PhysX. Longeron's pivot is to take over physics entirely — Jolt owns
broadphase, contact detection, contact solving, and integration;
Unity rigidbodies become kinematic proxies driven by Jolt for the
benefit of stock + modded code that reads `rb.velocity`,
`rb.position`, etc.

The implementation plan is at
`/Users/alx/.claude/plans/splendid-dancing-flute.md` (also referred
to as `PLAN.md` once committed).

## Architecture

### Ownership split

| Concern | Owner |
|---|---|
| Per-part Unity rigidbody poses | Derived from vessel pose; Longeron writes them post-step for stock-code consumers |
| Vessel rigid-body motion (source of truth) | **Jolt** (one Dynamic body per vessel) |
| Joint forces / stress accounting | **Longeron RNEA** (Phase 4; off until then) |
| Joint break detection | RNEA wrench vs. `breakForce` (Phase 4) |
| Numerical integration | **Jolt** |
| External wrench aggregation (thrust, aero, gravity) | **Longeron C#** (per-vessel record stream, applied at the originating part's world-CoM) |
| Collider geometry + broadphase | **Jolt** (mirror of every part's colliders into one compound shape per vessel) |
| Static-world contact detection | **Jolt** |
| Inter-vessel contact detection | **Jolt** |
| Terrain geometry | Unity PQS streams → mirrored to Jolt static `MeshShape`s in CB-frame |
| Unity rigidbodies | Kinematic proxies, pose-driven from the vessel body each tick |
| Docking alignment detection | KSP (`ModuleDockingNode`) → signals topology mutation |
| Decoupler fire | KSP modules → signals topology split |

Unity-PhysX simulation is effectively disabled for managed parts:
rigidbodies are kinematic, gravity off, all `PartJoint` /
`ConfigurableJoint` creation suppressed (or made a no-op). Unity is
kept around as a renderer (Transform hierarchy, MeshRenderer state)
and as the source of collider geometry mirrored into Jolt.

### Native bridge (`native/`, `mod/Longeron.Native/`)

The Jolt integration runs in a C++ shared library
(`longeron_native.{dll,dylib,so}`) bridged to C# via a single per-frame
P/Invoke. The bridge is deliberately thin: Jolt does its job, the
bridge just shuttles records in and out.

C ABI surface:

```c
LongeronWorld* longeron_world_create(const LongeronConfig* cfg);
void           longeron_world_destroy(LongeronWorld* w);
void           longeron_step(LongeronWorld* w,
                             const uint8_t* input, size_t input_len,
                             uint8_t* output, size_t output_cap, size_t* output_len,
                             float dt);
const char*    longeron_version(void);
```

Per-`FixedUpdate` flow:

1. **Accumulate inputs** (C# side, before the step):
   - Per-vessel force records — `ForceAtPosition` (force + world-CoM
     point) and `ForceDelta` (pure torque, no point), populated by
     Harmony patches on `Rigidbody.AddForce*` /
     `AddForceAtPosition` / `AddTorque`.
   - Mass-update records when a vessel's total mass crosses threshold
     (fuel burn, fairing eject, decouple).
   - Topology mutations — body create/destroy, optional inter-vessel
     constraint create/destroy — driven by the diff-based reconciler.
   - Step `dt` and scene-level config (gravity vector at vessel CoM).
2. **Single P/Invoke**: `longeron_step(world, input_buf, …,
   output_buf, …, dt)`. The native side parses input records,
   mutates the Jolt world, runs `JPH::PhysicsSystem::Update(dt, …)`,
   and emits output records.
3. **Read outputs** (C# side, after the step):
   - Per-vessel pose (position, orientation), linear/angular
     velocity. One pose per vessel — propagated to all member parts.
   - Per-pair contact records (body A, body B, point, normal, depth,
     impulse). Phase 4 will feed these into the RNEA pass as external
     wrenches at the appropriate part.
4. **Propagate pose to parts** from output. For each part:
   `rb.position = vesselPos + vesselRot * partLocalPos`,
   `rb.rotation = vesselRot * partLocalRot`,
   `rb.velocity = vesselLinVel + vesselAngVel × (vesselRot * partLocalPos)`,
   `rb.angularVelocity = vesselAngVel`. Offsets are frozen at
   body-create time.

Buffer schemas are typed records with a 1-byte type tag; the bridge
parses streams of records rather than fixed-layout arrays. Allows
additive schema evolution without breaking either side.

Marshalling discipline:

- Blittable structs only.
- Buffers pinned via `Marshal.AllocHGlobal` once at session start,
  reused every frame. No per-tick allocations.
- All native API uses `unsafe` C# with `byte*` and explicit offsets.
- No `string` marshalling; identifiers are integer body IDs.
- `[SuppressUnmanagedCodeSecurity]` on the P/Invoke decl.

Jolt is built with `JPH_DOUBLE_PRECISION=ON`. World coordinates are
doubles, eliminating the need for Krakensbane on the physics side.
Physics-space is the active mainBody's body-fixed rotating frame
(CB-frame), so terrain quads (PQS) and landed vessels stay
stationary by construction; the bridge's `CbFrame` helper handles
the boundary transform between Unity world and CB-frame at every
write/read.

`mod/Longeron.Native/` is the C# wrapper around the bridge —
`NativeBridge.cs` for the P/Invoke decls, `InputBuffer.cs` /
`OutputBuffer.cs` for serialization, `World.cs` for session lifetime.

### Featherstone solver (Phase 4 target — `mod/Longeron.Physics/`)

Phase 4 wires inverse-dynamics joint-force accounting onto the
single-body vessel motion. The solver doesn't drive poses; it
*reads* the vessel's overall acceleration and external wrenches
from Jolt and decomposes them into per-edge stresses.

- One spanning tree per vessel (while loaded + unpacked), built from
  `part.parent` walks. Every part knows its parent and the joint
  through which it's connected.
- **RNEA pass** — Featherstone Ch. 5. Two recursions:
  - Leaf-to-root forward: propagate kinematics (each subtree's
    velocity, acceleration, and momentum at the parent joint).
  - Root-to-leaf backward: propagate forces (the wrench transmitted
    across each joint, given external forces at every part).
- The output wrenches feed straight into the stock joint-break
  comparison: `|F| > breakForce || |τ| > breakTorque` triggers
  `Part.decouple()` via the existing decouple path; the topology
  reconciler then rebuilds the affected vessel(s).
- **Loop closures** (docking rings, KAS struts forming cycles)
  contribute additional force pairs computed from the kinematic
  state of their two endpoints. They're not constraints needing KKT
  projection — just extra terms in the wrench accounting.
- No allocations in the hot path. SoA `NativeArray<T>`-style layout.

Optional **forward-dynamics ABA mode** (Featherstone Ch. 7 + 9, off
by default): if a future user *wants* visible flex (space-station
ring sag, soft landing legs), the same data structures support
forward dynamics in reduced coordinates with stiff implicit-Euler
joint integration. ABA writes per-part poses into Jolt as kinematic
overrides, and Jolt sees no inter-body constraints from the vessel —
just per-tick pose updates. Unconditional stability at 50 Hz without
substepping. **This is opt-in; the default vessel model is rigid.**

In Phases 0–3, `mod/Longeron.Physics/` exists but is not wired into
the flight path. Phase 4 wires the RNEA pass first (needed for
parity with stock breakage); the optional forward-dynamics ABA mode
is post-Phase-4 if anyone asks for it.

### KSPBurst / HPC# integration (applies to `Longeron.Physics`, Phase 4)

Physics project (`Longeron.Physics`) is written as a **HPC# subset**
so inner loops can be AOT-compiled via
[KSPBurst](https://github.com/KSPModdingLibs/KSPBurst) — ships
Unity.Burst 1.7.4 + Unity.Mathematics 1.2.6 + Unity.Collections
0.8.0-preview + Unity.Jobs 0.2.9-preview.

Discipline within `Longeron.Physics`:

- Structs only in hot paths; no managed references crossing into
  `[BurstCompile]` code.
- Blittable types only. `Unity.Mathematics` (`float3`, `float3x3`,
  `float4x4`) instead of Unity's `Vector3 / Quaternion` in numeric
  code.
- No exceptions in Burst-compiled code; preconditions via asserts
  or error codes.
- `NativeArray<T>` for body state, not managed arrays.
- No virtual dispatch, no generics over reference types.

The native bridge in `Longeron.Native` does *not* use Burst — it's
a plain managed wrapper around P/Invoke. Burst applies only to the
RNEA / ABA math in `Longeron.Physics` once Phase 4 wires it in.

Discipline stops at the module boundary. `Longeron` (KSP
integration) is ordinary C#.

### KSP integration (`mod/Longeron/`)

- `LongeronAddon` — session-level MonoBehaviour. Owns the
  `Longeron.Native.World` lifetime (one per flight scene), wires
  `GameEvents` to topology / SOI / rotating-frame handlers, installs
  Harmony patches.
- `LongeronVesselModule : VesselModule` — per-vessel kick-off. On
  `OnGoOffRails` applies kinematic takeover to every part rigidbody
  immediately, then marks the vessel dirty for the reconciler. On
  `OnGoOnRails`, tears down the vessel's Jolt body and restores
  rigidbodies to stock-physics defaults.
- `LongeronSceneDriver` — `[DefaultExecutionOrder(10000)]` driver
  bracketing every flight tick. Per-tick: snapshot CB-frame, run
  topology reconciler, emit per-vessel mass updates, step Jolt,
  drain output, propagate vessel pose to parts, refresh vessel-level
  derived velocity fields.
- Harmony patches in `Patches/` — force redirect
  (`Rigidbody.AddForce*` → vessel body at world-CoM), Krakensbane
  disable (patch `FixedUpdate` / `AddExcess` /
  `GetFrameVelocity[V3f]` to no-ops), `OrbitDriver.TrackRigidbody`
  bypass for kinematic vessels, mass updates, joint-creation
  suppression (Phase 4).

`FixedUpdate` shape:

1. **Pre-step** (early hook): drain accumulated forces, mass updates,
   topology mutations into the input buffer.
2. **Step**: single P/Invoke into `longeron_step`. Jolt advances dt.
3. **Post-step** (late hook): read vessel poses from output buffer,
   propagate to all per-part rigidbodies, drain contact records.

The force-redirect Harmony patches mean every modded force source
(FAR, RealFuels, RealPlume, KER, etc.) lands in our per-vessel record
stream without any per-mod special handling. Unity rigidbodies are
kinematic so the original `AddForce` is a no-op anyway; the Harmony
prefix replaces it cleanly.

### Resource lifecycle: ECS-style component ownership

Longeron leans on Unity's GameObject/MonoBehaviour ECS to own
per-part resources rather than tracking them in side dicts that have
to be manually invalidated.

Concrete pattern: `JoltBody : MonoBehaviour` is attached to every
Part GameObject we manage. It carries the **vessel's** `BodyHandle`
(every part in the vessel shares the same handle), the part's
frozen `PartLocalPos` / `PartLocalRot` offset from the vessel root,
the latest analytic velocity (`LastVelocity`,
`LastAngularVelocity`), and an `OwnsBody` flag. Two consequences
fall out for free:

- **Lookup**: `rb.GetComponent<JoltBody>()` is one Unity-native O(1)
  probe — used by `RigidbodyForceHooks` to find the vessel body
  instantly. No `Dictionary<Rigidbody, BodyHandle>` to keep coherent.
- **Cleanup**: when the *root* Part GameObject is destroyed (crash
  damage, splash damage, scene transition) Unity destroys its
  components; `JoltBody.OnDestroy` queues the corresponding
  `BodyDestroy` only if `OwnsBody == true`. Non-root parts share the
  resource and don't free it; their destruction is a topology
  mutation, observed by the reconciler which rebuilds the vessel
  body for the new (smaller) part list.

The reverse handle → Part lookup map (`_byHandle`) only stores the
owner JoltBody, since output records like `BodyPose` are
vessel-level — the driver propagates that one pose to all member
parts via the recorded offsets.

Decoupling (and other topology mutations) tear down the old vessel's
body and rebuild a fresh one for each resulting vessel. The
reconciler's diff trigger is "the part-set changed since last
reconcile" — a single condition that catches every event class
(decouple, dock, joint break, fairing eject, struct failure).

The pattern extends to any other per-part state we want to track
(per-part contact accumulators, debug visualization handles): give
it a MonoBehaviour, let `OnDestroy` speak for it. `QuadBody` for
PQS terrain quads is the same shape — the static pending-destroy
queue on `JoltBody` is shared so all resource owners drain through
the same site.

## PartJoint strategy: kill creation, stub for compatibility

Unity rigidbodies are kinematic; PhysX joints would be no-ops even
if created. Strategy (Phase 4):

1. **Suppress `PartJoint.CreateJoint`** — Harmony prefix that
   returns a stub `PartJoint` / `ConfigurableJoint` pair satisfying
   any null checks downstream but with no underlying PhysX state.
   Body relationships are implicit in our compound shape; nothing
   in Jolt represents an intra-vessel joint.
2. **Stub `PartJoint.DestroyJoint`** to fire stock events but skip
   the PhysX teardown.
3. **Stub `ConfigurableJoint.currentForce` etc.** if anything reads
   them — return RNEA-computed values from the latest tick.
4. **Inter-mod joint creation** (KAS/KIS extra `ConfigurableJoint`s)
   — intercept at creation, treat the connected parts as the same
   vessel for the compound-shape purposes, or as a non-tree edge
   for inverse-dynamics force accounting.

KSP 1.12.5 is a frozen target. Squad isn't shipping another patch.
Bytes don't move. Reflection against private fields is permanently
safe. Transpilers stay correct forever. A near-copy replacement of
a stock method is a perfectly reasonable tool.

## Topology lifecycle

Mid-flight topology mutations are observed via `GameEvents`, not
patched per-method. Stock fires `onVesselWasModified(vessel)` after
every structural change — decouple, couple, undock, joint break,
ModuleDockingNode engage. `onVesselCreate` covers any vessel born
from splits. `onVesselDestroy` covers end-of-life.

The handlers all funnel into `TopologyReconciler.MarkDirty(vessel)`.
At the start of the next `FixedUpdate`, the reconciler walks each
dirty vessel's current part set and **diffs against the recorded
`ManagedVessel.LastParts`**. Any difference triggers a full body
rebuild: `BodyDestroy` for the old vessel body + `BodyCreate` with
a fresh compound shape from the new part list.

Why diff-based reconciliation over per-method Harmony postfixes:

- One subscription set covers every stock and modded mutation path
  (KSP/KAS/KIS/Breaking Ground/custom decouplers all funnel through
  `Part.Couple` / `Part.decouple` / `PartJoint.DestroyJoint`, all
  of which fire `onVesselWasModified`).
- Idempotent: multiple events on the same vessel within one tick
  collapse to one diff.
- Order-independent: `onVesselWasModified(old)` and
  `onVesselCreate(new)` from a decouple can fire in either order;
  reconciliation observes final state.
- No internal-call assumptions about specific stock methods.

| Event | Mechanism | Jolt action |
|---|---|---|
| Vessel unpacks | `LongeronVesselModule.OnGoOffRails` → kinematic takeover + MarkDirty | Reconciler builds 1 compound body for the vessel |
| Vessel packs | `LongeronVesselModule.OnGoOnRails` (immediate) | Destroy the vessel body; restore rb to stock |
| Part coupled | `onVesselWasModified` → MarkDirty | Reconciler rebuilds the vessel body with new part list |
| Part decoupled | `onVesselWasModified` (old) + `onVesselCreate` (new) → MarkDirty both | Both vessels get fresh body builds |
| Joint break (Phase 4) | RNEA wrench exceeds `breakForce` → synthesize `Part.decouple()` | Same as decouple |
| Docking engaged | `onVesselWasModified` (and `onVesselDestroy` for absorbed vessel) | Both vessels destroyed; one merged body created |
| Undocking | `onVesselWasModified` (old) + `onVesselCreate` (new) | Both rebuilt |
| Part destroyed (root) | Unity `OnDestroy` → `JoltBody.OnDestroy` (OwnsBody=true) queues `BodyDestroy` | Drained by reconciler next tick |
| Part destroyed (non-root) | Unity `OnDestroy` → no body destroy | Reconciler rebuilds vessel without that part |

The vessel-body rebuild approach is uniformly cheap: one Jolt body
destroy + one create, regardless of how many parts changed.
Velocity/momentum inheritance comes from the new body reading its
starting state from the old body's pose; separation impulses (from
`ModuleDecouple.OnDecouple`'s `AddForce`) redirect normally to the
new vessels' bodies.

### Loop closures (Phase 4)

Stock KSP's docking joints + KAS struts can form cycles in the part
graph. With single-body-per-vessel + RNEA, cycles are handled
trivially: parts in the same connected component share a vessel
body (no internal flex), and any non-tree edges (docking ports
between different vessels, KAS struts spanning the cycle) become
additional force-pair contributions to the RNEA wrench accounting.
There is no KKT system, no Lagrange multipliers, no constraint
stabilization, and no "promote loop closure into spanning tree on
split" case.

## Struts in the rigid-vessel world

In Longeron, every intra-vessel attachment is already part of the
rigid compound shape. Auto-strut and explicit struts on rigid
stacks become cosmetic — there's no wobble for them to mitigate.
Struts retain meaning only for cycles and inter-vessel coupling:

- **Stiffening optional flex** (if forward-dynamics ABA mode is ever
  enabled for a vessel) — strutting across a BG servo or soft
  landing leg locks (or stiffens) a real DOF.
- **Closing inter-vessel topology** — space-station rings.
  Struts contribute additional force pairs to the RNEA stress
  accounting between docked vessels.

**Do not invent a "strut-across-decoupler survives fire" mechanic.**
Stock behavior: struts always break on decoupler fire. That is not
a backward-compatibility concern; it's a fact of the game.

## Open questions — verify before/during build

Concrete items to confirm via the `ksp-reference` skill, web
research, and/or live in-game probes:

1. **`WheelCollider` replacement.** Wheels are inherently per-part
   mobile and need their own Jolt body + a constraint or motor
   coupling them to the vessel. Custom raycast suspension on a
   small Jolt body vs. Jolt's `VehicleConstraint`. Phase 3.5
   decision.
2. **Breaking Ground robotic parts.** Servos and pistons are also
   per-part mobile. Each becomes its own Jolt body coupled to the
   vessel by a `SixDOFConstraint` (or, in the optional flex mode, an
   ABA `Compliant6DOF` record with a motor). Need to understand
   where stock PAM code calls into the joint before we replace it.
3. **KAS/KIS and other non-stock joint modules.** These add extra
   `ConfigurableJoint`s outside `Part.attachJoint`. Enumerate them
   in the joint-suppression pass; treat connected parts as a single
   vessel for compound-shape purposes when KAS forms a rigid link,
   or as inter-vessel constraints in Jolt when it forms a soft
   coupling.
4. **Which `Part` / `PartJoint` fields stock code reads
   during/after joint break.** If anything downstream of
   `DestroyJoint` expects specific rigidbody/joint state, the stub
   must preserve it.
5. **Per-part inertia accuracy.** Jolt's auto-inertia for the
   compound shape under uniform density is approximate; some parts
   (engines) have significant CoM offsets. May need explicit
   inertia overrides on the `BodyCreate` path. Punt until breakage
   is wired and we can measure parity gaps.

Per project memory (`feedback_verify_ksp_mechanics.md`): do not
assert stock KSP behavior from pattern-match; check decompiled
source before committing a design to it. Per
`feedback_dont_tune_architectural.md`: when knobs keep yielding
diminishing returns, the model is wrong — step back and reframe.

## Project layout

```
longeron/
  stubs/                        # KSP 1.12.5 reference assemblies (compile-time only)
    Assembly-CSharp.dll
    Assembly-CSharp-firstpass.dll
    UnityEngine.*.dll
    mscorlib.dll, System*.dll
  native/                       # C++ Jolt bridge
    CMakeLists.txt
    third_party/JoltPhysics/    # git submodule, pinned release
    src/
      bridge.{h,cpp}            # C ABI declarations + dispatch
      world.{h,cpp}             # LongeronWorld: PhysicsSystem,
                                #   body registry, layer manager
      records.h                 # shared wire-format enum (with C# mirror)
      contact_listener.{h,cpp}  # JPH::ContactListener impl
  mod/
    Longeron.sln
    Longeron.Native/            # C# wrapper around the bridge
      Longeron.Native.csproj    # net48
      src/
        NativeBridge.cs         # extern "C" P/Invoke
        InputBuffer.cs          # writer (RecordType enum mirrors records.h)
        OutputBuffer.cs         # reader
        BodyHandle.cs           # opaque integer body ID
        World.cs                # session lifetime owner
    Longeron.Physics/           # HPC# RNEA + (optional) ABA solver — Phase 4 target
      Longeron.Physics.csproj   # net48, Burst-compatible subset
      src/
        Spatial/                # 6D motion/force algebra
        ArticulatedBody.cs      # SoA flat arrays
        Solver/
          Rnea.cs               # recursive Newton-Euler (inverse dynamics) — primary
          ABA.cs                # forward dynamics (optional flex mode)
        Math/                   # float3, float3x3, quaternion
    Longeron/                   # KSP integration plugin
      Longeron.csproj           # net48 + Lib.Harmony + Assembly-CSharp refs
      src/
        LongeronAddon.cs        # session entry point + native World owner
        LongeronVesselModule.cs # per-vessel kick-off (kinematic takeover, mark dirty)
        LongeronSceneDriver.cs  # FixedUpdate bracket: input fill → step → output read → pose propagation
        JoltBody.cs             # per-part MonoBehaviour with shared vessel handle + frozen offsets
        Integration/            # SceneRegistry, ManagedVessel, ColliderWalker, CbFrame, DiagLogger
        Patches/                # Harmony: force redirect, Krakensbane disable, OrbitDriver, mass, joint-creation suppression
  justfile                      # just build / just install $KSP_PATH / just native-build
  LICENSE
  README.md
  CLAUDE.md                     # this file
```

## Build & install

Requires `just`, `dotnet`, `cmake`, a C++17 compiler. Native lib is
required for the mod to function.

```bash
just native-build               # build native shared library
just mod-build                  # build C# projects
just build                      # both
just dist                       # assemble release/Longeron.zip (managed + native)
just install ~/KSP_osx          # build + deploy into a KSP directory
```

The native lib must be rebuilt when Jolt is updated or the C ABI
changes. Local dev builds for the host platform; release matrix
produces Win64 / macOS universal / Linux x64 artifacts.

## Relationship to Hypergolic Systems

Longeron is a sibling mod to HGS, same author, same KSP target,
**independent**. HGS's persistent-world / background-simulation goals
benefit from Longeron's Jolt-based per-island parallelism (and
Phase 4's parallel RNEA passes — tick thousands of vessels at
microseconds each), but there's no hard dependency. Either mod
should load standalone.

## References

- Featherstone, *Rigid Body Dynamics Algorithms* (2008). Canonical
  RNEA + ABA + closed-loop treatment. Ch. 5 (RNEA) is the Phase 4
  workhorse; Ch. 7 + 9 cover ABA + spring-damper joints for the
  optional flex mode.
- [Jolt Physics](https://github.com/jrouwe/JoltPhysics) — the engine.
  See in particular [the architecture
  document](https://jrouwe.github.io/JoltPhysics/index.html).
- [KSPBurst](https://github.com/KSPModdingLibs/KSPBurst) — Unity
  Burst toolchain packaged for KSP (Phase 4).
- Unity [Mathematics](https://docs.unity3d.com/Packages/com.unity.mathematics@latest)
  / [Collections](https://docs.unity3d.com/Packages/com.unity.collections@latest)
  / [Jobs](https://docs.unity3d.com/Packages/com.unity.jobs@latest)
  — the HPC# surface `Longeron.Physics` targets.
- `ksp/Assembly-CSharp/` in sibling HGS repo — decompiled stock
  source, authoritative for behavior questions.
