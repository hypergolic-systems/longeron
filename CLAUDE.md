# Longeron

A KSP 1.12.5 mod that replaces Unity's PhysX with [Jolt
Physics](https://github.com/jrouwe/JoltPhysics) for vessel simulation, and
layers a Featherstone articulated-body solver on top of Jolt to model
joint compliance. Goal: eliminate the wobble that comes from KSP's chain-
of-`ConfigurableJoint`s architecture without giving up the structural flex
Squad's joint model intentionally provides.

The name is the aerospace structural member that keeps a fuselage rigid
along its length. That is approximately what this mod does.

## Why this exists

KSP's wobble is architectural, not a PhysX-tuning bug:

- Each `Part` is a separate `Rigidbody` connected to its parent by a
  `ConfigurableJoint`.
- PhysX's iterative PGS solver converges poorly on long chains of
  compliant constraints with high mass ratios — the residual on each
  constraint depends on its neighbours' residuals, and information
  propagates one link per iteration.
- Autostrut and KJR work because they short-circuit the chain, not
  because they fix anything fundamental.

Compliance itself is not the bug. Squad chose stiff-but-not-infinite
joints because they wanted real structural flex; the bug is PGS
diverging into wobble instead of converging into flex on long
maximal-coordinate chains.

Two earlier branches (`main` pre-pivot and `non-kinematic`) explored
keeping Unity-PhysX for collision and layering Longeron's solver on
top. Both hit the same wall from opposite sides:

- *Kinematic Unity rigidbodies* — clean integration ownership, but
  PhysX silences contact reports for kinematic-vs-static pairs and
  silently reverts position writes against a freeze anchor.
- *Non-kinematic Unity rigidbodies* — contacts come back, but
  computing a meaningful corrective wrench requires our solver to
  *predict* what PhysX's contact solver is about to do. Two contact
  solvers, neither authoritative.

Diagnosis: Unity won't let the rigidbody state be shared cleanly with
PhysX. Longeron's pivot is to take over physics entirely — Jolt owns
broadphase, contact detection, contact solving, and integration; Unity
rigidbodies become kinematic proxies driven by Jolt's output for the
benefit of stock + modded code that reads `rb.velocity`, `rb.position`,
etc.

Featherstone remains the long-term joint-force model. In Phases 0–3,
Jolt's native `SixDOFConstraint` carries joints with stiff spring/damper
drives — measurably better than PhysX/PGS at long chains and high mass
ratios, but still maximal-coordinate. Phase 4 replaces those Jolt
constraints with an ABA pass that writes per-part poses to Jolt each
tick. With all-6-DOF compliant joints, loop closures collapse out of the
math: cycle-closing graph edges become additional compliant force terms
on the kinematic state of their endpoints, not constraints needing KKT
projection.

The implementation plan is at
`/Users/alx/.claude/plans/splendid-dancing-flute.md` (also referred to as
`PLAN.md` once committed).

## Architecture

### Ownership split

| Concern | Owner |
|---|---|
| Part poses (source of truth) | **Jolt** (ABA writes to Jolt in Phase 4) |
| Inter-part forces / constraints | **Jolt** (Phase 2) → **ABA** (Phase 4) |
| Numerical integration | **Jolt** |
| External wrench aggregation (thrust, aero, gravity) | **Longeron C#** (per-tick accumulator) |
| Collider geometry + broadphase | **Jolt** (mirror of Unity collider data) |
| Static-world contact detection | **Jolt** |
| Inter-vessel contact detection | **Jolt** |
| Terrain geometry | Unity PQS streams → mirrored to Jolt static `MeshShape`s |
| Unity rigidbodies | Kinematic proxies, pose-driven from Jolt each tick |
| Docking alignment detection | KSP (`ModuleDockingNode`) → signals topology mutation |
| Decoupler fire | KSP modules → signals topology split |

Unity-PhysX simulation is effectively disabled for managed parts:
rigidbodies are kinematic, gravity off, all `PartJoint`/`ConfigurableJoint`
creation suppressed. Unity is kept around as a renderer (Transform
hierarchy, MeshRenderer state) and as the source of collider geometry
mirrored into Jolt.

### Native bridge (`native/`, `mod/Longeron.Native/`)

The Jolt integration runs in a C++ shared library
(`longeron_native.{dll,dylib,so}`) bridged to C# via a single per-frame
P/Invoke. The bridge is deliberately thin: Jolt does its job, the bridge
just shuttles records in and out.

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
   - Per-body force/torque deltas — sum of all `Rigidbody.AddForce*` /
     `AddTorque` calls intercepted via Harmony since the last tick.
   - Per-body kinematic velocity targets (Phase 4: ABA's predicted
     velocity).
   - Mass/inertia delta records for parts whose mass changed (fuel
     burn, fairing ejection, etc.).
   - Topology mutations — body create/destroy, constraint
     create/destroy, shape updates — queued by Harmony patches on
     `Part.Couple`, `Part.decouple`, `PartJoint.DestroyJoint`,
     `ModuleDockingNode` events.
   - Step `dt` and scene-level config (gravity vector at vessel COM).
2. **Single P/Invoke**: `longeron_step(world, input_buf, …, output_buf, …, dt)`.
   The native side parses input records, mutates the Jolt world, runs
   `JPH::PhysicsSystem::Update(dt, …)`, and emits output records.
3. **Read outputs** (C# side, after the step):
   - Per-body pose (position, orientation), linear/angular velocity.
   - Per-pair contact records (body A, body B, point, normal, depth,
     impulse) — fed back as external wrenches to ABA in Phase 4.
4. **Write Unity transforms** from output. Kinematic Unity `Rigidbody`
   gets `position`/`rotation`/`velocity` set so stock and modded code
   see consistent state.

Buffer schemas are typed records with a version prefix; the native bridge
parses streams of records rather than fixed-layout arrays. Allows
additive schema evolution without breaking either side.

Marshalling discipline:

- Blittable structs only.
- Buffers pinned via `GCHandle.Alloc(GCHandleType.Pinned)` once at session
  start, reused every frame. No per-tick allocations.
- All native API uses `unsafe` C# with `byte*` and explicit offsets.
- No `string` marshalling; identifiers are integer body IDs.
- `[SuppressUnmanagedCodeSecurity]` on the P/Invoke decl.

Jolt is built with `JPH_DOUBLE_PRECISION=ON`. World coordinates are
doubles, eliminating Krakensbane synchronization on the physics side.
Krakensbane keeps shifting Unity's rendering origin; Longeron applies
the offset on the C# side when writing Unity rigidbody poses.

`mod/Longeron.Native/` is the C# wrapper around the bridge —
`NativeBridge.cs` for the P/Invoke decls, `InputBuffer.cs` /
`OutputBuffer.cs` for serialization, `World.cs` for session lifetime.

### Featherstone solver (Phase 4 target — `mod/Longeron.Physics/`)

Reduced-coordinate articulated-body dynamics via Featherstone's ABA:

- One articulated body per vessel (while loaded + unpacked).
- Tree topology from `part.parent` walks; every body knows its
  spanning-tree parent and its joint-to-parent.
- **Single joint primitive**: `Compliant6DOF { K[6], C[6], q_neutral, motor[6] }`.
  Per-axis stiffness/damping/motor selects between what stock
  KSP would call fixed (high K, all axes), revolute (high K on 5,
  motor on 1), prismatic (high K on 5, motor on 1), free (zero K),
  or soft (medium K with high C). No tagged-union dispatch.
- **No loop closures.** Cycle-closing graph edges (docking rings,
  KAS struts forming cycles) are additional `Compliant6DOF` records
  not in the spanning tree, contributing force pairs to their two
  endpoints from the kinematic state of those endpoints. No KKT
  system, no Lagrange multipliers, no constraint stabilization.
- **Implicit ABA integration** for stiff joints (Featherstone Ch. 9
  spring-damper joint formulation) — unconditional stability without
  substepping below KSP's 50 Hz `FixedUpdate`.
- Joint break detection: `|K·Δq + C·q̇| > breakForce` triggers the
  stock joint-break path.
- No allocations in the hot path. SoA `NativeArray<T>`-style layout.

In Phases 0–3, `mod/Longeron.Physics/` exists but is not wired into the
flight path. The bridge uses Jolt's `SixDOFConstraint` instead.
Phase 4 swaps the joint-force model: Jolt sees per-tick kinematic pose
updates with no inter-body constraints, ABA owns joint forces, and the
contact listener feeds external wrenches to ABA.

### KSPBurst / HPC# integration (applies to `Longeron.Physics`, Phase 4)

Physics project (`Longeron.Physics`) is written as a **HPC# subset** so
inner loops can be AOT-compiled via
[KSPBurst](https://github.com/KSPModdingLibs/KSPBurst) — ships
Unity.Burst 1.7.4 + Unity.Mathematics 1.2.6 + Unity.Collections
0.8.0-preview + Unity.Jobs 0.2.9-preview.

Discipline within `Longeron.Physics`:

- Structs only in hot paths; no managed references crossing into
  `[BurstCompile]` code.
- Blittable types only. `Unity.Mathematics` (`float3`, `float3x3`,
  `float4x4`) instead of Unity's `Vector3/Quaternion` in numeric code.
- No exceptions in Burst-compiled code; preconditions via asserts or
  error codes.
- `NativeArray<T>` for body state, not managed arrays.
- No virtual dispatch, no generics over reference types.

The native bridge in `Longeron.Native` does *not* use Burst — it's a
plain managed wrapper around P/Invoke. Burst applies only to the
articulated-body math in `Longeron.Physics` once Phase 4 wires it in.

Discipline stops at the module boundary. `Longeron` (KSP integration) is
ordinary C#.

### KSP integration (`mod/Longeron/`)

- `LongeronAddon` — session-level MonoBehaviour. Owns the
  `Longeron.Native.World` lifetime (one per flight scene).
- `LongeronVesselModule : VesselModule` — per-vessel Jolt body lifecycle.
  On `Vessel.GoOffRails`, queues body-create records for each part. On
  `Vessel.GoOnRails`, queues body-destroy records.
- Harmony patches in `Patches/` — force redirect (`Rigidbody.AddForce*`),
  topology events (`Part.Couple`, `Part.decouple`,
  `PartJoint.DestroyJoint`, `ModuleDockingNode` state transitions), mass
  updates, joint-creation suppression.

`FixedUpdate` shape:

1. **Pre-step** (early hook): drain accumulated forces, mass updates,
   topology mutations into the input buffer.
2. **Step**: single P/Invoke into `longeron_step`. Jolt advances dt.
3. **Post-step** (late hook): read poses from output buffer, write
   Unity rigidbody position/rotation/velocity (with Krakensbane offset
   applied), drain contact records.

The force-redirect Harmony patches mean every modded force source (FAR,
RealFuels, RealPlume, KER, etc.) lands in our per-body accumulator
without any per-mod special handling. Unity rigidbodies are kinematic so
the original `AddForce` is a no-op anyway; the Harmony prefix replaces
it cleanly.

## PartJoint strategy: kill creation, stub for compatibility

Unity rigidbodies are kinematic; PhysX joints would be no-ops even if
created. Strategy:

1. **Suppress `PartJoint.CreateJoint`** — Harmony prefix that returns
   a stub `PartJoint`/`ConfigurableJoint` pair satisfying any null
   checks downstream but with no underlying PhysX state. Body
   relationships live in Jolt instead, addressed via the per-body
   `BodyHandle`.
2. **Stub `PartJoint.DestroyJoint`** to fire stock events but skip the
   PhysX teardown.
3. **Stub `ConfigurableJoint.currentForce` etc.** if anything reads
   them — return values derived from Jolt constraint reaction force or
   ABA Lagrange multipliers (Phase 4).
4. **Inter-mod joint creation** (KAS/KIS extra `ConfigurableJoint`s) —
   intercept at creation, forward to Jolt as `SixDOFConstraint`s
   (Phase 2) or ABA `Compliant6DOF` records (Phase 4).

KSP 1.12.5 is a frozen target. Squad isn't shipping another patch.
Bytes don't move. Reflection against private fields is permanently safe.
Transpilers stay correct forever. A near-copy replacement of a stock
method is a perfectly reasonable tool.

## Topology lifecycle

| Event | KSP source | Jolt action | ABA action (Phase 4) |
|---|---|---|---|
| Vessel unpacks | `Vessel.GoOffRails` postfix | Create N bodies + N−1 SixDOFConstraints | Build articulated body |
| Vessel packs | `Vessel.GoOnRails` postfix | Destroy bodies + constraints | Release articulated body |
| Part coupled | `Part.Couple` postfix | Add bodies + constraints (or merge ObjectLayers if inter-vessel) | Merge subtree |
| Part decoupled | `Part.decouple` postfix | Remove constraint between the two parts | Split tree |
| Joint destroyed | `PartJoint.DestroyJoint` postfix | Remove constraint | Potentially split tree |
| Docking engaged | `ModuleDockingNode` state transitions | Add cross-vessel constraint, merge ObjectLayers | Merge tree (compliant non-tree edge if cycle) |
| Undocking | `ModuleDockingNode.Undock` | Remove constraint, split ObjectLayer | Split tree (or remove non-tree edge) |

Topology rebuilds are **deferred to the next FixedUpdate start**, not
executed synchronously from the event handler. Stock event handlers run
against the pre-mutation state; Longeron observes the final state at
the start of the next tick and emits the corresponding mutation records.

### Loop closures collapse (Phase 4)

With Phase 4's all-`Compliant6DOF` model, the spanning-tree-vs-cycle
distinction is bookkeeping only — every joint is the same primitive,
and a non-tree edge just contributes force pairs from kinematic state
each tick. There is no KKT system, no Lagrange multipliers, no
constraint stabilization, and no "promote loop closure into spanning
tree on split" case. The split case becomes "drop one or more compliant
edges from the affected component."

## Struts in the all-compliant world

In Longeron, every stack/surface attach is already a `Compliant6DOF`
joint. Auto-strut and explicit struts on rigid stacks become cosmetic —
the wobble they were mitigating doesn't arise. Struts retain meaning
only as additional compliant joint records, useful for:

- **Stiffening intentionally compliant joints** — strutting across a BG
  servo or a soft landing gear locks (or stiffens) a real DOF, not a
  phantom one.
- **Adding compliance to closed topologies** — space-station rings.
  Without struts the cycle would still close via the docking joints,
  but adding struts adds parallel compliance, lowering the effective K.

**Do not invent a "strut-across-decoupler survives fire" mechanic.**
Stock behavior: struts always break on decoupler fire. That is not a
backward-compatibility concern; it's a fact of the game.

## Open questions — verify before/during build

Concrete items to confirm via the `ksp-reference` skill, web research,
and/or a live bridge probe:

1. **Jolt kinematic contact callback semantics** — verify in Phase 1
   that `OnContactPersisted` and `OnContactAdded` fire for
   kinematic-vs-static and kinematic-vs-kinematic body pairs. PhysX
   silenced these; Jolt should not, but the entire pivot rests on this
   working. **Day 1 of Phase 1 verification.**
2. **Jolt determinism flag** — confirm `JPH::PhysicsSettings::mDeterministicSimulation`
   actually delivers cross-platform reproducibility for our workload.
   Useful for save/load and HGS background simulation.
3. **Jolt body type for parts** — `EMotionType::Kinematic` driven by
   `SetPosition` per tick (simplest), vs driven by `SetLinearVelocity`
   (potentially supports CCD), vs `EMotionType::Dynamic` with all forces
   overridden. Default to position-driven kinematic; revisit in
   Phase 3.5 for wheels/CCD-needing parts.
4. **`WheelCollider` replacement.** Custom raycast suspension on Jolt
   bodies vs Jolt's `VehicleConstraint`. Phase 3.5 decision.
5. **Breaking Ground robotic parts.** Servos and pistons as
   `SixDOFConstraint` in Phase 2, ABA `Compliant6DOF` with motor in
   Phase 4. Need to understand where stock PAM code calls into the
   joint before we replace it.
6. **KAS/KIS and other non-stock joint modules.** These add extra
   `ConfigurableJoint`s outside `Part.attachJoint`. Enumerate them in
   the joint-suppression pass; mirror their joints into Jolt.
7. **Which `Part`/`PartJoint` fields stock code reads during/after
   joint break.** If anything downstream of `DestroyJoint` expects
   specific rigidbody/joint state, the stub has to preserve it.
8. **PQS collider streaming events** (Phase 3b) — exact event
   names/hooks for collider quad spawn/despawn.

Per project memory (`feedback_verify_ksp_mechanics.md`): do not assert
stock KSP behavior from pattern-match; check decompiled source before
committing a design to it.

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
      input_buffer.{h,cpp}      # C# input stream decoder
      output_buffer.{h,cpp}     # output stream encoder
      contact_listener.{h,cpp}  # JPH::ContactListener impl
  mod/
    Longeron.sln
    Longeron.Native/            # C# wrapper around the bridge
      Longeron.Native.csproj    # net48
      src/
        NativeBridge.cs         # extern "C" P/Invoke
        InputBuffer.cs          # writer
        OutputBuffer.cs         # reader
        BodyHandle.cs           # opaque integer body ID
        World.cs                # session lifetime owner
    Longeron.Physics/           # HPC# articulated-body solver (Phase 4 target)
      Longeron.Physics.csproj   # net48, Burst-compatible subset
      src/
        Spatial/                # 6D motion/force algebra
        ArticulatedBody.cs      # SoA flat arrays
        Solver/
          ABA.cs                # Featherstone forward dynamics
          Rnea.cs               # recursive Newton-Euler (inverse dynamics)
        Contact/                # narrowphase + manifolds (legacy; Phase 4 may not need)
        Math/                   # float3, float3x3, quaternion, math
    Longeron/                   # KSP integration plugin
      Longeron.csproj           # net48 + Lib.Harmony + Assembly-CSharp refs
      src/
        LongeronAddon.cs        # session entry point + native World owner
        LongeronVesselModule.cs # per-vessel body lifecycle
        LongeronSceneDriver.cs  # FixedUpdate bracket: input fill → step → output read
        Integration/            # InertiaEstimator, scene/vessel state
        Patches/                # Harmony patches: force redirect, topology, mass
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
benefit from Longeron's Jolt-based per-island parallelism and Phase 4's
parallel ABA passes (tick thousands of vessels at microseconds each),
but there's no hard dependency. Either mod should load standalone.

## References

- Featherstone, *Rigid Body Dynamics Algorithms* (2008). Canonical ABA
  + closed-loop treatment.
- [Jolt Physics](https://github.com/jrouwe/JoltPhysics) — the engine.
  See in particular [the architecture
  document](https://jrouwe.github.io/JoltPhysics/index.html).
- [KSPBurst](https://github.com/KSPModdingLibs/KSPBurst) — Unity Burst
  toolchain packaged for KSP (Phase 4).
- Unity [Mathematics](https://docs.unity3d.com/Packages/com.unity.mathematics@latest)
  / [Collections](https://docs.unity3d.com/Packages/com.unity.collections@latest)
  / [Jobs](https://docs.unity3d.com/Packages/com.unity.jobs@latest) — the
  HPC# surface `Longeron.Physics` targets.
- `ksp/Assembly-CSharp/` in sibling HGS repo — decompiled stock source,
  authoritative for behavior questions.
