# Longeron — Jolt pivot plan

## Context

The current architecture (Featherstone ABA in C# + Unity-PhysX for collision)
has been explored on two branches and hit the same structural wall from
opposite sides:

- **`main`** — kinematic Unity rigidbodies. Clean integration ownership, but
  PhysX does not generate contact reports for kinematic bodies against static
  geometry (and the kinematic-vs-static contact callbacks we did get
  required a load-bearing `RigidbodyConstraints.FreezeAll` toggle around every
  per-tick `rb.position` write to stop PhysX silently reverting our writes).
  Sub-mm rotation drift, joint slop under contact — residue of two
  integrators sharing one rigidbody.
- **`non-kinematic`** — PhysX integrates rbs, Longeron provides corrective
  wrenches via `AddForce`. To compute a correct residual `m·a − fExt`, ABA
  has to *predict* what PhysX's contact solver is about to apply; that means
  re-implementing a contact penalty model in our solver to chase PhysX's
  contact solver. Two contact solvers, neither authoritative. The PLAN.md on
  that branch documents the dead end.

Both branches confirm the same diagnosis: **Unity will not let us share the
rigidbody state cleanly with PhysX.** Either Unity owns integration (and we
fight position writes), or Unity owns contacts (and we run a worse contact
solver to stay in sync). There is no middle ground inside Unity's physics
ownership model.

This plan pivots to Jolt for **all of broadphase, contact detection,
contact solving, and integration**. Unity rigidbodies become kinematic
proxies driven by Jolt's output for the benefit of stock + modded code that
reads `rb.velocity`, `rb.position`, etc. PhysX simulation is effectively
disabled for parts (gravity off, all joints removed, kinematic flag set).

Featherstone is preserved as the long-term solver target but **layered on
top of Jolt**, not in place of it. Initial phases use Jolt's native
SixDOFConstraint with stiff spring/damper drives; Phase 4 replaces those
internal joint forces with an ABA pass that writes per-part poses to Jolt
each tick. Loop closures collapse out of the math because every joint is
6-DOF compliant — non-tree graph edges become additional compliant joints
contributing force pairs from kinematic state, not constraints needing KKT
projection.

## Architecture summary

Per-vessel design:

- **Many Jolt bodies per vessel.** One kinematic `JPH::Body` per `Part`,
  collision shape derived from the part's existing colliders. No
  `MutableCompoundShape` (sub-shape transforms would dirty every tick once
  any joint flexes). Self-collision filtered via `ObjectLayer` /
  `GroupFilter` keyed on vessel ID.
- **Joints internal to a vessel** — Phase 2: Jolt `SixDOFConstraint` with
  stiffness/damping matching stock's `ConfigurableJoint`. Phase 4: ABA
  computes joint forces, writes per-body poses; Jolt only sees rigid
  per-tick pose updates with no inter-body constraints. The "joint type
  collapse" — `Fixed` / `Revolute` / `Prismatic` / `Free6` all become one
  `Compliant6DOF` primitive parameterized by per-axis K/C/motor — happens
  at Phase 4.
- **Inter-vessel coupling** (docking, KAS strut between vessels) — Phase 2:
  Jolt constraint between bodies. Phase 4: ABA tree edge between bodies.
  Topology events become pure ABA-tree mutations; Jolt body counts don't
  change on docking/undocking.

Owner split (vs. current `CLAUDE.md`):

| Concern | Owner |
|---|---|
| Part poses (source of truth) | **Jolt** (ABA writes to Jolt in Phase 4) |
| Inter-part forces / constraints | Jolt (Phase 2) → ABA (Phase 4) |
| Numerical integration | **Jolt** |
| External wrench aggregation | C# accumulator → Jolt `AddForce` per tick |
| Collider geometry + broadphase | **Jolt** (mirror of Unity collider data) |
| Static-world contact detection | **Jolt** |
| Inter-vessel contact detection | **Jolt** |
| Terrain geometry | Unity PQS → mirrored to Jolt static `MeshShape`s |
| Unity rigidbodies | Kinematic proxies, pose-driven from Jolt |

## Boundary design — minimum P/Invoke surface

Goal: **one P/Invoke per `FixedUpdate` direction.** No per-event, per-body,
per-force boundary crossings.

C# side, per `FixedUpdate`:

1. **Accumulate inputs** in a pinned, blittable input buffer:
   - Per-body force/torque deltas (sum of all `AddForce*` / `AddTorque`
     calls intercepted via Harmony since last tick)
   - Per-body kinematic velocity targets (Phase 4: ABA's predicted velocity)
   - Mass/inertia updates (one record per body whose mass changed)
   - Topology mutations (body create/destroy, constraint create/destroy,
     shape update) — queued by Harmony patches on `Part.Couple`,
     `Part.decouple`, `PartJoint.DestroyJoint`, `ModuleDockingNode` events
   - Step `dt`, scene-level config (gravity vector at vessel COM, etc.)
2. **Single call**: `longeron_step(input_ptr, input_len, output_ptr, output_cap, &output_len)`.
3. **Read outputs** from pinned output buffer:
   - Per-body pose (position double3 if `JPH_DOUBLE_PRECISION`,
     orientation float4)
   - Per-body linear/angular velocity
   - Per-pair contact records (body A, body B, point, normal, depth,
     impulse) — fed back to ABA as external wrenches in Phase 4
4. **Write Unity transforms** from output. Kinematic Unity rb gets
   `rb.position` / `rb.rotation` / `rb.velocity` set so stock and modded
   code see consistent state.

Buffer schemas are typed records with version prefix; native bridge
parses streams of records rather than fixed-layout arrays. Allows additive
schema evolution without breaking either side.

Marshalling discipline:

- Blittable structs only.
- Buffers pinned via `GCHandle.Alloc(GCHandleType.Pinned)` once at session
  start, reused every frame. No per-tick allocations.
- All native API uses `unsafe` C# with `byte*` and explicit offsets.
- No `string` marshalling; all identifiers are integer body IDs.
- `[SuppressUnmanagedCodeSecurity]` on the P/Invoke decl to skip the
  full security stack walk.

## Native bridge — language and shape

**C++** for the bridge (Jolt is C++; one FFI boundary; no Rust↔C++ binding
generator + Rust↔C ABI to manage).

`JoltPhysicsSharp` is **not viable** — targets net9.0/net10.0 only, no
net48 or netstandard2.0 fallback, will not load under KSP's bundled Mono.
Confirmed via NuGet/repository inspection (2026-04-27).

C ABI surface:

```c
// session lifetime
LongeronWorld* longeron_world_create(const LongeronConfig* cfg);
void longeron_world_destroy(LongeronWorld* w);

// per-tick
void longeron_step(LongeronWorld* w,
                   const uint8_t* input, size_t input_len,
                   uint8_t* output, size_t output_cap, size_t* output_len,
                   float dt);

// debug
const char* longeron_version(void);
void longeron_set_debug_renderer(LongeronWorld* w, ...);
```

Implementation files:

```
native/
  CMakeLists.txt
  third_party/
    JoltPhysics/                 # git submodule, pinned to a release tag
  src/
    bridge.h                     # C ABI declarations
    bridge.cpp                   # C ABI → world dispatch
    world.cpp / world.h          # LongeronWorld: Jolt PhysicsSystem,
                                 #   body registry, contact listener,
                                 #   per-vessel ObjectLayer assignments
    input_buffer.cpp/.h          # decoder for the C# input stream
    output_buffer.cpp/.h         # encoder for the output stream
    contact_listener.cpp/.h      # JPH::ContactListener impl, accumulates
                                 #   contact records into the output stream
```

Build: CMake produces a single shared library
(`longeron_native.{dll,dylib,so}`) plus a copy of the Jolt static lib
embedded. Jolt built with `JPH_DOUBLE_PRECISION=ON` to eliminate
Krakensbane synchronization concerns on the physics side. Krakensbane
keeps shifting the Unity rendering origin — kinematic Unity rb pose
writes apply the offset on the C# side after reading from Jolt.

Cross-platform shipping (Phase 0):

- Win64 — `longeron_native.dll`
- macOS — `longeron_native.dylib`, universal binary (`x86_64;arm64`)
- Linux x64 — `longeron_native.so`

Local dev: build for the host platform only via CMake. Release: a
build matrix (GitHub Actions or just three local builds) producing all
three artifacts to bundle into `release/Longeron.zip`.

## Phasing

### Phase 0 — native foundation
**Goal:** Jolt builds, C++ bridge loads from C#, single round-trip
P/Invoke works.

- Add `native/` directory, CMakeLists, Jolt submodule pinned to a release
  tag, `JPH_DOUBLE_PRECISION=ON`.
- Implement `longeron_version()` and a no-op `longeron_step()` that just
  echoes input length.
- New C# project: `mod/Longeron.Native/` — P/Invoke bindings,
  pinned-buffer helpers, schema serialization.
- `justfile`: add `native-build`, `native-clean`, integrate into `dist`
  (copy native lib into `GameData/Longeron/Plugins/`).
- Verify: `Longeron.Native.dll` calls `longeron_version()` from KSP and
  prints to log. Round-trip a 1KB buffer with no real physics; confirm
  no marshalling allocations under `Profiler.GetMonoUsedSizeLong()`.

### Phase 1 — first light
**Goal:** one part, kinematic Unity rb, sitting on a static plane,
gravity applied per tick, contacts firing.

- Single-body world: one Jolt body for one part, plane Jolt body for the
  pad (no real terrain yet).
- Disable Jolt's gravity (`SetGravity(Vec3::sZero())`); apply gravity per
  body per tick from C# (`vessel.precalc.integrationAccel * mass`).
- Wire up `JPH::ContactListener::OnContactPersisted` → output buffer.
- **Verify the load-bearing assumption:** kinematic-vs-static and
  kinematic-vs-kinematic contact callbacks fire reliably under Jolt.
  This is the single biggest "if it doesn't work, we redesign" risk;
  catching it on Day 1 of Phase 1 is the entire point.
- Wire up Jolt's debug renderer (line/triangle output → Unity LineRenderer
  or `Debug.DrawLine`).
- Establish per-tick performance baseline: one body, one contact, full
  round-trip should be sub-100µs. If not, the bridge has a bug.

### Phase 2 — multi-part with Jolt joints
**Goal:** rocket flies. Multi-part vessel, stock force-producers work,
mass updates apply, vessel is flyable to orbit.

- One kinematic Jolt body per part. Vessel-scoped `ObjectLayer`
  prevents same-vessel self-collision.
- `JPH::SixDOFConstraint` between adjacent parts:
  - Stack/surface attach: stiff K, stiff C, all axes
  - BG hinges/pistons: stiff K on 5 axes, motor on the free axis
  - Pre-lock docking: free all 6 with light damping
  - Inter-vessel docking/KAS: Jolt constraint between separate vessel
    bodies (cross-`ObjectLayer` constraint allowed)
- Force redirect: Harmony prefix on `Rigidbody.AddForce*`,
  `Rigidbody.AddTorque*`, `Rigidbody.AddRelativeForce*`,
  `Rigidbody.AddForceAtPosition` (and `Rigidbody.AddExplosionForce` if
  used). Each call accumulates into a per-body force/torque scratch
  buffer; original method is replaced (Unity rb is kinematic, original
  is a no-op anyway). FlightIntegrator's gravity goes through the same
  redirect — no special case.
- Mass updates: hook the post-mass-change event (e.g., per `ModuleEngines`
  fuel burn). Per-body mass/inertia delta record in the input buffer.
- Krakensbane: kinematic rb pose write subtracts the current Krakensbane
  offset before assigning to `rb.position`. Native Jolt world stays in
  absolute double-precision space.

### Phase 2.5 — scene lifecycle
**Goal:** flying multiple missions, switching vessels, save/load all
work without state drift or memory leaks.

- `GameEvents.onVesselGoOnRails` / `GoOffRails` — destroy/create Jolt
  bodies for the affected vessel.
- `GameEvents.onVesselSwitching` / `onVesselWasModified` — verify
  state consistency; this is where most "world out of sync" bugs will
  live.
- Scene transitions (`onLevelWasLoaded`) — tear down `LongeronWorld`,
  rebuild on flight scene entry.
- Save/load — Jolt state is *derived* (not persisted); rebuild from KSP
  state on `onVesselGoOffRails`.

### Phase 3 — terrain
**Goal:** vessels rest on / drive on real terrain, not a plane.

Sub-decompose:

- **3a:** Static environment (KSC buildings, runway, launchpad mesh,
  hangar floor, water plane). One-shot mirror at scene load — enumerate
  Unity colliders, create Jolt static bodies. Tear down on scene exit.
- **3b:** PQS quad collider streaming. Hook PQS's collider quad
  spawn/despawn — verify the exact event names via the `ksp-reference`
  skill before committing. Each quad → one Jolt static `MeshShape` body,
  added/removed as PQS streams. This is the mechanically biggest part of
  Phase 3.
- **3c:** LOD swaps. PQS swaps colliders at LOD boundaries; treat as
  remove-old + add-new in Jolt with no special transitional state.
- **3d:** Asteroids/comets (procedural mesh per body) and water surface
  collider (planar). Asteroid collider streams in via stock; mirror the
  same way as PQS quads but per-body, not per-quad.

### Phase 3.5 — wheels
**Goal:** wheels work. Specifically: rovers drive on terrain without
falling over, landing gear absorbs touchdown impulses.

Wheels currently use Unity's `WheelCollider` (PhysX raycast suspension).
Two paths:

- Use Jolt's `VehicleConstraint` — built for cars, may not match KSP's
  wheel module API (`ModuleWheelBase` etc.) cleanly.
- Implement custom raycast suspension on Jolt bodies — more code but
  controllable and matches the stock `ModuleWheelBase` API surface.

Recommend the latter. Stock KSP wheels are a known pain point; rewriting
them on Jolt is an opportunity, not a cost.

### Phase 4 — Featherstone replaces Jolt's joints
**Goal:** wobble-free target hit. First public release.

- Joints internal to a vessel are removed from Jolt.
- ABA runs each tick over the vessel's spanning tree:
  - All joints become one `Compliant6DOF` primitive with per-axis K/C
  - Cycle-closing graph edges (docking rings, KAS struts forming cycles)
    become additional compliant joint forces, *not* KKT constraints
  - No Lagrange multipliers, no constraint stabilization
  - Stiff joints → implicit ABA integration (Featherstone Ch. 9
    spring-damper joint formulation) for unconditional stability
- ABA writes per-part kinematic velocities to Jolt each tick (Jolt
  integrates kinematic motion, applies pose to broadphase).
- Jolt's contact listener still feeds external contact wrenches into
  ABA's per-body fExt.
- Existing `mod/Longeron.Physics/` (ABA, RNEA, spatial vector library)
  is the implementation foundation here; preserved through Phases 0-3.

### Phase 5+ — secondary work
- BG robotics — servo/piston driver code, motor force/torque parameters
  fed to ABA per the unified primitive.
- KAS/KIS — patches to mirror their `ConfigurableJoint` creation into
  Longeron joint records.
- Determinism + multi-vessel parallel simulation.
- Profiling pass — likely the bridge call frequency, not Jolt itself.

## Repo layout changes

Additions:

```
native/                                # NEW
  CMakeLists.txt
  third_party/
    JoltPhysics/                       # git submodule
  src/
    bridge.h, bridge.cpp
    world.h, world.cpp
    input_buffer.{h,cpp}
    output_buffer.{h,cpp}
    contact_listener.{h,cpp}

mod/Longeron.Native/                   # NEW
  Longeron.Native.csproj               # net48
  src/
    NativeBridge.cs                    # extern "C" P/Invoke
    InputBuffer.cs                     # writer
    OutputBuffer.cs                    # reader
    BodyHandle.cs                      # opaque integer body ID
    World.cs                           # session-level lifetime

justfile                               # MODIFY: + native-build / dist integration
.gitmodules                            # NEW
```

Existing code:

- `mod/Longeron.Physics/` — **kept**. The ABA / RNEA / spatial library
  is the Phase 4 target. Phases 0-3 don't reference it.
- `mod/Longeron/` — gradually migrates: `LongeronSceneDriver`'s wrench
  computation paths get reworked to feed `Longeron.Native` instead of
  `Longeron.Physics` directly. Force-redirect Harmony patches added.
  `LongeronVesselModule` lifecycle hooks added for body create/destroy.
  Existing `Patches/PartCollisionRelay.cs` becomes obsolete (Jolt
  reports contacts) but is kept until Phase 1 verifies Jolt callbacks
  fire as expected.

## Documentation updates (CLAUDE.md, README.md)

Both need rewrites for self-consistency. Plan: do them as part of Phase 0
work, before code lands, so the docs match the design from day one.

### `README.md` changes

- "Approach" section — rewrite. The current text sells "PhysX's existing
  collider geometry and broadphase" as an explicit ownership boundary;
  that's no longer the design. New text: Jolt owns broadphase, contacts,
  and integration; Featherstone is the joint-force model layered on top.
- The four-bullet joint list (`Compliant 6-DOF`, `Revolute / Prismatic`,
  `Free 6-DOF`, `Fixed`) — collapse to a single `Compliant 6-DOF`
  primitive with per-axis stiffness/damping, with a note that revolute /
  prismatic / free / fixed are parameter choices, not separate types.
- Add a paragraph on Jolt: why a separate physics engine instead of
  PhysX, and what the boundary looks like.

### `CLAUDE.md` changes

- "Architecture / Ownership split" table — rewrite to match the new
  ownership table above.
- "Solver (Longeron.Physics)" section — reframe as "the Phase 4 target,"
  not "the current design." Add a new "Native bridge (Longeron.Native /
  native/)" section describing the C++ bridge, buffer protocol, P/Invoke
  surface.
- "KSPBurst / HPC# integration" section — keep but mark as "applies to
  `Longeron.Physics` (Phase 4)." The native bridge does not use Burst.
- "PartJoint strategy" section — rewrite. The current "neuter / transpile
  / delete" ladder doesn't apply because Jolt replaces PhysX entirely.
  New strategy: kill `PartJoint` creation outright (Phase 2), since Unity
  rbs are kinematic and PhysX joints would be no-ops anyway. Stock code
  paths that read `Part.attachJoint` get a stub `PartJoint` that satisfies
  null checks; calls into the joint object are no-ops or redirected.
- "Topology lifecycle" table — keep, but add column for "Jolt action"
  alongside the existing "Longeron action."
- "Loop closures on split" subsection — note that with all-6-DOF
  compliant joints, loop closures are no longer special; non-tree edges
  are just additional compliant force terms with no KKT machinery
  needed.
- "Struts are mostly obsolete" — update to acknowledge that with the
  Jolt design, struts and KAS struts are just additional `Compliant6DOF`
  records in the vessel graph.
- "Open questions" — replace items 1, 2 (PhysX-specific) with new
  Jolt-specific questions: Jolt kinematic contact callback semantics
  (Phase 1 verification), Jolt determinism flag, Jolt mass-update
  thread-safety, etc.
- "Project layout" — add `native/` and `mod/Longeron.Native/`; move
  `mod/Longeron.Physics/` description under the Phase 4 heading.
- "Build & install" — add `just native-build`.

## Critical files

Files this plan creates or substantially modifies:

- `native/CMakeLists.txt` — NEW
- `native/src/bridge.{h,cpp}` — NEW
- `native/src/world.{h,cpp}` — NEW
- `mod/Longeron.Native/Longeron.Native.csproj` — NEW
- `mod/Longeron.Native/src/NativeBridge.cs` — NEW
- `mod/Longeron.Native/src/{Input,Output}Buffer.cs` — NEW
- `mod/Longeron/src/LongeronSceneDriver.cs` — MODIFY: feed
  `Longeron.Native` instead of running ABA directly (Phases 0-3)
- `mod/Longeron/src/LongeronVesselModule.cs` — MODIFY: per-vessel Jolt
  body lifecycle on `GoOffRails`/`GoOnRails`
- `mod/Longeron/src/Patches/PartForceHooks.cs` — MODIFY: redirect to
  per-body force accumulator instead of (or in addition to) the current
  passthrough
- `mod/Longeron/src/Patches/PartCollisionRelay.cs` — DELETE eventually
  (Phase 1+)
- `mod/Longeron.Physics/` — UNTOUCHED through Phases 0-3; Phase 4 target
- `justfile` — MODIFY: native build steps
- `README.md` — MODIFY: Approach + joint list rewrites
- `CLAUDE.md` — MODIFY: substantial rewrite per above
- `.gitmodules` — NEW: pin Jolt to a release tag

## Verification

Per-phase end-to-end tests. Each phase ships only when the prior verifies.

**Phase 0:**
- `just native-build` produces `longeron_native.{dll,dylib,so}` on host
  platform with no warnings.
- KSP loads the mod; `[Longeron] native bridge version: <X>` appears in
  the log.
- Round-trip a buffer of N records; verify no mono allocations under
  `Profiler.GetMonoUsedSizeLong()` snapshot diff over 1000 ticks.

**Phase 1:**
- Single part placed on the launch pad sits stably at fixed height for 30s
  (no drift, no NaN, no crash).
- Jolt debug renderer shows the part body and pad plane at the right
  poses.
- Contact listener reports `OnContactPersisted` every tick while the part
  rests on the pad. **This is the load-bearing test for the entire
  pivot.** If it fails, redesign before continuing.

**Phase 2:**
- Saturn-V-class stack lifts off rigidly under engine thrust, gravity-
  turn flyable to orbit, decoupler fire splits the vessel cleanly.
- FAR / RealFuels / KER / mod-of-choice continues to function (force
  redirect path correctness).
- Mass-burn delta is reflected in vessel acceleration over the burn.
- Krakensbane handoff at rotation-speed threshold doesn't cause
  position discontinuity in either Jolt or Unity space.

**Phase 2.5:**
- Switch between two flying vessels, return to first vessel, position +
  velocity preserved.
- Save in flight, quit to main menu, reload save, vessel state restored.
- Quicksave/quickload during flight (`F5`/`F9`) stable.

**Phase 3:**
- Land on Mun in a manner that uses real terrain colliders, not a flat
  plane.
- Drive a rover from KSC to a nearby biome boundary, terrain collider
  streaming through PQS LOD doesn't drop the rover through the ground or
  cause stutter.

**Phase 3.5:**
- Rover handles cleanly on rough terrain at moderate speeds.
- Landing legs absorb a 5 m/s touchdown without breaking.

**Phase 4:**
- Long stack (30+ parts, mass ratio 50:1) under max-Q burn shows visible
  but bounded flex (~1° max) and does not oscillate. **The wobble
  target.**
- Re-run all prior phases' verification — no regressions vs Jolt-native-
  joints behavior except where the joint-collapse changes things
  intentionally.
- First public release tag.

## Notes / open questions to resolve before starting

- **Jolt kinematic contact callback semantics** — verify in Phase 1 that
  `OnContactPersisted` and `OnContactAdded` fire for kinematic-vs-static
  and kinematic-vs-kinematic body pairs. PhysX silenced these; Jolt
  should not but confirm before committing.
- **Jolt body type for parts** — `EMotionType::Kinematic` driven by
  `SetPosition` per tick (simplest), vs `EMotionType::Kinematic` driven
  by `SetLinearVelocity` (potentially supports CCD), vs
  `EMotionType::Dynamic` with all forces overridden (uglier but supports
  CCD between sub-shapes). Default to position-driven kinematic; revisit
  in Phase 3.5 for CCD-needing parts.
- **Jolt version pin** — pin to a tagged release rather than `main`.
  Choose the latest tag at Phase 0 start.
- **macOS universal build** — `lipo`-combined or `CMAKE_OSX_ARCHITECTURES`
  multi-arch. Latter is cleaner; verify Jolt builds for both arches
  cleanly first.
