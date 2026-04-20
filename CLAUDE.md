# Longeron

A KSP 1.12.5 mod that replaces vessel-internal joint physics with a reduced-coordinate articulated-body solver (Featherstone). Goal: eliminate the wobble that comes from KSP's chain-of-`ConfigurableJoint`s architecture. Unity still owns colliders, broadphase, terrain, and inter-vessel contacts — Longeron owns constraint solving and integration.

The name is the aerospace structural member that keeps a fuselage rigid along its length. That is literally what this mod does.

## Why this exists

KSP's wobble is architectural, not a PhysX bug:

- Each `Part` is a separate `Rigidbody` connected to its parent by a `ConfigurableJoint`.
- PhysX's iterative PGS solver converges poorly on long chains with high mass ratios — error compounds down the tree.
- Autostrut and KJR work because they short-circuit the chain, not because they fix anything fundamental.

Swapping engines (Jolt, PhysX 5) helps at the margins. Swapping the *formulation* — from maximal coordinates (N bodies + N−1 constraints) to reduced coordinates (one articulated body with DOFs only where they're real) — removes the wobble at its source. A rigid stack is, mathematically, zero DOFs. Featherstone's ABA integrates a zero-DOF tree at zero cost, because there's nothing to integrate.

## Architecture

### Ownership split

| Concern | Owner |
|---|---|
| Part transforms (source of truth) | **Longeron** (writes to Unity each FixedUpdate) |
| Inter-part constraints | **Longeron** (Featherstone ABA + loop closures) |
| External wrench aggregation (thrust, aero, gravity) | **Longeron** (inferred from rigidbody velocity delta) |
| Numerical integration | **Longeron** (semi-implicit Euler) |
| Collider geometry + broadphase | Unity |
| Static-world / inter-vessel contact detection | Unity → we consume as spatial impulses |
| Docking alignment detection | KSP (`ModuleDockingNode`) → signals topology merge |
| Decoupler fire | KSP modules → signal topology split |

No entity is simulated twice. Unity is not trying to enforce joint constraints (there aren't any) or gravity (`useGravity = false` on all managed parts); it's effectively a collision detector and a transform renderer for the parts we manage.

### Solver (Longeron.Physics)

Reduced-coordinate articulated-body dynamics via Featherstone's ABA:

- One articulated body per vessel (while loaded + unpacked).
- Tree topology from `part.parent` walks; every body knows its spanning-tree parent and its joint-to-parent.
- Joint types: `Fixed` (most stack/surface attachments), `Revolute` (BG hinges), `Prismatic` (BG pistons), `Free6` (docking ports pre-lock). Tagged-union struct + `switch`, no virtual dispatch.
- Loop closures (docked rings, struts, BG servo bridges, any cycle in the vessel graph) via Lagrange multipliers on top of the tree pass. KKT system `[M Jᵀ; J 0] [q̈; λ] = [τ − h; −J̇q̇ + stab]`. λ directly gives constraint reaction forces — `|λ| > breakForce` triggers the stock joint-break path.
- No allocations in the hot path. SoA `NativeArray<T>`-style layout.

### KSPBurst / HPC# integration

Physics project (`Longeron.Physics`) is written as a **HPC# subset** so inner loops can be AOT-compiled via [KSPBurst](https://github.com/KSPModdingLibs/KSPBurst) — ships Unity.Burst 1.7.4 + Unity.Mathematics 1.2.6 + Unity.Collections 0.8.0-preview + Unity.Jobs 0.2.9-preview.

Discipline within `Longeron.Physics`:

- Structs only in hot paths; no managed references crossing into `[BurstCompile]` code.
- Blittable types only. `Unity.Mathematics` (`float3`, `float3x3`, `float4x4`) instead of Unity's `Vector3/Quaternion` in numeric code — Burst vectorizes `Unity.Mathematics` far more aggressively.
- No exceptions in Burst-compiled code; preconditions via asserts or error codes.
- `NativeArray<T>` for body state, not managed arrays.
- No virtual dispatch, no generics over reference types.

Discipline stops at the module boundary. `Longeron` (KSP integration) is ordinary C# — it can hold references, throw, use managed collections. Burst-compiled static methods are called from managed code with `NativeArray`s passed through.

Jobs system is the big win: one `IJob` per vessel, scheduled in parallel via `JobHandle.CombineDependencies`, waited on once per `FixedUpdate`. This is what makes multi-vessel background simulation cheap later — one Burst-compiled ABA pass per vessel, all running in parallel on worker threads.

Fallback: the core solver must also build and run without Burst (plain Mono JIT). Same `IArticulatedScene` interface, slower path. Ship Longeron so that missing KSPBurst degrades gracefully instead of hard-crashing. Add KSPBurst as a CKAN dependency for the performance path.

### KSP integration (Longeron)

- `LongeronAddon` — session-level MonoBehaviour, owns the scheduler hook.
- `LongeronVesselModule : VesselModule` — per-vessel articulated body lifecycle.
- `LongeronPartModule` — per-part collision relay (collects `OnCollisionStay` contacts into a per-vessel buffer).
- Harmony patches for topology-change events + joint neutering (see below).

FixedUpdate shape, using Unity script execution order to bracket the physics step:

1. **Early hook** (before Unity physics): write solver's predicted velocity to each `Rigidbody`.
2. **Unity physics step**: stock and modded code calls `Rigidbody.AddForce` from `ModuleEngines`, aero, wheels, RCS, reaction wheels. Velocities get integrated.
3. **Late hook** (after Unity physics): read `rigidbody.velocity - predictedVelocity` → delta, scale by `mass/dt` → net external force this tick. Feed as spatial wrench. Drain accumulated contacts from the relay. Step ABA. Write new positions/rotations/velocities to rigidbodies. Unity uses these for the next broadphase pass.

The velocity-delta force inference means **we do not patch `AddForce` or any force-producing module**. Stock, FAR, RealFuels, every modded force source just works. We only see the sum in the rigidbody's velocity delta — which is exactly what we want.

## PartJoint strategy: start soft, cut deep as needed

The honest design range, cheapest to most invasive:

1. **Neuter `ConfigurableJoint`s** — keep the `PartJoint`/`ConfigurableJoint` GameObjects, set all motions `Free`, all drives zero, `breakForce = Infinity`. Stock and modded code that pokes `Part.attachJoint` still sees a live object. Physically inert. Longeron decides when to break and calls the stock `PartJoint.DestroyJoint()` so the standard event/topology-split paths fire unchanged.

2. **Transpile specific stock call sites** — when we find code paths that read `ConfigurableJoint.currentForce`, inspect drives/limits, or otherwise assume the joint is physically simulating, surgically rewrite the IL to read from our solver state instead.

3. **Delete joints entirely** — skip creation in `PartJoint.CreateJoint`, and patch out all call sites that assume the joint exists.

**Default posture: be aggressive.** KSP 1.12.5 is a frozen target. The usual objections to transpilers and deep cuts — upstream drift, maintenance burden — don't apply. Squad isn't shipping another patch. Bytes don't move. Reflection against private fields is permanently safe. Transpilers stay correct forever. A near-copy replacement of a stock method is a perfectly reasonable tool.

Start at level 1 because it's fast to get running. Promote individual call sites to level 2 or 3 as profiling / correctness requires. Don't contort the design around "what if stock needs to read this" — stock is a known quantity and every path is in `ksp/Assembly-CSharp/` to be read directly.

## Topology lifecycle

Events that change the tree, and what we hook:

| Event | KSP source | Longeron action |
|---|---|---|
| Vessel unpacks | `Vessel.GoOffRails` postfix | Build articulated body from `part.parent` walk |
| Vessel packs | `Vessel.GoOnRails` postfix | Release articulated body |
| Part coupled | `Part.Couple` postfix | Merge subtree (or whole vessels, if inter-vessel couple) |
| Part decoupled | `Part.decouple` postfix | Split tree |
| Joint destroyed | `PartJoint.DestroyJoint` postfix | Potentially split tree; may promote a loop closure into the spanning tree of one half |
| Docking engaged | `ModuleDockingNode` state transitions | Merge; may introduce loop closure if closing a cycle |
| Undocking | `ModuleDockingNode.Undock` | Split; remove loop closure if it was the only thing closing the cycle |

Topology rebuilds are **deferred to the next FixedUpdate start**, not executed synchronously from the event handler. This gives stock event handlers a chance to run against consistent state before we rebuild around them.

### Loop closures on split

The subtle bit: when the tree splits, a loop closure living on the cut side might need to be promoted into the spanning tree of one half, or remain as a loop closure in a different half, or dangle across the split (and simply break). Three cases, all correct in different scenarios. This is bookkeeping, not math — and it is where correctness bugs will hide. It deserves its own test suite with scripted topology mutations.

## Struts are mostly obsolete

In a reduced-coordinate world, rigid stack/surface attachment is a zero-DOF joint by construction. The wobble mitigation that autostrut and explicit struts exist to provide is the default behavior. Where struts retain meaning:

- **Closed topologies** — space stations with docking-port rings. The vessel graph is a cycle; this isn't optional, and the loop-closure machinery handles it.
- **Stiffening intentionally compliant joints** — BG robotic servos, soft landing gear. Here a strut is locking a real DOF, not fighting phantom flex.

Auto-strut as a gameplay mechanic quietly evaporates. Explicit struts on rigid stacks become cosmetic. Neither is a regression — it's the thing players were hacking around never existing in the first place.

**Do not invent a "strut-across-decoupler survives fire" mechanic.** Stock behavior: struts always break on decoupler fire. That is not a backward-compatibility concern; it's a fact of the game.

## Open questions — verify before building

Concrete items to confirm via the `ksp-reference` skill and/or a live bridge probe before real implementation:

1. **Script execution order vs. Unity's physics step.** Where exactly the late hook sits relative to `OnCollisionStay` callbacks and `Rigidbody.velocity` being finalized.
2. **`ConfigurableJoint.currentForce` readers in stock.** If anything reads this, the neuter-only approach needs to spoof it.
3. **`WheelCollider` behavior.** Unity's wheel subsystem has its own suspension simulation. Likely treat the wheel root as a compliant 6-DOF joint and let Unity own the rest.
4. **Breaking Ground robotic parts.** Servos and pistons as explicit revolute/prismatic joints; robotic controllers drive joint positions via stock PAM code — need to understand where they call into the joint before we replace it.
5. **KAS/KIS and other non-stock joint modules.** These add extra `ConfigurableJoint`s outside `Part.attachJoint`. Enumerate them in the neuter pass.
6. **Which `Part` fields stock code reads during/after joint break.** If anything downstream of `DestroyJoint` expects specific rigidbody/joint state, we have to preserve it.

Per project memory (`feedback_verify_ksp_mechanics.md`): do not assert stock KSP behavior from pattern-match; check decompiled source before committing a design to it.

## Project layout

```
longeron/
  stubs/                        # KSP 1.12.5 reference assemblies (compile-time only)
    Assembly-CSharp.dll
    Assembly-CSharp-firstpass.dll
    UnityEngine.*.dll
    mscorlib.dll, System*.dll
  mod/
    Longeron.sln
    Longeron.Physics/           # HPC# articulated-body solver
      Longeron.Physics.csproj   # net48, Burst-compatible subset
      src/
        Spatial/                # 6D motion/force algebra
        ArticulatedBody.cs      # SoA flat arrays: parent[], jointType[], Xtree[], I[]
        Solver/
          ABA.cs                # Featherstone forward dynamics
          CRBA.cs               # mass matrix for loop closures
          LoopClosure.cs        # KKT with Lagrange multipliers
          Integrator.cs
        Contacts/
    Longeron/                   # KSP integration
      Longeron.csproj           # net48 + Lib.Harmony + Assembly-CSharp refs
      src/
        LongeronAddon.cs        # session entry point
        LongeronVesselModule.cs # per-vessel articulated body lifecycle
        LongeronPartModule.cs   # collision relay
        Patches/                # Harmony patches
  justfile                      # just build / just install $KSP_PATH
  LICENSE
  CLAUDE.md                     # this file
```

## Build & install

Requires `just`, `dotnet`. No Rust, no Node, no native plugin.

```bash
just mod-build                  # build both projects
just dist                       # assemble release/Longeron.zip
just install ~/KSP_osx          # build + deploy into a KSP directory
```

## Relationship to Hypergolic Systems

Longeron is a sibling mod to HGS, same author, same KSP target, **independent**. HGS's persistent-world / background-simulation goals benefit from Longeron's parallel Jobs-based articulated dynamics (tick thousands of vessels at microseconds each), but there's no hard dependency. Either mod should load standalone.

## References

- Featherstone, *Rigid Body Dynamics Algorithms* (2008). The canonical ABA + loop-closure treatment.
- [KSPBurst](https://github.com/KSPModdingLibs/KSPBurst) — Unity Burst toolchain packaged for KSP.
- Unity [Mathematics](https://docs.unity3d.com/Packages/com.unity.mathematics@latest) / [Collections](https://docs.unity3d.com/Packages/com.unity.collections@latest) / [Jobs](https://docs.unity3d.com/Packages/com.unity.jobs@latest) — the HPC# surface we target.
- `ksp/Assembly-CSharp/` in sibling HGS repo — decompiled stock source, authoritative for behavior questions.
