// CbFrame — boundary transform between Unity world and the active
// CelestialBody's body-fixed (rotating) frame.
//
// Phase 3b's reference-frame redesign: Jolt operates in CB-fixed
// coords, not Unity world. Terrain is stationary by construction;
// landed vessels stay still; FloatingOrigin / Krakensbane / planet
// rotation no longer require per-event plumbing into Jolt — they
// fall out of this transform.
//
// Implementation lives entirely on stock's existing double-precision
// frame helpers:
//   - CelestialBody.position is Vector3d
//   - CelestialBody.rotation is QuaternionD (CelestialBody.cs:203)
//   - CelestialBody.BodyFrame is Planetarium.CelestialFrame, with
//     WorldToLocal / LocalToWorld accepting Vector3d
//     (CelestialBody.cs:782, 798)
//   - CelestialBody.getRFrmVel(p) = ω × (p − body.position)
//     (CelestialBody.cs:736)
//
// We compose those — no custom math, no precision loss until the
// final cast to float at the rb.position / rb.rotation /
// rb.velocity boundary.
//
// .xzy swizzle convention: KSP stores math-frame quantities right-
// handed; Unity is left-handed. BodyFrame.WorldToLocal wants the
// math-frame ordering, so we swizzle in and back out. Same pattern
// stock uses in GetWorldSurfacePosition (CelestialBody.cs:808) and
// GetLatLonAlt (CelestialBody.cs:911).

using UnityEngine;

namespace Longeron.Integration
{
    public readonly struct CbFrame
    {
        public readonly CelestialBody Body;

        public CbFrame(CelestialBody body)
        {
            Body = body;
        }

        // Snapshot the active vessel's mainBody as the current frame.
        // Cheap: just wraps a reference. Recomputable per call.
        public static CbFrame Current()
        {
            var v = FlightGlobals.ActiveVessel;
            var body = v?.mainBody ?? FlightGlobals.GetHomeBody();
            return new CbFrame(body);
        }

        public bool IsValid => Body != null;

        // ---- Position (translates + rotates) ----

        public Vector3d WorldToCb(Vector3d p_world)
            => Body.BodyFrame.WorldToLocal((p_world - Body.position).xzy).xzy;

        public Vector3d CbToWorld(Vector3d p_cb)
            => Body.BodyFrame.LocalToWorld(p_cb.xzy).xzy + Body.position;

        // ---- Direction (rotates only — for force vectors, axes,
        //      direction-typed velocity differences) ----

        public Vector3d WorldDirToCb(Vector3d v)
            => Body.BodyFrame.WorldToLocal(v.xzy).xzy;

        public Vector3d CbDirToWorld(Vector3d v)
            => Body.BodyFrame.LocalToWorld(v.xzy).xzy;

        // ---- Rotation (CB ↔ Unity world) ----

        public QuaternionD WorldToCb(QuaternionD r_world)
            => QuaternionD.Inverse(Body.rotation) * r_world;

        public QuaternionD CbToWorld(QuaternionD r_cb)
            => Body.rotation * r_cb;

        // ---- Linear velocity at a point ----
        //
        // CB-rotating frame velocity at point p is the surface velocity:
        // remove the planet's surface-rotation contribution (ω×r), then
        // re-express in CB axes.
        //
        // Krakensbane is patched out (KrakensbaneDisable.cs), so
        // FrameVel ≡ 0 and rb.velocity is the inertial-frame Unity-world
        // velocity directly. No FrameVel correction here.

        public Vector3d WorldVelToCb(Vector3d v_world, Vector3d p_world)
        {
            Vector3d v_surface = v_world - Body.getRFrmVel(p_world);
            return WorldDirToCb(v_surface);
        }

        public Vector3d CbVelToWorld(Vector3d v_cb, Vector3d p_world)
        {
            Vector3d v_surface = CbDirToWorld(v_cb);
            return v_surface + Body.getRFrmVel(p_world);
        }

        // ---- Angular velocity ----
        //
        // The part's angular velocity in Unity world axes vs the CB
        // rotating frame differs by the planet's spin. Subtract
        // body.angularVelocity, then re-express in CB axes.

        public Vector3d WorldAngVelToCb(Vector3d ω_world)
            => WorldDirToCb(ω_world - Body.angularVelocity);

        public Vector3d CbAngVelToWorld(Vector3d ω_cb)
            => CbDirToWorld(ω_cb) + Body.angularVelocity;
    }
}
