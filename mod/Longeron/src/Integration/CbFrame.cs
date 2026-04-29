// CbFrame — boundary transform between Unity world and the active
// CelestialBody's body-fixed (rotating) frame.
//
// Phase 3b's reference-frame redesign: Jolt operates in CB-fixed
// coords, not Unity world. Terrain is stationary by construction;
// landed vessels stay still.
//
// **Crucial subtlety: KSP's `body.inverseRotation` flag.** When the
// active mainBody is in "rotating frame" mode (typically below ~70km
// on Kerbin), Unity world IS the body-fixed rotating frame:
//   - bodyTransform.rotation stays identity; planet doesn't rotate
//     in Unity world
//   - PQS terrain transforms don't drift in Unity world
//   - rb.velocity is already in the surface frame
//   - body.position is anchored (Sun & other planets re-positioned
//     each tick to maintain dominantBody fixity, via reverse=true)
// In this mode CbFrame = Unity world; the helpers degenerate to a
// pure translation by body.position. No rotation correction, no
// getRFrmVel subtraction — those would over-correct.
//
// Above the threshold (inverseRotation=false), Unity world IS
// inertial. bodyTransform.rotation rotates with the planet, vessels
// move at orbital speeds in inertial coords, terrain quads rotate
// in Unity world. Here CbFrame applies the full rotation transform
// via stock's BodyFrame.WorldToLocal/LocalToWorld and subtracts
// getRFrmVel from velocities to recover surface-frame.
//
// Stock handles the transition by re-stamping rb.velocity (see
// OrbitPhysicsManager.setRotatingFrame, ksp-reference at
// OrbitPhysicsManager.cs:337 — adds/subtracts rFrmVel per part).
// Our world rebuilds on `GameEvents.onRotatingFrameTransition` to
// re-bake all body poses in the new frame.
//
// Implementation uses stock's existing double-precision helpers:
//   - CelestialBody.position (Vector3d)
//   - CelestialBody.rotation (QuaternionD; CelestialBody.cs:203)
//   - CelestialBody.BodyFrame.WorldToLocal/LocalToWorld
//   - CelestialBody.getRFrmVel(p)
// Compose, no custom math, double precision throughout. Cast to
// float only at the rb.position / rb.rotation / rb.velocity write.
//
// .xzy swizzle convention: same pattern stock uses in
// GetWorldSurfacePosition (CelestialBody.cs:808) and GetLatLonAlt.

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

        public static CbFrame Current()
        {
            var v = FlightGlobals.ActiveVessel;
            var body = v?.mainBody ?? FlightGlobals.GetHomeBody();
            return new CbFrame(body);
        }

        public bool IsValid => Body != null;

        bool Rotating => Body.inverseRotation;

        // ---- Direction (translation-free) ----

        public Vector3d WorldDirToCb(Vector3d v)
            => Rotating ? v : Body.BodyFrame.WorldToLocal(v.xzy).xzy;

        public Vector3d CbDirToWorld(Vector3d v)
            => Rotating ? v : Body.BodyFrame.LocalToWorld(v.xzy).xzy;

        // ---- Position ----

        public Vector3d WorldToCb(Vector3d p_world)
            => WorldDirToCb(p_world - Body.position);

        public Vector3d CbToWorld(Vector3d p_cb)
            => Body.position + CbDirToWorld(p_cb);

        // ---- Rotation (CB ↔ Unity world) ----

        public QuaternionD WorldToCb(QuaternionD r_world)
            => Rotating ? r_world : QuaternionD.Inverse(Body.rotation) * r_world;

        public QuaternionD CbToWorld(QuaternionD r_cb)
            => Rotating ? r_cb : Body.rotation * r_cb;

        // ---- Linear velocity at point p ----
        //
        // Krakensbane is patched out, so rb.velocity is the actual
        // Unity-world velocity (no FrameVel correction needed).
        //
        // Rotating mode: Unity world == surface frame, so rb.velocity
        // IS already the surface velocity. Pass through.
        //
        // Inertial mode: rb.velocity is in inertial Unity coords; the
        // surface velocity is rb.velocity − ω×r at the part. Then
        // re-express in CB-fixed axes.

        public Vector3d WorldVelToCb(Vector3d v_world, Vector3d p_world)
        {
            Vector3d v_surface = Rotating
                ? v_world
                : v_world - Body.getRFrmVel(p_world);
            return WorldDirToCb(v_surface);
        }

        public Vector3d CbVelToWorld(Vector3d v_cb, Vector3d p_world)
        {
            Vector3d v_surface = CbDirToWorld(v_cb);
            return Rotating
                ? v_surface
                : v_surface + Body.getRFrmVel(p_world);
        }

        // ---- Angular velocity ----
        //
        // Rotating mode: rb.angularVelocity is already the relative-to-
        // surface angular velocity. No correction.
        //
        // Inertial mode: rb.angularVelocity is in inertial frame; the
        // body's spin contribution is ω_planet. Subtract to get the
        // vessel's rotation relative to the rotating ground.

        public Vector3d WorldAngVelToCb(Vector3d ω_world)
        {
            Vector3d ω_relative = Rotating
                ? ω_world
                : ω_world - Body.angularVelocity;
            return WorldDirToCb(ω_relative);
        }

        public Vector3d CbAngVelToWorld(Vector3d ω_cb)
        {
            Vector3d ω_world_rotated = CbDirToWorld(ω_cb);
            return Rotating
                ? ω_world_rotated
                : ω_world_rotated + Body.angularVelocity;
        }
    }
}
