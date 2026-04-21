// Rough per-part inertia estimate.
//
// Stock KSP parts expose `Part.mass` + `Part.GetResourceMass()` but not a
// principal inertia tensor. For the integration spike we estimate
// I_c = (2/5)·m·R²·I_3 where R is the part's bounding radius — an isotropic
// sphere of equivalent mass + bounding size.
//
// This is physically wrong for cuboid fuel tanks and elongated engines, but
// good enough to make the solver non-singular and make translational motion
// track F/M_total. Rotational dynamics will look off under torque; that's a
// known risk flagged in the plan. A real inertia tensor from part geometry
// is future work.

using UnityEngine;

namespace Longeron.Integration
{
    internal static class InertiaEstimator
    {
        public static Physics.SpatialInertia Estimate(Part part)
        {
            float mass = part.mass + part.GetResourceMass();
            if (mass < 1e-6f) mass = 0.01f;

            float radius = EstimateBoundingRadius(part);
            float i = 0.4f * mass * radius * radius;   // solid sphere
            var I_c = new Physics.float3x3(
                new Physics.float3(i, 0, 0),
                new Physics.float3(0, i, 0),
                new Physics.float3(0, 0, i));
            return new Physics.SpatialInertia(mass, Physics.float3.zero, I_c);
        }

        static float EstimateBoundingRadius(Part part)
        {
            // Prefer the collider bounds; fall back to a coarse guess.
            var col = part.collider;
            if (col != null)
            {
                Bounds b = col.bounds;
                return b.extents.magnitude;
            }
            return 0.5f;
        }
    }
}
