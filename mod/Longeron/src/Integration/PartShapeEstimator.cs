// Fit a CapsuleShape to a KSP Part's collider, expressed in the part's body
// frame. Mirrors the InertiaEstimator pattern: cheap heuristic, one-shot at
// VesselScene build.
//
// ASSUMPTIONS
//
// - Capsule axis is along body +Y. Consistent with the KSP rocket-part
//   convention (pods, tanks, engines all use Y as thrust/length axis).
//   Parts with other natural axes (antennas, wings) will get a mis-aligned
//   capsule, acceptable for the pad-demo scope; revisit when those matter.
//
// - Local-space bounds come from MeshCollider.sharedMesh.bounds when
//   available (the sharedMesh is authored in part-local coords for stock
//   parts). Otherwise we fall back to Collider.bounds (world) and assume
//   the part's transform is aligned with world axes — true when the vessel
//   is upright on the pad, approximate when tilted.
//
// - Capsule endpoints are inset by `radius` from the ends of the bounding
//   region so the swept-sphere extent matches the bounding extent. For a
//   shape shorter than it is wide, the capsule degenerates to a sphere.

using Longeron.Physics;
using Longeron.Physics.Contact;
using UnityEngine;

namespace Longeron.Integration
{
    internal static class PartShapeEstimator
    {
        public static CapsuleShape Estimate(Part part)
        {
            var col = part.collider;
            if (col == null)
                return new CapsuleShape(float3.zero, float3.zero, 0.25f);

            // Local-space bounds: mesh-local if this is a MeshCollider (most
            // KSP stock parts); otherwise project the world bounds back into
            // the part's local frame — approximate for tilted orientations.
            Vector3 localCenter;
            Vector3 localExtents;
            if (col is MeshCollider mc && mc.sharedMesh != null)
            {
                var b = mc.sharedMesh.bounds;
                localCenter  = b.center;
                localExtents = b.extents;
            }
            else
            {
                var wb = col.bounds;
                localCenter  = part.transform.InverseTransformPoint(wb.center);
                Vector3 ext  = part.transform.InverseTransformVector(wb.extents);
                localExtents = new Vector3(Mathf.Abs(ext.x), Mathf.Abs(ext.y), Mathf.Abs(ext.z));
            }

            float halfHeight = localExtents.y;
            float radius = (localExtents.x + localExtents.z) * 0.5f;
            if (radius < 0.05f) radius = 0.05f;

            // Center of the capsule segment lives at the collider's local center.
            // Axis half-length = halfHeight - radius so the swept sphere brings
            // total extent back to halfHeight (avoids over-estimating size).
            float3 center = new float3(localCenter.x, localCenter.y, localCenter.z);
            float axisHalfLen = halfHeight - radius;
            if (axisHalfLen < 0f) axisHalfLen = 0f;

            float3 axis = new float3(0f, axisHalfLen, 0f);
            return new CapsuleShape(center - axis, center + axis, radius);
        }
    }
}
