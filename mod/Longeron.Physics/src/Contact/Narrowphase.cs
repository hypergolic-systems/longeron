// Analytic capsule-vs-AABB narrowphase.
//
// For Longeron's demo scope this is the only pair type the solver needs.
// Everything else (mesh terrain, cross-vessel, wheel colliders) is out of
// scope for now and will arrive as dedicated pair types when we need them.
//
// ALGORITHM
//
// Capsule = segment (axisStart..axisEnd) swept by a sphere of `radius`. AABB
// = min/max corners in world space. We decompose the query into:
//
//   1. Each capsule endpoint: is it inside the AABB?
//        If yes — contact on the nearest face, plus hemisphere-equator samples
//        around the endpoint to fill out a support-polygon manifold.
//        If no — nothing from this endpoint (the other endpoint or external
//        segment-to-AABB distance handle it).
//
//   2. If neither endpoint is inside the AABB, fall back to external distance:
//        closest point on the segment to the AABB, closest point on the AABB
//        back to that segment point. If the pair distance is less than
//        radius, emit a single tangent-contact point.
//
// The four-point manifold on endpoint penetration is what lets a vertical
// rocket standing on its engine bell resist tipping: when the rocket tilts,
// the samples on the down-tilt side penetrate deeper than the ones on the
// up-tilt side, and the PGS solver reads that as a restoring moment. A
// single endpoint contact would have no moment arm around itself — a knife
// edge balance, exactly the "fall over" failure mode Phase A suffered from.
//
// LIMITATIONS
//
// - Side-lying contact (capsule axis parallel to a face) currently produces
//   a single external-distance contact point. For horizontal-lying-rocket
//   behavior we'd want a line-contact enrichment — follow-up.
// - Capsule edge-on-AABB-edge and capsule-past-AABB-corner geometry are
//   handled correctly only by the external-distance path; edge cases near
//   AABB corners may pick the wrong face. Good enough for flat-top pad.

namespace Longeron.Physics.Contact
{
    public static class Narrowphase
    {
        // Number of azimuthal hemisphere samples per penetrating endpoint.
        // Four yields a 4-point manifold (endpoint center + 3 perimeter
        // samples) which is the ManifoldCapacity — add this here and the
        // manifold will fill up on a single endpoint contact.
        const int HEMISPHERE_SAMPLES = 3;

        public static void CapsuleVsAabb(
            float3 segStart,
            float3 segEnd,
            float radius,
            AabbShape aabb,
            ref ContactManifold mf)
        {
            mf.Clear();

            bool startInside = aabb.Contains(segStart);
            bool endInside   = aabb.Contains(segEnd);

            if (startInside || endInside)
            {
                float3 axis = segEnd - segStart;
                float axisLen = math.length(axis);
                float3 axisDir = axisLen > 1e-6f ? axis / axisLen : new float3(0f, 1f, 0f);

                if (startInside)
                    AddEndpointContacts(segStart, axisDir, radius, aabb, ref mf);
                if (endInside && mf.count < ContactManifold.Capacity)
                    AddEndpointContacts(segEnd, -axisDir, radius, aabb, ref mf);
                return;
            }

            // Neither endpoint inside — external distance test. Find the
            // closest point on the segment to the AABB; project that point
            // back onto the AABB. If the pair gap is < radius, we have
            // exactly one contact.
            float3 segClose;
            float3 boxClose;
            ClosestSegmentToAabb(segStart, segEnd, aabb, out segClose, out boxClose);

            float3 delta = segClose - boxClose;
            float dsq = math.dot(delta, delta);
            if (dsq > radius * radius) return;

            float d = math.sqrt(dsq);
            float3 normal;
            if (d > 1e-6f) normal = delta / d;
            else           normal = new float3(0f, 1f, 0f);   // coincident; fall back to +Y

            mf.Add(boxClose, normal, radius - d);
        }

        // Endpoint is inside AABB — emit the primary-face contact plus up to
        // HEMISPHERE_SAMPLES azimuthal samples around the capsule axis to
        // spread the support polygon.
        static void AddEndpointContacts(
            float3 endpoint,
            float3 axisDirOutward,    // unit, pointing from this endpoint into the capsule
            float radius,
            AabbShape aabb,
            ref ContactManifold mf)
        {
            NearestFace(endpoint, aabb, out float3 faceNormal, out float distToFace);

            // Primary contact: endpoint projected onto the nearest face, normal
            // = outward face normal, depth = distance-to-face + radius.
            float3 primaryPoint = endpoint + faceNormal * distToFace;
            mf.Add(primaryPoint, faceNormal, distToFace + radius);
            if (mf.count >= ContactManifold.Capacity) return;

            // Hemisphere samples — points on the capsule's hemisphere equator
            // plane (perpendicular to the capsule axis, through the endpoint),
            // at radius out. Each sample contributes an independent contact
            // if it's also inside the AABB, which gives us restoring-torque
            // differential when the capsule tilts.
            BuildPerpendicularFrame(axisDirOutward, out float3 u, out float3 v);
            for (int i = 0; i < HEMISPHERE_SAMPLES && mf.count < ContactManifold.Capacity; i++)
            {
                float theta = (2f * math.PI * i) / HEMISPHERE_SAMPLES;
                float c = math.cos(theta);
                float s = math.sin(theta);
                float3 sample = endpoint + radius * (c * u + s * v);
                if (!aabb.Contains(sample)) continue;

                NearestFace(sample, aabb, out float3 sn, out float sd);
                float3 samplePoint = sample + sn * sd;
                mf.Add(samplePoint, sn, sd);
            }
        }

        // Given a point inside an AABB, find the nearest face. Returns the
        // outward-pointing unit normal for that face and the distance from
        // the point to the face plane (≥ 0).
        static void NearestFace(float3 p, AabbShape aabb, out float3 normal, out float dist)
        {
            float dxNeg = p.x - aabb.min.x;
            float dxPos = aabb.max.x - p.x;
            float dyNeg = p.y - aabb.min.y;
            float dyPos = aabb.max.y - p.y;
            float dzNeg = p.z - aabb.min.z;
            float dzPos = aabb.max.z - p.z;

            dist = dxNeg; normal = new float3(-1f, 0f, 0f);
            if (dxPos < dist) { dist = dxPos; normal = new float3( 1f, 0f, 0f); }
            if (dyNeg < dist) { dist = dyNeg; normal = new float3( 0f,-1f, 0f); }
            if (dyPos < dist) { dist = dyPos; normal = new float3( 0f, 1f, 0f); }
            if (dzNeg < dist) { dist = dzNeg; normal = new float3( 0f, 0f,-1f); }
            if (dzPos < dist) { dist = dzPos; normal = new float3( 0f, 0f, 1f); }
        }

        // Any two unit vectors perpendicular to `n` and to each other. Used
        // to build a hemisphere-equator frame at the capsule endpoint.
        static void BuildPerpendicularFrame(float3 n, out float3 u, out float3 v)
        {
            // Pick the reference axis least aligned with n to avoid near-zero cross.
            float3 refAxis = math.abs(n.x) < 0.9f
                ? new float3(1f, 0f, 0f)
                : new float3(0f, 1f, 0f);
            u = math.normalize(math.cross(n, refAxis));
            v = math.cross(n, u);
        }

        // Closest-point pair between a segment and an AABB. Uses the
        // iterative Arvo / Larsen approach: alternately project the segment
        // point onto the AABB and the AABB point onto the segment until
        // convergence. Two iterations are enough for our uses.
        static void ClosestSegmentToAabb(
            float3 segA, float3 segB,
            AabbShape aabb,
            out float3 segPoint,
            out float3 boxPoint)
        {
            // Initial guess: midpoint of segment.
            segPoint = (segA + segB) * 0.5f;

            for (int iter = 0; iter < 3; iter++)
            {
                boxPoint = aabb.ClosestPoint(segPoint);
                segPoint = ClosestPointOnSegment(segA, segB, boxPoint);
            }
            boxPoint = aabb.ClosestPoint(segPoint);
        }

        static float3 ClosestPointOnSegment(float3 a, float3 b, float3 p)
        {
            float3 ab = b - a;
            float len2 = math.dot(ab, ab);
            if (len2 < 1e-12f) return a;
            float t = math.dot(p - a, ab) / len2;
            if (t < 0f) t = 0f;
            else if (t > 1f) t = 1f;
            return a + t * ab;
        }
    }
}
