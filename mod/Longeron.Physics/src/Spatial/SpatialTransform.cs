// FRAME CONVENTION — Featherstone RBDA (2008), link-fixed body frame.
//
// SpatialTransform X_A→B represents a rigid change-of-frame from A to B.
// Stored compactly as (rotation, translation) where:
//   - `rotation` q is the *active* quaternion giving frame B's orientation
//     in frame A (i.e., basis_B[i] = q · basis_A[i]). math.mul(q, v) rotates
//     vectors actively.
//   - `translation` t is the position of frame B's origin, expressed in A.
//
// Transforming a vector's *components* from A to B is a passive operation:
// we apply q⁻¹ (the inverse of the active rotation) to the components. This
// is the source of the asymmetry throughout this file — the stored rotation
// is active, the formulas use the inverse.
//
// Motion transform (RBDA eq. 2.23):
//   ω_B = q⁻¹ · ω_A
//   v_B = q⁻¹ · (v_A - t × ω_A)
//
// Force transform (RBDA eq. 2.24):
//   n_B = q⁻¹ · (n_A - t × f_A)
//   f_B = q⁻¹ · f_A
//
// Composition: X_A→C = X_B→C ∘ X_A→B. If X_A→B = (q1, t1), X_B→C = (q2, t2):
//   combined rotation    = q1 · q2         (active; intrinsic joint order)
//   combined translation = t1 + q1·t2
//
// Inverse of (q, t) is (q⁻¹, -q⁻¹·t).

namespace Longeron.Physics
{
    public readonly struct SpatialTransform
    {
        public readonly quaternion rotation;   // active rotation: frame B's orientation in A
        public readonly float3 translation;    // origin of B, expressed in A

        public SpatialTransform(quaternion rotation, float3 translation)
        {
            this.rotation = rotation;
            this.translation = translation;
        }

        public static readonly SpatialTransform identity =
            new SpatialTransform(quaternion.identity, float3.zero);

        public SpatialMotion TransformMotion(SpatialMotion m)
        {
            quaternion qInv = math.inverse(rotation);
            float3 omega = math.mul(qInv, m.angular);
            float3 v     = math.mul(qInv, m.linear - math.cross(translation, m.angular));
            return new SpatialMotion(omega, v);
        }

        public SpatialForce TransformForce(SpatialForce f)
        {
            quaternion qInv = math.inverse(rotation);
            float3 n      = math.mul(qInv, f.angular - math.cross(translation, f.linear));
            float3 fprime = math.mul(qInv, f.linear);
            return new SpatialForce(n, fprime);
        }

        // Inverse of TransformMotion: given m_B, recover m_A.
        //   ω_A = q · ω_B
        //   v_A = q · v_B + t × ω_A
        public SpatialMotion InverseTransformMotion(SpatialMotion m)
        {
            float3 omega = math.mul(rotation, m.angular);
            float3 v = math.mul(rotation, m.linear) + math.cross(translation, omega);
            return new SpatialMotion(omega, v);
        }

        // Inverse of TransformForce: given f_B, recover f_A.
        //   f_A = q · f_B
        //   n_A = q · n_B + t × f_A
        public SpatialForce InverseTransformForce(SpatialForce f)
        {
            float3 fprime = math.mul(rotation, f.linear);
            float3 n = math.mul(rotation, f.angular) + math.cross(translation, fprime);
            return new SpatialForce(n, fprime);
        }

        // X_A→C = then ∘ first. `first` is X_A→B, `then` is X_B→C.
        public static SpatialTransform operator *(SpatialTransform then, SpatialTransform first)
        {
            quaternion q = math.mul(first.rotation, then.rotation);
            float3 t = first.translation + math.mul(first.rotation, then.translation);
            return new SpatialTransform(q, t);
        }

        public SpatialTransform Inverse()
        {
            quaternion qInv = math.inverse(rotation);
            float3 tInv = -math.mul(qInv, translation);
            return new SpatialTransform(qInv, tInv);
        }
    }
}
