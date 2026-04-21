// Joint primitive — the small set of kinematic connections we support.
//
// FRAME CONVENTION — Featherstone RBDA (2008), link-fixed body frame.
//
// Factoring follows RBDA: each body has a constant `Xtree` (bolted
// parent-to-joint-predecessor transform, independent of q) and a
// q-dependent `Xj(q)` from the joint predecessor to the body frame.
// Spatial transform from parent body frame to child body frame is
// X_parent_to_child = Xj(q) * Xtree.
//
// Motion subspace S[i] is constant in the body frame:
//   Fixed:     S undefined (zero DOF; handled as a distinct code path).
//   Revolute:  S = (axis, 0)      — angular velocity along axis.
//   Prismatic: S = (0,    axis)   — linear velocity along axis.

namespace Longeron.Physics
{
    public enum JointKind : byte
    {
        Fixed    = 0,
        Revolute = 1,
        Prismatic = 2,
        // 6-DOF joint. Supported only as a root ("floating base"); the joint's
        // configuration and velocity live in ArticulatedBody.rootPose and
        // ArticulatedBody.rootVelocity rather than in q[] / qdot[]. A Floating
        // joint on a non-root body is not a supported configuration.
        Floating = 3,
    }

    public readonly struct Joint
    {
        public readonly JointKind kind;
        public readonly float3 axis;   // unit vector in body frame; ignored for Fixed/Floating

        public Joint(JointKind kind, float3 axis)
        {
            this.kind = kind;
            this.axis = axis;
        }

        public static Joint Fixed()                  => new Joint(JointKind.Fixed, float3.zero);
        public static Joint Revolute(float3 axis)    => new Joint(JointKind.Revolute,  axis);
        public static Joint Prismatic(float3 axis)   => new Joint(JointKind.Prismatic, axis);
        public static Joint Floating()               => new Joint(JointKind.Floating, float3.zero);

        public int Dof
        {
            get
            {
                switch (kind)
                {
                    case JointKind.Fixed: return 0;
                    case JointKind.Floating: return 6;
                    default: return 1;
                }
            }
        }

        // Motion subspace S_i expressed in body frame. Fixed joints have
        // no DOF and callers must not query S on them; return zero.
        public SpatialMotion MotionSubspace()
        {
            switch (kind)
            {
                case JointKind.Revolute:  return new SpatialMotion(axis, float3.zero);
                case JointKind.Prismatic: return new SpatialMotion(float3.zero, axis);
                default:                  return SpatialMotion.zero;
            }
        }

        // Joint transform X_j(q): from the joint predecessor frame to the
        // child body frame. For revolute: rotation about `axis` by q.
        // For prismatic: translation `axis * q`. For fixed: identity.
        //
        // Parameterization note: the rotation/translation placed here
        // encodes the *configuration* across the joint, in the child
        // body's frame — applied *after* the parent-to-predecessor Xtree.
        public SpatialTransform JointTransform(float q)
        {
            switch (kind)
            {
                case JointKind.Revolute:
                    return new SpatialTransform(math.axisAngle(axis, q), float3.zero);
                case JointKind.Prismatic:
                    return new SpatialTransform(quaternion.identity, axis * q);
                default:
                    return SpatialTransform.identity;
            }
        }
    }
}
