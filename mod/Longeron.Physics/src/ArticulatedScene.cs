// ArticulatedScene — ownership layer.
//
// Holds the ArticulatedBody (topology + state), scratch buffers for ABA,
// and the world-frame gravity vector. Step() runs one ABA solve plus a
// hardcoded semi-implicit Euler integration; GetWorldTransform walks the
// parent chain to compose the body-to-world transform on demand.

using System;

namespace Longeron.Physics
{
    public sealed class ArticulatedScene
    {
        readonly ArticulatedBody body;
        ABA.Scratch scratch;
        public float3 gravity;   // world frame; (0, 0, 0) = no gravity

        public ArticulatedScene(int initialCapacity = 16)
        {
            body = new ArticulatedBody(initialCapacity);
            scratch = ABA.AllocateScratch(initialCapacity);
            gravity = float3.zero;
        }

        public int BodyCount => body.Count;
        public ArticulatedBody Body => body;

        public BodyId AddBody(BodyId parent, Joint j, SpatialInertia I, SpatialTransform Xtree)
        {
            BodyId id = body.AddBody(parent, j, I, Xtree);
            EnsureScratchCapacity(body.Capacity);
            return id;
        }

        public void SetExternalWrench(BodyId b, SpatialForce f) => body.SetExternalWrench(b, f);
        public void SetJointPosition(BodyId b, float value) => body.SetJointPosition(b, value);
        public void SetJointVelocity(BodyId b, float value) => body.SetJointVelocity(b, value);
        public float GetJointPosition(BodyId b) => body.GetJointPosition(b);
        public float GetJointVelocity(BodyId b) => body.GetJointVelocity(b);
        public void SetGravity(float3 g) => gravity = g;
        public void Validate() => body.Validate();

        // Single forward-dynamics step with hardcoded semi-implicit Euler.
        public void Step(float dt)
        {
            ABA.Solve(body, gravity, ref scratch);

            // 1-DOF joints: classic q-qdot advance. Fixed joints skipped.
            for (int i = 0; i < body.Count; i++)
            {
                Joint j = body.joint[i];
                if (j.kind != JointKind.Revolute && j.kind != JointKind.Prismatic) continue;
                body.qdot[i] += scratch.qddot[i] * dt;
                body.q[i]    += body.qdot[i] * dt;
            }

            // Floating root: integrate rootVelocity (body frame) via semi-implicit
            // Euler, then advance rootPose by the SE(3) first-order increment.
            if (body.Count > 0 && body.joint[0].kind == JointKind.Floating)
            {
                body.rootVelocity = body.rootVelocity + scratch.rootAccel * dt;

                float3 omega = body.rootVelocity.angular;
                float3 v = body.rootVelocity.linear;
                float omegaMag = math.length(omega);
                quaternion qInc = omegaMag > 1e-8f
                    ? math.axisAngle(omega / omegaMag, omegaMag * dt)
                    : quaternion.identity;
                float3 tInc = v * dt;
                var increment = new SpatialTransform(qInc, tInc);
                body.rootPose = increment * body.rootPose;
            }
        }

        // Spatial acceleration of body b in body b's frame, from the most
        // recent Step(). Useful for tests that want to measure tip motion.
        public SpatialMotion GetSpatialAcceleration(BodyId b) => scratch.a[b.index];

        // Spatial velocity of body b in body b's frame.
        public SpatialMotion GetSpatialVelocity(BodyId b) => scratch.v[b.index];

        // Body-to-world spatial transform. Composed by walking the parent
        // chain — X_world_to_body = Xup[i] ∘ Xup[parent[i]] ∘ ... We then
        // invert to get body → world.
        //
        // This is O(depth); for per-body-per-frame queries we'd cache a
        // X_world_to_body table. Deferred until KSP integration needs it.
        public SpatialTransform GetWorldTransform(BodyId b)
        {
            // World→body composed by walking root-to-leaf, using `then * first`
            // with first.target == then.source. Walking up means we hit Xup[i]
            // in reverse (leaf first, root last), so accumulate on the right.
            //
            // We rebuild Xup[i] from the *current* q rather than reading the
            // scratch buffer: the scratch entries were computed during the
            // last ABA pass 1, *before* integration updated q, so they lag by
            // one step. Rebuilding here keeps the query consistent with the
            // freshly-integrated joint state.
            SpatialTransform wToBody = SpatialTransform.identity;
            int i = b.index;
            while (i != -1)
            {
                SpatialTransform xup = body.joint[i].kind == JointKind.Floating
                    ? body.rootPose
                    : body.joint[i].JointTransform(body.q[i]) * body.Xtree[i];
                wToBody = wToBody * xup;
                i = body.parent[i];
            }
            return wToBody;
        }

        void EnsureScratchCapacity(int cap)
        {
            if (scratch.v.Length >= cap) return;
            scratch = ABA.AllocateScratch(cap);
        }
    }
}
