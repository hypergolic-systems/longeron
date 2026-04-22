// ArticulatedScene — ownership layer.
//
// Holds the ArticulatedBody (topology + state), scratch buffers for ABA,
// and the world-frame gravity vector. Step() runs one ABA solve plus a
// hardcoded semi-implicit Euler integration; GetWorldTransform walks the
// parent chain to compose the body-to-world transform on demand.

using System;
using System.Collections.Generic;
using Longeron.Physics.Contact;

namespace Longeron.Physics
{
    public sealed class ArticulatedScene
    {
        readonly ArticulatedBody body;
        ABA.Scratch scratch;
        Rnea.Scratch rneaScratch;
        public float3 gravity;   // world frame; (0, 0, 0) = no gravity

        public ArticulatedScene(int initialCapacity = 16)
        {
            body = new ArticulatedBody(initialCapacity);
            scratch = ABA.AllocateScratch(initialCapacity);
            rneaScratch = Rnea.AllocateScratch(initialCapacity);
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

        // Single forward-dynamics step, no contacts. Equivalent to
        // StepWithContacts(null, dt). Kept as a convenience + stable test
        // surface for the solver-only test suite.
        public void Step(float dt) => StepWithContacts(null, dt);

        // Forward dynamics + (optional) contact constraint application +
        // semi-implicit Euler integration.
        //
        // Contract: when `contacts` is null or empty, output matches Step(dt)
        // exactly (all pre-existing solver tests verify this). When contacts
        // are provided, the PGS loop runs between the ABA free-dynamics
        // computation and the integration step, applying normal (and, once
        // implemented, friction) impulses that satisfy the contact
        // inequalities before positions advance.
        public void StepWithContacts(IList<ContactConstraint> contacts, float dt)
        {
            ABA.Solve(body, gravity, ref scratch);
            // Run RNEA with the ABA-computed accelerations so each body's
            // reaction wrench at its parent-side joint is available for
            // diagnostics / break detection via GetJointReactionWrench().
            // RNEA's Pass 2 populates scratch.f[i] as the spatial force
            // transmitted through joint i, in body i's frame.
            Rnea.Solve(body, gravity, scratch.qddot, rneaScratch, scratch.rootAccel);

            // 1-DOF joints: classic q-qdot advance. Fixed joints skipped.
            for (int i = 0; i < body.Count; i++)
            {
                Joint j = body.joint[i];
                if (j.kind != JointKind.Revolute && j.kind != JointKind.Prismatic) continue;
                body.qdot[i] += scratch.qddot[i] * dt;
                body.q[i]    += body.qdot[i] * dt;
            }

            // Floating root: predict free-dynamics velocity first, then let
            // the constraint solver fold in contact impulses, then integrate
            // position from the corrected velocity.
            if (body.Count > 0 && body.joint[0].kind == JointKind.Floating)
            {
                body.rootVelocity = body.rootVelocity + scratch.rootAccel * dt;

                // Skeleton hook: contact-impulse application lives here.
                // Step 4 of the Phase B plan fills this in with the PGS
                // normal-constraint loop + Baumgarte; Step 5 adds friction.
                if (contacts != null && contacts.Count > 0)
                    ApplyContactImpulses(contacts, dt);

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

        // PGS contact impulse application. Currently a skeleton — the
        // iteration loop lands in Phase B step 4. Leaving the hook in place
        // keeps the integrate-after-impulse ordering correct even when the
        // body is still empty.
        void ApplyContactImpulses(IList<ContactConstraint> contacts, float dt)
        {
            // Intentional no-op until PGS lands. Contacts are discarded.
        }

        // Spatial acceleration of body b in body b's frame, from the most
        // recent Step(). Useful for tests that want to measure tip motion.
        public SpatialMotion GetSpatialAcceleration(BodyId b) => scratch.a[b.index];

        // Spatial velocity of body b in body b's frame.
        public SpatialMotion GetSpatialVelocity(BodyId b) => scratch.v[b.index];

        // Spatial reaction wrench at body b's parent-side joint, in body b's
        // frame. Populated each Step() from RNEA pass 2. For the root body
        // this is the net external wrench required to hold the base in its
        // current motion (normally ~0 for a Floating root). For a Fixed or
        // low-DOF child this is the total wrench the joint is transmitting
        // from the subtree into its parent — compare against breakForce /
        // breakTorque to detect joint failure.
        public SpatialForce GetJointReactionWrench(BodyId b) => rneaScratch.f[b.index];

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
            if (scratch.v.Length < cap) scratch = ABA.AllocateScratch(cap);
            if (rneaScratch.v.Length < cap) rneaScratch = Rnea.AllocateScratch(cap);
        }
    }
}
