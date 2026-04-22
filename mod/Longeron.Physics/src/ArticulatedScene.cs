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

        // PGS contact-impulse loop. Operates on the Floating root's spatial
        // velocity (body frame). Each contact i is represented by:
        //   J_i : row 1×6, body-frame Jacobian mapping rootVelocity to the
        //         contact point's normal-projected velocity.
        //         J_i = [ (r × n), n ]   (angular | linear, spatial-force convention)
        //   ΔV_i = IA_root⁻¹ · J_iᵀ  — the rootVelocity increment produced by
        //                              a unit λ impulse at this contact.
        //   m_eff_i = 1 / (J_i · ΔV_i) — the contact-frame effective mass
        //                               along the normal direction.
        //
        // IA_root is the articulated-body inertia ABA Pass 2 just computed,
        // which for an all-Fixed-joint subtree gives us the whole assembly's
        // effective inertia at the root — free of approximation.
        //
        // Inner loop per iteration: recompute current v_n, compare to the
        // Baumgarte target (positive separation proportional to penetration
        // depth), compute the required impulse delta, project accumulated λ
        // to non-negative, apply the net impulse via rootVelocity += δλ · ΔV.
        //
        // Per-contact scratch is stack-allocated to the constraint-solver
        // scratch buffers so the hot loop never allocates.

        ContactScratch contactScratch;

        struct ContactScratch
        {
            public SpatialForce[]  J_T;
            public SpatialMotion[] dV;
            public float[]         mEff;
            public float[]         lambda;
            public int Capacity => J_T != null ? J_T.Length : 0;

            public void EnsureCapacity(int n)
            {
                if (Capacity >= n) return;
                int cap = Capacity == 0 ? 16 : Capacity * 2;
                while (cap < n) cap *= 2;
                J_T    = new SpatialForce[cap];
                dV     = new SpatialMotion[cap];
                mEff   = new float[cap];
                lambda = new float[cap];
            }
        }

        const int   PGS_ITERATIONS = 15;
        const float BAUMGARTE_BETA = 0.2f;

        void ApplyContactImpulses(IList<ContactConstraint> contacts, float dt)
        {
            int N = contacts.Count;
            if (N == 0) return;
            if (body.joint[0].kind != JointKind.Floating) return;

            contactScratch.EnsureCapacity(N);

            var IA_root = scratch.IA[0];
            var Rinv = math.inverse(body.rootPose.rotation);
            var rootOrigin = body.rootPose.translation;

            // --- Pre-pass: per-contact Jacobian, ΔV response, effective mass.
            for (int i = 0; i < N; i++)
            {
                var c = contacts[i];
                float3 r_body = math.mul(Rinv, c.point - rootOrigin);
                float3 n_body = math.mul(Rinv, c.normal);
                float3 rxn    = math.cross(r_body, n_body);

                var jt = new SpatialForce(rxn, n_body);
                var dv = IA_root.Solve6(jt);
                float jdv = math.dot(rxn, dv.angular) + math.dot(n_body, dv.linear);

                contactScratch.J_T[i]    = jt;
                contactScratch.dV[i]     = dv;
                contactScratch.mEff[i]   = jdv > 1e-9f ? 1f / jdv : 0f;
                contactScratch.lambda[i] = 0f;
            }

            // --- PGS iterations. Each pass walks all contacts in order,
            //     applying whatever correction is needed given the accumulated
            //     impulses from prior passes. Typical convergence: a handful
            //     of contacts settle in 5-10 iterations, 15 is a conservative
            //     cap until we have a real convergence tolerance.
            for (int iter = 0; iter < PGS_ITERATIONS; iter++)
            {
                for (int i = 0; i < N; i++)
                {
                    float mEff = contactScratch.mEff[i];
                    if (mEff <= 0f) continue;

                    var c = contacts[i];
                    var jt = contactScratch.J_T[i];
                    var dv = contactScratch.dV[i];

                    // Current normal-directional velocity at contact.
                    float v_n = math.dot(jt.angular, body.rootVelocity.angular)
                              + math.dot(jt.linear,  body.rootVelocity.linear);

                    // Baumgarte-style position drift correction. Push-out
                    // velocity proportional to current penetration; clamped
                    // to non-negative so "already separating fast" doesn't
                    // reduce the target.
                    float target_v_n = math.max(0f, BAUMGARTE_BETA * c.depth / dt);

                    float delta = mEff * (target_v_n - v_n);
                    float newLambda = math.max(0f, contactScratch.lambda[i] + delta);
                    float actualDelta = newLambda - contactScratch.lambda[i];
                    contactScratch.lambda[i] = newLambda;

                    // Apply the impulse as a root-velocity delta.
                    body.rootVelocity = body.rootVelocity + actualDelta * dv;
                }
            }
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
