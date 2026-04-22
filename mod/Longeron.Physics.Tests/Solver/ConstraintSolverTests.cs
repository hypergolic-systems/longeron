using System.Collections.Generic;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Longeron.Physics;
using Longeron.Physics.Contact;

namespace Longeron.Physics.Tests
{
    // End-to-end gate for the PGS constraint solver. Pairs the narrowphase
    // with StepWithContacts so we exercise the real integration path:
    //   narrowphase → ContactConstraint list → StepWithContacts → settled.
    //
    // These tests pin the behavior the Phase-A penalty solver could never
    // provide: zero-velocity steady state, no energy pumping, no bounce.
    [TestClass]
    public class ConstraintSolverTests
    {
        const float g = 9.81f;

        static float3x3 IsotropicInertia(float i) =>
            new float3x3(
                new float3(i, 0, 0),
                new float3(0, i, 0),
                new float3(0, 0, i));

        // A 10×10×2 "pad" centered at origin; top face at y = 1.
        static readonly AabbShape Pad = new AabbShape(
            new float3(-5f, -1f, -5f),
            new float3( 5f,  1f,  5f));

        const float Radius = 0.5f;

        static ArticulatedScene MakeSingleBody(float mass, float inertia, float3 startPos)
        {
            var scene = new ArticulatedScene();
            scene.AddBody(BodyId.None, Joint.Floating(),
                new SpatialInertia(mass, float3.zero, IsotropicInertia(inertia)),
                SpatialTransform.identity);
            scene.Body.rootPose = new SpatialTransform(quaternion.identity, startPos);
            scene.Body.rootVelocity = SpatialMotion.zero;
            scene.SetGravity(new float3(0, -g, 0));
            return scene;
        }

        // Construct the capsule-in-world for a vertically-oriented body
        // centered at rootPose, axis ±Y of length h - 2r, radius r.
        static void BuildContacts(ArticulatedScene scene, float halfH, float radius,
                                    List<ContactConstraint> contacts)
        {
            contacts.Clear();
            var X = scene.GetWorldTransform(new BodyId(0));
            float3 seg0 = X.translation + math.mul(X.rotation, new float3(0f, -(halfH - radius), 0f));
            float3 seg1 = X.translation + math.mul(X.rotation, new float3(0f,  (halfH - radius), 0f));

            var mf = new ContactManifold();
            Narrowphase.CapsuleVsAabb(seg0, seg1, radius, Pad, ref mf);
            for (int i = 0; i < mf.count; i++)
            {
                mf.Get(i, out var p, out var n, out var d);
                contacts.Add(new ContactConstraint { bodyId = new BodyId(0), point = p, normal = n, depth = d });
            }
        }

        [TestMethod]
        public void BodySettlesOnFlatPad_NoOscillation()
        {
            // 100 kg capsule body, radius 0.5 m, half-height 1.0 m. Start
            // 0.1 m above the pad — bottom endpoint is just clear. After a
            // brief fall the contact should catch it, Baumgarte closes the
            // gap, and steady state is weight ≈ normal impulse with zero
            // residual velocity.
            float halfH = 1.0f;
            float inertia = 0.4f * 100f * halfH * halfH;
            var scene = MakeSingleBody(100f, inertia, new float3(0f, 1f + halfH + 0.1f, 0f));

            var contacts = new List<ContactConstraint>();
            float dt = 0.02f;

            for (int i = 0; i < 200; i++)
            {
                BuildContacts(scene, halfH, Radius, contacts);
                scene.StepWithContacts(contacts, dt);
            }

            float vLin = math.length(scene.Body.rootVelocity.linear);
            float vAng = math.length(scene.Body.rootVelocity.angular);
            float y = scene.Body.rootPose.translation.y;

            Assert.IsTrue(vLin < 0.05f, $"linear velocity should be near zero, got {vLin:F3}");
            Assert.IsTrue(vAng < 0.05f, $"angular velocity should be near zero, got {vAng:F3}");

            // Capsule bottom endpoint is at y - halfH; pad top at y = 1.
            // Baumgarte converges toward exactly-touching (bottom endpoint at
            // y=1, body origin at y=1+halfH). Allow a couple cm of tolerance.
            Assert.AreEqual(1f + halfH, y, 0.02f,
                $"body should rest with bottom endpoint near pad top, got y={y:F3}");
        }

        [TestMethod]
        public void ContactImpulseCannotPumpEnergy()
        {
            // Drop a body from 2m above the pad. After 200 ticks at rest,
            // total kinetic energy should be small — there's no restitution
            // so bounces absorb.
            float halfH = 1.0f;
            float inertia = 0.4f * 100f * halfH * halfH;
            var scene = MakeSingleBody(100f, inertia, new float3(0f, 1f + halfH + 2f, 0f));

            var contacts = new List<ContactConstraint>();
            float dt = 0.02f;
            float peakKE = 0f;

            for (int i = 0; i < 500; i++)
            {
                BuildContacts(scene, halfH, Radius, contacts);
                scene.StepWithContacts(contacts, dt);

                float v = math.length(scene.Body.rootVelocity.linear);
                float ke = 0.5f * 100f * v * v;
                if (ke > peakKE) peakKE = ke;
            }

            // Final state: negligible KE.
            float finalV = math.length(scene.Body.rootVelocity.linear);
            float finalKE = 0.5f * 100f * finalV * finalV;

            Assert.IsTrue(finalKE < 1f,
                $"settled KE should be ~0 J; got {finalKE:F3} J (peak was {peakKE:F1} J)");
        }

        [TestMethod]
        public void ZeroContacts_ReproducesFreeFall()
        {
            // Sanity: StepWithContacts with empty list should match Step(dt)
            // exactly — verified by comparing to the free-fall drop.
            var scene = MakeSingleBody(10f, 1f, new float3(0f, 100f, 0f));

            var empty = new List<ContactConstraint>();
            float dt = 1e-3f;
            for (int i = 0; i < 1000; i++)
                scene.StepWithContacts(empty, dt);

            // z after 1s: -½·g·t² ≈ -4.905 m below start.
            float expectedY = 100f - 0.5f * g * 1f * 1f;
            Assert.AreEqual(expectedY, scene.Body.rootPose.translation.y, 0.02f);
        }
    }
}
