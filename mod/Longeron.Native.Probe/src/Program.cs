// Longeron.Native.Probe — Phase 1 verification harness.
//
// Asserts that Jolt fires kinematic-vs-static and kinematic-vs-
// kinematic contact callbacks. If these checks fail, the entire Jolt
// pivot is wrong and the design needs to be revisited before Phase 1
// proceeds. Exit code 0 = pass, non-zero = fail.

using System;
using Longeron.Native;

namespace Longeron.Native.Probe
{
    internal static class Program
    {
        private static int sFailures;

        public static int Main(string[] args)
        {
            Console.WriteLine($"== Longeron native bridge probe ==");
            Console.WriteLine($"native: {World.NativeVersion}");
            Console.WriteLine();

            Run("native bridge loads + version readable", VersionStringIsReadable);
            Run("World creates and disposes",            WorldCreatesAndDisposes);
            Run("World steps with empty input",          WorldStepsWithEmptyInput);
            Run("dynamic vs static fires contact (sanity)", DynamicVsStatic_FiresContactReport);
            Run("kinematic vs static fires contact",     KinematicOverlappingStatic_FiresContactReport);
            Run("kinematic vs kinematic fires contact",  KinematicVsKinematic_FiresContactReport);
            Run("non-overlapping bodies fire no contact", KinematicNotOverlapping_NoContactReport);
            Run("persisted contact fires every tick",    KinematicPersistedContact_FiresEveryTick);
            Run("sphere shape mirrored + contact fires",       SphereShape_FiresContactReport);
            Run("convex hull mirrored + contact fires",        ConvexHull_FiresContactReport);
            Run("static compound (box+sphere) fires contact",  CompoundShape_FiresContactReport);
            Run("contact normal points roughly upward",        ContactNormal_PointsUpward);
            Run("FixedConstraint holds two dynamic bodies together", FixedConstraint_HoldsTwoBodies);

            // -- ABA scenarios (M1+) ---------------------------------------
            Console.WriteLine();
            Console.WriteLine("-- ABA scenarios --");
            Run("M2 spatial-vector primitives self-test",       AbaScenarios.Spatial_PrimitivesSelfTest);
            Run("M1.1 single-part vessel emits no PartPose",    AbaScenarios.SinglePartVessel_NoFlexEmitted);
            Run("M1.2 two-part vessel at rest stays unflexed",  AbaScenarios.TwoPartVessel_AtRest_NoFlex);
            Run("M1.3 two-part vessel: force on child → flex",  AbaScenarios.TwoPartVessel_ForceOnChild_FlexEmerges);
            Run("M3.1 pendulum at rest, no motion",             AbaScenarios.Pendulum_AtRest_NoMotion);
            Run("M3.2 pendulum equilibrium = F·L/K_ang",        AbaScenarios.Pendulum_EquilibriumMatchesAnalytic);
            Run("M3.3 undamped period = 2π√(I/K)",              AbaScenarios.Pendulum_UndampedPeriodMatchesAnalytic);
            Run("M3.4 critical-damped decay to rest",           AbaScenarios.Pendulum_CriticalDampingDecaysToRest);
            Run("M4.1 3-stack at rest stays unflexed",          AbaScenarios.ThreeStack_AtRest_NoFlex);
            Run("M4.2 3-stack axial gravity: compression = m_above·g",       AbaScenarios.ThreeStack_AxialGravity_CompressionMatchesAnalytic);
            Run("M4.3 3-stack lateral force: bend = Σ(lever)·F/K",            AbaScenarios.ThreeStack_LateralForce_BendMatchesAnalytic);
            Run("M4.4 3-stack transient release damps to rest", AbaScenarios.ThreeStack_TransientRelease_DampsToRest);

            Console.WriteLine();
            if (sFailures == 0)
            {
                Console.WriteLine("== ALL CHECKS PASSED ==");
                return 0;
            }
            else
            {
                Console.WriteLine($"== {sFailures} CHECK(S) FAILED ==");
                return 1;
            }
        }

        // -- Harness ---------------------------------------------------

        private static void Run(string name, Action body)
        {
            try
            {
                body();
                Console.WriteLine($"  PASS  {name}");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"  FAIL  {name}");
                Console.WriteLine($"        {ex.GetType().Name}: {ex.Message}");
                ++sFailures;
            }
        }

        private static void AssertTrue(bool condition, string message)
        {
            if (!condition) throw new InvalidOperationException(message);
        }

        // -- Tests -----------------------------------------------------

        private static void VersionStringIsReadable()
        {
            string version = World.NativeVersion;
            AssertTrue(version != null && version.StartsWith("longeron_native"),
                $"unexpected version string: {version}");
        }

        private static void WorldCreatesAndDisposes()
        {
            using (var world = new World(LongeronConfig.Default))
            {
                AssertTrue(world != null, "World ctor returned null");
            }
        }

        private static void WorldStepsWithEmptyInput()
        {
            using (var world = new World(LongeronConfig.Default))
            {
                world.Step(1f / 60f);
                world.Step(1f / 60f);
                world.Step(1f / 60f);
                AssertTrue(world.Output.Length == 0,
                    $"expected empty output, got {world.Output.Length} bytes");
            }
        }

        private static void DynamicVsStatic_FiresContactReport()
        {
            // Sanity check: replicate Jolt's HelloWorld pattern with a
            // dynamic body. If this fails, our setup is broken at a
            // fundamental level (filters, layers, etc) rather than
            // being a kinematic-specific issue.
            using (var world = new World(LongeronConfig.Default))
            {
                var ground = new BodyHandle(1);
                var box    = new BodyHandle(2);

                world.Input.WriteBodyCreateBox(
                    ground, BodyType.Static, Layer.Static,
                    halfX: 5f, halfY: 0.5f, halfZ: 5f,
                    posX: 0, posY: -0.5, posZ: 0,
                    rotX: 0, rotY: 0, rotZ: 0, rotW: 1,
                    mass: 0f);

                world.Input.WriteBodyCreateBox(
                    box, BodyType.Dynamic, Layer.Kinematic,
                    halfX: 0.5f, halfY: 0.5f, halfZ: 0.5f,
                    posX: 0, posY: 0.4, posZ: 0,
                    rotX: 0, rotY: 0, rotZ: 0, rotW: 1,
                    mass: 1f);

                world.Step(1f / 60f);

                AssertTrue(HasContactBetween(world, 1, 2),
                    "dynamic-vs-static contact callback did not fire on the first step");
            }
        }

        private static void KinematicOverlappingStatic_FiresContactReport()
        {
            using (var world = new World(LongeronConfig.Default))
            {
                var ground = new BodyHandle(1);
                var box    = new BodyHandle(2);

                world.Input.WriteBodyCreateBox(
                    ground, BodyType.Static, Layer.Static,
                    halfX: 5f, halfY: 0.5f, halfZ: 5f,
                    posX: 0, posY: -0.5, posZ: 0,
                    rotX: 0, rotY: 0, rotZ: 0, rotW: 1,
                    mass: 0f);

                // Kinematic box overlapping ground top by 0.1m.
                world.Input.WriteBodyCreateBox(
                    box, BodyType.Kinematic, Layer.Kinematic,
                    halfX: 0.5f, halfY: 0.5f, halfZ: 0.5f,
                    posX: 0, posY: 0.4, posZ: 0,
                    rotX: 0, rotY: 0, rotZ: 0, rotW: 1,
                    mass: 1f);

                world.Step(1f / 60f);

                AssertTrue(HasContactBetween(world, 1, 2),
                    "kinematic-vs-static contact callback did not fire on the first step");
            }
        }

        private static void KinematicVsKinematic_FiresContactReport()
        {
            using (var world = new World(LongeronConfig.Default))
            {
                var a = new BodyHandle(10);
                var b = new BodyHandle(11);

                world.Input.WriteBodyCreateBox(
                    a, BodyType.Kinematic, Layer.Kinematic,
                    halfX: 0.5f, halfY: 0.5f, halfZ: 0.5f,
                    posX: 0, posY: 5, posZ: 0,
                    rotX: 0, rotY: 0, rotZ: 0, rotW: 1,
                    mass: 1f);

                world.Input.WriteBodyCreateBox(
                    b, BodyType.Kinematic, Layer.Kinematic,
                    halfX: 0.5f, halfY: 0.5f, halfZ: 0.5f,
                    posX: 0.5, posY: 5, posZ: 0,
                    rotX: 0, rotY: 0, rotZ: 0, rotW: 1,
                    mass: 1f);

                world.Step(1f / 60f);

                AssertTrue(HasContactBetween(world, 10, 11),
                    "kinematic-vs-kinematic contact callback did not fire");
            }
        }

        private static void KinematicNotOverlapping_NoContactReport()
        {
            using (var world = new World(LongeronConfig.Default))
            {
                world.Input.WriteBodyCreateBox(
                    new BodyHandle(1), BodyType.Static, Layer.Static,
                    halfX: 5f, halfY: 0.5f, halfZ: 5f,
                    posX: 0, posY: -0.5, posZ: 0,
                    rotX: 0, rotY: 0, rotZ: 0, rotW: 1,
                    mass: 0f);

                // Kinematic 5m above ground — well-separated.
                world.Input.WriteBodyCreateBox(
                    new BodyHandle(2), BodyType.Kinematic, Layer.Kinematic,
                    halfX: 0.5f, halfY: 0.5f, halfZ: 0.5f,
                    posX: 0, posY: 5, posZ: 0,
                    rotX: 0, rotY: 0, rotZ: 0, rotW: 1,
                    mass: 1f);

                world.Step(1f / 60f);

                AssertTrue(!HasAnyContact(world),
                    "got a ContactReport for non-overlapping bodies (false positive)");
            }
        }

        private static void KinematicPersistedContact_FiresEveryTick()
        {
            using (var world = new World(LongeronConfig.Default))
            {
                world.Input.WriteBodyCreateBox(
                    new BodyHandle(1), BodyType.Static, Layer.Static,
                    halfX: 5f, halfY: 0.5f, halfZ: 5f,
                    posX: 0, posY: -0.5, posZ: 0,
                    rotX: 0, rotY: 0, rotZ: 0, rotW: 1,
                    mass: 0f);

                world.Input.WriteBodyCreateBox(
                    new BodyHandle(2), BodyType.Kinematic, Layer.Kinematic,
                    halfX: 0.5f, halfY: 0.5f, halfZ: 0.5f,
                    posX: 0, posY: 0.4, posZ: 0,
                    rotX: 0, rotY: 0, rotZ: 0, rotW: 1,
                    mass: 1f);

                int contactsSeen = 0;
                for (int i = 0; i < 30; ++i)
                {
                    world.Step(1f / 60f);
                    if (HasContactBetween(world, 1, 2)) ++contactsSeen;
                }

                AssertTrue(contactsSeen >= 25,
                    $"expected persistent contact for ~30 ticks, saw {contactsSeen} — " +
                    "OnContactPersisted may not be firing for kinematic-vs-static");
            }
        }

        // -- Shape mirroring tests (Phase 1.5) --------------------------

        private static void SphereShape_FiresContactReport()
        {
            using (var world = new World(LongeronConfig.Default))
            {
                var ground = new BodyHandle(1);
                var sphere = new BodyHandle(2);

                world.Input.WriteBodyCreateBox(
                    ground, BodyType.Static, Layer.Static,
                    halfX: 5f, halfY: 0.5f, halfZ: 5f,
                    posX: 0, posY: -0.5, posZ: 0,
                    rotX: 0, rotY: 0, rotZ: 0, rotW: 1,
                    mass: 0f);

                // Sphere of radius 0.5 centered at y=0.4 → bottom at y=-0.1,
                // overlapping ground top by 0.1.
                world.Input.WriteBodyCreateSphere(
                    sphere, BodyType.Kinematic, Layer.Kinematic,
                    radius: 0.5f,
                    posX: 0, posY: 0.4, posZ: 0,
                    rotX: 0, rotY: 0, rotZ: 0, rotW: 1,
                    mass: 1f);

                world.Step(1f / 60f);

                AssertTrue(HasContactBetween(world, 1, 2),
                    "sphere body did not produce a ContactReport against ground");
            }
        }

        private static void ConvexHull_FiresContactReport()
        {
            using (var world = new World(LongeronConfig.Default))
            {
                var ground = new BodyHandle(1);
                var hull   = new BodyHandle(2);

                world.Input.WriteBodyCreateBox(
                    ground, BodyType.Static, Layer.Static,
                    halfX: 5f, halfY: 0.5f, halfZ: 5f,
                    posX: 0, posY: -0.5, posZ: 0,
                    rotX: 0, rotY: 0, rotZ: 0, rotW: 1,
                    mass: 0f);

                // Tetrahedron vertices in body-local frame.
                // Apex at +y, base triangle below.
                float[] tetraVerts = {
                     0f,  1f,  0f,    // apex
                     1f, -1f,  1f,    // base front-right
                    -1f, -1f,  1f,    // base front-left
                     0f, -1f, -1f,    // base back
                };

                // Place body so the base (y_local=-1) is at world y=-0.1
                // — overlapping ground top.
                world.Input.WriteBodyCreateConvexHull(
                    hull, BodyType.Kinematic, Layer.Kinematic,
                    vertices: tetraVerts,
                    posX: 0, posY: 0.9, posZ: 0,
                    rotX: 0, rotY: 0, rotZ: 0, rotW: 1,
                    mass: 1f);

                world.Step(1f / 60f);

                AssertTrue(HasContactBetween(world, 1, 2),
                    "convex hull body did not produce a ContactReport against ground");
            }
        }

        private static void CompoundShape_FiresContactReport()
        {
            // Two sub-shapes within one body: a box on the left, a
            // sphere on the right, both above the ground but with the
            // sphere's bottom dipping into ground.
            using (var world = new World(LongeronConfig.Default))
            {
                var ground = new BodyHandle(1);
                var compound = new BodyHandle(2);

                world.Input.WriteBodyCreateBox(
                    ground, BodyType.Static, Layer.Static,
                    halfX: 5f, halfY: 0.5f, halfZ: 5f,
                    posX: 0, posY: -0.5, posZ: 0,
                    rotX: 0, rotY: 0, rotZ: 0, rotW: 1,
                    mass: 0f);

                // Body origin at (0, 0.6, 0). Sub-shapes:
                //   Box (half 0.4) at sub_pos (-1, 0, 0): center (-1, 0.6, 0),
                //     bottom y=0.2 — clear of ground.
                //   Sphere (radius 0.7) at sub_pos (+1, 0, 0): center (+1, 0.6, 0),
                //     bottom y=-0.1 — overlapping ground.
                world.Input.BeginBodyCreate(
                    compound, BodyType.Kinematic, Layer.Kinematic,
                    posX: 0, posY: 0.6, posZ: 0,
                    rotX: 0, rotY: 0, rotZ: 0, rotW: 1,
                    mass: 1f, shapeCount: 2);
                world.Input.AppendShapeBox(
                    -1, 0, 0,  0, 0, 0, 1,
                    halfX: 0.4f, halfY: 0.4f, halfZ: 0.4f);
                world.Input.AppendShapeSphere(
                    +1, 0, 0,  0, 0, 0, 1,
                    radius: 0.7f);

                world.Step(1f / 60f);

                AssertTrue(HasContactBetween(world, 1, 2),
                    "compound body (box+sphere) did not produce a ContactReport — " +
                    "one of the sub-shapes was supposed to overlap the ground");
            }
        }

        private static void ContactNormal_PointsUpward()
        {
            // Body resting on ground: contact normal should point
            // approximately +Y (away from the static ground into the
            // kinematic body). Verifies that we're actually decoding
            // contact data correctly, not just that contacts fire.
            using (var world = new World(LongeronConfig.Default))
            {
                var ground = new BodyHandle(1);
                var sphere = new BodyHandle(2);

                world.Input.WriteBodyCreateBox(
                    ground, BodyType.Static, Layer.Static,
                    halfX: 5f, halfY: 0.5f, halfZ: 5f,
                    posX: 0, posY: -0.5, posZ: 0,
                    rotX: 0, rotY: 0, rotZ: 0, rotW: 1,
                    mass: 0f);

                world.Input.WriteBodyCreateSphere(
                    sphere, BodyType.Kinematic, Layer.Kinematic,
                    radius: 0.5f,
                    posX: 0, posY: 0.4, posZ: 0,
                    rotX: 0, rotY: 0, rotZ: 0, rotW: 1,
                    mass: 1f);

                world.Step(1f / 60f);

                bool found = false;
                float observedY = 0f;
                RecordType type;
                while ((type = world.Output.Next()) != RecordType.None)
                {
                    if (type == RecordType.ContactReport)
                    {
                        world.Output.ReadContactReport(out var c);
                        // Jolt sorts the body pair by ID, so normal direction
                        // depends on which is body1 vs body2 in Jolt's terms.
                        // Take |normal.Y| > 0.5 — pointing roughly along +/-Y
                        // is what "resting on ground" should look like.
                        found = true;
                        observedY = Math.Abs(c.NormalY);
                    }
                    else if (type == RecordType.BodyPose)
                    {
                        world.Output.ReadBodyPose(out _);
                    }
                    else
                    {
                        throw new InvalidOperationException($"unexpected record type {type}");
                    }
                }
                AssertTrue(found, "no ContactReport emitted at all");
                AssertTrue(observedY > 0.5f,
                    $"contact normal Y={observedY:F3} — expected close to 1 for a body sitting on a flat ground");
            }
        }

        private static void FixedConstraint_HoldsTwoBodies()
        {
            // Two dynamic boxes 1 m apart along X, joined by a
            // FixedConstraint. Gravity along -Y. Static ground 1 m
            // below the lower box. After settling, both bodies should
            // have the same Y (constraint keeps them at the original
            // relative offset). Without the constraint, one box would
            // fall onto the ground and the other would fall forever
            // since they're independent dynamic bodies.
            using (var world = new World(LongeronConfig.Default))
            {
                world.Input.WriteSetGravity(0, -9.81, 0);

                var ground = new BodyHandle(1);
                var a      = new BodyHandle(2);
                var b      = new BodyHandle(3);

                world.Input.WriteBodyCreateBox(
                    ground, BodyType.Static, Layer.Static,
                    halfX: 50f, halfY: 0.5f, halfZ: 50f,
                    posX: 0, posY: -1.5, posZ: 0,
                    rotX: 0, rotY: 0, rotZ: 0, rotW: 1,
                    mass: 0f);

                // Two boxes side-by-side at y=2.
                world.Input.WriteBodyCreateBox(
                    a, BodyType.Dynamic, Layer.Kinematic,
                    halfX: 0.5f, halfY: 0.5f, halfZ: 0.5f,
                    posX: -1.0, posY: 2.0, posZ: 0,
                    rotX: 0, rotY: 0, rotZ: 0, rotW: 1,
                    mass: 1f);
                world.Input.WriteBodyCreateBox(
                    b, BodyType.Dynamic, Layer.Kinematic,
                    halfX: 0.5f, halfY: 0.5f, halfZ: 0.5f,
                    posX: +1.0, posY: 2.0, posZ: 0,
                    rotX: 0, rotY: 0, rotZ: 0, rotW: 1,
                    mass: 1f);
                world.Input.WriteConstraintCreateFixed(
                    constraintId: 100, bodyA: a, bodyB: b);

                // Step for ~3 s — long enough to settle.
                BodyPoseRecord poseA = default, poseB = default;
                bool sawA = false, sawB = false;
                for (int i = 0; i < 180; ++i)
                {
                    world.Step(1f / 60f);
                    sawA = false; sawB = false;
                    RecordType type;
                    while ((type = world.Output.Next()) != RecordType.None)
                    {
                        if (type == RecordType.BodyPose)
                        {
                            world.Output.ReadBodyPose(out var p);
                            if (p.Body.Id == 2) { poseA = p; sawA = true; }
                            else if (p.Body.Id == 3) { poseB = p; sawB = true; }
                        }
                        else if (type == RecordType.ContactReport)
                        {
                            world.Output.ReadContactReport(out _);
                        }
                    }
                }

                AssertTrue(sawA && sawB, "missing pose record for A or B");
                // After settling on the ground, both bodies' Y must
                // match within tolerance. If the constraint is broken,
                // they diverge.
                double dy = System.Math.Abs(poseA.PosY - poseB.PosY);
                AssertTrue(dy < 0.05,
                    $"FixedConstraint broken — bodies diverged in Y: A.y={poseA.PosY:F3}, B.y={poseB.PosY:F3}, |Δy|={dy:F3}");
                // Distance between A and B should be approximately
                // their original 2 m offset (X axis). FixedConstraint
                // preserves the relative pose at the moment it was
                // created.
                double dx = poseA.PosX - poseB.PosX;
                double dz = poseA.PosZ - poseB.PosZ;
                double dist = System.Math.Sqrt(dx * dx + dy * dy + dz * dz);
                AssertTrue(dist > 1.5 && dist < 2.5,
                    $"FixedConstraint distance drifted: |A-B|={dist:F3} (expected ≈2.0)");
            }
        }

        // -- Helpers ---------------------------------------------------

        private static bool HasContactBetween(World world, uint a, uint b)
        {
            bool found = false;
            RecordType type;
            while ((type = world.Output.Next()) != RecordType.None)
            {
                switch (type)
                {
                    case RecordType.BodyPose:
                        world.Output.ReadBodyPose(out _);
                        break;
                    case RecordType.ContactReport:
                        world.Output.ReadContactReport(out var c);
                        if ((c.BodyA.Id == a && c.BodyB.Id == b) ||
                            (c.BodyA.Id == b && c.BodyB.Id == a))
                            found = true;
                        break;
                    default:
                        throw new InvalidOperationException($"unexpected record type {type}");
                }
            }
            return found;
        }

        private static bool HasAnyContact(World world)
        {
            bool found = false;
            RecordType type;
            while ((type = world.Output.Next()) != RecordType.None)
            {
                switch (type)
                {
                    case RecordType.BodyPose:
                        world.Output.ReadBodyPose(out _);
                        break;
                    case RecordType.ContactReport:
                        world.Output.ReadContactReport(out _);
                        found = true;
                        break;
                    default:
                        throw new InvalidOperationException($"unexpected record type {type}");
                }
            }
            return found;
        }
    }
}
