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
