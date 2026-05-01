// ABA scenario tests — assertions exercising the native ABA forward
// pass through the wire format.
//
// M1 (this file at ship): smoke tests that prove the test rig wires
// up end-to-end. M3+ adds analytic gates (pendulum equilibrium,
// oscillation period, damping decay) once canonical Featherstone
// lands.

using System;
using Longeron.Native;

namespace Longeron.Native.Probe
{
    public static class AbaScenarios
    {
        // KSP physics tick.
        public const float kDt = 1f / 50f;

        // ABA skips integration during the first kAbaWarmupTicks (=5
        // in tree.cpp) so finite-diff a_body / α stabilize. Step a
        // few extras for safety in tests that look at steady state.
        public const int kWarmupSteps = 10;

        // -- M2 -------------------------------------------------------------

        /// <summary>
        /// M2: native-side self-test of the Featherstone spatial-vector
        /// primitives in <c>native/src/spatial.h</c>. Runs in-process on
        /// the C++ side; failure here means an algorithmic bug in the
        /// spatial layer that all of ABA depends on.
        /// </summary>
        public static void Spatial_PrimitivesSelfTest()
        {
            int rc = World.SpatialSelfTest();
            AssertTrue(rc == 0,
                $"longeron_spatial_selftest failed at check #{rc} " +
                "(1=identity-transform, 2=cross-duality, 3=transform-inverse-roundtrip, " +
                "4=spatial-inertia-mul, 5=matrix6-from-inertia agrees, 6=mat33 inverse)");
        }

        // -- M1.1 -----------------------------------------------------------

        /// <summary>
        /// Single-part vessel, no forces. ABA returns early when the
        /// tree has fewer than 2 nodes; no PartPose records flow back
        /// for the (only) root part. The Jolt body still emits BodyPose.
        /// </summary>
        public static void SinglePartVessel_NoFlexEmitted()
        {
            using (var rig = new AbaTestRig())
            {
                var v = rig.Vessel().At(0, 100, 0).Root(mass: 1f).Build();

                // Step a handful of times to let any output records
                // surface. We expect: BodyPose for the vessel; no
                // PartPose (single-part vessel produces none — emission
                // loop in world.cpp:1093 starts at i=1).
                for (int i = 0; i < 5; ++i) rig.Step(kDt);

                AssertTrue(rig.HasBodyPose(v.Id),
                    "single-part vessel should emit BodyPose every tick");
                AssertTrue(!rig.HasPartPose(v.Id, 0),
                    "root part should not emit PartPose (only non-root parts do)");
            }
        }

        // -- M1.2 -----------------------------------------------------------

        /// <summary>
        /// Two-part vessel, no external forces, no gravity. Both parts
        /// at rest pose. After ABA warm-up the child should still have
        /// negligible flex (delta_pos ≈ 0, delta_rot ≈ identity).
        ///
        /// This is the structural baseline: an undisturbed vessel must
        /// not develop spurious flex from numerical noise.
        /// </summary>
        public static void TwoPartVessel_AtRest_NoFlex()
        {
            using (var rig = new AbaTestRig())
            {
                // Tower: root at origin (CoM at 0,0,0), child stacked
                // above with its CoM at (0,1,0); joint anchor at the
                // contact plane (0, 0.5, 0).
                var v = rig.Vessel()
                    .At(0, 100, 0)
                    .Root(mass: 1f)
                    .Child(parentIdx: 0, mass: 1f,
                           comX: 0f, comY: 1f, comZ: 0f,
                           attachX: 0f, attachY: 0.5f, attachZ: 0f,
                           kAng: 100f, cAng: 5f)
                    .Build();

                for (int i = 0; i < kWarmupSteps + 10; ++i) rig.Step(kDt);

                AssertTrue(rig.HasPartPose(v.Id, 1),
                    "non-root part should emit PartPose every tick");

                var pp = rig.PartPose(v.Id, 1);
                float posMag = Mag(pp.DeltaPosX, pp.DeltaPosY, pp.DeltaPosZ);
                float angMag = AxisAngleMag(pp.DeltaRotX, pp.DeltaRotY, pp.DeltaRotZ, pp.DeltaRotW);
                // 1 cm / 0.01 rad (~0.6°) is generous; an at-rest vessel
                // should have orders-of-magnitude tighter than this.
                AssertTrue(posMag < 0.01f,
                    $"at-rest vessel: unexpected |delta_pos|={posMag:F4} m");
                AssertTrue(angMag < 0.01f,
                    $"at-rest vessel: unexpected |delta_rot|={angMag:F4} rad");
            }
        }

        // -- M1.3 -----------------------------------------------------------

        /// <summary>
        /// Two-part vessel with a sustained per-part force on the child.
        /// We expect SOMETHING to happen — flex emerges, joint wrench
        /// non-zero. Magnitudes are not asserted (that's M3's job once
        /// canonical Featherstone is in place).
        /// </summary>
        public static void TwoPartVessel_ForceOnChild_FlexEmerges()
        {
            using (var rig = new AbaTestRig())
            {
                var v = rig.Vessel()
                    .At(0, 100, 0)
                    .Root(mass: 1f)
                    .Child(parentIdx: 0, mass: 1f,
                           comX: 0f, comY: 1f, comZ: 0f,
                           attachX: 0f, attachY: 0.5f, attachZ: 0f,
                           kAng: 100f, cAng: 5f)
                    .Build();

                // Warm up unforced.
                for (int i = 0; i < kWarmupSteps; ++i) rig.Step(kDt);

                // Apply a sustained lateral force at the child's CoM
                // (world-frame X = 100 N). With K_ang = 100 kN·m/rad
                // and lever ≈ 0.5 m from anchor → expected steady-state
                // bend angle θ_ss ≈ F·lever / K_ang = 100 × 0.5 / 100000
                // = 5e-4 rad. Tiny, but well above numerical noise.
                for (int i = 0; i < 60; ++i)
                {
                    rig.World.Input.WriteForceAtPosition(
                        v,
                        fx: 100, fy: 0, fz: 0,
                        // child's CoM in world frame: vessel pos + (0,1,0).
                        px: 0, py: 100 + 1, pz: 0,
                        partIdx: 1);
                    rig.Step(kDt);
                }

                AssertTrue(rig.HasPartPose(v.Id, 1),
                    "expected PartPose for non-root child every tick");

                var pp = rig.PartPose(v.Id, 1);
                float angMag = AxisAngleMag(pp.DeltaRotX, pp.DeltaRotY, pp.DeltaRotZ, pp.DeltaRotW);
                float posMag = Mag(pp.DeltaPosX, pp.DeltaPosY, pp.DeltaPosZ);
                // Smoke assertion only: SOMETHING moved.
                AssertTrue(angMag > 1e-6f || posMag > 1e-6f,
                    $"expected flex response under sustained force; got |Δθ|={angMag:E2} |Δp|={posMag:E2}");

                // JointWrench should also be non-trivial.
                var jw = rig.JointWrench(v.Id, 1);
                float fMag = Mag(jw.FX, jw.FY, jw.FZ);
                AssertTrue(fMag > 1e-3f,
                    $"expected non-trivial joint wrench under force; |F|={fMag:E2}");
            }
        }

        // -- M3 -------------------------------------------------------------
        //
        // Single-link spherical pendulum tests. Configuration:
        //   Root: 1×1×1 box at origin. Mass 1 t.
        //   Child: 1×1×1 box, com at (0, -1.5, 0), anchor at (0, -0.5, 0)
        //          → lever from anchor to com = (0, -1, 0), |L| = 1 m.
        //   Joint: spherical, K_ang and C_ang configurable per test.
        //
        // Forces applied via WriteForceAtPosition: F_x on child + counter
        // -F_x on root → net vessel force = 0 → a_body = 0 → F_flex per
        // part equals F_ext per part. Steady-state analytic for child:
        //   τ_ext_about_anchor (body frame) = (com_in_body) × F_body
        //                                   = (0, -1, 0) × (F_x, 0, 0)
        //                                   = (0, 0, F_x)
        //   τ_spring + τ_ext = 0 ⇒ φ_z = F_x · L / K_ang.

        private const float kPendL    = 1.0f;        // lever from anchor to com (m)
        private const float kPendMass = 1.0f;        // tonnes
        private const float kPendIdiag = 1.0f / 6.0f; // box I_xx ≈ m·(4h²+4h²)/12, h=0.5 → 1/6

        // Effective rotational inertia about the joint anchor:
        // I_anchor = I_c + m·L² (parallel axis).
        private const float kPendIAnchor = kPendIdiag + kPendMass * kPendL * kPendL;

        private static BodyHandle BuildPendulum(AbaTestRig rig, float kAng, float cAng)
        {
            // Kinematic vessel — Jolt body's CoM motion stays frozen so
            // a_body finite-diff = 0 and F_flex equals F_ext per part.
            // This isolates the joint flex dynamics for analytic
            // verification; it isn't representative of an in-game
            // free-floating vessel (M5+ tests cover dynamic motion).
            return rig.Vessel()
                .Kinematic()
                .At(0, 100, 0)
                .Root(mass: kPendMass)                 // half=0.5 default
                .Child(parentIdx: 0, mass: kPendMass,
                       comX: 0f, comY: -1.5f, comZ: 0f,
                       attachX: 0f, attachY: -0.5f, attachZ: 0f,
                       kAng: kAng, cAng: cAng)
                .Build();
        }

        /// <summary>
        /// M3.1: spherical pendulum at rest, no external forces, no
        /// gravity. Expect zero motion after warm-up.
        /// </summary>
        public static void Pendulum_AtRest_NoMotion()
        {
            using (var rig = new AbaTestRig())
            {
                var v = BuildPendulum(rig, kAng: 100f, cAng: 0f);
                for (int i = 0; i < kWarmupSteps + 50; ++i) rig.Step(kDt);

                var pp = rig.PartPose(v.Id, 1);
                float posMag = Mag(pp.DeltaPosX, pp.DeltaPosY, pp.DeltaPosZ);
                float angMag = AxisAngleMag(pp.DeltaRotX, pp.DeltaRotY, pp.DeltaRotZ, pp.DeltaRotW);
                AssertTrue(posMag < 1e-3f,
                    $"at-rest pendulum drifted: |Δp|={posMag:E2}");
                AssertTrue(angMag < 1e-3f,
                    $"at-rest pendulum drifted: |Δθ|={angMag:E2}");
            }
        }

        /// <summary>
        /// M3.2: spherical pendulum equilibrium under a sustained
        /// lateral force on the child. Expected steady-state bend
        /// angle |φ_z| = F · L / K_ang. Tolerance 5%.
        /// </summary>
        public static void Pendulum_EquilibriumMatchesAnalytic()
        {
            const float kAng = 100f;       // kN·m/rad
            const float Fx_kN = 10f;       // 10 kN lateral on child
            const float expectedAngle = Fx_kN * kPendL / kAng;  // = 0.1 rad
            // Critical damping for fast settle.
            float cAng = 2f * (float)Math.Sqrt(kAng * kPendIAnchor);

            using (var rig = new AbaTestRig())
            {
                var v = BuildPendulum(rig, kAng: kAng, cAng: cAng);

                // Settle for 200 ticks (= 4 s = ~6× critical time constant).
                // Vessel is kinematic so the per-part force on the child
                // doesn't propagate to the Jolt body — F_flex equals F_ext.
                for (int i = 0; i < 200; ++i)
                {
                    rig.World.Input.WriteForceAtPosition(
                        v, fx: Fx_kN, fy: 0, fz: 0,
                        px: 0, py: 100 - 1.5, pz: 0,
                        partIdx: 1);
                    rig.Step(kDt);
                }

                var pp = rig.PartPose(v.Id, 1);
                float angMag = AxisAngleMag(pp.DeltaRotX, pp.DeltaRotY, pp.DeltaRotZ, pp.DeltaRotW);
                float relErr = Math.Abs(angMag - expectedAngle) / expectedAngle;
                AssertTrue(relErr < 0.05f,
                    $"steady-state bend angle |φ|={angMag:F4} rad, expected {expectedAngle:F4} rad (relErr={relErr:P1})");

                // Direction: +X force on child with -Y lever produces +Z torque
                // ⇒ rotation about +Z. Verify by sign of DeltaRotZ.
                AssertTrue(pp.DeltaRotZ > 0.01f,
                    $"rotation should be primarily about +Z; got DeltaRotZ={pp.DeltaRotZ:F4}");
            }
        }

        /// <summary>
        /// M3.3: undamped oscillation period matches T = 2π√(I_anchor/K).
        /// Push the pendulum with an impulse, then release, count
        /// zero-crossings to estimate period. Tolerance 5%.
        /// </summary>
        public static void Pendulum_UndampedPeriodMatchesAnalytic()
        {
            const float kAng = 100f;
            float omega_n   = (float)Math.Sqrt(kAng / kPendIAnchor);
            float T_expected = 2f * (float)Math.PI / omega_n;  // ≈ 0.68 s
            const float Fx_kN = 10f;

            using (var rig = new AbaTestRig())
            {
                var v = BuildPendulum(rig, kAng: kAng, cAng: 0f);

                // Warm up first.
                for (int i = 0; i < kWarmupSteps; ++i) rig.Step(kDt);

                // Impulse: apply force for 5 ticks (= 0.1 s) to push the
                // pendulum, then release and let it oscillate.
                for (int i = 0; i < 5; ++i)
                {
                    rig.World.Input.WriteForceAtPosition(
                        v, fx: Fx_kN, fy: 0, fz: 0,
                        px: 0, py: 100 - 1.5, pz: 0,
                        partIdx: 1);
                    rig.Step(kDt);
                }

                // Track DeltaRotZ for 4 expected periods, find peaks.
                int N = (int)(4f * T_expected / kDt);
                float[] z = new float[N];
                for (int i = 0; i < N; ++i)
                {
                    rig.Step(kDt);
                    var pp = rig.PartPose(v.Id, 1);
                    z[i] = pp.DeltaRotZ;
                }

                // Find indices of consecutive same-sign extrema (peaks);
                // distance between same-direction peaks = period.
                int firstPeak = -1, secondPeak = -1;
                for (int i = 1; i < N - 1; ++i)
                {
                    bool isLocalMax = z[i] > z[i-1] && z[i] > z[i+1] && z[i] > 0;
                    if (isLocalMax)
                    {
                        if (firstPeak == -1) firstPeak = i;
                        else if (secondPeak == -1) { secondPeak = i; break; }
                    }
                }
                AssertTrue(firstPeak != -1 && secondPeak != -1,
                    $"could not find two oscillation peaks (firstPeak={firstPeak}, secondPeak={secondPeak})");

                float T_observed = (secondPeak - firstPeak) * kDt;
                float relErr = Math.Abs(T_observed - T_expected) / T_expected;
                AssertTrue(relErr < 0.05f,
                    $"oscillation period T={T_observed:F3} s, expected {T_expected:F3} s (relErr={relErr:P1})");
            }
        }

        /// <summary>
        /// M3.4: critical-damped decay. Initial impulse, then verify
        /// the angle settles to within 1% of zero in roughly 5τ where
        /// τ = 1/(ζ ω_n) = 1/ω_n at critical damping.
        /// </summary>
        public static void Pendulum_CriticalDampingDecaysToRest()
        {
            const float kAng = 100f;
            float omega_n = (float)Math.Sqrt(kAng / kPendIAnchor);
            float cAng = 2f * (float)Math.Sqrt(kAng * kPendIAnchor);  // critical
            const float Fx_kN = 10f;

            using (var rig = new AbaTestRig())
            {
                var v = BuildPendulum(rig, kAng: kAng, cAng: cAng);
                for (int i = 0; i < kWarmupSteps; ++i) rig.Step(kDt);

                // Apply impulse for 5 ticks.
                for (int i = 0; i < 5; ++i)
                {
                    rig.World.Input.WriteForceAtPosition(
                        v, fx: Fx_kN, fy: 0, fz: 0,
                        px: 0, py: 100 - 1.5, pz: 0,
                        partIdx: 1);
                    rig.Step(kDt);
                }

                // Settle for 8 / ω_n (≈ 8 critical-damping time constants).
                int settleSteps = (int)(8f / omega_n / kDt);
                for (int i = 0; i < settleSteps; ++i) rig.Step(kDt);

                var pp = rig.PartPose(v.Id, 1);
                float angMag = AxisAngleMag(pp.DeltaRotX, pp.DeltaRotY, pp.DeltaRotZ, pp.DeltaRotW);
                AssertTrue(angMag < 0.005f,
                    $"critical-damped pendulum should have settled to ~0; got |φ|={angMag:F4} rad");
            }
        }

        // -- M4 -------------------------------------------------------------
        //
        // Three-part stack tower extending along +Y. Each part is a unit
        // box; the stack spans (0,0,0) → (0,2,0) with 0.5m air-gap
        // overlap at each joint.
        //
        //   Root   (idx 0): com (0,0,0), half (0.5,0.5,0.5).
        //   Middle (idx 1): com (0,1,0), attach (0,0.5,0).  Joint to root.
        //   Top    (idx 2): com (0,2,0), attach (0,1.5,0).  Joint to middle.
        //
        // Joint axis convention (parent CoM → child anchor):
        //   bottom joint (root → middle): axis = +Y in vessel frame.
        //   middle joint (middle → top):  axis = +Y in vessel frame.
        // So JointWrench.FX (axial in joint frame) is +Y in vessel frame.
        //   FX > 0 → compression (stack squeezed together)
        //   FX < 0 → tension     (stack pulled apart)

        private const float kStackMass = 1.0f;   // tonnes per part
        private const float kStackGrav = 9.81f;  // m/s² (KSP-realistic g)

        // Effective rotational inertia of one stack-part about its joint
        // anchor: I_c + m·L² where L = distance(com, anchor) = 0.5 m.
        private const float kStackPartIAnchor = (1.0f / 6.0f)
                                              + kStackMass * 0.5f * 0.5f;

        private static BodyHandle BuildThreeStack(
            AbaTestRig rig, float kAng, float cAng, bool kinematic = true)
        {
            var b = rig.Vessel().At(0, 100, 0);
            if (kinematic) b = b.Kinematic();
            return b
                .Root(mass: kStackMass)
                .Child(parentIdx: 0, mass: kStackMass,
                       comX: 0f, comY: 1f, comZ: 0f,
                       attachX: 0f, attachY: 0.5f, attachZ: 0f,
                       kAng: kAng, cAng: cAng)
                .Child(parentIdx: 1, mass: kStackMass,
                       comX: 0f, comY: 2f, comZ: 0f,
                       attachX: 0f, attachY: 1.5f, attachZ: 0f,
                       kAng: kAng, cAng: cAng)
                .Build();
        }

        /// <summary>
        /// M4.1: 3-part stack at rest, no forces. Both non-root parts
        /// stay unflexed; joint wrenches near zero.
        /// </summary>
        public static void ThreeStack_AtRest_NoFlex()
        {
            using (var rig = new AbaTestRig())
            {
                float cAng = 2f * (float)Math.Sqrt(1000f * kStackPartIAnchor);
                var v = BuildThreeStack(rig, kAng: 1000f, cAng: cAng);

                for (int i = 0; i < kWarmupSteps + 50; ++i) rig.Step(kDt);

                for (ushort idx = 1; idx <= 2; ++idx)
                {
                    var pp = rig.PartPose(v.Id, idx);
                    float pos = Mag(pp.DeltaPosX, pp.DeltaPosY, pp.DeltaPosZ);
                    float ang = AxisAngleMag(pp.DeltaRotX, pp.DeltaRotY,
                                              pp.DeltaRotZ, pp.DeltaRotW);
                    AssertTrue(pos < 1e-3f,
                        $"part {idx}: at-rest 3-stack drifted, |Δp|={pos:E2} m");
                    AssertTrue(ang < 1e-3f,
                        $"part {idx}: at-rest 3-stack drifted, |Δθ|={ang:E2} rad");
                }
            }
        }

        /// <summary>
        /// M4.2: 3-part stack under axial gravity (-Y per part, applied
        /// to the kinematic vessel as F_ext on each part). The stack
        /// stays straight (no lateral component) but each joint sees
        /// compression from the parts above it:
        ///   bottom joint: 2 parts above × m·g
        ///   middle joint: 1 part  above × m·g
        /// JointWrench.FX (axial, +compression) matches to 1%.
        /// </summary>
        public static void ThreeStack_AxialGravity_CompressionMatchesAnalytic()
        {
            // Stiff joints — under pure axial load there's no bending
            // moment to deflect them, but the test isolates the axial
            // force calculation. Make them stiff anyway to keep the
            // stack pristine.
            const float kAng = 10000f;
            float cAng = 2f * (float)Math.Sqrt(kAng * kStackPartIAnchor);

            using (var rig = new AbaTestRig())
            {
                var v = BuildThreeStack(rig, kAng: kAng, cAng: cAng);

                // Sustained per-part gravity. 200 ticks = 4 s, comfortably
                // past any transient.
                for (int settle = 0; settle < 200; ++settle)
                {
                    for (ushort idx = 0; idx <= 2; ++idx)
                    {
                        rig.World.Input.WriteForceAtPosition(
                            v,
                            fx: 0, fy: -kStackMass * kStackGrav, fz: 0,
                            // part i's CoM in world frame: (0, 100+i, 0).
                            px: 0, py: 100 + idx, pz: 0,
                            partIdx: idx);
                    }
                    rig.Step(kDt);
                }

                // Bottom joint (between root and middle, part_idx=1):
                // RNEA computes the wrench the PARENT transmits to the
                // child to hold the subtree against external forces.
                // Under -Y gravity the parent must push child +Y →
                // axial-component f·axis = +m_above·g (compression).
                var jw1 = rig.JointWrench(v.Id, 1);
                float expected1 = 2f * kStackMass * kStackGrav;
                float relErr1 = Math.Abs(jw1.FX - expected1) / expected1;
                AssertTrue(relErr1 < 0.01f,
                    $"bottom joint axial = {jw1.FX:F3} kN, expected {expected1:F3} (relErr={relErr1:P2})");

                var jw2 = rig.JointWrench(v.Id, 2);
                float expected2 = 1f * kStackMass * kStackGrav;
                float relErr2 = Math.Abs(jw2.FX - expected2) / expected2;
                AssertTrue(relErr2 < 0.01f,
                    $"middle joint axial = {jw2.FX:F3} kN, expected {expected2:F3} (relErr={relErr2:P2})");
            }
        }

        /// <summary>
        /// M4.3: 3-part stack under lateral force (+X per part). Each
        /// joint deflects to balance the moment from the parts above:
        ///   q_bottom = (lever_middle + lever_top) · F / K
        ///            = (0.5 + 1.5) · F / K = 2·F / K
        ///   q_middle = lever_top · F / K = 0.5·F / K
        /// Cumulative vessel-frame rotations:
        ///   middle (idx 1): q_bottom
        ///   top    (idx 2): q_bottom + q_middle
        /// Tolerance 5%.
        /// </summary>
        public static void ThreeStack_LateralForce_BendMatchesAnalytic()
        {
            const float kAng  = 1000f;          // softer to get measurable bend
            const float Fx_kN = 10f;            // 10 kN lateral per part
            float cAng = 2f * (float)Math.Sqrt(kAng * kStackPartIAnchor);

            float qBottom = (0.5f + 1.5f) * Fx_kN / kAng;   // = 0.02 rad
            float qMiddle = 0.5f * Fx_kN / kAng;             // = 0.005 rad
            float angMiddleExpected = qBottom;               // ≈ 0.02
            float angTopExpected    = qBottom + qMiddle;     // ≈ 0.025

            using (var rig = new AbaTestRig())
            {
                var v = BuildThreeStack(rig, kAng: kAng, cAng: cAng);

                // Critical damping settles quickly: ~5/ω_n is comfortable.
                // ω_n ≈ √(K/I) = √(1000/0.42) ≈ 49 rad/s ⇒ 5/ω_n ≈ 0.1 s.
                // Use 200 ticks = 4 s ≫ 0.1 s for safety.
                for (int settle = 0; settle < 200; ++settle)
                {
                    for (ushort idx = 0; idx <= 2; ++idx)
                    {
                        rig.World.Input.WriteForceAtPosition(
                            v, fx: Fx_kN, fy: 0, fz: 0,
                            px: 0, py: 100 + idx, pz: 0,
                            partIdx: idx);
                    }
                    rig.Step(kDt);
                }

                var pp1 = rig.PartPose(v.Id, 1);
                var pp2 = rig.PartPose(v.Id, 2);
                float angObs1 = AxisAngleMag(pp1.DeltaRotX, pp1.DeltaRotY,
                                              pp1.DeltaRotZ, pp1.DeltaRotW);
                float angObs2 = AxisAngleMag(pp2.DeltaRotX, pp2.DeltaRotY,
                                              pp2.DeltaRotZ, pp2.DeltaRotW);

                float relErr1 = Math.Abs(angObs1 - angMiddleExpected) / angMiddleExpected;
                float relErr2 = Math.Abs(angObs2 - angTopExpected)    / angTopExpected;
                AssertTrue(relErr1 < 0.05f,
                    $"middle bend |φ|={angObs1:F4} rad, expected {angMiddleExpected:F4} (relErr={relErr1:P2})");
                AssertTrue(relErr2 < 0.05f,
                    $"top bend |φ|={angObs2:F4} rad, expected {angTopExpected:F4} (relErr={relErr2:P2})");
            }
        }

        /// <summary>
        /// M4.4: 3-part stack transient release. Apply lateral force to
        /// develop bend, release, verify the chain damps back to rest
        /// within ~5 critical-damping time constants.
        /// </summary>
        public static void ThreeStack_TransientRelease_DampsToRest()
        {
            const float kAng  = 1000f;
            const float Fx_kN = 10f;
            float omega_n = (float)Math.Sqrt(kAng / kStackPartIAnchor);
            float cAng    = 2f * (float)Math.Sqrt(kAng * kStackPartIAnchor);

            using (var rig = new AbaTestRig())
            {
                var v = BuildThreeStack(rig, kAng: kAng, cAng: cAng);

                // Load phase: 30 ticks of lateral force to develop bend.
                for (int i = 0; i < 30; ++i)
                {
                    for (ushort idx = 0; idx <= 2; ++idx)
                    {
                        rig.World.Input.WriteForceAtPosition(
                            v, fx: Fx_kN, fy: 0, fz: 0,
                            px: 0, py: 100 + idx, pz: 0,
                            partIdx: idx);
                    }
                    rig.Step(kDt);
                }

                // Sanity: confirm we DID develop bend (else the test is meaningless).
                var ppLoaded = rig.PartPose(v.Id, 2);
                float bentAngle = AxisAngleMag(ppLoaded.DeltaRotX, ppLoaded.DeltaRotY,
                                                ppLoaded.DeltaRotZ, ppLoaded.DeltaRotW);
                AssertTrue(bentAngle > 0.005f,
                    $"load phase did not develop bend; |φ|={bentAngle:F4}");

                // Release phase: no forces, settle. The multi-link
                // chain's lower-order modes are slower than a single
                // pendulum's: each joint's effective inertia is the
                // sum-of-parallel-axis of all subtree parts, which
                // exceeds the single-part kStackPartIAnchor we used to
                // compute cAng. So C_ang is under-critical for the
                // chain modes; use a generous 30/ω_n (single-link τ as
                // upper bound on chain τ) for the settle window.
                int settleSteps = (int)(30f / omega_n / kDt);
                for (int i = 0; i < settleSteps; ++i) rig.Step(kDt);

                for (ushort idx = 1; idx <= 2; ++idx)
                {
                    var pp = rig.PartPose(v.Id, idx);
                    float ang = AxisAngleMag(pp.DeltaRotX, pp.DeltaRotY,
                                              pp.DeltaRotZ, pp.DeltaRotW);
                    AssertTrue(ang < 0.005f,
                        $"part {idx} did not damp to rest after release: |Δθ|={ang:F4}");
                }
            }
        }

        // -- M5 -------------------------------------------------------------
        //
        // Booster + side masses: a 3-part central stack with two side
        // masses attached radially to the middle part.
        //
        //   Root   (idx 0): com (0, 0, 0).            Mass kStackMass.
        //   Middle (idx 1): com (0, 1, 0). attach (0,0.5,0).  Joint to root.
        //   Top    (idx 2): com (0, 2, 0). attach (0,1.5,0).  Joint to middle.
        //   SideA  (idx 3): com (1, 1, 0). attach (0.5,1,0).  Joint to middle.
        //   SideB  (idx 4): com (-1, 1, 0). attach (-0.5,1,0). Joint to middle.
        //
        // Side mass = ½ central. The side joints' axes are radial
        // (parent CoM → child anchor): (+0.5,1,0)→(+0.5,0,0) for A,
        // (-0.5,1,0)→(-0.5,0,0) for B (after subtracting middle's CoM
        // (0,1,0)).

        private const float kSideMass = 0.5f;
        private const float kSidePartIAnchor = (1.0f / 6.0f) * kSideMass
                                              + kSideMass * 0.5f * 0.5f;

        private static BodyHandle BuildBoosterPlusSides(
            AbaTestRig rig, float kAng, float cAng, bool kinematic = true)
        {
            var b = rig.Vessel().At(0, 100, 0);
            if (kinematic) b = b.Kinematic();
            return b
                .Root(mass: kStackMass)
                .Child(parentIdx: 0, mass: kStackMass,
                       comX: 0f, comY: 1f, comZ: 0f,
                       attachX: 0f, attachY: 0.5f, attachZ: 0f,
                       kAng: kAng, cAng: cAng)
                .Child(parentIdx: 1, mass: kStackMass,
                       comX: 0f, comY: 2f, comZ: 0f,
                       attachX: 0f, attachY: 1.5f, attachZ: 0f,
                       kAng: kAng, cAng: cAng)
                .Child(parentIdx: 1, mass: kSideMass,
                       comX: 1f, comY: 1f, comZ: 0f,
                       attachX: 0.5f, attachY: 1f, attachZ: 0f,
                       kAng: kAng, cAng: cAng)
                .Child(parentIdx: 1, mass: kSideMass,
                       comX: -1f, comY: 1f, comZ: 0f,
                       attachX: -0.5f, attachY: 1f, attachZ: 0f,
                       kAng: kAng, cAng: cAng)
                .Build();
        }

        /// <summary>
        /// M5.1: booster + side masses at rest under axial gravity.
        /// Side joints (idx 3, 4) bear no axial load (their axes are
        /// radial — perpendicular to gravity). The central stack joints
        /// bear weight plus the share routed through the side-mass
        /// anchors (which connect to the middle part).
        /// </summary>
        public static void Booster_AtRest_AxialGravity_Stable()
        {
            const float kAng = 10000f;
            float cAng = 2f * (float)Math.Sqrt(kAng * kStackPartIAnchor);

            using (var rig = new AbaTestRig())
            {
                var v = BuildBoosterPlusSides(rig, kAng: kAng, cAng: cAng);

                for (int settle = 0; settle < 200; ++settle)
                {
                    // Per-part gravity (kinematic vessel):
                    //   Root, middle, top: kStackMass.
                    //   Side A (idx 3), Side B (idx 4): kSideMass.
                    rig.World.Input.WriteForceAtPosition(
                        v, fx: 0, fy: -kStackMass * kStackGrav, fz: 0,
                        px: 0, py: 100, pz: 0, partIdx: 0);
                    rig.World.Input.WriteForceAtPosition(
                        v, fx: 0, fy: -kStackMass * kStackGrav, fz: 0,
                        px: 0, py: 101, pz: 0, partIdx: 1);
                    rig.World.Input.WriteForceAtPosition(
                        v, fx: 0, fy: -kStackMass * kStackGrav, fz: 0,
                        px: 0, py: 102, pz: 0, partIdx: 2);
                    rig.World.Input.WriteForceAtPosition(
                        v, fx: 0, fy: -kSideMass * kStackGrav, fz: 0,
                        px: 1, py: 101, pz: 0, partIdx: 3);
                    rig.World.Input.WriteForceAtPosition(
                        v, fx: 0, fy: -kSideMass * kStackGrav, fz: 0,
                        px: -1, py: 101, pz: 0, partIdx: 4);
                    rig.Step(kDt);
                }

                // Side masses: their joint axis is radial X. Gravity
                // (-Y) is perpendicular to axis → no axial component.
                // Just verify no oscillation (q stays small).
                for (ushort idx = 3; idx <= 4; ++idx)
                {
                    var pp = rig.PartPose(v.Id, idx);
                    float ang = AxisAngleMag(pp.DeltaRotX, pp.DeltaRotY,
                                              pp.DeltaRotZ, pp.DeltaRotW);
                    AssertTrue(ang < 0.01f,
                        $"side mass {idx} oscillating under axial gravity: |Δθ|={ang:F4} rad");
                }

                // Central stack joints carry weight. Bottom joint sees
                // (top + middle + side_A + side_B + their gravity).
                // Approximate: 2·kStackMass + 2·kSideMass = 3 t worth.
                var jw1 = rig.JointWrench(v.Id, 1);
                float expected1 = (2f * kStackMass + 2f * kSideMass) * kStackGrav;
                float relErr1 = Math.Abs(jw1.FX - expected1) / expected1;
                AssertTrue(relErr1 < 0.02f,
                    $"bottom joint axial = {jw1.FX:F3} kN, expected {expected1:F3} (relErr={relErr1:P2})");
            }
        }

        /// <summary>
        /// M5.2: booster under steady rotation. The v1 XPBD-style
        /// implementation failed this test — under SAS / vessel-frame
        /// rotation, side masses' q_joint diverged from identity
        /// because the per-link integration in vessel frame and the
        /// angular spring were fighting each other.
        ///
        /// Pinocchio's reduced-coord ABA handles this by construction:
        /// q is integrated in joint-relative coordinates, so any
        /// rigid-body rotation of the vessel doesn't enter the joint
        /// state. Centripetal stress (ω×ω×r per part) does enter
        /// F_flex — but its torque about each joint anchor stays
        /// small for moderate ω and stiff joints.
        /// </summary>
        public static void Booster_SteadyRotation_JointsStayPinned()
        {
            const float kAng = 10000f;
            float cAng = 2f * (float)Math.Sqrt(kAng * kStackPartIAnchor);
            const float omega_z = 1.0f;       // rad/s steady spin

            using (var rig = new AbaTestRig())
            {
                // Kinematic body with ang_vel set; Jolt integrates the
                // pose forward each tick. ABA's α finite-diff = 0 (steady
                // ω); F_flex captures only centripetal contribution.
                var v = BuildBoosterPlusSides(rig, kAng: kAng, cAng: cAng);

                // Set initial ang_vel — kinematic pose stays at start
                // and Jolt advances it each tick using the velocity.
                rig.World.Input.WriteSetKinematicPose(
                    v, posX: 0, posY: 100, posZ: 0,
                    rotX: 0, rotY: 0, rotZ: 0, rotW: 1,
                    linX: 0, linY: 0, linZ: 0,
                    angX: 0, angY: 0, angZ: omega_z);

                // Step for 100 ticks (= 2 s real, ≈ 2 rad rotation).
                // Long enough for any v1-style divergent flex to
                // accumulate visibly.
                for (int i = 0; i < 100; ++i) rig.Step(kDt);

                // Side masses: q_joint should stay near identity. The
                // analytic centripetal-induced equilibrium is
                // (com_in_body × F_centrifugal) / K_ang. For ω = 1 rad/s,
                // r_side = (1,1,0), com_in_body_side = (0.5,0,0):
                //   ω×(ω×r) = (-1,-1,0). F_inertial = m·(-1,-1,0).
                //   F_flex = -F_inertial = m·(1,1,0).
                //   M = com_in_body × F_flex = (0,0,0.5·m).
                //   q = M / K_ang. For m=0.5, K=10000: q ≈ 2.5e-5 rad ≈ 0.0014°.
                // Threshold: 0.5° = 0.00873 rad. Plenty of margin.
                for (ushort idx = 3; idx <= 4; ++idx)
                {
                    var pp = rig.PartPose(v.Id, idx);
                    // Compute joint-relative q (vs vessel-frame delta_rot).
                    // For Pinocchio's reduced coords, the joint q is
                    // R_v[parent].conj() * R_v[child]. Since R_v[parent]
                    // (the middle part) might also have flex, we use
                    // the cumulative delta_rot which is good enough at
                    // these small magnitudes.
                    float ang = AxisAngleMag(pp.DeltaRotX, pp.DeltaRotY,
                                              pp.DeltaRotZ, pp.DeltaRotW);
                    AssertTrue(ang < 0.00873f,   // 0.5°
                        $"under steady rotation, side mass {idx}: |Δθ|={ang:F4} rad (>0.5°)");
                }
            }
        }

        /// <summary>
        /// M5.3: gimbal-style offset thrust. We approximate an engine
        /// gimbaled 5° off-axis by attributing a tilted thrust vector
        /// to the middle part (the "engine" stand-in) with the
        /// application point at the root's bottom face — i.e., 1.5 m
        /// below the middle's CoM. The lever-arm geometry is what
        /// produces a bending moment at the bottom joint:
        ///     T_ext_at_middle = (point - com_middle) × F
        ///                     = (0, -1.5, 0) × (Fx, Fy, 0)
        ///                     = (0, 0, 1.5·Fx)
        ///
        /// Verifies (a) bend response is observable, (b) bounded, (c)
        /// side masses don't diverge from rigid-body kinematics.
        /// </summary>
        public static void Booster_GimbalOffsetThrust_BendBounded()
        {
            const float kAng = 1000f;            // softer for observable bend
            float cAng = 2f * (float)Math.Sqrt(kAng * kStackPartIAnchor);
            const float Fmag = 50.0f;            // kN
            float angleRad = (float)(5.0 * Math.PI / 180.0);
            float Fx = Fmag * (float)Math.Sin(angleRad);
            float Fy = Fmag * (float)Math.Cos(angleRad);

            using (var rig = new AbaTestRig())
            {
                var v = BuildBoosterPlusSides(rig, kAng: kAng, cAng: cAng);

                // Sustained thrust attributed to the middle part,
                // applied at the root's bottom face. The application
                // point being 1.5m below middle's CoM is what generates
                // the bending moment.
                for (int i = 0; i < 200; ++i)
                {
                    rig.World.Input.WriteForceAtPosition(
                        v, fx: Fx, fy: Fy, fz: 0,
                        px: 0, py: 100 - 0.5, pz: 0,
                        partIdx: 1);
                    rig.Step(kDt);
                }

                // Middle part should bend in the +Z direction (the
                // moment generated by the lever-arm gimbal force).
                // Top inherits middle's rotation cumulatively.
                var pp_middle = rig.PartPose(v.Id, 1);
                var pp_top    = rig.PartPose(v.Id, 2);
                float angMiddle = AxisAngleMag(pp_middle.DeltaRotX, pp_middle.DeltaRotY,
                                                pp_middle.DeltaRotZ, pp_middle.DeltaRotW);
                float angTop    = AxisAngleMag(pp_top.DeltaRotX, pp_top.DeltaRotY,
                                                pp_top.DeltaRotZ, pp_top.DeltaRotW);
                AssertTrue(angMiddle > 1e-3f,
                    $"middle should show bend response to gimbal: |Δθ|={angMiddle:E2}");
                AssertTrue(angTop >= angMiddle - 1e-4f,
                    $"top should inherit middle's bend: top |Δθ|={angTop:F4}, middle |Δθ|={angMiddle:F4}");
                AssertTrue(angTop < 0.5f,
                    $"top bend exceeds bound: |Δθ|={angTop:F4} rad (~{angTop*180.0f/(float)Math.PI:F1}°)");

                // Side masses should NOT diverge — they're attached to
                // the middle part rigidly, so their cumulative
                // delta_rot should track the middle part's bend
                // (since they rotate WITH the middle, not relative to
                // it). The joint-relative q_side stays near identity.
                for (ushort idx = 3; idx <= 4; ++idx)
                {
                    var pp = rig.PartPose(v.Id, idx);
                    float ang = AxisAngleMag(pp.DeltaRotX, pp.DeltaRotY,
                                              pp.DeltaRotZ, pp.DeltaRotW);
                    // Side's delta_rot should be CLOSE to the middle's
                    // delta_rot (within ~1° absolute, since we're using
                    // cumulative delta_rot, not joint-relative).
                    AssertTrue(Math.Abs(ang - angMiddle) < 0.02f,
                        $"side mass {idx} not tracking middle's rotation: side |Δθ|={ang:F4}, middle |Δθ|={angMiddle:F4}");
                }
            }
        }

        // -- helpers --------------------------------------------------------

        private static float Mag(float x, float y, float z)
            => (float)Math.Sqrt(x * x + y * y + z * z);

        private static float AxisAngleMag(float qx, float qy, float qz, float qw)
        {
            if (qw < 0) { qx = -qx; qy = -qy; qz = -qz; qw = -qw; }
            float vmag = (float)Math.Sqrt(qx * qx + qy * qy + qz * qz);
            return vmag < 1e-6f ? 0f : 2f * (float)Math.Atan2(vmag, qw);
        }

        private static void AssertTrue(bool cond, string message)
        {
            if (!cond) throw new InvalidOperationException(message);
        }
    }
}
