// Penalty-based contact solver — Phase A.
//
// For each buffered ContactEntry, compute a world-frame normal force as a
// spring-damper on penetration depth and approach velocity, then accumulate
// it into the solver's per-body fExt via VesselScene.AddWorldForceAtPosition
// (which handles the lever-arm torque + body-frame conversion for us).
//
//   Fn_mag = max(0, k · depth - c_damp · v_n)
//
//   depth = max(0, -ContactPoint.separation)    // ≥ 0 while penetrating
//   v_n   = dot(v_point, normal)                // +ve = separating
//
// The clamp to ≥ 0 means contacts never pull (no sticking). Damping kicks in
// on approach (v_n < 0), opposing penetration rate; on separation (v_n > 0)
// the damping term would subtract from the spring, which the clamp also
// handles correctly (contact disengages softly rather than yanking back).
//
// Friction is deferred to Phase A.2 — we ship normal-force-only first so
// resting-stiffness tuning can happen without a coupled variable.
//
// This file is the designated migration seam for Phase B: the penalty math
// will be replaced with a proper PGS / LCP constraint solver that treats the
// contact normals as inequality constraints on contact-point relative
// velocity. Public API (Apply(VesselScene, dt)) stays the same; the driver
// and VesselScene do not need to change for that swap.

using Longeron.Physics;

namespace Longeron.Integration
{
    internal static class ContactSolver
    {
        // Launchpad-tuned defaults. With ContactDiscovery casting up to 5 rays
        // per part, a 3-part vessel gets ~15 contacts in contact with the pad,
        // giving effective stiffness K = N·k. Critical damping scales as
        // c ~ 2·sqrt(N·k·m); with N=5..15 for 7 t vessels, per-contact c_damp
        // ≈ 3e4 puts us in the 0.6..0.9 × critical range across that band —
        // near-critical for vertical rockets, still compliant on landing
        // impacts where only a few rays strike initially.
        const float k       = 5.0e5f;
        const float c_damp  = 3.0e4f;
        const float mu      = 0.8f;
        const float v_t_eps = 1e-4f;

        // Normal-force magnitude cap. Without a cap, damping (c_damp · |v_n|)
        // is unbounded for high approach velocities — a 74 m/s landing produces
        // c_damp × v_n = 2.2 MN per contact, and Euler integration with that
        // force overshoots on the rebound, adding energy each bounce. The cap
        // pins the impulse to something physically reasonable (~3× the static
        // spring force at max clamped depth).
        const float MAX_FN_PER_CONTACT = 3.0e5f;

        public static void Apply(VesselScene scene, float dt)
        {
            var contacts = scene.Contacts;
            if (contacts.Count == 0) return;
            if (dt <= 0f) return;

            var bodies = scene.Scene.Body;

            for (int idx = 0; idx < contacts.Count; idx++)
            {
                var c = contacts[idx];

                var X      = scene.Scene.GetWorldTransform(c.bodyId);
                var v_body = scene.Scene.GetSpatialVelocity(c.bodyId);

                float3 r_w    = c.point - X.translation;
                float3 omegaW = math.mul(X.rotation, v_body.angular);
                float3 vlinW  = math.mul(X.rotation, v_body.linear);
                float3 vPoint = vlinW + math.cross(omegaW, r_w);

                float depth = math.max(0f, -c.separation);
                float v_n   = math.dot(vPoint, c.normal);

                float fnMag = math.max(0f, k * depth - c_damp * v_n);
                if (fnMag > MAX_FN_PER_CONTACT) fnMag = MAX_FN_PER_CONTACT;
                if (fnMag <= 0f) continue;

                float3 Fn = fnMag * c.normal;

                // Linearized Coulomb friction. Ft opposes tangential velocity
                // at the contact point; magnitude capped by the friction cone
                // (mu·Fn) and by the "do-not-overshoot stopping velocity"
                // term m_eff·|v_t|/dt — that second cap prevents the discrete
                // Euler step from reversing tangential motion under static
                // friction, which would look like spontaneous jitter.
                float3 v_t = vPoint - v_n * c.normal;
                float v_t_mag = math.length(v_t);
                float3 Ft = float3.zero;
                if (v_t_mag > v_t_eps)
                {
                    float3 t_hat = v_t / v_t_mag;
                    float m_eff = bodies.I[c.bodyId.index].mass;
                    if (m_eff < 1f) m_eff = 1f;
                    float ft_cap_cone  = mu * fnMag;
                    float ft_cap_stop  = m_eff * v_t_mag / dt;
                    float ft_mag = math.min(ft_cap_cone, ft_cap_stop);
                    Ft = -ft_mag * t_hat;
                }

                scene.AddWorldForceAtPosition(c.bodyId, Fn + Ft, c.point);
            }
        }
    }
}
