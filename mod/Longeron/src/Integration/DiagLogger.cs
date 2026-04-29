// DiagLogger — capture per-tick part-pose drift for the first few
// seconds after each vessel registers.
//
// FixedConstraint is supposed to be a rigid weld. If parts drift in
// their relative-to-root pose over time, the constraint is failing.
// Static tilt at rest implies the rest pose itself was recorded wrong;
// time-varying drift implies the solver is leaking.
//
// One snapshot per managed vessel: tick-0 world rotation + position of
// every part. Per tick, log the largest deviation in relative-to-root
// pose. Captures at tight cadence (every tick) for the first 10 ticks
// (200 ms) so we can see the spawn-time dynamics, then sparsely (every
// 10 ticks) out to 250 ticks (5 s).

using System.Collections.Generic;
using UnityEngine;

namespace Longeron.Integration
{
    internal static class DiagLogger
    {
        const string LogPrefix = "[Longeron/diag] ";
        const int kTickWindow = 250;       // 5 s @ 50 Hz

        static int _globalTick = 0;
        static readonly Dictionary<Vessel, VesselSnap> _snaps = new Dictionary<Vessel, VesselSnap>();

        class VesselSnap
        {
            public int Tick0;
            public Dictionary<Part, Quaternion> InitialRotWorld;
            public Dictionary<Part, Vector3> InitialPosWorld;
        }

        public static void Clear()
        {
            _snaps.Clear();
            _globalTick = 0;
        }

        public static void OnTickStart() { _globalTick++; }

        public static void LogVessel(Vessel v)
        {
            if (v == null || !v.loaded || v.packed) return;
            if (v.parts == null || v.parts.Count == 0) return;
            var root = v.rootPart;
            if (root == null || root.transform == null) return;

            if (!_snaps.TryGetValue(v, out var snap))
            {
                snap = new VesselSnap
                {
                    Tick0 = _globalTick,
                    InitialRotWorld = new Dictionary<Part, Quaternion>(),
                    InitialPosWorld = new Dictionary<Part, Vector3>(),
                };
                foreach (var p in v.parts)
                {
                    if (p == null || p.transform == null) continue;
                    snap.InitialRotWorld[p] = p.transform.rotation;
                    snap.InitialPosWorld[p] = p.transform.position;
                }
                _snaps[v] = snap;
                Debug.Log(LogPrefix + $"BEGIN '{v.vesselName}' parts={v.parts.Count} root={root.partInfo?.name}");
            }

            int dt = _globalTick - snap.Tick0;
            if (dt > kTickWindow) return;

            // Cadence: every tick for the first 10, then every 10th out
            // to the window edge. Plenty for spotting "drift starts at
            // tick N" patterns without flooding KSP.log.
            bool emit = (dt < 10) || ((dt % 10) == 0) || (dt == kTickWindow);
            if (!emit) return;

            if (!snap.InitialRotWorld.TryGetValue(root, out var rootRot0)) return;
            if (!snap.InitialPosWorld.TryGetValue(root, out var rootPos0)) return;

            var rootRotNow = root.transform.rotation;
            var rootPosNow = root.transform.position;

            float maxAngDrift = 0f;
            Part maxAngDriftPart = null;
            float maxPosDrift = 0f;
            Part maxPosDriftPart = null;

            foreach (var p in v.parts)
            {
                if (p == null || p.transform == null || p == root) continue;
                if (!snap.InitialRotWorld.TryGetValue(p, out var pRot0)) continue;
                if (!snap.InitialPosWorld.TryGetValue(p, out var pPos0)) continue;

                // Relative-to-root rotation now and at t0.
                Quaternion relRotNow = Quaternion.Inverse(rootRotNow) * p.transform.rotation;
                Quaternion relRot0   = Quaternion.Inverse(rootRot0)   * pRot0;
                Quaternion delta     = Quaternion.Inverse(relRot0) * relRotNow;

                delta.ToAngleAxis(out float angle, out Vector3 _);
                if (angle > 180f) angle = 360f - angle;
                if (angle > maxAngDrift) { maxAngDrift = angle; maxAngDriftPart = p; }

                // Relative-to-root position drift.
                Vector3 relPosNow = Quaternion.Inverse(rootRotNow) * (p.transform.position - rootPosNow);
                Vector3 relPos0   = Quaternion.Inverse(rootRot0)   * (pPos0 - rootPos0);
                float posDrift = (relPosNow - relPos0).magnitude;
                if (posDrift > maxPosDrift) { maxPosDrift = posDrift; maxPosDriftPart = p; }
            }

            // Also probe the root's own absolute rotation drift (should
            // also be zero if the rocket is sitting still on the pad).
            Quaternion rootDelta = Quaternion.Inverse(rootRot0) * rootRotNow;
            rootDelta.ToAngleAxis(out float rootAng, out Vector3 _);
            if (rootAng > 180f) rootAng = 360f - rootAng;

            Debug.Log(LogPrefix + string.Format(
                "t={0:D3} '{1}' rootDriftDeg={2:F2} maxRelAngDeg={3:F2} on={4} maxRelPosCm={5:F1} on={6}",
                dt,
                v.vesselName,
                rootAng,
                maxAngDrift,
                maxAngDriftPart?.partInfo?.name ?? "-",
                maxPosDrift * 100f,
                maxPosDriftPart?.partInfo?.name ?? "-"));
        }
    }
}
