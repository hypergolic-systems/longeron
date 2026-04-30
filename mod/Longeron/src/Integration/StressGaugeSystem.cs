// StressGaugeSystem — scene-level driver for Longeron's structural-
// stress UI overlay. Mirrors the shape of stock's
// TemperatureGaugeSystem (per-frame Update walks active vessel,
// instantiates gauges, drives their GaugeUpdate) but cooperates with
// our SceneRegistry instead of FlightGlobals.ActiveVessel directly,
// and creates two gauges per non-root part (force + torque channels).
//
// The visual prefab is reused from stock — TemperatureGaugeSystem
// exposes its temperatureGaugePrefab as a public field. We
// Instantiate, strip the inherited TemperatureGauge component (so
// stock thermal logic doesn't run), and add our StressGauge
// component which drives the slider against JoltPart's per-tick
// joint wrench.

using System.Collections.Generic;
using KSP.UI.Screens.Flight;
using UnityEngine;

namespace Longeron.Integration
{
    public sealed class StressGaugeSystem : MonoBehaviour
    {
        const string LogPrefix = "[Longeron/gauge] ";

        // Y-pixel offsets for the two gauges on the same joint.
        // Stacked vertically: torque slightly above force.
        const float kForceYOffset  = 0f;
        const float kTorqueYOffset = 16f;

        struct GaugeKey
        {
            public Part Part;
            public StressGauge.Channel Channel;
            public override int GetHashCode() =>
                (Part != null ? Part.GetInstanceID() : 0) ^ (int)Channel;
            public override bool Equals(object o) =>
                o is GaugeKey g && g.Part == Part && g.Channel == Channel;
        }

        readonly Dictionary<GaugeKey, StressGauge> _gauges =
            new Dictionary<GaugeKey, StressGauge>();
        readonly List<GaugeKey> _staleScratch = new List<GaugeKey>();
        readonly HashSet<GaugeKey> _liveScratch = new HashSet<GaugeKey>();

        void Update()
        {
            if (!HighLogic.LoadedSceneIsFlight) return;
            if (CameraManager.Instance == null
                || CameraManager.Instance.currentCameraMode
                   != CameraManager.CameraMode.Flight)
                return;

            var stockSys = TemperatureGaugeSystem.Instance;
            if (stockSys == null || stockSys.temperatureGaugePrefab == null)
                return;

            EnsureGaugesForManagedVessels(stockSys);

            foreach (var kv in _gauges)
            {
                if (kv.Value == null) continue;
                kv.Value.GaugeUpdate();
            }
        }

        void EnsureGaugesForManagedVessels(TemperatureGaugeSystem stockSys)
        {
            _liveScratch.Clear();

            foreach (var mv in SceneRegistry.Vessels)
            {
                if (mv.Vessel == null || mv.Vessel.packed) continue;
                foreach (var part in mv.Vessel.parts)
                {
                    if (part == null) continue;
                    // Root part has no joint to a parent — nothing to
                    // measure stress on.
                    if (part.parent == null) continue;

                    var fkey = new GaugeKey { Part = part, Channel = StressGauge.Channel.Force };
                    var tkey = new GaugeKey { Part = part, Channel = StressGauge.Channel.Torque };
                    _liveScratch.Add(fkey);
                    _liveScratch.Add(tkey);

                    if (!_gauges.ContainsKey(fkey))
                        _gauges[fkey] = CreateGauge(stockSys, part,
                            StressGauge.Channel.Force, kForceYOffset);
                    if (!_gauges.ContainsKey(tkey))
                        _gauges[tkey] = CreateGauge(stockSys, part,
                            StressGauge.Channel.Torque, kTorqueYOffset);
                }
            }

            // Drop gauges whose part is gone, on rails, or no longer
            // tracked. Stash keys first so we don't mutate while
            // iterating.
            _staleScratch.Clear();
            foreach (var kv in _gauges)
            {
                if (!_liveScratch.Contains(kv.Key)
                    || kv.Key.Part == null
                    || kv.Key.Part.gameObject == null
                    || kv.Value == null)
                {
                    _staleScratch.Add(kv.Key);
                }
            }
            for (int i = 0; i < _staleScratch.Count; ++i)
            {
                var k = _staleScratch[i];
                if (_gauges.TryGetValue(k, out var g) && g != null)
                    Destroy(g.gameObject);
                _gauges.Remove(k);
            }
        }

        StressGauge CreateGauge(TemperatureGaugeSystem stockSys, Part part,
                                 StressGauge.Channel ch, float yOffset)
        {
            // Clone the stock gauge prefab to inherit its visual style
            // (Slider, fill Image, RectTransform).
            var clone = Object.Instantiate(stockSys.temperatureGaugePrefab.gameObject);

            // Strip the inherited TemperatureGauge so stock's thermal
            // GaugeUpdate doesn't run when something somewhere calls
            // it. Our StressGauge drives the slider directly.
            var inherited = clone.GetComponent<TemperatureGauge>();
            if (inherited != null) Object.Destroy(inherited);

            // Parent under the stock manager's transform so we share
            // the same canvas hierarchy as stock gauges. Stock's own
            // sibling-index sort only touches its own list, so it
            // won't reorder our entries.
            clone.transform.SetParent(stockSys.transform, worldPositionStays: false);

            var sg = clone.AddComponent<StressGauge>();
            sg.Setup(part, ch, yOffset);
            return sg;
        }

        void OnDestroy()
        {
            foreach (var kv in _gauges)
            {
                if (kv.Value != null) Destroy(kv.Value.gameObject);
            }
            _gauges.Clear();
        }
    }
}
