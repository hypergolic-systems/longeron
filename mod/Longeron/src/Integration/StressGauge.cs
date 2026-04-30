// StressGauge — per-part-per-channel UI bar showing Phase 4 joint
// stress against the part's break thresholds. Mimics stock's
// TemperatureGauge UX (a small slider that floats next to the part,
// green-to-red gradient). Two gauges per part: one for force
// (combined tension + shear vs Part.breakingForce), one for torque
// (bending + torsion vs Part.breakingTorque).
//
// We don't subclass KSP's TemperatureGauge — its GaugeUpdate is hard-
// wired to thermal data, and stock manages its own gauge list per
// vessel via TemperatureGaugeSystem. Instead we instantiate the
// stock prefab (same visual style) for free, swap our component for
// the inherited TemperatureGauge, and drive the slider directly.
//
// Phase 4.5 ships always-on (threshold = 0) for debug visibility.
// Auto-hide can be wired later when joint break detection lands.

using KSP.UI.Util;
using UnityEngine;
using UnityEngine.UI;

namespace Longeron.Integration
{
    public sealed class StressGauge : MonoBehaviour
    {
        public enum Channel { Force, Torque }

        public Part Part;
        public Channel Mode;

        Slider _slider;
        Image  _fill;
        RectTransform _rt;

        // Vertical pixel offset so the two gauges on the same part
        // don't overlap. Force at 0, Torque shifted up.
        float _yOffset;

        public void Setup(Part p, Channel mode, float yOffsetPx)
        {
            Part = p;
            Mode = mode;
            _yOffset = yOffsetPx;
            _rt = GetComponent<RectTransform>();
            _slider = GetComponentInChildren<Slider>(includeInactive: true);
            if (_slider != null && _slider.fillRect != null)
            {
                _fill = _slider.fillRect.GetComponent<Image>();
            }
        }

        public void GaugeUpdate()
        {
            if (Part == null || _slider == null) { Hide(); return; }

            var jp = Part.gameObject.GetComponent<JoltPart>();
            if (jp == null || !jp.Handle.IsValid)
            {
                Hide();
                return;
            }

            float ratio = Mathf.Clamp01(ComputeRatio(jp));

            // Position via stock util — projects the joint anchor's
            // world position to canvas-local UI space. Gauges sit on
            // the joint they describe, not the part center, so the
            // visual stress reading lines up with where the parts
            // actually meet (radial decoupler joint, stack interface,
            // etc.).
            var cam = FlightCamera.fetch != null ? FlightCamera.fetch.mainCamera : null;
            if (cam == null) { Hide(); return; }

            bool onScreen = false;
            Vector3 uiPos = RectUtil.WorldToUISpacePos(
                GetJointWorldPos(Part), cam,
                MainCanvasUtil.MainCanvasRect, ref onScreen);
            if (!onScreen) { Hide(); return; }

            uiPos.y += _yOffset;
            _rt.localPosition = uiPos;

            Show();
            _slider.value = ratio;
            if (_fill != null) _fill.color = Color.Lerp(Color.green, Color.red, ratio);
        }

        void Show()
        {
            if (_slider != null && !_slider.gameObject.activeSelf)
                _slider.gameObject.SetActive(true);
        }

        void Hide()
        {
            if (_slider != null && _slider.gameObject.activeSelf)
                _slider.gameObject.SetActive(false);
        }

        // The joint between this part and its parent. Prefer the
        // ConfigurableJoint's connectedAnchor (transformed via the
        // parent's rb, which is what attachJoint.Joint.connectedBody
        // points at) — that's the precise pivot where the joint
        // resists stress. Fall back to part-parent CoM midpoint if
        // attachJoint isn't available, then to the part itself.
        static Vector3 GetJointWorldPos(Part part)
        {
            if (part.attachJoint != null && part.attachJoint.Joint != null
                && part.attachJoint.Joint.connectedBody != null)
            {
                var joint = part.attachJoint.Joint;
                return joint.connectedBody.transform.TransformPoint(joint.connectedAnchor);
            }
            if (part.parent != null && part.parent.transform != null)
            {
                return Vector3.Lerp(part.transform.position,
                                     part.parent.transform.position, 0.5f);
            }
            return part.transform.position;
        }

        float ComputeRatio(JoltPart jp)
        {
            if (Mode == Channel.Force)
            {
                // Effective stock-equivalent threshold (matches the live
                // ConfigurableJoint.breakForce stock would set on this
                // attachment, modulo Phase 4 joint-creation suppression).
                // Verified against stock: for our radial decouplers,
                // jp.EffectiveBreakForce == part.attachJoint.Joint.breakForce.
                // Falls back to raw Part.breakingForce for parts without an
                // attachJoint yet (vessel root or pre-couple).
                float bf = jp.EffectiveBreakForce > 0f
                    ? jp.EffectiveBreakForce : Part.breakingForce;
                if (bf <= 0f) return 0f;
                // Combined tension + shear; compression is excluded
                // (it doesn't break joints, see Phase 4.3 reasoning).
                float tens  = jp.LastJointTension;
                float shear = jp.LastJointShear;
                return Mathf.Sqrt(tens * tens + shear * shear) / bf;
            }
            else // Channel.Torque
            {
                float bt = jp.EffectiveBreakTorque > 0f
                    ? jp.EffectiveBreakTorque : Part.breakingTorque;
                if (bt <= 0f) return 0f;
                float tors = Mathf.Abs(jp.LastJointTorsion);
                float bend = jp.LastJointBending;
                return Mathf.Sqrt(tors * tors + bend * bend) / bt;
            }
        }
    }
}
