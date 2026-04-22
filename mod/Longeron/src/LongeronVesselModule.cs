// Per-vessel lifecycle: build a VesselScene when the vessel goes off-rails
// (physics active), release it when it goes on-rails (packed / on orbit).
//
// KSPAddon-injected VesselModule — KSP instantiates one per Vessel and
// calls the OnGoOffRails/OnGoOnRails handlers on scene transitions.

using Longeron.Integration;
using UnityEngine;

namespace Longeron
{
    public class LongeronVesselModule : VesselModule
    {
        const string LogPrefix = "[Longeron/Vessel] ";

        VesselScene _scene;

        protected override void OnStart()
        {
            // No-op at mod-start; the real work happens on GoOffRails.
        }

        public override void OnGoOffRails()
        {
            if (vessel == null) return;
            if (SceneRegistry.Instance.TryGet(vessel, out _)) return;

            _scene = VesselScene.Build(vessel);
            SceneRegistry.Instance.Register(_scene);

            // Kinematic takeover of rigidbodies: we own transforms from here on.
            // With isKinematic = true, Unity silently ignores any Rigidbody.AddForce
            // calls — so FlightIntegrator's direct gravity add (line 817 of
            // FlightIntegrator.cs) becomes a no-op for free, without needing a
            // patch. Force accumulators (part.force / .torque / .forces) are
            // zeroed by our Part.Add* prefixes returning false, so lines 818 / 819
            // / 823 also add zero. No FlightIntegrator patch required for this spike.
            foreach (var p in vessel.parts)
            {
                var rb = p.rb;
                if (rb == null) continue;
                rb.useGravity = false;
                rb.isKinematic = true;
                // Contact detection is query-based (see ContactDiscovery), not
                // event-based, so we don't need ContinuousSpeculative's swept
                // contact generation — Discrete is fine and avoids any
                // interaction with our MovePosition writeback.
                rb.collisionDetectionMode = CollisionDetectionMode.Discrete;
            }

            // Delete joints outright: destroy the ConfigurableJoint components
            // and null out the entries in PartJoint.joints. PartJoint.Joint (the
            // getter at joints[0]) then returns null. Stock code that does
            // `part.attachJoint.Joint.<member>` will NRE — patch as they surface.
            foreach (var p in vessel.parts)
            {
                var pj = p.attachJoint;
                if (pj == null || pj.joints == null) continue;
                for (int i = 0; i < pj.joints.Count; i++)
                {
                    var j = pj.joints[i];
                    if (j != null) UnityEngine.Object.Destroy(j);
                    pj.joints[i] = null;
                }
            }

            // Intra-vessel self-contact filtering is handled inside
            // ContactDiscovery via VesselScene.OwnColliders — no
            // Physics.IgnoreCollision pass needed (overlap queries don't honor
            // it anyway). No per-part MonoBehaviour relay either; discovery
            // is query-based.

            Debug.Log(LogPrefix + $"managing vessel '{vessel.vesselName}' with {_scene.BodyToPart.Length} bodies");
        }

        public override void OnGoOnRails()
        {
            if (_scene == null) return;
            Debug.Log(LogPrefix + $"releasing vessel '{vessel.vesselName}'");
            SceneRegistry.Instance.Unregister(vessel);

            // Restore rigidbodies to stock physics-friendly defaults.
            foreach (var p in vessel.parts)
            {
                var rb = p.rb;
                if (rb == null) continue;
                rb.isKinematic = false;
                // `useGravity` left false — FlightIntegrator owns gravity via
                // direct rb.AddForce; stock sets useGravity = false across the
                // board anyway.
            }
            _scene = null;
        }
    }
}
