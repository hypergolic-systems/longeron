// Session-level FixedUpdate driver for all Longeron-managed vessel scenes.
//
// Relies on Unity script execution order to bracket stock physics:
//   - [DefaultExecutionOrder(-10000)] `LongeronScenePreTick`   (early hook)
//     zeros fExt and reads vessel.precalc.integrationAccel into the solver's
//     gravity field, so the ABA pass sees the same gravity stock does.
//   - [DefaultExecutionOrder(+10000)] `LongeronSceneDriver`    (late hook)
//     steps every managed scene and writes transforms back to rigidbodies
//     via MovePosition / MoveRotation.
//
// Stock and modded force-producers run their FixedUpdates between these
// two brackets. Their Part.AddForce* calls are intercepted by our Harmony
// hooks (see Patches/PartForceHooks.cs) and populate fExt per body.

using Longeron.Integration;
using Longeron.Physics;
using UnityEngine;

namespace Longeron
{
    [DefaultExecutionOrder(-10000)]
    public class LongeronScenePreTick : MonoBehaviour
    {
        public void FixedUpdate()
        {
            foreach (var scene in SceneRegistry.Instance.AllScenes)
            {
                scene.ResetExternalWrenches();
                // Spike milestone: zero gravity so we can demo thrust → rigid
                // motion without needing a contact solver for ground collision.
                // Next milestone replaces this with:
                //   var g = scene.Vessel.precalc.integrationAccel;
                // plus a real contact pass for OnCollisionStay events.
                scene.Scene.SetGravity(float3.zero);
            }
        }
    }

    [DefaultExecutionOrder(10000)]
    public class LongeronSceneDriver : MonoBehaviour
    {
        public void FixedUpdate()
        {
            float dt = Time.fixedDeltaTime;
            foreach (var scene in SceneRegistry.Instance.AllScenes)
            {
                EnsureKinematic(scene);
                // Seed rootVelocity from stock's orbital velocity once rb is
                // non-null and populated — deferred from scene-build time.
                scene.TrySeedRootVelocity();
                scene.Scene.Step(dt);
                WriteTransformsBack(scene);
            }
        }

        // VesselModule.OnGoOffRails can fire before part rigidbodies are
        // fully instantiated, so our one-shot takeover in the module may
        // silently skip parts. Idempotently reapply kinematic+CCD every
        // tick — cheap, and bulletproofs against timing races with
        // stock physics setup.
        static void EnsureKinematic(VesselScene scene)
        {
            for (int i = 0; i < scene.BodyToPart.Length; i++)
            {
                var rb = scene.BodyToPart[i].rb;
                if (rb == null) continue;
                if (!rb.isKinematic) rb.isKinematic = true;
                if (rb.useGravity) rb.useGravity = false;
                // Discrete (the kinematic default): no speculative push-out
                // that fights our writeback. We don't need collision events
                // until the contact solver lands.
                if (rb.collisionDetectionMode != CollisionDetectionMode.Discrete)
                    rb.collisionDetectionMode = CollisionDetectionMode.Discrete;
            }
        }

        static void WriteTransformsBack(VesselScene scene)
        {
            for (int i = 0; i < scene.BodyToPart.Length; i++)
            {
                var part = scene.BodyToPart[i];
                // Physicsless parts (no own rb) are parented in Unity to a
                // physics-parent; their transforms follow automatically. Skip.
                var rb = part.rb;
                if (rb == null) continue;
                var X = scene.Scene.GetWorldTransform(new BodyId(i));
                rb.MovePosition(new Vector3(X.translation.x, X.translation.y, X.translation.z));
                rb.MoveRotation(new Quaternion(X.rotation.x, X.rotation.y, X.rotation.z, X.rotation.w));
            }
        }
    }
}
