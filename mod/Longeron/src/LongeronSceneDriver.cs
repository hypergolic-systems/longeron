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
                // Isolating the Krakensbane test: gravity stays zero until
                // the contact solver lands. With gravity on we'd fall
                // through the launchpad before reaching Krakensbane's
                // activation threshold. Flipping this is a one-liner once
                // contacts work.
                scene.Scene.SetGravity(float3.zero);
            }
        }
    }

    [DefaultExecutionOrder(10000)]
    public class LongeronSceneDriver : MonoBehaviour
    {
        long _tick;

        readonly System.Collections.Generic.List<Vessel> _dead = new System.Collections.Generic.List<Vessel>();

        public void FixedUpdate()
        {
            float dt = Time.fixedDeltaTime;
            _tick++;
            _dead.Clear();

            foreach (var scene in SceneRegistry.Instance.AllScenes)
            {
                if (scene.Vessel == null || scene.Vessel.state == Vessel.State.DEAD)
                {
                    _dead.Add(scene.Vessel);
                    continue;
                }
                EnsureKinematic(scene);
                scene.TrySeedRootVelocity();
                // Absolute position is stock's domain — FloatingOrigin can
                // teleport rb.position, Krakensbane can translate it per
                // tick. Snap rootPose.translation to rb.position at the
                // start of each tick and let our solver contribute only
                // velocity/acceleration dynamics from there.
                scene.SyncRootPoseFromRigidbody();
                // Krakensbane velocity-drain: if FrameVelocity changed this
                // tick, drain the same delta from rootVelocity.
                scene.ApplyKrakensbaneStep(dt);
                scene.Scene.Step(dt);
                WriteTransformsBack(scene);
                Trace(scene);
            }

            // Reap dead vessels — their VesselModule.OnGoOnRails won't fire
            // if the vessel exploded, so we clean up on our own.
            foreach (var v in _dead)
            {
                if (v != null) SceneRegistry.Instance.Unregister(v);
            }
        }

        void Trace(VesselScene scene)
        {
            var vessel = scene.Vessel;
            var root = scene.BodyToPart[0];
            var rb = root != null ? root.rb : null;
            var pose = scene.Scene.Body.rootPose;
            var rv = scene.Scene.Body.rootVelocity;
            float3 rvWorld = Physics.math.mul(pose.rotation, rv.linear);
            Vector3d fv = Krakensbane.GetFrameVelocity();

            // Accumulate |fExt| across the vessel to detect thrust input.
            float fExtMag = 0f;
            for (int i = 0; i < scene.BodyToPart.Length; i++)
            {
                var f = scene.Scene.Body.fExt[i];
                fExtMag += Physics.math.length(f.angular) + Physics.math.length(f.linear);
            }

            Debug.Log(string.Format(
                "[Longeron/trace] t={0} v={1} rb_pos=({2:F1},{3:F1},{4:F1}) rb_vel=({5:F1},{6:F1},{7:F1}) " +
                "fv=({8:F1},{9:F1},{10:F1}) root_pos=({11:F1},{12:F1},{13:F1}) " +
                "root_v_w=({14:F1},{15:F1},{16:F1}) alt={17:F0} obt_v={18:F1} fext={19:F1}",
                _tick, vessel.vesselName,
                rb != null ? rb.position.x : 0f, rb != null ? rb.position.y : 0f, rb != null ? rb.position.z : 0f,
                rb != null ? rb.velocity.x : 0f, rb != null ? rb.velocity.y : 0f, rb != null ? rb.velocity.z : 0f,
                (float)fv.x, (float)fv.y, (float)fv.z,
                pose.translation.x, pose.translation.y, pose.translation.z,
                rvWorld.x, rvWorld.y, rvWorld.z,
                vessel.altitude, vessel.obt_velocity.magnitude,
                fExtMag));
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
