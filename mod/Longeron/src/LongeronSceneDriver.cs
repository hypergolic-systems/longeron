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

            float fExtMag = 0f;
            for (int i = 0; i < scene.BodyToPart.Length; i++)
            {
                var f = scene.Scene.Body.fExt[i];
                fExtMag += Physics.math.length(f.angular) + Physics.math.length(f.linear);
            }

            // Everything the navball / aero / heating read:
            //   vessel.velocityD = rb.GetPointVelocity(CoM) + FrameVelocity
            //   vessel.srf_velocity = velocityD - surface rotation at vessel
            //   vessel.obt_velocity = velocityD mapped into the orbit's frame
            var pointV = rb != null ? rb.GetPointVelocity(vessel.CoMD) : Vector3.zero;
            Debug.Log(string.Format(
                "[Longeron/trace] t={0} v={1} rb_pos=({2:F1},{3:F1},{4:F1}) rb_vel=({5:F2},{6:F2},{7:F2}) " +
                "rb_pv=({8:F2},{9:F2},{10:F2}) fv=({11:F1},{12:F1},{13:F1}) " +
                "rv_w=({14:F2},{15:F2},{16:F2}) alt={17:F0} " +
                "srf_v={18:F2} obt_v={19:F2} vel_d={20:F2} fext={21:F1}",
                _tick, vessel.vesselName,
                rb != null ? rb.position.x : 0f, rb != null ? rb.position.y : 0f, rb != null ? rb.position.z : 0f,
                rb != null ? rb.velocity.x : 0f, rb != null ? rb.velocity.y : 0f, rb != null ? rb.velocity.z : 0f,
                pointV.x, pointV.y, pointV.z,
                (float)fv.x, (float)fv.y, (float)fv.z,
                rvWorld.x, rvWorld.y, rvWorld.z,
                vessel.altitude,
                vessel.srf_velocity.magnitude,
                vessel.obt_velocity.magnitude,
                vessel.velocityD.magnitude,
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

            // Set the root rigidbody's velocity explicitly. KSP computes
            // vessel.velocityD as rootPart.rb.GetPointVelocity(CoM) +
            // Krakensbane.FrameVelocity — so for the navball / aero / etc.
            // to read stable values, rb.velocity must be a deterministic
            // per-tick write rather than whatever Unity happens to have
            // inferred from MovePosition deltas (which also collides with
            // Krakensbane's ChangeWorldVelocity mid-tick).
            if (scene.BodyToPart.Length > 0)
            {
                var rootRb = scene.BodyToPart[0].rb;
                if (rootRb != null)
                {
                    var rv = scene.Scene.Body.rootVelocity;
                    var rot = scene.Scene.Body.rootPose.rotation;
                    float3 vWorld = Physics.math.mul(rot, rv.linear);
                    float3 omWorld = Physics.math.mul(rot, rv.angular);
                    rootRb.velocity = new Vector3(vWorld.x, vWorld.y, vWorld.z);
                    rootRb.angularVelocity = new Vector3(omWorld.x, omWorld.y, omWorld.z);
                }
            }
        }
    }
}
