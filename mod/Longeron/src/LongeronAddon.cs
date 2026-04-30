// Longeron plugin entry point.
//
// Loaded once at MainMenu and persists for the session. Owns the
// Native.World lifetime (created on flight scene entry, disposed on
// exit), installs Harmony patches, and spawns the per-FixedUpdate
// SceneDriver.

using HarmonyLib;
using Longeron.Integration;
using Longeron.Native;
using UnityEngine;

namespace Longeron
{
    [KSPAddon(KSPAddon.Startup.MainMenu, true)]
    public class LongeronAddon : MonoBehaviour
    {
        const string LogPrefix = "[Longeron] ";
        const string HarmonyId = "dev.longeron.main";

        public static World ActiveWorld { get; private set; }

        public void Awake()
        {
            DontDestroyOnLoad(this);

            var harmony = new Harmony(HarmonyId);
            harmony.PatchAll(typeof(LongeronAddon).Assembly);

            string nativeVersion = World.NativeVersion;
            Debug.Log(LogPrefix + "loaded — Harmony patches installed. native bridge: " + nativeVersion);

            GameEvents.onLevelWasLoadedGUIReady.Add(OnLevelLoaded);
            GameEvents.onGameSceneLoadRequested.Add(OnSceneLoadRequested);
            GameEvents.onVesselWasModified.Add(OnVesselWasModified);
            GameEvents.onVesselCreate.Add(OnVesselCreated);
            GameEvents.onVesselDestroy.Add(OnVesselDestroyed);
            GameEvents.onVesselSOIChanged.Add(OnVesselSOIChanged);
            GameEvents.onRotatingFrameTransition.Add(OnRotatingFrameTransition);

            gameObject.AddComponent<LongeronSceneDriver>();
        }

        public void OnDestroy()
        {
            GameEvents.onLevelWasLoadedGUIReady.Remove(OnLevelLoaded);
            GameEvents.onGameSceneLoadRequested.Remove(OnSceneLoadRequested);
            GameEvents.onVesselWasModified.Remove(OnVesselWasModified);
            GameEvents.onVesselCreate.Remove(OnVesselCreated);
            GameEvents.onVesselDestroy.Remove(OnVesselDestroyed);
            GameEvents.onVesselSOIChanged.Remove(OnVesselSOIChanged);
            GameEvents.onRotatingFrameTransition.Remove(OnRotatingFrameTransition);
            DisposeWorld();
        }

        // Topology dirty signals. All three funnel into the reconciler;
        // it decides per-call whether to register, mutate, or tear
        // down based on the vessel's current state.
        void OnVesselWasModified(Vessel v) => TopologyReconciler.MarkDirty(v);
        void OnVesselCreated(Vessel v) => TopologyReconciler.MarkDirty(v);
        void OnVesselDestroyed(Vessel v) => TopologyReconciler.MarkDirty(v);

        // FloatingOrigin shift handling deliberately removed (Phase 3b
        // frame redesign). Jolt now operates in CB-fixed coordinates;
        // CbFrame.WorldToCb / CbToWorld at the bridge boundary absorb
        // FloatingOrigin shifts because mainBody.position and rb.position
        // shift by identical deltas — their difference is invariant.
        // See /Users/alx/.claude/plans/splendid-dancing-flute.md.

        // Active vessel transitions to a new mainBody. The Jolt world
        // is anchored to the previous mainBody's frame, so all body
        // positions / velocities / terrain are now expressed in the
        // wrong frame. Tear down + rebuild — the player is in
        // transition, a few hundred ms is acceptable.
        void OnVesselSOIChanged(GameEvents.HostedFromToAction<Vessel, CelestialBody> ev)
        {
            if (ev.host != FlightGlobals.ActiveVessel) return;
            if (ActiveWorld == null) return;
            Debug.Log(LogPrefix + $"SOI change for active vessel: {ev.from?.name} -> {ev.to?.name} — rebuilding world");
            RebuildWorld();
        }

        // Dominant body's inverseRotation flag flips (typically when
        // the active vessel crosses inverseRotThresholdAltitude — 70km
        // on Kerbin). Stock's OrbitPhysicsManager.setRotatingFrame
        // re-stamps every part's rb.velocity to match the new frame
        // (OrbitPhysicsManager.cs:337); our CbFrame branches on
        // body.inverseRotation, so the boundary-transform meaning
        // changes mid-tick. Rebuild to re-bake every Jolt body in
        // the new frame from the now-current Unity-world poses.
        void OnRotatingFrameTransition(GameEvents.HostTargetAction<CelestialBody, bool> ev)
        {
            if (ActiveWorld == null) return;
            if (ev.host != FlightGlobals.ActiveVessel?.mainBody) return;
            Debug.Log(LogPrefix + $"rotating-frame transition on {ev.host?.name} → inverseRotation={ev.target} — rebuilding world");
            RebuildWorld();
        }

        void RebuildWorld()
        {
            DisposeWorld();
            ActiveWorld = new World(LongeronConfig.Default);
            var driver = GetComponent<LongeronSceneDriver>();
            if (driver != null) driver.NotifyWorldCreated();
            Streamer.AttachToAllPQS();
            StaticSceneStreamer.MirrorAllPQSCities();
            foreach (var v in FlightGlobals.Vessels)
            {
                if (v == null) continue;
                if (v.packed) continue;
                var module = v.FindVesselModuleImplementing<LongeronVesselModule>();
                if (module != null) module.OnGoOffRails();
            }
        }

        // Flight scene entry. The World is created here; vessel
        // modules whose OnGoOffRails fires *after* this point register
        // into the live world automatically. Vessels that were already
        // unpacked when this hook fires (the typical scene-load case)
        // get a manual register sweep below.
        void OnLevelLoaded(GameScenes scene)
        {
            if (scene != GameScenes.FLIGHT) return;

            if (ActiveWorld != null) DisposeWorld();
            ActiveWorld = new World(LongeronConfig.Default);
            Debug.Log(LogPrefix + "world created for flight scene");

            var driver = GetComponent<LongeronSceneDriver>();
            if (driver != null) driver.NotifyWorldCreated();

            // Phase 3b: inject our PQSMod into every CelestialBody's
            // PQS so we receive OnQuadBuilt/Update/Destroy callbacks
            // for streaming terrain mirroring. Includes a sweep of
            // already-built quads so terrain near the launchpad lands
            // in Jolt without waiting for the player to move.
            Streamer.AttachToAllPQS();

            // Mirror PQSCity[2] static prefab geometry — KSC's
            // launchpad concrete, runway tarmac, building shells.
            // PQS terrain is the Kerbin sphere; this is the static
            // overlay sitting on top of it.
            StaticSceneStreamer.MirrorAllPQSCities();

            // Sweep already-unpacked vessels — VesselModule.OnGoOffRails
            // for these fired before ActiveWorld existed. Each vessel's
            // module instance can re-issue the register manually.
            foreach (var v in FlightGlobals.Vessels)
            {
                if (v == null) continue;
                if (v.packed) continue;  // OnGoOffRails will fire when it unpacks
                var module = v.FindVesselModuleImplementing<LongeronVesselModule>();
                if (module != null) module.OnGoOffRails();
            }
        }

        // Flight scene exit: tear down the world before vessels start
        // unloading. Vessel modules' OnGoOnRails may also fire on
        // scene change but we don't depend on the order; the world
        // dispose path is independent of the registry.
        void OnSceneLoadRequested(GameScenes nextScene)
        {
            if (nextScene != GameScenes.FLIGHT && ActiveWorld != null)
            {
                Debug.Log(LogPrefix + $"leaving flight for {nextScene} — disposing world");
                DisposeWorld();
            }
        }

        static void DisposeWorld()
        {
            // Tear down KSC static-mesh MonoBehaviours before the world
            // dies. The bodies themselves are gone with the World; this
            // sweep clears the components so a subsequent re-mirror on
            // RebuildWorld doesn't trip the "already-mirrored" idempotency
            // check on stale MonoBehaviours.
            StaticBody.DestroyAll();

            if (ActiveWorld != null)
            {
                ActiveWorld.Dispose();
                ActiveWorld = null;
            }
            SceneRegistry.Clear();
            TopologyReconciler.Clear();
            JoltPart.ClearPendingDestroys();
        }
    }
}
