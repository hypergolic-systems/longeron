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
            GameEvents.onFloatingOriginShift.Add(OnFloatingOriginShift);
            GameEvents.onVesselWasModified.Add(OnVesselWasModified);
            GameEvents.onVesselCreate.Add(OnVesselCreated);
            GameEvents.onVesselDestroy.Add(OnVesselDestroyed);

            gameObject.AddComponent<LongeronSceneDriver>();
        }

        public void OnDestroy()
        {
            GameEvents.onLevelWasLoadedGUIReady.Remove(OnLevelLoaded);
            GameEvents.onGameSceneLoadRequested.Remove(OnSceneLoadRequested);
            GameEvents.onFloatingOriginShift.Remove(OnFloatingOriginShift);
            GameEvents.onVesselWasModified.Remove(OnVesselWasModified);
            GameEvents.onVesselCreate.Remove(OnVesselCreated);
            GameEvents.onVesselDestroy.Remove(OnVesselDestroyed);
            DisposeWorld();
        }

        // Topology dirty signals. All three funnel into the reconciler;
        // it decides per-call whether to register, mutate, or tear
        // down based on the vessel's current state.
        void OnVesselWasModified(Vessel v) => TopologyReconciler.MarkDirty(v);
        void OnVesselCreated(Vessel v) => TopologyReconciler.MarkDirty(v);
        void OnVesselDestroyed(Vessel v) => TopologyReconciler.MarkDirty(v);

        // Krakensbane / FloatingOrigin shifted Unity's world origin.
        // Translate every Jolt body by the same delta KSP applied to
        // active flying vessel rigidbodies — that's just the
        // Krakensbane component (`offset`), NOT
        // `offsetNonKrakensbane` (= `offset + nonKrakensbane`). See
        // ~/dev/ksp-reference/source/Assembly-CSharp/FloatingOrigin.cs:
        //   line 311: vessel.SetPosition(transform.position - vector3d2)
        //   line 252: vector3d2 = offset (for non-packed, non-landed)
        //   line 257: vector3d2 = offsetNonKrakensbane (for packed/landed)
        // We over-shifted by `nonKrakensbane` per event before, which
        // visibly desynced the vessel from the camera's view as
        // FloatingOrigin shifts accumulated. Synthetic ground and
        // future static geometry technically want
        // `offsetNonKrakensbane` to track terrain — Phase 3
        // territory. For now Phase 2.4 covers active vessels.
        void OnFloatingOriginShift(Vector3d offset, Vector3d nonKrakensbane)
        {
            if (ActiveWorld == null) return;
            Vector3d delta = -offset;
            ActiveWorld.Input.WriteShiftWorld(delta.x, delta.y, delta.z);
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
            if (ActiveWorld != null)
            {
                ActiveWorld.Dispose();
                ActiveWorld = null;
            }
            SceneRegistry.Clear();
            TopologyReconciler.Clear();
            JoltBody.ClearPendingDestroys();
        }
    }
}
