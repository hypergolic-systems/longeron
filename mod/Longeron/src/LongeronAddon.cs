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

            gameObject.AddComponent<LongeronSceneDriver>();
        }

        public void OnDestroy()
        {
            GameEvents.onLevelWasLoadedGUIReady.Remove(OnLevelLoaded);
            GameEvents.onGameSceneLoadRequested.Remove(OnSceneLoadRequested);
            DisposeWorld();
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
        }
    }
}
