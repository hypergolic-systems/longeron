// Longeron plugin entry point.
//
// Loaded once at MainMenu and persists for the session. Installs Harmony
// patches and spawns the two session-level FixedUpdate drivers
// (LongeronScenePreTick early, LongeronSceneDriver late) that bracket
// stock physics for every managed vessel.

using HarmonyLib;
using UnityEngine;

namespace Longeron
{
    [KSPAddon(KSPAddon.Startup.MainMenu, true)]
    public class LongeronAddon : MonoBehaviour
    {
        const string LogPrefix = "[Longeron] ";
        const string HarmonyId = "dev.longeron.main";

        public void Awake()
        {
            DontDestroyOnLoad(this);

            var harmony = new Harmony(HarmonyId);
            harmony.PatchAll(typeof(LongeronAddon).Assembly);

            // Session-scoped drivers — one of each, persist for the session.
            gameObject.AddComponent<LongeronScenePreTick>();
            gameObject.AddComponent<LongeronSceneDriver>();

            Debug.Log(LogPrefix + "loaded — Harmony patches installed, drivers running");
        }
    }
}
