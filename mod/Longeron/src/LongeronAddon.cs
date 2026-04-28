// Longeron plugin entry point.
//
// Phase 1.5 stub: loads at MainMenu, installs Harmony patches (none yet),
// logs a load line. The real KSP integration (World lifetime, vessel
// module, scene driver, collider walker) lands in the next commit.

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

            string version = Native.World.NativeVersion;
            Debug.Log(LogPrefix + "loaded — Harmony patches installed. native bridge: " + version);
        }
    }
}
