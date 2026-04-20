// Longeron plugin entry point.
//
// Loaded once at MainMenu and persists for the session. Owns the
// scene-level wiring that the per-vessel VesselModule + Harmony
// patches hook into.

using UnityEngine;

namespace Longeron
{
    [KSPAddon(KSPAddon.Startup.MainMenu, true)]
    public class LongeronAddon : MonoBehaviour
    {
        private const string LogPrefix = "[Longeron] ";

        public void Awake()
        {
            DontDestroyOnLoad(this);
            Debug.Log(LogPrefix + "loaded");
        }
    }
}
