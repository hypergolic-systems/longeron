// Session-level registry of Longeron-managed vessel scenes.
//
// Two maps:
//   - Vessel  → VesselScene: for lifecycle (GoOffRails/GoOnRails, iteration in
//     the FixedUpdate driver).
//   - Part    → (VesselScene, BodyId): for Harmony force hooks, which receive
//     a Part and need fast lookup of the managed body.

using System.Collections.Generic;
using Longeron.Physics;

namespace Longeron.Integration
{
    internal sealed class SceneRegistry
    {
        public static SceneRegistry Instance { get; } = new SceneRegistry();

        readonly Dictionary<Vessel, VesselScene> byVessel = new Dictionary<Vessel, VesselScene>();
        readonly Dictionary<Part, VesselScene>   byPart   = new Dictionary<Part, VesselScene>();

        SceneRegistry() { }

        public int Count => byVessel.Count;

        public void Register(VesselScene scene)
        {
            byVessel[scene.Vessel] = scene;
            foreach (var kv in scene.PartToBody)
                byPart[kv.Key] = scene;
        }

        public void Unregister(Vessel vessel)
        {
            if (!byVessel.TryGetValue(vessel, out var scene)) return;
            byVessel.Remove(vessel);
            foreach (var kv in scene.PartToBody)
                byPart.Remove(kv.Key);
        }

        public bool TryGet(Vessel vessel, out VesselScene scene) =>
            byVessel.TryGetValue(vessel, out scene);

        public bool TryGet(Part part, out VesselScene scene, out BodyId id)
        {
            if (byPart.TryGetValue(part, out scene))
            {
                id = scene.PartToBody[part];
                return true;
            }
            id = BodyId.None;
            return false;
        }

        public IEnumerable<VesselScene> AllScenes => byVessel.Values;
    }
}
