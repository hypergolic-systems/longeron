// Global table of vessels currently managed by the Jolt bridge, plus
// the id-minting counters.
//
// Vessel registration happens via TopologyReconciler (the per-vessel
// reconciler creates a ManagedVessel on first sight and tears it down
// on vessel destruction). OnGoOnRails calls Unregister directly for
// the synchronous "leaving physics" case.
//
// Body / part lookup is NOT here — JoltPart.TryGet on the static
// component-keyed dict gives bodyId → JoltPart → Part. This module
// only owns the vessel-level grouping.
//
// Body handle IDs are minted from a per-process monotonic counter so
// they're unique across vessels and through topology mutations.
// BodyHandle 0 is reserved as Invalid by Longeron.Native; we start
// at 1.

using System.Collections.Generic;
using Longeron.Native;

namespace Longeron.Integration
{
    internal static class SceneRegistry
    {
        static readonly Dictionary<Vessel, ManagedVessel> _vessels =
            new Dictionary<Vessel, ManagedVessel>();

        // Body handle ids start at 1 — BodyHandle 0 is reserved as
        // Invalid by Longeron.Native. All bodies (vessel parts, KSC
        // static colliders, PQS terrain quads) draw from this single
        // monotonic counter.
        static uint _nextBodyId = 1;

        // Constraint IDs are minted from a separate counter so the
        // namespaces don't collide. The native side keys constraints
        // independently of bodies.
        static uint _nextConstraintId = 1;

        // Per-vessel collision group IDs. 0 is reserved for "no
        // group" (terrain / synthetic ground / global static
        // geometry). Each vessel that registers gets a fresh ID.
        static uint _nextGroupId = 1;

        public static IEnumerable<ManagedVessel> Vessels => _vessels.Values;

        public static int Count => _vessels.Count;

        public static BodyHandle MintBodyHandle()
        {
            return new BodyHandle(_nextBodyId++);
        }

        public static uint MintConstraintId()
        {
            return _nextConstraintId++;
        }

        public static uint MintVesselGroupId()
        {
            return _nextGroupId++;
        }

        public static ManagedVessel Register(Vessel vessel)
        {
            if (_vessels.TryGetValue(vessel, out var existing)) return existing;
            var mv = new ManagedVessel(vessel, MintVesselGroupId());
            _vessels[vessel] = mv;
            return mv;
        }

        public static bool Unregister(Vessel vessel, out ManagedVessel mv)
        {
            if (!_vessels.TryGetValue(vessel, out mv)) return false;
            _vessels.Remove(vessel);
            return true;
        }

        public static bool TryGet(Vessel vessel, out ManagedVessel mv) =>
            _vessels.TryGetValue(vessel, out mv);

        public static void Clear()
        {
            _vessels.Clear();
        }
    }
}
