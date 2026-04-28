// Global table of vessels currently managed by the Jolt bridge.
//
// LongeronVesselModule.OnGoOffRails registers a vessel here when its
// physics activate; OnGoOnRails removes it. The session driver
// iterates the registry each FixedUpdate to drive bridge inputs.
//
// Body handle IDs are minted from a per-process monotonic counter so
// they're unique across vessels and through topology mutations.
// BodyHandle 0 is reserved as Invalid by Longeron.Native; we start
// at 1.

using System.Collections.Generic;
using Longeron.Native;
using UnityEngine;

namespace Longeron.Integration
{
    internal static class SceneRegistry
    {
        static readonly Dictionary<Vessel, ManagedVessel> _vessels =
            new Dictionary<Vessel, ManagedVessel>();

        // BodyHandle.Id → Part lookup, populated as parts register.
        // Used by the driver to dispatch per-body output records (BodyPose,
        // ContactReport) back to the right Unity rigidbody. Ground bodies
        // and other non-Part-backed handles are absent from this map; the
        // driver skips them silently.
        static readonly Dictionary<uint, Part> _bodyToPart =
            new Dictionary<uint, Part>();

        // Reserved range:
        //   1                — first synthetic ground (Phase 1.5)
        //   2..              — vessel parts and any future ground bodies
        // The synthetic-ground id is fixed so the driver can reference
        // it without a lookup; vessel parts get sequentially-allocated
        // ids from there on.
        public const uint SyntheticGroundBodyId = 1;
        static uint _nextBodyId = 2;

        public static IEnumerable<ManagedVessel> Vessels => _vessels.Values;

        public static int Count => _vessels.Count;

        public static BodyHandle MintBodyHandle()
        {
            return new BodyHandle(_nextBodyId++);
        }

        public static ManagedVessel Register(Vessel vessel)
        {
            if (_vessels.TryGetValue(vessel, out var existing)) return existing;
            var mv = new ManagedVessel(vessel);
            _vessels[vessel] = mv;
            return mv;
        }

        public static bool Unregister(Vessel vessel, out ManagedVessel mv)
        {
            if (!_vessels.TryGetValue(vessel, out mv)) return false;
            foreach (var h in mv.BodyHandles)
                _bodyToPart.Remove(h.Id);
            _vessels.Remove(vessel);
            return true;
        }

        public static bool TryGet(Vessel vessel, out ManagedVessel mv) =>
            _vessels.TryGetValue(vessel, out mv);

        // Called by ManagedVessel.Add to keep _bodyToPart in sync.
        public static void RegisterBodyHandle(BodyHandle handle, Part part)
        {
            _bodyToPart[handle.Id] = part;
        }

        public static bool TryGetPart(uint bodyId, out Part part) =>
            _bodyToPart.TryGetValue(bodyId, out part);

        public static void Clear()
        {
            _vessels.Clear();
            _bodyToPart.Clear();
        }
    }
}
