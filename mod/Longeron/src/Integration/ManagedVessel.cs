// Per-vessel state held by SceneRegistry. Tracks the Part ↔ BodyHandle
// mapping so the driver can find the bodies for SetKinematicPose each
// tick, and so OnGoOnRails can queue BodyDestroy records for the
// correct handles.

using System.Collections.Generic;
using Longeron.Native;

namespace Longeron.Integration
{
    internal sealed class ManagedVessel
    {
        public Vessel Vessel { get; }
        public List<Part> Parts { get; } = new List<Part>();
        public List<BodyHandle> BodyHandles { get; } = new List<BodyHandle>();

        public ManagedVessel(Vessel vessel) { Vessel = vessel; }

        public void Add(Part part, BodyHandle handle)
        {
            Parts.Add(part);
            BodyHandles.Add(handle);
        }
    }
}
