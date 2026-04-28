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
        public uint GroupId { get; }
        public List<Part> Parts { get; } = new List<Part>();
        public List<BodyHandle> BodyHandles { get; } = new List<BodyHandle>();
        public List<uint> ConstraintIds { get; } = new List<uint>();

        public ManagedVessel(Vessel vessel, uint groupId)
        {
            Vessel = vessel;
            GroupId = groupId;
        }

        public void Add(Part part, BodyHandle handle)
        {
            Parts.Add(part);
            BodyHandles.Add(handle);
            SceneRegistry.RegisterBodyHandle(handle, part);
            JoltBody.AttachTo(part, handle);
        }

        public void AddConstraint(uint constraintId)
        {
            ConstraintIds.Add(constraintId);
        }

        public bool TryGetHandle(Part part, out BodyHandle handle)
        {
            for (int i = 0; i < Parts.Count; i++)
            {
                if (ReferenceEquals(Parts[i], part))
                {
                    handle = BodyHandles[i];
                    return true;
                }
            }
            handle = BodyHandle.Invalid;
            return false;
        }
    }
}
