// Per-vessel state held by SceneRegistry. Tracks the Part ↔ BodyHandle
// mapping and the (parent, child) ↔ ConstraintId mapping so the
// reconciler can diff against the vessel's current part tree each
// FixedUpdate and emit minimal mutations.

using System.Collections.Generic;
using Longeron.Native;

namespace Longeron.Integration
{
    internal sealed class ManagedVessel
    {
        public Vessel Vessel { get; }
        public uint GroupId { get; }

        // Part → its Jolt body. Keyed for O(1) reconciliation diffs.
        public Dictionary<Part, BodyHandle> Bodies { get; }
            = new Dictionary<Part, BodyHandle>();

        // (parent, child) → constraint id. Tuple keys let the
        // reconciler detect re-parenting (a child whose parent
        // changed mid-flight needs the old edge destroyed and the new
        // one created), and tag-by-edge means we don't need a
        // separate part-pair → constraint lookup.
        public Dictionary<(Part Parent, Part Child), uint> ConstraintEdges { get; }
            = new Dictionary<(Part, Part), uint>();

        public ManagedVessel(Vessel vessel, uint groupId)
        {
            Vessel = vessel;
            GroupId = groupId;
        }

        public IEnumerable<BodyHandle> BodyHandles => Bodies.Values;
        public IEnumerable<uint> ConstraintIds => ConstraintEdges.Values;

        public void AddBody(Part part, BodyHandle handle)
        {
            // The reverse (handle → JoltBody → Part) lookup lives on
            // JoltBody itself; ManagedVessel just records its
            // membership in this vessel's body set.
            Bodies[part] = handle;
        }

        public void RemoveBody(Part part)
        {
            // We only relinquish vessel membership here. The JoltBody
            // MonoBehaviour stays alive on the GameObject — another
            // ManagedVessel may claim it (decouple → new vessel) or
            // Unity will eventually destroy the component, at which
            // point JoltBody.OnDestroy queues the BodyDestroy.
            Bodies.Remove(part);
        }

        public void AddEdge(Part parent, Part child, uint constraintId)
        {
            ConstraintEdges[(parent, child)] = constraintId;
        }

        public bool RemoveEdge(Part parent, Part child, out uint constraintId)
        {
            var key = (parent, child);
            if (ConstraintEdges.TryGetValue(key, out constraintId))
            {
                ConstraintEdges.Remove(key);
                return true;
            }
            return false;
        }

        public bool TryGetHandle(Part part, out BodyHandle handle) =>
            Bodies.TryGetValue(part, out handle);
    }
}
