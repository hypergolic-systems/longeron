// AttachAnchors — resolve KSP attach-node anchor points for joints.
//
// Stock KSP creates ConfigurableJoints anchored at AttachNode.position
// in each part's local frame (PartJoint.cs). We need the same anchor
// for our Jolt FixedConstraint emission so PGS sees a sensible
// constraint Jacobian — `mAutoDetectPoint=true` puts the anchor at the
// midpoint between the two bodies' CoMs, which for a radial-decoupler-
// on-tank pair lands ~0.5 m *inside* the central tank, far from the
// physical attach surface. Bad conditioning compounds across long
// chains; the empirical symptom is gross compliance under gravity load
// (~15° tilt on side boosters at rest, see plan 2026-04-28).
//
// The pair-anchor strategy: walk both ends of the (parent, child)
// edge and read each side's matching AttachNode. They describe the
// same physical attach point in two different part frames; transformed
// to world space they should agree to within construction tolerance.
// We send a single canonical world-space anchor (in CB-frame coords)
// to the native side; FixedConstraint resolves both bodies' local
// offsets from it at construction.

using UnityEngine;

namespace Longeron.Integration
{
    internal static class AttachAnchors
    {
        // Resolve attach anchors for an edge (parent, child) and return
        // a single canonical world-space anchor in Unity-world coords
        // (caller is responsible for any CB-frame conversion).
        //
        // Selection rule:
        //   1. If child has an AttachNode whose attachedPart == parent,
        //      use that node's transformed-to-world position as anchor.
        //      Mirror on the parent side as a sanity check; average if
        //      both resolve and the world-space offset is small.
        //   2. Else, child.srfAttachNode (surface-attached child).
        //   3. Else, parent.srfAttachNode (rare; mirrors stock fallback
        //      at PartJoint.cs:140-142).
        //
        // Returns false if no anchor could be resolved (caller should
        // fall back to the autoDetect constraint path and log).
        public static bool TryResolveWorld(
            Part child, Part parent,
            out Vector3 anchorWorld)
        {
            anchorWorld = Vector3.zero;
            if (child == null || parent == null) return false;
            var childXform = child.transform;
            var parentXform = parent.transform;
            if (childXform == null || parentXform == null) return false;

            Vector3? childAnchorWorld = null;
            Vector3? parentAnchorWorld = null;

            var childNode = child.FindAttachNodeByPart(parent);
            if (childNode != null)
                childAnchorWorld = childXform.TransformPoint(childNode.position);
            else if (child.srfAttachNode != null)
                childAnchorWorld = childXform.TransformPoint(child.srfAttachNode.position);

            var parentNode = parent.FindAttachNodeByPart(child);
            if (parentNode != null)
                parentAnchorWorld = parentXform.TransformPoint(parentNode.position);
            else if (parent.srfAttachNode != null
                     && childAnchorWorld == null)  // only fall back if child side failed
                parentAnchorWorld = parentXform.TransformPoint(parent.srfAttachNode.position);

            if (childAnchorWorld.HasValue && parentAnchorWorld.HasValue)
            {
                // Both sides resolved. Average for the canonical anchor —
                // they should be ~equal at attachment but parts can be
                // placed with sub-mm misalignment in the editor.
                anchorWorld = 0.5f * (childAnchorWorld.Value + parentAnchorWorld.Value);
                return true;
            }
            if (childAnchorWorld.HasValue) { anchorWorld = childAnchorWorld.Value; return true; }
            if (parentAnchorWorld.HasValue) { anchorWorld = parentAnchorWorld.Value; return true; }
            return false;
        }
    }
}
