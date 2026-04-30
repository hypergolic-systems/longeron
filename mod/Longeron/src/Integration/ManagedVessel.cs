// Per-vessel state held by SceneRegistry. Single-body model: one Jolt
// body per vessel. Bodies dict + ConstraintEdges from the multi-body
// era are gone.
//
// Tracks:
//   - The vessel's BodyHandle (single).
//   - Per-part offsets from the vessel root (in root-local coords) so
//     the scene driver can derive each part's Unity-world pose from
//     the body's pose.
//   - The set of parts contributing to the body, used for change
//     detection during reconcile.

using System.Collections.Generic;
using Longeron.Native;
using UnityEngine;

namespace Longeron.Integration
{
    internal sealed class ManagedVessel
    {
        public Vessel Vessel { get; }
        public uint GroupId { get; }

        // Vessel-level Jolt body handle. Invalid until first BodyCreate.
        public BodyHandle Body;

        // Each part's offset from the vessel root, captured at
        // BodyCreate time and held constant until the next rebuild.
        public struct PartOffset
        {
            public Vector3 LocalPos;     // root-local frame
            public Quaternion LocalRot;  // root-local frame
        }

        public Dictionary<Part, PartOffset> PartOffsets { get; }
            = new Dictionary<Part, PartOffset>();

        // BFS-order part list (parallel to the native side's vessel
        // tree indices). PartsByIdx[i] is the Part at tree index i.
        // Populated by TopologyReconciler.EmitVesselTree alongside
        // the VesselTreeUpdate record. Used by SceneDriver to map
        // incoming JointWrench records back to a Part for stashing
        // on its JoltPart.
        public List<Part> PartsByIdx { get; }
            = new List<Part>();

        // Last-seen part list, used to detect topology changes that
        // require a rebuild (decouple, dock, joint break, fairing
        // eject, structural failure).
        public HashSet<Part> LastParts { get; }
            = new HashSet<Part>();

        // Per-vessel mass aggregate (sum of part.mass +
        // part.GetResourceMass over LastParts). Reconciler / scene
        // driver compares against the live sum and emits a single
        // MassUpdate record when it drifts past threshold.
        public float LastMass;

        public ManagedVessel(Vessel vessel, uint groupId)
        {
            Vessel = vessel;
            GroupId = groupId;
        }
    }
}
