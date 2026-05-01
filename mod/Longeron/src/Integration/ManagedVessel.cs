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

        // SubShape index → part_idx (BFS) mapping. Parallel array to
        // the SubShape stream sent in BodyCreate; populated alongside
        // the WriteSubShapeMap call in ColliderWalker. Used by
        // contact-report diag to translate native sub-shape ID to a
        // Part name for "which collider hit" attribution.
        public List<ushort> SubShapeMap { get; }
            = new List<ushort>();

        // Parallel to SubShapeMap: GameObject name of the source
        // Unity collider for each sub-shape. Diagnostic-only — used
        // by the contact log to identify which specific collider is
        // touching, beyond just the part. Indexed the same way.
        public List<string> SubShapeColliderNames { get; }
            = new List<string>();

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
