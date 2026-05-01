using System;
using System.Runtime.InteropServices;

namespace Longeron.Native
{
    /// <summary>
    /// One-byte tag identifying each typed record in the input/output
    /// streams. Mirrors <c>longeron::RecordType</c> in
    /// <c>native/src/records.h</c>. Schema additions append new values;
    /// existing values do not change. Both sides assert their schema
    /// version matches at world-create time.
    /// </summary>
    public enum RecordType : byte
    {
        None              = 0,
        ForceDelta        = 1,
        MassUpdate        = 2,
        BodyCreate        = 3,
        BodyDestroy       = 4,
        ConstraintCreate  = 5,
        ConstraintDestroy = 6,
        SetGravity        = 7,
        SetKinematicPose  = 8,

        ShiftWorld        = 9,
        SetBodyGroup      = 10,
        ConstraintCreateFixedAt = 11,
        ForceAtPosition   = 12,
        VesselTreeUpdate  = 13,
        SubShapeMap       = 14,

        // Output-only:
        BodyPose          = 64,
        ContactReport     = 65,
        RneaSummary       = 66,
        JointWrench       = 67,
        PartPose          = 68,
        BodyMassDiag      = 69,
        AbaPartDiag       = 70,
    }

    /// <summary>
    /// Mirrors <c>longeron::BodyType</c>. Same values as
    /// <c>JPH::EMotionType</c>.
    /// </summary>
    public enum BodyType : byte
    {
        Static    = 0,
        Kinematic = 1,
        Dynamic   = 2,
    }

    /// <summary>
    /// Mirrors <c>longeron::ShapeKind</c>. Phase 1.5 supports Box,
    /// Sphere, ConvexHull. Phase 2+ extends with Capsule, Cylinder,
    /// Mesh.
    /// </summary>
    public enum ShapeKind : byte
    {
        Box          = 0,
        Sphere       = 1,
        ConvexHull   = 2,
        TriangleMesh = 3,
    }

    /// <summary>
    /// Wire-level cap on vertices passed to a single ConvexHull
    /// sub-shape. Mirrors <c>longeron::kMaxConvexHullVertices</c>.
    /// Caller is expected to pre-simplify large meshes; the bridge
    /// truncates with a warning if exceeded.
    /// </summary>
    public static class ShapeLimits
    {
        public const int MaxConvexHullVertices = 256;
        public const int MaxMeshTriangles = 4096;
    }

    /// <summary>
    /// Mirrors <c>longeron::Layer</c>. Phase 2 will partition Kinematic
    /// into per-vessel layers for self-collision filtering.
    /// </summary>
    public enum Layer : byte
    {
        Static    = 0,
        Kinematic = 1,
    }

    /// <summary>
    /// Mirrors <c>longeron::ConstraintKind</c>. Phase 2.2 supports
    /// Fixed (rigid weld); compliant SixDOF lands later.
    /// </summary>
    public enum ConstraintKind : byte
    {
        Fixed = 0,
    }

    /// <summary>
    /// Append-only writer for the input record stream. The buffer is
    /// allocated once via <see cref="Marshal.AllocHGlobal"/> and reused
    /// every tick — <see cref="Reset"/> clears the length without
    /// freeing memory. No managed allocations on the hot path.
    /// </summary>
    public sealed unsafe class InputBuffer : IDisposable
    {
        private byte* _ptr;
        private int   _capacity;
        private int   _len;

        public InputBuffer(int capacityBytes)
        {
            if (capacityBytes <= 0) throw new ArgumentOutOfRangeException(nameof(capacityBytes));
            _ptr      = (byte*)Marshal.AllocHGlobal(capacityBytes);
            _capacity = capacityBytes;
            _len      = 0;
        }

        public byte* Pointer  => _ptr;
        public int   Length   => _len;
        public int   Capacity => _capacity;

        public void Reset() { _len = 0; }

        public void Dispose()
        {
            if (_ptr != null)
            {
                Marshal.FreeHGlobal((IntPtr)_ptr);
                _ptr = null;
            }
        }

        // -- Record writers -----------------------------------------------
        //
        // BodyCreate is variable-length: a fixed prefix (user_id,
        // body_type, layer, group_id, body pose, mass, shape_count)
        // followed by shape_count sub-shape records. Each sub-shape
        // carries its own local transform (relative to the body's
        // frame) plus the shape kind and kind-specific params.
        //
        // Wire layout for the BodyCreate record:
        //   tag(1) + user_id(4) + body_type(1) + layer(1) + group_id(4)
        //          + body_pos(double3=24) + body_rot(float4=16) + mass(4)
        //          + lin_v(float3=12) + ang_v(float3=12)
        //          + shape_count(u8)
        //          + shape_count × {
        //              sub_pos(float3=12) + sub_rot(float4=16) + kind(u8)
        //              + density(4)
        //              + kind-specific params (Box: 12, Sphere: 4,
        //                ConvexHull: 4 + 12·N)
        //            }
        //
        // lin_v / ang_v are in CB-frame, applied as
        // BodyCreationSettings.mLinearVelocity / mAngularVelocity. Used
        // to seed dynamic vessel bodies on goOffRails (unpack from
        // rails); static / kinematic ignore (kinematic motion comes
        // from SetKinematicPose).
        //
        // density (per sub-shape, tonnes/m³ in KSP convention) lets
        // each part contribute its real mass distribution to the
        // compound's auto-computed CoM and inertia. C# computes
        // density = part_mass / part_total_volume so all sub-shapes of
        // a single part share one density; CompoundShape's
        // GetMassProperties() aggregates correctly via parallel-axis.
        // Sentinel density == 0 means "use Jolt default 1000" (legacy
        // / convenience-wrapper path; mass on BodyCreate is then taken
        // as the explicit override via CalculateInertia).
        //
        // group_id semantics: 0 = collide with everything (terrain,
        // synthetic ground, anything outside any vessel). Non-zero =
        // bodies sharing the same group_id skip collision with each
        // other (intra-vessel filtering).
        //
        // Public API:
        //   - BeginBodyCreate(...) writes the fixed prefix.
        //   - AppendShape{Box,Sphere,ConvexHull} appends one sub-shape.
        //   - Caller must know the shape_count up front and call
        //     Append exactly that many times. There is no End/finalize
        //     — once the count and all sub-shapes are written, the
        //     record is complete.
        //
        // Convenience single-shape writers (WriteBodyCreate{Box,Sphere,
        // ConvexHull}) wrap the above for the common one-collider case.

        /// <summary>
        /// Write the BodyCreate record's fixed prefix (everything up
        /// to and including <paramref name="shapeCount"/>). Caller
        /// then appends <paramref name="shapeCount"/> sub-shapes via
        /// <see cref="AppendShapeBox"/> / <see cref="AppendShapeSphere"/>
        /// / <see cref="AppendShapeConvexHull"/>.
        /// </summary>
        public void BeginBodyCreate(
            BodyHandle body, BodyType bodyType, Layer layer,
            double posX, double posY, double posZ,
            float rotX, float rotY, float rotZ, float rotW,
            float mass, byte shapeCount,
            float linVx = 0f, float linVy = 0f, float linVz = 0f,
            float angVx = 0f, float angVy = 0f, float angVz = 0f,
            uint groupId = 0)
        {
            const int kPrefix = 1 /*tag*/ + 4 /*user_id*/ + 1 /*body_type*/ + 1 /*layer*/
                              + 4 /*group_id*/
                              + 24 /*pos*/ + 16 /*rot*/ + 4 /*mass*/
                              + 12 /*lin_v*/ + 12 /*ang_v*/
                              + 1 /*shape_count*/;
            EnsureCapacity(kPrefix);
            byte* p = _ptr + _len;
            *p++ = (byte)RecordType.BodyCreate;
            *(uint*)p = body.Id;            p += 4;
            *p++ = (byte)bodyType;
            *p++ = (byte)layer;
            *(uint*)p = groupId;            p += 4;
            *(double*)p = posX;             p += 8;
            *(double*)p = posY;             p += 8;
            *(double*)p = posZ;             p += 8;
            *(float*)p = rotX;              p += 4;
            *(float*)p = rotY;              p += 4;
            *(float*)p = rotZ;              p += 4;
            *(float*)p = rotW;              p += 4;
            *(float*)p = mass;              p += 4;
            *(float*)p = linVx;             p += 4;
            *(float*)p = linVy;             p += 4;
            *(float*)p = linVz;             p += 4;
            *(float*)p = angVx;             p += 4;
            *(float*)p = angVy;             p += 4;
            *(float*)p = angVz;             p += 4;
            *p++ = shapeCount;
            _len += kPrefix;
        }

        public void AppendShapeBox(
            float subPosX, float subPosY, float subPosZ,
            float subRotX, float subRotY, float subRotZ, float subRotW,
            float halfX, float halfY, float halfZ,
            float density = 0f)
        {
            // sub_pos(12) + sub_rot(16) + kind(1) + density(4) + half_extents(12) = 45
            const int kSize = 45;
            EnsureCapacity(kSize);
            byte* p = _ptr + _len;
            *(float*)p = subPosX;           p += 4;
            *(float*)p = subPosY;           p += 4;
            *(float*)p = subPosZ;           p += 4;
            *(float*)p = subRotX;           p += 4;
            *(float*)p = subRotY;           p += 4;
            *(float*)p = subRotZ;           p += 4;
            *(float*)p = subRotW;           p += 4;
            *p++ = (byte)ShapeKind.Box;
            *(float*)p = density;           p += 4;
            *(float*)p = halfX;             p += 4;
            *(float*)p = halfY;             p += 4;
            *(float*)p = halfZ;             p += 4;
            _len += kSize;
        }

        public void AppendShapeSphere(
            float subPosX, float subPosY, float subPosZ,
            float subRotX, float subRotY, float subRotZ, float subRotW,
            float radius,
            float density = 0f)
        {
            // sub_pos(12) + sub_rot(16) + kind(1) + density(4) + radius(4) = 37
            const int kSize = 37;
            EnsureCapacity(kSize);
            byte* p = _ptr + _len;
            *(float*)p = subPosX;           p += 4;
            *(float*)p = subPosY;           p += 4;
            *(float*)p = subPosZ;           p += 4;
            *(float*)p = subRotX;           p += 4;
            *(float*)p = subRotY;           p += 4;
            *(float*)p = subRotZ;           p += 4;
            *(float*)p = subRotW;           p += 4;
            *p++ = (byte)ShapeKind.Sphere;
            *(float*)p = density;           p += 4;
            *(float*)p = radius;            p += 4;
            _len += kSize;
        }

        /// <summary>
        /// Append a ConvexHull sub-shape. <paramref name="vertices"/>
        /// is xyz-packed (length must be a multiple of 3) in the
        /// sub-shape's local frame. Vertices beyond
        /// <see cref="ShapeLimits.MaxConvexHullVertices"/> are still
        /// transmitted but the bridge truncates with a warning.
        /// </summary>
        public void AppendShapeConvexHull(
            float subPosX, float subPosY, float subPosZ,
            float subRotX, float subRotY, float subRotZ, float subRotW,
            float[] vertices,
            float density = 0f)
        {
            if (vertices == null || vertices.Length == 0 || (vertices.Length % 3) != 0)
                throw new ArgumentException("vertices must be a non-empty xyz-packed array",
                                             nameof(vertices));

            uint vertCount = (uint)(vertices.Length / 3);
            // sub_pos(12) + sub_rot(16) + kind(1) + density(4) + count(4) + vertCount*12 = 37 + 12N
            int kSize = 37 + (int)vertCount * 12;
            EnsureCapacity(kSize);
            byte* p = _ptr + _len;
            *(float*)p = subPosX;           p += 4;
            *(float*)p = subPosY;           p += 4;
            *(float*)p = subPosZ;           p += 4;
            *(float*)p = subRotX;           p += 4;
            *(float*)p = subRotY;           p += 4;
            *(float*)p = subRotZ;           p += 4;
            *(float*)p = subRotW;           p += 4;
            *p++ = (byte)ShapeKind.ConvexHull;
            *(float*)p = density;           p += 4;
            *(uint*)p = vertCount;          p += 4;
            for (int i = 0; i < vertices.Length; ++i)
            {
                *(float*)p = vertices[i];   p += 4;
            }
            _len += kSize;
        }

        /// <summary>
        /// Append a TriangleMesh sub-shape. <paramref name="vertices"/>
        /// is xyz-packed (length must be a multiple of 3) in the
        /// sub-shape's local frame; <paramref name="triangles"/> is
        /// index-packed (length must be a multiple of 3) referring to
        /// vertex indices. Used for streaming PQS terrain quads —
        /// static-only on the native side.
        /// </summary>
        public void AppendShapeTriangleMesh(
            float subPosX, float subPosY, float subPosZ,
            float subRotX, float subRotY, float subRotZ, float subRotW,
            float[] vertices, int[] triangles)
        {
            if (vertices == null || vertices.Length == 0 || (vertices.Length % 3) != 0)
                throw new ArgumentException("vertices must be a non-empty xyz-packed array",
                                             nameof(vertices));
            if (triangles == null || triangles.Length == 0 || (triangles.Length % 3) != 0)
                throw new ArgumentException("triangles must be a non-empty index-triplet array",
                                             nameof(triangles));

            uint vertCount = (uint)(vertices.Length / 3);
            uint triCount  = (uint)(triangles.Length / 3);
            // sub_pos(12) + sub_rot(16) + kind(1) + density(4) + vert_count(4) + verts(12*N)
            //   + tri_count(4) + tris(12*M) = 37 + 12N + 4 + 12M
            int kSize = 37 + (int)vertCount * 12 + 4 + (int)triCount * 12;
            EnsureCapacity(kSize);
            byte* p = _ptr + _len;
            *(float*)p = subPosX;           p += 4;
            *(float*)p = subPosY;           p += 4;
            *(float*)p = subPosZ;           p += 4;
            *(float*)p = subRotX;           p += 4;
            *(float*)p = subRotY;           p += 4;
            *(float*)p = subRotZ;           p += 4;
            *(float*)p = subRotW;           p += 4;
            *p++ = (byte)ShapeKind.TriangleMesh;
            // TriangleMesh is static-only; density is meaningless. Pad
            // the wire so all sub-shape records share the same prefix.
            *(float*)p = 0f;                p += 4;
            *(uint*)p = vertCount;          p += 4;
            for (int i = 0; i < vertices.Length; ++i)
            {
                *(float*)p = vertices[i];   p += 4;
            }
            *(uint*)p = triCount;           p += 4;
            for (int i = 0; i < triangles.Length; ++i)
            {
                *(uint*)p = (uint)triangles[i]; p += 4;
            }
            _len += kSize;
        }

        // -- Convenience wrappers (single sub-shape, identity transform) --

        public void WriteBodyCreateBox(
            BodyHandle body, BodyType bodyType, Layer layer,
            float halfX, float halfY, float halfZ,
            double posX, double posY, double posZ,
            float rotX, float rotY, float rotZ, float rotW,
            float mass, uint groupId = 0)
        {
            BeginBodyCreate(body, bodyType, layer,
                            posX, posY, posZ,
                            rotX, rotY, rotZ, rotW,
                            mass, shapeCount: 1, groupId: groupId);
            AppendShapeBox(0, 0, 0, 0, 0, 0, 1, halfX, halfY, halfZ);
        }

        public void WriteBodyCreateSphere(
            BodyHandle body, BodyType bodyType, Layer layer,
            float radius,
            double posX, double posY, double posZ,
            float rotX, float rotY, float rotZ, float rotW,
            float mass, uint groupId = 0)
        {
            BeginBodyCreate(body, bodyType, layer,
                            posX, posY, posZ,
                            rotX, rotY, rotZ, rotW,
                            mass, shapeCount: 1, groupId: groupId);
            AppendShapeSphere(0, 0, 0, 0, 0, 0, 1, radius);
        }

        public void WriteBodyCreateConvexHull(
            BodyHandle body, BodyType bodyType, Layer layer,
            float[] vertices,
            double posX, double posY, double posZ,
            float rotX, float rotY, float rotZ, float rotW,
            float mass, uint groupId = 0)
        {
            BeginBodyCreate(body, bodyType, layer,
                            posX, posY, posZ,
                            rotX, rotY, rotZ, rotW,
                            mass, shapeCount: 1, groupId: groupId);
            AppendShapeConvexHull(0, 0, 0, 0, 0, 0, 1, vertices);
        }

        public void WriteBodyCreateTriangleMesh(
            BodyHandle body, Layer layer,
            float[] vertices, int[] triangles,
            double posX, double posY, double posZ,
            float rotX, float rotY, float rotZ, float rotW,
            uint groupId = 0)
        {
            // Triangle meshes are static-only — Jolt's MeshShape isn't
            // valid on Dynamic / Kinematic bodies. Mass is meaningless
            // for static bodies; pass 0.
            BeginBodyCreate(body, BodyType.Static, layer,
                            posX, posY, posZ,
                            rotX, rotY, rotZ, rotW,
                            mass: 0f, shapeCount: 1, groupId: groupId);
            AppendShapeTriangleMesh(0, 0, 0, 0, 0, 0, 1, vertices, triangles);
        }

        /// <summary>
        /// Set a kinematic body's pose and velocity. Layout:
        /// tag(1) + user_id(4) + pos(24) + rot(16) + lin_v(12) + ang_v(12) = 69.
        /// </summary>
        public void WriteSetKinematicPose(
            BodyHandle body,
            double posX, double posY, double posZ,
            float rotX, float rotY, float rotZ, float rotW,
            float linX, float linY, float linZ,
            float angX, float angY, float angZ)
        {
            const int kSize = 69;
            EnsureCapacity(kSize);
            byte* p = _ptr + _len;
            *p++ = (byte)RecordType.SetKinematicPose;
            *(uint*)p = body.Id;            p += 4;
            *(double*)p = posX;             p += 8;
            *(double*)p = posY;             p += 8;
            *(double*)p = posZ;             p += 8;
            *(float*)p = rotX;              p += 4;
            *(float*)p = rotY;              p += 4;
            *(float*)p = rotZ;              p += 4;
            *(float*)p = rotW;              p += 4;
            *(float*)p = linX;              p += 4;
            *(float*)p = linY;              p += 4;
            *(float*)p = linZ;              p += 4;
            *(float*)p = angX;              p += 4;
            *(float*)p = angY;              p += 4;
            *(float*)p = angZ;              p += 4;
            _len += kSize;
        }

        public void WriteForceDelta(BodyHandle body,
                                    double fx, double fy, double fz,
                                    double tx, double ty, double tz)
        {
            const int kSize = 53;
            EnsureCapacity(kSize);
            byte* p = _ptr + _len;
            *p++ = (byte)RecordType.ForceDelta;
            *(uint*)p = body.Id;            p += 4;
            *(double*)p = fx;               p += 8;
            *(double*)p = fy;               p += 8;
            *(double*)p = fz;               p += 8;
            *(double*)p = tx;               p += 8;
            *(double*)p = ty;               p += 8;
            *(double*)p = tz;               p += 8;
            _len += kSize;
        }

        /// <summary>
        /// Apply a force at a specific world-space point on the body
        /// (CB-frame coords). Native side computes the implicit torque
        /// from (point - body_CoM) × force. <paramref name="partIdx"/>
        /// tags the force with the originating part's RNEA tree index
        /// so Phase 4 can subtract per-part external wrenches; pass
        /// <c>0xFFFF</c> for unattributed (static / non-tree) sources.
        /// Layout: tag(1) + body_id(4) + force(double3=24) +
        /// point(double3=24) + part_idx(2) = 55 bytes.
        /// </summary>
        public void WriteForceAtPosition(BodyHandle body,
                                         double fx, double fy, double fz,
                                         double px, double py, double pz,
                                         ushort partIdx = 0xFFFF)
        {
            const int kSize = 55;
            EnsureCapacity(kSize);
            byte* p = _ptr + _len;
            *p++ = (byte)RecordType.ForceAtPosition;
            *(uint*)p = body.Id;            p += 4;
            *(double*)p = fx;               p += 8;
            *(double*)p = fy;               p += 8;
            *(double*)p = fz;               p += 8;
            *(double*)p = px;               p += 8;
            *(double*)p = py;               p += 8;
            *(double*)p = pz;               p += 8;
            *(ushort*)p = partIdx;          p += 2;
            _len += kSize;
        }

        public void WriteSetGravity(double gx, double gy, double gz)
        {
            const int kSize = 25;
            EnsureCapacity(kSize);
            byte* p = _ptr + _len;
            *p++ = (byte)RecordType.SetGravity;
            *(double*)p = gx;               p += 8;
            *(double*)p = gy;               p += 8;
            *(double*)p = gz;               p += 8;
            _len += kSize;
        }

        /// <summary>
        /// Translate every body in the world by the given delta in
        /// absolute coordinates. Used to keep Jolt's world aligned
        /// with Unity's view when KSP's Krakensbane / FloatingOrigin
        /// shifts the Unity origin out from under us. Layout:
        /// tag(1) + delta(double3=24) = 25 bytes.
        /// </summary>
        public void WriteShiftWorld(double dx, double dy, double dz)
        {
            const int kSize = 25;
            EnsureCapacity(kSize);
            byte* p = _ptr + _len;
            *p++ = (byte)RecordType.ShiftWorld;
            *(double*)p = dx;               p += 8;
            *(double*)p = dy;               p += 8;
            *(double*)p = dz;               p += 8;
            _len += kSize;
        }

        public void WriteBodyDestroy(BodyHandle body)
        {
            const int kSize = 5;
            EnsureCapacity(kSize);
            byte* p = _ptr + _len;
            *p++ = (byte)RecordType.BodyDestroy;
            *(uint*)p = body.Id;
            _len += kSize;
        }

        /// <summary>
        /// Create a rigid (FixedConstraint) joint between two bodies.
        /// Layout: tag(1) + constraint_id(4) + kind(1) + body_a(4) + body_b(4) = 14.
        /// </summary>
        public void WriteConstraintCreateFixed(uint constraintId, BodyHandle bodyA, BodyHandle bodyB)
        {
            const int kSize = 14;
            EnsureCapacity(kSize);
            byte* p = _ptr + _len;
            *p++ = (byte)RecordType.ConstraintCreate;
            *(uint*)p = constraintId;       p += 4;
            *p++ = (byte)ConstraintKind.Fixed;
            *(uint*)p = bodyA.Id;           p += 4;
            *(uint*)p = bodyB.Id;           p += 4;
            _len += kSize;
        }

        /// <summary>
        /// Create a FixedConstraint with an explicit world-space anchor
        /// (CB-frame coords) for better PGS conditioning. Both bodies
        /// resolve their local-CoM offsets to this world point at
        /// construction time. Layout: tag(1) + constraint_id(4) + kind(1)
        /// + body_a(4) + body_b(4) + anchor(double3=24) = 38 bytes.
        /// </summary>
        public void WriteConstraintCreateFixedAt(
            uint constraintId, BodyHandle bodyA, BodyHandle bodyB,
            double anchorX, double anchorY, double anchorZ)
        {
            const int kSize = 38;
            EnsureCapacity(kSize);
            byte* p = _ptr + _len;
            *p++ = (byte)RecordType.ConstraintCreateFixedAt;
            *(uint*)p = constraintId;       p += 4;
            *p++ = (byte)ConstraintKind.Fixed;
            *(uint*)p = bodyA.Id;           p += 4;
            *(uint*)p = bodyB.Id;           p += 4;
            *(double*)p = anchorX;          p += 8;
            *(double*)p = anchorY;          p += 8;
            *(double*)p = anchorZ;          p += 8;
            _len += kSize;
        }

        /// <summary>
        /// Update a body's collision filter group post-creation. Used
        /// when a part migrates between vessels (decouple, dock) so
        /// the body's GroupFilterTable membership matches its new
        /// vessel's group ID. Layout: tag(1) + user_id(4) + group_id(4) = 9.
        /// </summary>
        public void WriteSetBodyGroup(BodyHandle body, uint groupId)
        {
            const int kSize = 9;
            EnsureCapacity(kSize);
            byte* p = _ptr + _len;
            *p++ = (byte)RecordType.SetBodyGroup;
            *(uint*)p = body.Id;            p += 4;
            *(uint*)p = groupId;            p += 4;
            _len += kSize;
        }

        /// <summary>
        /// Update a body's mass post-creation. Used to track fuel burn
        /// and discrete mass changes (fairing eject, decouple of
        /// physics-significant subparts). Native side rescales the
        /// shape-derived inertia tensor with the new mass so angular
        /// dynamics stay correct.
        /// Layout: tag(1) + user_id(4) + mass(4) = 9 bytes. mass in
        /// tonnes (KSP convention).
        /// </summary>
        public void WriteMassUpdate(BodyHandle body, float mass)
        {
            const int kSize = 9;
            EnsureCapacity(kSize);
            byte* p = _ptr + _len;
            *p++ = (byte)RecordType.MassUpdate;
            *(uint*)p = body.Id;            p += 4;
            *(float*)p = mass;              p += 4;
            _len += kSize;
        }

        /// <summary>
        /// Per-part data for a single node in a vessel's spanning tree.
        /// All quantities expressed in the vessel-root local frame.
        /// </summary>
        public struct VesselTreeNode
        {
            public ushort  ParentIdx;     // 0xFFFF for root
            public float   Mass;          // tonnes
            public float   ComLocalX, ComLocalY, ComLocalZ;
            public float   InertiaDiagX, InertiaDiagY, InertiaDiagZ;
            public float   AttachLocalX, AttachLocalY, AttachLocalZ;
            // Phase 5.1: per-edge spherical-joint compliance (joint to
            // parent). 3 angular DOF only — translation between anchors
            // is rigidly pinned. K_ang in kN·m/rad, C_ang in kN·m·s/rad.
            // Root's values are unused (no edge to a non-existent parent).
            public float   KAng, CAng;
        }

        /// <summary>
        /// Send a vessel's spanning tree to the native RNEA solver. Called
        /// after the corresponding BodyCreate. <paramref name="vesselBodyId"/>
        /// must match the vessel body's user_id. Nodes must be in topology
        /// order (every parent_idx strictly less than the node's own
        /// index, or 0xFFFF for the root).
        /// Layout: tag(1) + body_id(4) + part_count(2)
        ///         + part_count × (parent_idx(2) + mass(4) + 3 × float3(36)
        ///                         + compliance(8))
        ///         = 7 + N*50 bytes.
        /// </summary>
        public void WriteVesselTreeUpdate(BodyHandle vesselBody, VesselTreeNode[] nodes)
        {
            int n = nodes != null ? nodes.Length : 0;
            const int kHeader = 1 /*tag*/ + 4 /*body_id*/ + 2 /*part_count*/;
            const int kPerNode = 2 + 4 + 12 + 12 + 12 + 8;  // 50 bytes
            int kSize = kHeader + n * kPerNode;
            EnsureCapacity(kSize);
            byte* p = _ptr + _len;
            *p++ = (byte)RecordType.VesselTreeUpdate;
            *(uint*)p = vesselBody.Id;     p += 4;
            *(ushort*)p = (ushort)n;       p += 2;
            for (int i = 0; i < n; ++i)
            {
                ref var node = ref nodes[i];
                *(ushort*)p = node.ParentIdx;     p += 2;
                *(float*)p  = node.Mass;          p += 4;
                *(float*)p  = node.ComLocalX;     p += 4;
                *(float*)p  = node.ComLocalY;     p += 4;
                *(float*)p  = node.ComLocalZ;     p += 4;
                *(float*)p  = node.InertiaDiagX;  p += 4;
                *(float*)p  = node.InertiaDiagY;  p += 4;
                *(float*)p  = node.InertiaDiagZ;  p += 4;
                *(float*)p  = node.AttachLocalX;  p += 4;
                *(float*)p  = node.AttachLocalY;  p += 4;
                *(float*)p  = node.AttachLocalZ;  p += 4;
                *(float*)p  = node.KAng;          p += 4;
                *(float*)p  = node.CAng;          p += 4;
            }
            _len += kSize;
        }

        /// <summary>
        /// Send a SubShape→part_idx mapping for a vessel body so Phase 5
        /// ABA can apply per-part flex transforms to every collider
        /// belonging to that part (multi-collider parts share one
        /// flex). Layout: tag(1) + body_id(4) + count(2) + count × u16.
        /// Send right after BodyCreate; the mapping is keyed by the
        /// body and persists until BodyDestroy.
        /// </summary>
        public void WriteSubShapeMap(BodyHandle body, ushort[] subShapeToPart)
        {
            int n = subShapeToPart != null ? subShapeToPart.Length : 0;
            int kSize = 1 + 4 + 2 + n * 2;
            EnsureCapacity(kSize);
            byte* p = _ptr + _len;
            *p++ = (byte)RecordType.SubShapeMap;
            *(uint*)p = body.Id;            p += 4;
            *(ushort*)p = (ushort)n;        p += 2;
            for (int i = 0; i < n; ++i)
            {
                *(ushort*)p = subShapeToPart[i];
                p += 2;
            }
            _len += kSize;
        }

        public void WriteConstraintDestroy(uint constraintId)
        {
            const int kSize = 5;
            EnsureCapacity(kSize);
            byte* p = _ptr + _len;
            *p++ = (byte)RecordType.ConstraintDestroy;
            *(uint*)p = constraintId;
            _len += kSize;
        }

        // -- Internals -----------------------------------------------------

        private void EnsureCapacity(int recordBytes)
        {
            if (_len + recordBytes > _capacity)
                Grow(_len + recordBytes);
        }

        private void Grow(int required)
        {
            int newCap = _capacity;
            while (newCap < required) newCap *= 2;
            byte* @new = (byte*)Marshal.AllocHGlobal(newCap);
            Buffer.MemoryCopy(_ptr, @new, newCap, _len);
            Marshal.FreeHGlobal((IntPtr)_ptr);
            _ptr      = @new;
            _capacity = newCap;
        }
    }
}
