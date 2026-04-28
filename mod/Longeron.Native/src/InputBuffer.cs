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

        // Output-only:
        BodyPose          = 64,
        ContactReport     = 65,
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
    /// Mirrors <c>longeron::ShapeKind</c>. Phase 1 only Box; Phase 2+
    /// extends with Sphere, Capsule, ConvexHull, Mesh.
    /// </summary>
    public enum ShapeKind : byte
    {
        Box = 0,
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

        /// <summary>
        /// Create a body with a box collider. Layout matches
        /// <c>longeron::LongeronWorld::HandleBodyCreate</c>:
        /// tag(1) + user_id(4) + body_type(1) + shape_kind(1)
        /// + half_extents(float3=12) + pos(double3=24) + rot(float4=16)
        /// + mass(float=4) + layer(1) = 64 bytes.
        /// </summary>
        public void WriteBodyCreateBox(
            BodyHandle body, BodyType bodyType, Layer layer,
            float halfX, float halfY, float halfZ,
            double posX, double posY, double posZ,
            float rotX, float rotY, float rotZ, float rotW,
            float mass)
        {
            const int kSize = 64;
            EnsureCapacity(kSize);
            byte* p = _ptr + _len;
            *p++ = (byte)RecordType.BodyCreate;
            *(uint*)p = body.Id;            p += 4;
            *p++ = (byte)bodyType;
            *p++ = (byte)ShapeKind.Box;
            *(float*)p = halfX;             p += 4;
            *(float*)p = halfY;             p += 4;
            *(float*)p = halfZ;             p += 4;
            *(double*)p = posX;             p += 8;
            *(double*)p = posY;             p += 8;
            *(double*)p = posZ;             p += 8;
            *(float*)p = rotX;              p += 4;
            *(float*)p = rotY;              p += 4;
            *(float*)p = rotZ;              p += 4;
            *(float*)p = rotW;              p += 4;
            *(float*)p = mass;              p += 4;
            *p++ = (byte)layer;
            _len += kSize;
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

        public void WriteBodyDestroy(BodyHandle body)
        {
            const int kSize = 5;
            EnsureCapacity(kSize);
            byte* p = _ptr + _len;
            *p++ = (byte)RecordType.BodyDestroy;
            *(uint*)p = body.Id;
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
