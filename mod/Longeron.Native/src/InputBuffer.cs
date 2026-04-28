using System;
using System.Runtime.InteropServices;

namespace Longeron.Native
{
    /// <summary>
    /// One-byte tag identifying each typed record in the input/output
    /// streams. Schema additions append new values; existing values do
    /// not change. Both sides assert their schema version matches at
    /// world-create time.
    /// </summary>
    public enum RecordType : byte
    {
        None              = 0,
        ForceDelta        = 1,  // u32 body, double3 force, double3 torque (world-frame)
        MassUpdate        = 2,  // u32 body, float mass, float[6] inertia (Ixx,Iyy,Izz,Ixy,Ixz,Iyz)
        BodyCreate        = 3,  // u32 body, shape descriptor, double3 pos, float4 rot, float mass
        BodyDestroy       = 4,  // u32 body
        ConstraintCreate  = 5,  // Phase 2+
        ConstraintDestroy = 6,  // Phase 2+
        SetGravity        = 7,  // double3 gravity (per-vessel COM-resolved)
        SetKinematicPose  = 8,  // u32 body, double3 pos, float4 rot (Phase 4 ABA writeback)

        // Output-only:
        BodyPose          = 64, // u32 body, double3 pos, float4 rot, float3 lin_vel, float3 ang_vel
        ContactReport     = 65, // u32 body_a, u32 body_b, double3 point, float3 normal, float depth, float impulse
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

        public byte* Pointer => _ptr;
        public int   Length  => _len;
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

        // -- Record writers ------------------------------------------------

        public void WriteForceDelta(BodyHandle body,
                                    double fx, double fy, double fz,
                                    double tx, double ty, double tz)
        {
            // tag(1) + body(4) + force(24) + torque(24) = 53 bytes
            EnsureCapacity(53);
            byte* p = _ptr + _len;
            *p = (byte)RecordType.ForceDelta;
            *(uint*)(p + 1)   = body.Id;
            *(double*)(p + 5) = fx;  *(double*)(p + 13) = fy;  *(double*)(p + 21) = fz;
            *(double*)(p + 29) = tx; *(double*)(p + 37) = ty;  *(double*)(p + 45) = tz;
            _len += 53;
        }

        public void WriteSetGravity(double gx, double gy, double gz)
        {
            // tag(1) + gravity(24) = 25
            EnsureCapacity(25);
            byte* p = _ptr + _len;
            *p = (byte)RecordType.SetGravity;
            *(double*)(p + 1)  = gx;
            *(double*)(p + 9)  = gy;
            *(double*)(p + 17) = gz;
            _len += 25;
        }

        public void WriteBodyDestroy(BodyHandle body)
        {
            EnsureCapacity(5);
            byte* p = _ptr + _len;
            *p = (byte)RecordType.BodyDestroy;
            *(uint*)(p + 1) = body.Id;
            _len += 5;
        }

        // Phase 2+ record writers (BodyCreate with shape, ConstraintCreate,
        // MassUpdate, SetKinematicPose) land here as the bridge grows.

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
