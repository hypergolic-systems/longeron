using System;
using System.Runtime.InteropServices;

namespace Longeron.Native
{
    /// <summary>
    /// Body pose record from <c>longeron_step</c>. Payload only — the
    /// 1-byte tag is consumed by <see cref="OutputBuffer.Next"/> before
    /// the read.
    /// </summary>
    public struct BodyPoseRecord
    {
        public BodyHandle Body;
        public double PosX, PosY, PosZ;
        public float  RotX, RotY, RotZ, RotW;
        public float  LinX, LinY, LinZ;
        public float  AngX, AngY, AngZ;
    }

    /// <summary>
    /// Contact report record from <c>longeron_step</c>. Phase 1 emits
    /// one report per body pair per tick, at the deepest contact point.
    /// </summary>
    public struct ContactReportRecord
    {
        public BodyHandle BodyA;
        public BodyHandle BodyB;
        public double PointX, PointY, PointZ;
        public float  NormalX, NormalY, NormalZ;
        public float  Depth;
        public float  Impulse;
    }

    /// <summary>
    /// Read-only cursor over the output record stream produced by
    /// <c>longeron_step</c>. Backed by a pinned native buffer; the C#
    /// caller iterates records via <see cref="Next"/> and reads typed
    /// payloads via the corresponding <c>Read…</c> methods. No managed
    /// allocations on the hot path.
    /// </summary>
    public sealed unsafe class OutputBuffer : IDisposable
    {
        private byte* _ptr;
        private int   _capacity;
        private int   _len;     // bytes actually written by the bridge
        private int   _cursor;  // current read offset

        public OutputBuffer(int capacityBytes)
        {
            if (capacityBytes <= 0) throw new ArgumentOutOfRangeException(nameof(capacityBytes));
            _ptr      = (byte*)Marshal.AllocHGlobal(capacityBytes);
            _capacity = capacityBytes;
            _len      = 0;
            _cursor   = 0;
        }

        public byte* Pointer  => _ptr;
        public int   Capacity => _capacity;
        public int   Length   => _len;
        public int   Cursor   => _cursor;

        /// <summary>
        /// Called by <see cref="World.Step"/> to set the actual byte
        /// length the bridge wrote, and reset the read cursor.
        /// </summary>
        internal void OnStepReturned(int actualLen)
        {
            _len    = actualLen;
            _cursor = 0;
        }

        public void Dispose()
        {
            if (_ptr != null)
            {
                Marshal.FreeHGlobal((IntPtr)_ptr);
                _ptr = null;
            }
        }

        /// <summary>
        /// Returns the next record's type and consumes its 1-byte tag,
        /// or <see cref="RecordType.None"/> if the stream is exhausted.
        /// Caller must then call the matching <c>Read…</c> method to
        /// consume the payload, or <see cref="SkipUnknown"/> to skip
        /// (currently throws — schema is not forwards-compatible at the
        /// reader without explicit lengths).
        /// </summary>
        public RecordType Next()
        {
            if (_cursor >= _len) return RecordType.None;
            return (RecordType)_ptr[_cursor++];
        }

        public void ReadBodyPose(out BodyPoseRecord record)
        {
            const int kPayload = 68;  // user_id(4) + pos(24) + rot(16) + lin(12) + ang(12)
            EnsureBytes(kPayload);
            byte* p = _ptr + _cursor;
            record = default;
            record.Body = new BodyHandle(*(uint*)p);   p += 4;
            record.PosX = *(double*)p;                 p += 8;
            record.PosY = *(double*)p;                 p += 8;
            record.PosZ = *(double*)p;                 p += 8;
            record.RotX = *(float*)p;                  p += 4;
            record.RotY = *(float*)p;                  p += 4;
            record.RotZ = *(float*)p;                  p += 4;
            record.RotW = *(float*)p;                  p += 4;
            record.LinX = *(float*)p;                  p += 4;
            record.LinY = *(float*)p;                  p += 4;
            record.LinZ = *(float*)p;                  p += 4;
            record.AngX = *(float*)p;                  p += 4;
            record.AngY = *(float*)p;                  p += 4;
            record.AngZ = *(float*)p;                  p += 4;
            _cursor += kPayload;
        }

        public void ReadContactReport(out ContactReportRecord record)
        {
            const int kPayload = 52;  // a(4) + b(4) + point(24) + normal(12) + depth(4) + impulse(4)
            EnsureBytes(kPayload);
            byte* p = _ptr + _cursor;
            record = default;
            record.BodyA   = new BodyHandle(*(uint*)p);   p += 4;
            record.BodyB   = new BodyHandle(*(uint*)p);   p += 4;
            record.PointX  = *(double*)p;                 p += 8;
            record.PointY  = *(double*)p;                 p += 8;
            record.PointZ  = *(double*)p;                 p += 8;
            record.NormalX = *(float*)p;                  p += 4;
            record.NormalY = *(float*)p;                  p += 4;
            record.NormalZ = *(float*)p;                  p += 4;
            record.Depth   = *(float*)p;                  p += 4;
            record.Impulse = *(float*)p;                  p += 4;
            _cursor += kPayload;
        }

        private void EnsureBytes(int n)
        {
            if (_cursor + n > _len)
                throw new InvalidOperationException(
                    $"Output stream truncated: need {n} bytes at offset {_cursor}, only {_len - _cursor} available.");
        }
    }
}
