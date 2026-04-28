using System;
using System.Runtime.InteropServices;

namespace Longeron.Native
{
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
        /// Returns the next record's type, or <see cref="RecordType.None"/>
        /// if the stream is exhausted. Caller then reads the typed
        /// payload with one of the <c>Read…</c> methods. Mismatched
        /// reads corrupt the cursor.
        /// </summary>
        public RecordType Next()
        {
            if (_cursor >= _len) return RecordType.None;
            return (RecordType)_ptr[_cursor];
        }

        /// <summary>
        /// Phase 0 echo record: 8-byte input length followed by 8-byte
        /// step counter, both u64. No record-type tag in front (Phase 0
        /// short-circuit; Phase 1+ removes this method when the real
        /// schema lands).
        /// </summary>
        public bool TryReadPhase0Echo(out ulong inputLen, out ulong stepCount)
        {
            if (_len < 16)
            {
                inputLen  = 0;
                stepCount = 0;
                return false;
            }
            inputLen  = *(ulong*)(_ptr + 0);
            stepCount = *(ulong*)(_ptr + 8);
            _cursor   = 16;
            return true;
        }
    }
}
