using System;

namespace Longeron.Native
{
    /// <summary>
    /// Session-level wrapper around a native <c>LongeronWorld*</c>.
    /// Owns the input/output buffers and per-tick step path. One
    /// instance per flight scene; <see cref="Dispose"/> on scene exit.
    /// </summary>
    public sealed unsafe class World : IDisposable
    {
        private IntPtr       _handle;
        private InputBuffer  _input;
        private OutputBuffer _output;

        public World(LongeronConfig config, int inputCap = 1 << 16, int outputCap = 1 << 18)
        {
            _handle = NativeBridge.longeron_world_create(ref config);
            if (_handle == IntPtr.Zero)
                throw new InvalidOperationException(
                    "longeron_world_create returned null — native bridge may be missing or stale.");
            _input  = new InputBuffer(inputCap);
            _output = new OutputBuffer(outputCap);
        }

        public InputBuffer  Input  => _input;
        public OutputBuffer Output => _output;

        /// <summary>
        /// Native version string ("longeron_native X.Y.Z (jolt: …)").
        /// Useful for sanity-checking that the right native binary is
        /// loaded.
        /// </summary>
        public static string NativeVersion => NativeBridge.GetVersion();

        /// <summary>
        /// Runs the per-FixedUpdate step: feeds the accumulated input
        /// records into the native bridge, advances the Jolt world by
        /// <paramref name="dt"/>, and surfaces output records via
        /// <see cref="Output"/>. Caller is expected to have populated
        /// <see cref="Input"/> beforehand and to drain
        /// <see cref="Output"/> afterwards. Resets the input buffer
        /// length after the call.
        /// </summary>
        public void Step(float dt)
        {
            if (_handle == IntPtr.Zero)
                throw new ObjectDisposedException(nameof(World));

            UIntPtr outLen;
            int rc = NativeBridge.longeron_step(
                _handle,
                _input.Pointer,
                (UIntPtr)_input.Length,
                _output.Pointer,
                (UIntPtr)_output.Capacity,
                &outLen,
                dt);

            // Reset input buffer regardless of success — otherwise a
            // single failure compounds: next tick's records get appended
            // to the un-parsed half of the failed tick's records, and
            // every subsequent tick fails for the same reason.
            int len = _input.Length;
            _input.Reset();

            if (rc != 0)
                throw new InvalidOperationException(
                    $"longeron_step failed (rc={rc}, input bytes={len}, output bytes={(ulong)outLen})");

            _output.OnStepReturned((int)(ulong)outLen);
        }

        public void Dispose()
        {
            if (_handle != IntPtr.Zero)
            {
                NativeBridge.longeron_world_destroy(_handle);
                _handle = IntPtr.Zero;
            }
            _input?.Dispose();
            _output?.Dispose();
            _input  = null;
            _output = null;
        }
    }
}
