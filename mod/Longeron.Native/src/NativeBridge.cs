// extern "C" P/Invoke declarations matching native/src/bridge.h.
//
// Library name `longeron_native` resolves to longeron_native.dll on
// Windows, longeron_native.dylib on macOS, longeron_native.so on Linux.
// Mono's DllImport resolution handles the platform-specific suffix.

using System;
using System.Runtime.InteropServices;
using System.Security;

namespace Longeron.Native
{
    /// <summary>
    /// Native bridge configuration. Mirrors C struct
    /// <c>LongeronConfig</c> in native/src/bridge.h.
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Pack = 8)]
    public struct LongeronConfig
    {
        public uint   SchemaVersion;
        public uint   Reserved0;
        public double GravityX;
        public double GravityY;
        public double GravityZ;
        public uint   MaxBodies;
        public uint   MaxConstraints;

        public static LongeronConfig Default => new LongeronConfig
        {
            SchemaVersion  = 1,
            Reserved0      = 0,
            GravityX       = 0.0,
            GravityY       = 0.0,
            GravityZ       = 0.0,
            MaxBodies      = 4096,
            MaxConstraints = 4096,
        };
    }

    [SuppressUnmanagedCodeSecurity]
    internal static class NativeBridge
    {
        private const string Lib = "longeron_native";

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr longeron_world_create(ref LongeronConfig cfg);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        public static extern void longeron_world_destroy(IntPtr w);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        public static extern unsafe int longeron_step(
            IntPtr  w,
            byte*   input,
            UIntPtr input_len,
            byte*   output,
            UIntPtr output_cap,
            UIntPtr* output_len,
            float   dt);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr longeron_version();

        public static string GetVersion()
        {
            IntPtr p = longeron_version();
            return p == IntPtr.Zero ? "(null)" : Marshal.PtrToStringAnsi(p);
        }
    }
}
