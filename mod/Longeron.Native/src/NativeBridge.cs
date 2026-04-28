// extern "C" P/Invoke declarations matching native/src/bridge.h.
//
// Library name `longeron_native` resolves to longeron_native.dll on
// Windows, longeron_native.dylib on macOS, longeron_native.so on Linux.
// Mono's DllImport resolution handles the platform-specific suffix.

using System;
using System.IO;
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
            // Schema version 2: BodyCreate gained a variable-length
            // sub-shape list (Phase 1.5) — Box, Sphere, ConvexHull
            // packed with per-sub-shape local transform.
            SchemaVersion  = 2,
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

        // Mono on macOS does not search the calling assembly's
        // directory for native libraries, and DYLD_LIBRARY_PATH is
        // stripped by SIP for many launchers. Force-load the lib from
        // a known location (next to this assembly, or a few parent
        // dirs in dev) before any DllImport call resolves. Once
        // dlopen succeeds the lib is in the process and subsequent
        // DllImports bind to it by base name.
        static NativeBridge()
        {
            try { Preload(); }
            catch (Exception ex)
            {
                Console.Error.WriteLine($"[Longeron.Native] preload failed: {ex.GetType().Name}: {ex.Message}");
            }
        }

        private static void Preload()
        {
            bool diag = Environment.GetEnvironmentVariable("LONGERON_NATIVE_DIAG") == "1";

            string asmDir = Path.GetDirectoryName(typeof(NativeBridge).Assembly.Location);
            if (diag) Console.Error.WriteLine($"[Longeron.Native] asmDir={asmDir}");
            if (string.IsNullOrEmpty(asmDir)) return;

            string ext = Environment.OSVersion.Platform == PlatformID.MacOSX
                          || (Environment.OSVersion.Platform == PlatformID.Unix
                              && DetectMacOS())
                          ? ".dylib"
                          : Environment.OSVersion.Platform == PlatformID.Unix ? ".so" : ".dll";
            if (diag) Console.Error.WriteLine($"[Longeron.Native] ext={ext}");

            // Search candidates: assembly dir, then a few parent dirs
            // (covers `bin/Release/net48/` ↔ repo `native/build/` in
            // local dev). On Unix CMake produces a `lib` prefix; on
            // Windows it doesn't.
            string fileName = (ext == ".dll" ? Lib : "lib" + Lib) + ext;
            var candidates = new[]
            {
                Path.Combine(asmDir, fileName),
                Path.Combine(asmDir, "..", "..", "..", "..", "..", "native", "build", fileName),
                Path.Combine(asmDir, "..", "..", "..", "..", "native", "build", fileName),
                Path.Combine(asmDir, "..", "..", "..", "native", "build", fileName),
            };

            foreach (var c in candidates)
            {
                bool exists = File.Exists(c);
                if (diag) Console.Error.WriteLine($"[Longeron.Native] candidate={c} exists={exists}");
                if (!exists) continue;
                IntPtr h = LoadLibrary(c, ext);
                if (diag) Console.Error.WriteLine($"[Longeron.Native] LoadLibrary handle=0x{h.ToInt64():x}");
                if (h != IntPtr.Zero) return;
            }
            if (diag) Console.Error.WriteLine("[Longeron.Native] no candidate loaded successfully");
        }

        private static bool DetectMacOS()
        {
            try { return File.Exists("/System/Library/CoreServices/SystemVersion.plist"); }
            catch { return false; }
        }

        private static IntPtr LoadLibrary(string path, string ext)
        {
            const int RTLD_NOW = 2;
            switch (ext)
            {
                case ".dylib": return dlopen_macos(path, RTLD_NOW);
                case ".so":    return dlopen_linux(path, RTLD_NOW);
                case ".dll":   return LoadLibraryW(path);
                default:       return IntPtr.Zero;
            }
        }

        [DllImport("libdl.dylib", EntryPoint = "dlopen")]
        private static extern IntPtr dlopen_macos(string file, int flags);

        [DllImport("libdl.so.2", EntryPoint = "dlopen")]
        private static extern IntPtr dlopen_linux(string file, int flags);

        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, EntryPoint = "LoadLibraryW")]
        private static extern IntPtr LoadLibraryW(string path);

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
