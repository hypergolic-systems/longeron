// Longeron native bridge — C ABI exposed to C# (mod/Longeron.Native/).
//
// All symbols here are extern "C" with stable layout. C# pins input/output
// buffers and calls longeron_step() once per FixedUpdate; the bridge
// parses the input record stream, advances the Jolt world by dt, and
// emits an output record stream. See CLAUDE.md for the per-record
// schema and ownership split.

#ifndef LONGERON_BRIDGE_H
#define LONGERON_BRIDGE_H

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
    #if defined(LONGERON_NATIVE_BUILDING_DLL)
        #define LONGERON_EXPORT __declspec(dllexport)
    #else
        #define LONGERON_EXPORT __declspec(dllimport)
    #endif
#else
    #define LONGERON_EXPORT __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle. C# treats this as IntPtr; bridge implementation owns it.
typedef struct LongeronWorld LongeronWorld;

// Mirror of the C# `LongeronConfig` struct. Blittable, fixed layout.
typedef struct LongeronConfig {
    uint32_t schema_version;     // bumped when the layout changes
    uint32_t reserved0;
    double   gravity_x;          // default per-tick gravity (0,0,0 if
    double   gravity_y;          //   per-body gravity is supplied via
    double   gravity_z;          //   input records each step)
    uint32_t max_bodies;         // initial body capacity
    uint32_t max_constraints;    // initial constraint capacity
} LongeronConfig;

// --------------------------------------------------------------------
// Session lifetime

LONGERON_EXPORT LongeronWorld* longeron_world_create(const LongeronConfig* cfg);
LONGERON_EXPORT void            longeron_world_destroy(LongeronWorld* w);

// --------------------------------------------------------------------
// Per-FixedUpdate step
//
// Inputs:  pinned C# buffer of typed records (force deltas, mass updates,
//          topology mutations, etc.). See InputBuffer.cs for the schema.
// Outputs: pinned C# buffer; bridge writes a record stream of poses,
//          velocities, and contact reports back. *output_len is set to
//          the actual number of bytes written. If output_cap is too
//          small the bridge writes nothing and sets *output_len to the
//          required size.
// dt:      simulation step in seconds.
//
// Returns 0 on success, non-zero error code on failure.

LONGERON_EXPORT int32_t longeron_step(
    LongeronWorld*  w,
    const uint8_t*  input,
    size_t          input_len,
    uint8_t*        output,
    size_t          output_cap,
    size_t*         output_len,
    float           dt);

// --------------------------------------------------------------------
// Diagnostics

// Returns a static, null-terminated version string ("longeron_native
// X.Y.Z (jolt: ...)"). Caller does not own the pointer.
LONGERON_EXPORT const char* longeron_version(void);

// Runs in-process sanity checks on the Featherstone spatial-vector
// primitives in spatial.h. Returns 0 on success, or the 1-indexed
// number of the first failing check. Used by the M2 gate in
// Longeron.Native.Probe — failure here signals a bug in the spatial
// layer that all of ABA depends on.
LONGERON_EXPORT int32_t longeron_spatial_selftest(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // LONGERON_BRIDGE_H
