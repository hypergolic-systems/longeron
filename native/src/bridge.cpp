// Phase 0 bridge implementation.
//
// LongeronWorld is a stub struct; longeron_step is a no-op echo.
// Real Jolt integration begins in Phase 1.

#include "bridge.h"

#include <cstring>
#include <new>

namespace {

constexpr const char* kVersionString =
#if defined(LONGERON_HAS_JOLT)
    "longeron_native 0.0.1 (jolt: enabled, double-precision)";
#else
    "longeron_native 0.0.1 (jolt: stub)";
#endif

} // namespace

// Minimal opaque world. Phase 1 grows this into the Jolt PhysicsSystem
// owner; Phase 2 adds body registry, layer manager, contact listener.
struct LongeronWorld {
    LongeronConfig config;
    uint64_t       step_count;
};

extern "C" {

LONGERON_EXPORT LongeronWorld* longeron_world_create(const LongeronConfig* cfg) {
    if (cfg == nullptr) {
        return nullptr;
    }
    auto* w = new (std::nothrow) LongeronWorld{};
    if (w == nullptr) {
        return nullptr;
    }
    w->config     = *cfg;
    w->step_count = 0;
    return w;
}

LONGERON_EXPORT void longeron_world_destroy(LongeronWorld* w) {
    delete w;
}

LONGERON_EXPORT int32_t longeron_step(
    LongeronWorld* w,
    const uint8_t* /*input*/,
    size_t         input_len,
    uint8_t*       output,
    size_t         output_cap,
    size_t*        output_len,
    float          /*dt*/)
{
    if (w == nullptr || output_len == nullptr) {
        return -1;
    }

    // Phase 0: no-op echo. Write a single 8-byte record back to the C#
    // side echoing input_len + step_count so the round-trip can be
    // verified end-to-end. Real schema decoding lands in Phase 1.
    constexpr size_t kEchoSize = 16;
    if (output != nullptr && output_cap >= kEchoSize) {
        const uint64_t inlen = static_cast<uint64_t>(input_len);
        const uint64_t step  = w->step_count;
        std::memcpy(output + 0, &inlen, sizeof(inlen));
        std::memcpy(output + 8, &step,  sizeof(step));
        *output_len = kEchoSize;
    } else {
        *output_len = kEchoSize;
        if (output != nullptr) {
            return -2; // buffer too small
        }
    }

    ++w->step_count;
    return 0;
}

LONGERON_EXPORT const char* longeron_version(void) {
    return kVersionString;
}

} // extern "C"
