// C ABI dispatcher: thin shim that forwards to longeron::LongeronWorld.

#include "bridge.h"
#include "world.h"

namespace {

constexpr const char* kVersionString =
#if defined(LONGERON_HAS_JOLT)
    #if defined(JPH_DOUBLE_PRECISION)
        "longeron_native 0.0.1 (jolt: enabled, double-precision)";
    #else
        "longeron_native 0.0.1 (jolt: enabled, single-precision)";
    #endif
#else
    "longeron_native 0.0.1 (jolt: stub)";
#endif

} // namespace

// LongeronWorld is the opaque struct exposed to C. Internally it
// inherits from longeron::LongeronWorld.
struct LongeronWorld : public longeron::LongeronWorld {
    using longeron::LongeronWorld::LongeronWorld;
};

extern "C" {

LONGERON_EXPORT LongeronWorld* longeron_world_create(const LongeronConfig* cfg) {
    if (cfg == nullptr) return nullptr;
    try {
        return new LongeronWorld(*cfg);
    } catch (...) {
        return nullptr;
    }
}

LONGERON_EXPORT void longeron_world_destroy(LongeronWorld* w) {
    delete w;
}

LONGERON_EXPORT int32_t longeron_step(
    LongeronWorld* w,
    const uint8_t* input,
    size_t         input_len,
    uint8_t*       output,
    size_t         output_cap,
    size_t*        output_len,
    float          dt)
{
    if (w == nullptr || output_len == nullptr) return -1;
    return w->Step(input, input_len, output, output_cap, output_len, dt);
}

LONGERON_EXPORT const char* longeron_version(void) {
    return kVersionString;
}

} // extern "C"
