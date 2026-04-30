# Longeron native bridge — Jolt-backed physics engine for KSP.
#
# Produces a shared library `longeron_native` (.dll/.dylib/.so) with a
# C ABI defined in src/bridge.h. C# side lives in mod/Longeron.Native/.
#
# Phase 0: bridge compiles standalone with no Jolt dependency and
# exposes longeron_version() + a no-op longeron_step() for round-trip
# smoke testing from C#. Jolt enters at Phase 1 via the
# third_party/JoltPhysics submodule.

cmake_minimum_required(VERSION 3.20)

project(longeron_native
    VERSION 0.0.1
    DESCRIPTION "Longeron native physics bridge"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Symbol visibility: hide everything by default; the bridge explicitly
# marks its C ABI exports via LONGERON_EXPORT (see src/bridge.h).
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

# macOS universal binary by default. Override with
# -DCMAKE_OSX_ARCHITECTURES=arm64 for a single-arch dev build.
if(APPLE AND NOT CMAKE_OSX_ARCHITECTURES)
    set(CMAKE_OSX_ARCHITECTURES "arm64;x86_64" CACHE STRING
        "macOS architectures" FORCE)
endif()

# --------------------------------------------------------------------
# Optional Jolt dependency
#
# Phase 1+ wires Jolt in via the submodule at third_party/JoltPhysics.
# When present, build with double-precision world coordinates so the
# physics side doesn't need to track Krakensbane shifts (Unity-side
# rendering still does — but the Jolt world stays in absolute doubles).

set(LONGERON_HAS_JOLT OFF)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/JoltPhysics/Build/CMakeLists.txt")
    set(LONGERON_HAS_JOLT ON)
    message(STATUS "Longeron: Jolt submodule found, enabling JOLT integration")
    # Jolt's own build flags (see third_party/JoltPhysics/Build/CMakeLists.txt).
    # Double precision lets the Jolt world stay in absolute coordinates so the
    # physics side doesn't need to track Krakensbane shifts.
    set(DOUBLE_PRECISION                  ON  CACHE BOOL "" FORCE)
    set(GENERATE_DEBUG_SYMBOLS            ON  CACHE BOOL "" FORCE)
    set(ENABLE_ASSERTS                    ON  CACHE BOOL "" FORCE)
    set(USE_AVX2                          OFF CACHE BOOL "" FORCE)
    set(USE_AVX                           OFF CACHE BOOL "" FORCE)
    set(USE_SSE4_2                        OFF CACHE BOOL "" FORCE)
    set(USE_SSE4_1                        OFF CACHE BOOL "" FORCE)
    set(USE_LZCNT                         OFF CACHE BOOL "" FORCE)
    set(USE_TZCNT                         OFF CACHE BOOL "" FORCE)
    set(USE_F16C                          OFF CACHE BOOL "" FORCE)
    set(USE_FMADD                         OFF CACHE BOOL "" FORCE)
    set(INTERPROCEDURAL_OPTIMIZATION      OFF CACHE BOOL "" FORCE)
    set(TARGET_HELLO_WORLD                OFF CACHE BOOL "" FORCE)
    set(TARGET_PERFORMANCE_TEST           OFF CACHE BOOL "" FORCE)
    set(TARGET_SAMPLES                    OFF CACHE BOOL "" FORCE)
    set(TARGET_UNIT_TESTS                 OFF CACHE BOOL "" FORCE)
    set(TARGET_VIEWER                     OFF CACHE BOOL "" FORCE)
    add_subdirectory(third_party/JoltPhysics/Build EXCLUDE_FROM_ALL)
else()
    message(STATUS "Longeron: Jolt submodule not present yet — building bridge stub only")
endif()

# --------------------------------------------------------------------
# Bridge shared library

add_library(longeron_native SHARED
    src/bridge.cpp
    src/world.cpp
    src/contact_listener.cpp
    src/contact_solve_listener.cpp
    src/tree.cpp
    src/aba.cpp
)

target_include_directories(longeron_native PRIVATE src)

target_compile_definitions(longeron_native PRIVATE
    LONGERON_NATIVE_BUILDING_DLL=1
)

if(LONGERON_HAS_JOLT)
    target_link_libraries(longeron_native PRIVATE Jolt)
    target_compile_definitions(longeron_native PRIVATE
        LONGERON_HAS_JOLT=1
        JPH_DOUBLE_PRECISION=1
    )
endif()

set_target_properties(longeron_native PROPERTIES
    OUTPUT_NAME "longeron_native"
    POSITION_INDEPENDENT_CODE ON
)

# Keep CMake's default platform conventions: longeron_native.dll on
# Windows, liblongeron_native.{dylib,so} on Unix. Mono's DllImport
# resolver automatically prepends `lib` on Unix when looking up
# `[DllImport("longeron_native")]`, so this is the form it actually
# searches for.

# Warnings.
if(MSVC)
    target_compile_options(longeron_native PRIVATE /W4 /permissive-)
else()
    target_compile_options(longeron_native PRIVATE
        -Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()
