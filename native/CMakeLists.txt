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

# CMP0069 (NEW) lets target-level INTERPROCEDURAL_OPTIMIZATION actually
# enable -flto on Clang/GCC. Without this policy, the property is silently
# ignored and we'd see no LTO in the link line. Required for the
# Pinocchio header-only DCE story to work.
cmake_policy(SET CMP0069 NEW)

project(longeron_native
    VERSION 0.0.1
    DESCRIPTION "Longeron native physics bridge"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# If no build type was specified, default to Release. Many of our size /
# perf optimizations (LTO, dead_strip, -O3) only apply in Release config.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

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
# Pinocchio dependency — Phase 5.1.2 forward-dynamics provider.
# Replaces the prior in-tree canonical Featherstone ABA in src/aba.cpp.
# Pinocchio is BSD-2; it transitively pulls in Eigen (vendored at
# third_party/eigen) and Boost.Filesystem + Boost.Serialization at
# *build time* (system, e.g. brew). All optional Pinocchio features
# (Python, URDF, hpp-fcl/coal collision, autodiff, codegen, OpenMP,
# tests, benchmarks, docs) are forced OFF — we only need core
# dynamics (aba, rnea, integrate, forwardKinematics) and the joint
# zoo (Spherical at this phase; revolute / prismatic additive later).

set(LONGERON_HAS_PINOCCHIO OFF)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/pinocchio/CMakeLists.txt")
    set(LONGERON_HAS_PINOCCHIO ON)
    message(STATUS "Longeron: Pinocchio submodule found, enabling Pinocchio integration")

    # Eigen — header-only, vendored. Make Pinocchio find it without a
    # system Eigen install via EIGEN3_INCLUDE_DIR + cmake hint path.
    set(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen"
        CACHE PATH "Eigen include directory" FORCE)
    list(APPEND CMAKE_MODULE_PATH "${EIGEN3_INCLUDE_DIR}/cmake")
    list(APPEND CMAKE_PREFIX_PATH "${EIGEN3_INCLUDE_DIR}/cmake")

    # Disable every optional Pinocchio feature. Anything left on must
    # be justified — we use core dynamics only.
    set(BUILD_PYTHON_INTERFACE        OFF CACHE BOOL "" FORCE)
    set(BUILD_TESTING                 OFF CACHE BOOL "" FORCE)
    set(BUILD_BENCHMARK               OFF CACHE BOOL "" FORCE)
    set(BUILD_UTILS                   OFF CACHE BOOL "" FORCE)
    set(BUILD_WITH_URDF_SUPPORT       OFF CACHE BOOL "" FORCE)
    set(BUILD_WITH_PARSERS_SUPPORT    OFF CACHE BOOL "" FORCE)
    set(BUILD_WITH_HPP_FCL_SUPPORT    OFF CACHE BOOL "" FORCE)
    set(BUILD_WITH_COAL_SUPPORT       OFF CACHE BOOL "" FORCE)
    set(BUILD_WITH_COLLISION_SUPPORT  OFF CACHE BOOL "" FORCE)
    set(BUILD_WITH_AUTODIFF_SUPPORT   OFF CACHE BOOL "" FORCE)
    set(BUILD_WITH_CASADI_SUPPORT     OFF CACHE BOOL "" FORCE)
    set(BUILD_WITH_CODEGEN_SUPPORT    OFF CACHE BOOL "" FORCE)
    set(BUILD_WITH_OPENMP_SUPPORT     OFF CACHE BOOL "" FORCE)
    set(BUILD_WITH_EXTRA_SUPPORT      OFF CACHE BOOL "" FORCE)
    set(BUILD_WITH_SDF_SUPPORT        OFF CACHE BOOL "" FORCE)
    set(BUILD_WITH_MJCF_SUPPORT       OFF CACHE BOOL "" FORCE)
    set(BUILD_WITH_LIEGROUP_TESTS     OFF CACHE BOOL "" FORCE)
    set(INSTALL_DOCUMENTATION         OFF CACHE BOOL "" FORCE)
    set(GENERATE_PYTHON_STUBS         OFF CACHE BOOL "" FORCE)
    set(BUILD_ADVANCED_TESTING        OFF CACHE BOOL "" FORCE)
    set(BUILD_EXAMPLES                OFF CACHE BOOL "" FORCE)
    set(BUILD_WITH_COMMIT_VERSION     OFF CACHE BOOL "" FORCE)

    # Pinocchio universal arm64+x86_64 build: macOS multi-arch is fragile
    # under heavily-templated code (per-arch instantiations multiply
    # both compile time and OOM risk). We force host-arch only here and
    # plan a `lipo`-merge approach in the cross-platform deployment plan.
    if(APPLE AND DEFINED CMAKE_OSX_ARCHITECTURES)
        list(LENGTH CMAKE_OSX_ARCHITECTURES _n_arch)
        if(_n_arch GREATER 1)
            message(WARNING "Longeron: forcing host-arch only "
                "(was ${CMAKE_OSX_ARCHITECTURES}). Universal binary "
                "deferred to the cross-platform deployment plan.")
            execute_process(COMMAND uname -m OUTPUT_VARIABLE _host_arch
                            OUTPUT_STRIP_TRAILING_WHITESPACE)
            set(CMAKE_OSX_ARCHITECTURES "${_host_arch}" CACHE STRING "" FORCE)
        endif()
    endif()

    add_subdirectory(third_party/pinocchio EXCLUDE_FROM_ALL)
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

if(LONGERON_HAS_PINOCCHIO)
    # Link to pinocchio_headers (INTERFACE target) rather than the
    # precompiled-template pinocchio_default shared lib. With headers-only,
    # template instantiations happen in our single TU (aba.cpp) and only
    # the symbols we touch get pulled in. Pinocchio_default precompiles
    # ALL of {aba, rnea, crba, contact-dynamics, *-derivatives, ...} for
    # the (double, 0) instantiation and ships ~67 MB of symbols, of which
    # we use ~5%. Headers-only + LTO trades aba.cpp compile time
    # (instantiations move into our TU) for an order-of-magnitude smaller
    # final binary and elimination of the runtime libpinocchio_default
    # dylib dependency.
    target_link_libraries(longeron_native PRIVATE pinocchio_headers)
    target_compile_definitions(longeron_native PRIVATE
        LONGERON_HAS_PINOCCHIO=1
    )

    # Link-Time Optimization for cross-TU dead-code elimination on the
    # Pinocchio templates that are instantiated in aba.cpp but unused
    # downstream. macOS ld also runs -dead_strip in Release; LTO catches
    # the cases dead_strip alone misses (e.g. internal helpers used only
    # in non-instantiated paths).
    include(CheckIPOSupported)
    check_ipo_supported(RESULT _ipo_ok OUTPUT _ipo_err)
    if(_ipo_ok)
        set_target_properties(longeron_native PROPERTIES
            INTERPROCEDURAL_OPTIMIZATION_RELEASE        ON
            INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO ON)
        message(STATUS "Longeron: LTO enabled for longeron_native (Release/RelWithDebInfo)")
    else()
        message(WARNING "Longeron: LTO requested but not supported by toolchain: ${_ipo_err}")
    endif()

    # Belt-and-braces: explicit -dead_strip on macOS (clang already does
    # this in Release but we want to be sure under header-only Pinocchio
    # which leaves a lot of unused template instantiations to eliminate).
    # -dead_strip_dylibs drops LC_LOAD_DYLIB load commands for libraries
    # we don't actually reference. Pinocchio's pinocchio_headers PUBLIC-
    # links Boost::serialization for the sake of consumers that *use*
    # the serialization templates; we don't instantiate any of those, so
    # nm reports zero undefined Boost symbols and the load command is
    # pure dead weight at runtime.
    if(APPLE)
        target_link_options(longeron_native PRIVATE
            -Wl,-dead_strip
            -Wl,-dead_strip_dylibs)
    endif()

    # Strip local symbols from the final dylib. The header-only Pinocchio
    # path leaves __LINKEDIT (symbol/string tables) bloated with mangled
    # template instantiation names; `strip -x` removes locals while
    # keeping the exported C ABI. This typically halves __LINKEDIT.
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        find_program(STRIP_EXEC strip)
        if(STRIP_EXEC)
            add_custom_command(TARGET longeron_native POST_BUILD
                COMMAND ${STRIP_EXEC} -x $<TARGET_FILE:longeron_native>
                COMMENT "Stripping local symbols from longeron_native (Release)")
        endif()
    endif()
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
