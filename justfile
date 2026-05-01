# Longeron — top-level build orchestration

default:
    @just --list

# --- Native bridge (native/) ---

# Configure + build the C++ Jolt bridge (longeron_native.{dll,dylib,so}).
# `cmake_build_type` controls the CMake build type (default Release).
native-build cmake_build_type="Release":
    #!/usr/bin/env bash
    set -euo pipefail
    cd native
    cmake -B build -DCMAKE_BUILD_TYPE={{cmake_build_type}}
    # Pinocchio's template-heavy compilation uses 2-3 GB per parallel job;
    # default `-j` would launch one per core and OOM on a 24 GB box. Cap to
    # LONGERON_BUILD_JOBS (default 4) so peak RAM stays bounded. Override
    # for fast machines with `LONGERON_BUILD_JOBS=8 just native-build`.
    cmake --build build --config {{cmake_build_type}} -j ${LONGERON_BUILD_JOBS:-4}

native-clean:
    rm -rf native/build

# Print the loaded native library version (sanity check from the host
# platform — does not exercise KSP's Mono).
native-version: native-build
    #!/usr/bin/env bash
    set -euo pipefail
    case "$(uname)" in
        Darwin) lib=native/build/liblongeron_native.dylib ;;
        Linux)  lib=native/build/liblongeron_native.so ;;
        *)      lib=native/build/longeron_native.dll ;;
    esac
    if [ ! -f "$lib" ]; then
        echo "error: native lib not built at $lib" >&2
        exit 1
    fi
    nm -gU "$lib" 2>/dev/null | grep '_longeron' || nm "$lib" | grep ' T longeron_'

# --- C# (mod/) ---

mod-build config="Release":
    cd mod && dotnet build -c {{config}}

mod-test:
    cd mod && dotnet test Longeron.Physics.Tests/Longeron.Physics.Tests.csproj

# Run the native bridge probe (Phase 1 verification: kinematic-vs-static
# contact callbacks fire). Goes through `mono` because dotnet's net48
# testhost is unstable on macOS arm64 with custom native libs.
native-probe: native-build (mod-build "Release")
    cd mod/Longeron.Native.Probe/build && mono Longeron.Native.Probe.exe

mod-clean:
    cd mod && dotnet clean

# --- Release packaging ---

# Stage GameData/Longeron with managed DLLs + native lib (for the host
# platform) and zip into release/Longeron.zip. Multi-platform release
# (Win64 + macOS universal + Linux x64) is a separate matrix build; this
# recipe ships only the host platform's native artifact.
dist: (mod-build "Release") native-build
    #!/usr/bin/env bash
    set -euo pipefail
    stage=$(mktemp -d)
    root="$stage/GameData/Longeron"

    mkdir -p "$root/Plugins"
    cp mod/Longeron/build/Longeron.dll                  "$root/Plugins/"
    cp mod/Longeron.Physics/build/Longeron.Physics.dll  "$root/Plugins/"
    cp mod/Longeron.Native/build/Longeron.Native.dll    "$root/Plugins/"

    case "$(uname)" in
        Darwin) cp native/build/liblongeron_native.dylib "$root/Plugins/" ;;
        Linux)  cp native/build/liblongeron_native.so    "$root/Plugins/" ;;
        *)      cp native/build/longeron_native.dll      "$root/Plugins/" ;;
    esac

    cp LICENSE "$root/"

    mkdir -p release
    rm -f "{{justfile_directory()}}/release/Longeron.zip"
    cd "$stage"
    zip -qr "{{justfile_directory()}}/release/Longeron.zip" GameData/
    rm -rf "$stage"
    echo "Built → release/Longeron.zip"

# --- Install into KSP ---

# Build and install into a KSP directory.
#   just install ~/KSP_osx
install ksp_path: dist
    #!/usr/bin/env bash
    set -euo pipefail
    ksp="{{ksp_path}}"

    if [ ! -d "$ksp" ]; then
        echo "error: KSP directory not found: $ksp" >&2
        exit 1
    fi

    rm -rf "$ksp/GameData/Longeron"
    unzip -qo release/Longeron.zip -d "$ksp"

    echo "Installed → $ksp/GameData/Longeron"

# --- All ---

build: native-build mod-build

clean: native-clean mod-clean

check:
    cd mod && dotnet build --no-restore
