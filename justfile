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
    cmake --build build --config {{cmake_build_type}} -j

native-clean:
    rm -rf native/build

# Print the loaded native library version (sanity check from the host
# platform — does not exercise KSP's Mono).
native-version: native-build
    #!/usr/bin/env bash
    set -euo pipefail
    case "$(uname)" in
        Darwin) lib=native/build/longeron_native.dylib ;;
        Linux)  lib=native/build/longeron_native.so ;;
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
        Darwin) cp native/build/longeron_native.dylib "$root/Plugins/" ;;
        Linux)  cp native/build/longeron_native.so    "$root/Plugins/" ;;
        *)      cp native/build/longeron_native.dll   "$root/Plugins/" ;;
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
