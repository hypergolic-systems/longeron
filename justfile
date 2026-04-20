# Longeron — top-level build orchestration

default:
    @just --list

# --- C# (mod/) ---

mod-build config="Release":
    cd mod && dotnet build -c {{config}}

mod-test:
    cd mod && dotnet test Longeron.Physics.Tests/Longeron.Physics.Tests.csproj

mod-clean:
    cd mod && dotnet clean

# --- Release packaging ---

dist: (mod-build "Release")
    #!/usr/bin/env bash
    set -euo pipefail
    stage=$(mktemp -d)
    root="$stage/GameData/Longeron"

    mkdir -p "$root/Plugins"
    cp mod/Longeron/build/Longeron.dll                  "$root/Plugins/"
    cp mod/Longeron.Physics/build/Longeron.Physics.dll  "$root/Plugins/"

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

build: mod-build

check:
    cd mod && dotnet build --no-restore
