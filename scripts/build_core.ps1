# build_core.ps1 - Build C++ Core only

param(
    [string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot

Set-Location "$root\src\core"

if (-not (Test-Path "build")) {
    cmake -B build -G "Visual Studio 17 2022" -A x64 `
        -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake"
}

cmake --build build --config $Configuration

Set-Location $root
