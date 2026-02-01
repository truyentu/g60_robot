# build_all.ps1 - Build entire Robot Controller project

param(
    [string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot

Write-Host "=== Building Robot Controller ===" -ForegroundColor Cyan
Write-Host "Configuration: $Configuration"
Write-Host ""

# Build C++ Core
Write-Host "--- Building C++ Core ---" -ForegroundColor Yellow
Set-Location "$root\src\core"

if (-not (Test-Path "build")) {
    cmake -B build -G "Visual Studio 17 2022" -A x64 `
        -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake"
}

cmake --build build --config $Configuration
if ($LASTEXITCODE -ne 0) { throw "C++ build failed" }
Write-Host "[OK] C++ Core built" -ForegroundColor Green

# Build C# UI
Write-Host ""
Write-Host "--- Building C# UI ---" -ForegroundColor Yellow
Set-Location "$root\src\ui"

dotnet build --configuration $Configuration
if ($LASTEXITCODE -ne 0) { throw "C# build failed" }
Write-Host "[OK] C# UI built" -ForegroundColor Green

Write-Host ""
Write-Host "=== Build Complete ===" -ForegroundColor Cyan

Set-Location $root
