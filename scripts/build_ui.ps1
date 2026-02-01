# build_ui.ps1 - Build C# UI only

param(
    [string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot

Set-Location "$root\src\ui"
dotnet build --configuration $Configuration

Set-Location $root
