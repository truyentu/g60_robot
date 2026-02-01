# Robot Controller

6-DOF Robot Controller for MIG/MAG Welding Applications.

## Architecture

- **UI Layer:** C# WPF (.NET 8)
- **Core Layer:** C++ 17/20
- **Firmware:** Teensy 4.1 + grblHAL

## Prerequisites

- Visual Studio 2022
- CMake 3.20+
- .NET 8 SDK
- vcpkg (for C++ dependencies)

## Build Instructions

### C++ Core

```powershell
cd src/core
cmake -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

### C# UI

```powershell
cd src/ui
dotnet restore
dotnet build
```

## Project Structure

```
Robot_controller/
├── src/
│   ├── ui/           # C# WPF Application
│   ├── core/         # C++ Core Logic
│   └── firmware/     # Teensy Firmware
├── config/           # Configuration files
├── resources/        # 3D models, icons
├── docs/             # Documentation
└── scripts/          # Build scripts
```

## Documentation

See [docs/00_MASTER_ROADMAP.md](docs/00_MASTER_ROADMAP.md) for full documentation.

## License

Proprietary - All Rights Reserved
