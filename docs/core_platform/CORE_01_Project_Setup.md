# CORE_01: Project Setup

## Document Info
| Item | Value |
|------|-------|
| **Module** | Project Setup & Build System |
| **Layer** | All Layers (UI, Core, Firmware) |
| **Priority** | P0 - Foundation |
| **Dependencies** | None (First module) |
| **Last Updated** | 2026-02-01 |

---

## 1. Overview

### 1.1. Purpose
Thiết lập cấu trúc dự án, build system, và development environment cho toàn bộ Robot Controller project. Đây là module nền tảng, phải hoàn thành trước tất cả các module khác.

### 1.2. Scope
- **C++ Core**: CMake build system với vcpkg package manager
- **C# UI**: .NET 6+ solution với WPF
- **Firmware**: PlatformIO cho Teensy 4.1
- **Integration**: Build scripts để build toàn bộ project

### 1.3. Architecture Context

```
┌─────────────────────────────────────────────────────────────────┐
│                     DEVELOPMENT ENVIRONMENT                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                    Visual Studio 2022                     │   │
│  │  ┌─────────────────┐  ┌─────────────────────────────────┐│   │
│  │  │   C# Solution   │  │         CMake Project           ││   │
│  │  │   (.sln)        │  │         (CMakeLists.txt)        ││   │
│  │  │                 │  │                                  ││   │
│  │  │  ├─ RobotUI     │  │  ├─ RobotCore (static lib)      ││   │
│  │  │  │  (WPF App)   │  │  │  ├─ Kinematics               ││   │
│  │  │  │              │  │  │  ├─ Trajectory               ││   │
│  │  │  └─ RobotUI.    │  │  │  ├─ StateManager             ││   │
│  │  │     Tests       │  │  │  └─ ...                      ││   │
│  │  │                 │  │  │                               ││   │
│  │  │                 │  │  ├─ RobotBridge (C++/CLI DLL)   ││   │
│  │  │                 │  │  │                               ││   │
│  │  │                 │  │  └─ RobotCore.Tests             ││   │
│  │  └─────────────────┘  └─────────────────────────────────┘│   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                     PlatformIO IDE                        │   │
│  │  ┌─────────────────────────────────────────────────────┐ │   │
│  │  │              Firmware (Teensy 4.1)                   │ │   │
│  │  │              grblHAL + Custom Extensions             │ │   │
│  │  └─────────────────────────────────────────────────────┘ │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Project Structure

### 2.1. Root Directory Layout

```
📁 Robot_controller/
│
├── 📄 CMakeLists.txt              # Root CMake (C++ projects)
├── 📄 CMakePresets.json           # CMake presets (Debug/Release)
├── 📄 vcpkg.json                  # C++ dependencies manifest
├── 📄 .gitignore                  # Git ignore rules
├── 📄 .clang-format               # C++ code formatting
├── 📄 .editorconfig               # Editor settings
│
├── 📁 src/
│   ├── 📁 core/                   # C++ Core Logic
│   │   ├── 📄 CMakeLists.txt
│   │   ├── 📁 include/            # Public headers
│   │   │   └── 📁 robot/
│   │   │       ├── 📄 kinematics.hpp
│   │   │       ├── 📄 trajectory.hpp
│   │   │       ├── 📄 state_manager.hpp
│   │   │       ├── 📄 config.hpp
│   │   │       ├── 📄 ipc_server.hpp
│   │   │       └── 📄 grbl_driver.hpp
│   │   │
│   │   └── 📁 src/                # Implementation files
│   │       ├── 📄 kinematics.cpp
│   │       ├── 📄 trajectory.cpp
│   │       ├── 📄 state_manager.cpp
│   │       ├── 📄 config.cpp
│   │       ├── 📄 ipc_server.cpp
│   │       └── 📄 grbl_driver.cpp
│   │
│   ├── 📁 bridge/                 # C++/CLI Bridge (Optional)
│   │   ├── 📄 CMakeLists.txt
│   │   └── 📁 src/
│   │       └── 📄 robot_bridge.cpp
│   │
│   ├── 📁 ui/                     # C# WPF Application
│   │   ├── 📄 RobotController.sln
│   │   ├── 📁 RobotUI/
│   │   │   ├── 📄 RobotUI.csproj
│   │   │   ├── 📄 App.xaml
│   │   │   ├── 📄 MainWindow.xaml
│   │   │   ├── 📁 Views/
│   │   │   ├── 📁 ViewModels/
│   │   │   ├── 📁 Models/
│   │   │   ├── 📁 Services/
│   │   │   └── 📁 Assets/
│   │   │
│   │   └── 📁 RobotUI.Tests/
│   │       └── 📄 RobotUI.Tests.csproj
│   │
│   └── 📁 firmware/               # Teensy 4.1 Firmware
│       ├── 📄 platformio.ini
│       ├── 📁 src/
│       │   └── 📄 main.cpp
│       ├── 📁 lib/
│       │   └── 📁 grblHAL/
│       └── 📁 include/
│
├── 📁 config/                     # Configuration files
│   ├── 📄 robot_config.yaml       # Robot parameters
│   ├── 📄 app_config.yaml         # Application settings
│   └── 📁 robots/                 # Robot-specific configs
│       └── 📄 default_6dof.yaml
│
├── 📁 docs/                       # Documentation
│   ├── 📄 00_MASTER_ROADMAP.md
│   ├── 📁 core/
│   ├── 📁 phases/
│   └── 📁 modes/
│
├── 📁 tests/                      # Integration tests
│   ├── 📁 cpp/
│   │   ├── 📄 CMakeLists.txt
│   │   └── 📁 integration/
│   └── 📁 scripts/
│       └── 📄 run_all_tests.ps1
│
├── 📁 scripts/                    # Build & utility scripts
│   ├── 📄 build_all.ps1
│   ├── 📄 setup_dev.ps1
│   └── 📄 deploy.ps1
│
├── 📁 tools/                      # Development tools
│   └── 📁 vcpkg/                  # vcpkg submodule
│
└── 📁 out/                        # Build output (gitignored)
    ├── 📁 build/
    │   ├── 📁 Debug/
    │   └── 📁 Release/
    └── 📁 install/
```

---

## 3. Build System Configuration

### 3.1. CMake Root Configuration

**CMakeLists.txt** (Root):

```cmake
cmake_minimum_required(VERSION 3.20)

# ==============================================================================
# Project Configuration
# ==============================================================================
project(RobotController
    VERSION 1.0.0
    LANGUAGES CXX
    DESCRIPTION "6-DOF Robot Controller"
)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands for IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ==============================================================================
# Build Options
# ==============================================================================
option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_BRIDGE "Build C++/CLI bridge (Windows only)" OFF)
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
option(ENABLE_SIMULATION "Enable simulation mode" ON)

# ==============================================================================
# Output Directories
# ==============================================================================
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# ==============================================================================
# vcpkg Integration
# ==============================================================================
if(DEFINED ENV{VCPKG_ROOT})
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "vcpkg toolchain file")
endif()

# Windows-specific: Use static CRT for Release
if(MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

# ==============================================================================
# Find Dependencies
# ==============================================================================
find_package(Eigen3 CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(cppzmq CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

# Ruckig (header-only or find_package)
find_package(ruckig CONFIG QUIET)
if(NOT ruckig_FOUND)
    # Fallback: Use FetchContent
    include(FetchContent)
    FetchContent_Declare(
        ruckig
        GIT_REPOSITORY https://github.com/pantor/ruckig.git
        GIT_TAG v0.9.2
    )
    FetchContent_MakeAvailable(ruckig)
endif()

# Robotics Library (RL) - May need custom find module
find_package(RL QUIET)
if(NOT RL_FOUND)
    message(STATUS "Robotics Library not found via find_package, using manual path")
    # Set RL_INCLUDE_DIR and RL_LIBRARIES manually if needed
endif()

# ==============================================================================
# Add Subdirectories
# ==============================================================================
add_subdirectory(src/core)

if(BUILD_BRIDGE AND WIN32)
    add_subdirectory(src/bridge)
endif()

if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests/cpp)
endif()

# ==============================================================================
# Installation
# ==============================================================================
include(GNUInstallDirs)

install(TARGETS RobotCore
    EXPORT RobotControllerTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY src/core/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
```

### 3.2. Core Library CMake

**src/core/CMakeLists.txt**:

```cmake
# ==============================================================================
# RobotCore Static Library
# ==============================================================================

set(CORE_SOURCES
    src/kinematics.cpp
    src/trajectory.cpp
    src/state_manager.cpp
    src/config.cpp
    src/ipc_server.cpp
    src/grbl_driver.cpp
)

set(CORE_HEADERS
    include/robot/kinematics.hpp
    include/robot/trajectory.hpp
    include/robot/state_manager.hpp
    include/robot/config.hpp
    include/robot/ipc_server.hpp
    include/robot/grbl_driver.hpp
    include/robot/types.hpp
    include/robot/safety.hpp
)

add_library(RobotCore STATIC ${CORE_SOURCES} ${CORE_HEADERS})

target_include_directories(RobotCore
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(RobotCore
    PUBLIC
        Eigen3::Eigen
        ruckig::ruckig
    PRIVATE
        spdlog::spdlog
        cppzmq
        yaml-cpp::yaml-cpp
        nlohmann_json::nlohmann_json
)

# Compiler warnings
if(MSVC)
    target_compile_options(RobotCore PRIVATE /W4 /WX-)
else()
    target_compile_options(RobotCore PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Precompiled headers (optional, for faster builds)
target_precompile_headers(RobotCore
    PRIVATE
        <Eigen/Core>
        <vector>
        <memory>
        <string>
        <optional>
)

# Define simulation mode
if(ENABLE_SIMULATION)
    target_compile_definitions(RobotCore PUBLIC SIMULATION_MODE=1)
endif()
```

### 3.3. CMake Presets

**CMakePresets.json**:

```json
{
    "version": 6,
    "cmakeMinimumRequired": {
        "major": 3,
        "minor": 20,
        "patch": 0
    },
    "configurePresets": [
        {
            "name": "base",
            "hidden": true,
            "generator": "Ninja",
            "binaryDir": "${sourceDir}/out/build/${presetName}",
            "installDir": "${sourceDir}/out/install/${presetName}",
            "cacheVariables": {
                "CMAKE_TOOLCHAIN_FILE": {
                    "type": "FILEPATH",
                    "value": "$env{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
                }
            }
        },
        {
            "name": "x64-debug",
            "displayName": "x64 Debug",
            "inherits": "base",
            "architecture": {
                "value": "x64",
                "strategy": "external"
            },
            "cacheVariables": {
                "CMAKE_BUILD_TYPE": "Debug",
                "BUILD_TESTS": "ON",
                "ENABLE_SIMULATION": "ON"
            }
        },
        {
            "name": "x64-release",
            "displayName": "x64 Release",
            "inherits": "base",
            "architecture": {
                "value": "x64",
                "strategy": "external"
            },
            "cacheVariables": {
                "CMAKE_BUILD_TYPE": "Release",
                "BUILD_TESTS": "OFF",
                "ENABLE_SIMULATION": "OFF"
            }
        },
        {
            "name": "x64-relwithdebinfo",
            "displayName": "x64 Release with Debug Info",
            "inherits": "base",
            "architecture": {
                "value": "x64",
                "strategy": "external"
            },
            "cacheVariables": {
                "CMAKE_BUILD_TYPE": "RelWithDebInfo",
                "BUILD_TESTS": "ON",
                "ENABLE_SIMULATION": "ON"
            }
        }
    ],
    "buildPresets": [
        {
            "name": "x64-debug",
            "configurePreset": "x64-debug"
        },
        {
            "name": "x64-release",
            "configurePreset": "x64-release"
        }
    ],
    "testPresets": [
        {
            "name": "x64-debug",
            "configurePreset": "x64-debug",
            "output": {
                "outputOnFailure": true
            }
        }
    ]
}
```

---

## 4. Dependencies Management

### 4.1. vcpkg Manifest

**vcpkg.json**:

```json
{
    "$schema": "https://raw.githubusercontent.com/microsoft/vcpkg-tool/main/docs/vcpkg.schema.json",
    "name": "robot-controller",
    "version": "1.0.0",
    "description": "6-DOF Robot Controller",
    "homepage": "https://github.com/your-org/robot-controller",
    "dependencies": [
        {
            "name": "eigen3",
            "version>=": "3.4.0"
        },
        {
            "name": "spdlog",
            "version>=": "1.12.0"
        },
        {
            "name": "cppzmq",
            "version>=": "4.10.0"
        },
        {
            "name": "zeromq",
            "version>=": "4.3.5"
        },
        {
            "name": "yaml-cpp",
            "version>=": "0.8.0"
        },
        {
            "name": "nlohmann-json",
            "version>=": "3.11.0"
        },
        {
            "name": "gtest",
            "version>=": "1.14.0"
        },
        {
            "name": "benchmark",
            "version>=": "1.8.0"
        }
    ],
    "overrides": [],
    "builtin-baseline": "2024.01.12"
}
```

### 4.2. Dependencies Summary

| Library | Version | Purpose | License |
|---------|---------|---------|---------|
| **Eigen3** | 3.4+ | Linear algebra, transforms | MPL2 |
| **Ruckig** | 0.9+ | Online trajectory generation | MIT |
| **Robotics Library** | Latest | IK/FK solver | BSD |
| **spdlog** | 1.12+ | Fast logging | MIT |
| **cppzmq** | 4.10+ | ZeroMQ C++ binding | MIT |
| **yaml-cpp** | 0.8+ | YAML config parsing | MIT |
| **nlohmann/json** | 3.11+ | JSON parsing | MIT |
| **Google Test** | 1.14+ | Unit testing | BSD-3 |

### 4.3. Robotics Library Setup

Robotics Library (RL) không có sẵn trên vcpkg, cần build thủ công:

```powershell
# Clone RL
git clone https://github.com/roboticslibrary/rl.git
cd rl

# Build with CMake
cmake -B build -G Ninja ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DCMAKE_INSTALL_PREFIX=C:/Libraries/rl

cmake --build build --config Release
cmake --install build
```

Sau đó thêm vào CMake:

```cmake
# CMakeLists.txt
list(APPEND CMAKE_PREFIX_PATH "C:/Libraries/rl")
find_package(RL REQUIRED COMPONENTS mdl kin)
target_link_libraries(RobotCore PRIVATE RL::mdl RL::kin)
```

---

## 5. C# UI Solution

### 5.1. Solution Structure

**src/ui/RobotController.sln**:

```
Solution 'RobotController'
│
├── RobotUI/                          # Main WPF Application
│   ├── RobotUI.csproj
│   ├── App.xaml
│   ├── MainWindow.xaml
│   ├── Views/
│   │   ├── JogView.xaml
│   │   ├── ProgramView.xaml
│   │   ├── ConfigView.xaml
│   │   └── Visualization3DView.xaml
│   ├── ViewModels/
│   │   ├── MainViewModel.cs
│   │   ├── JogViewModel.cs
│   │   └── ...
│   ├── Models/
│   │   ├── RobotState.cs
│   │   ├── JointPosition.cs
│   │   └── ...
│   └── Services/
│       ├── IPCClient.cs
│       ├── ConfigService.cs
│       └── RobotModelService.cs
│
├── RobotUI.Core/                     # Shared models/interfaces
│   └── RobotUI.Core.csproj
│
└── RobotUI.Tests/                    # Unit tests
    └── RobotUI.Tests.csproj
```

### 5.2. Main Project File

**RobotUI.csproj**:

```xml
<Project Sdk="Microsoft.NET.Sdk">

  <PropertyGroup>
    <OutputType>WinExe</OutputType>
    <TargetFramework>net8.0-windows</TargetFramework>
    <Nullable>enable</Nullable>
    <UseWPF>true</UseWPF>
    <ImplicitUsings>enable</ImplicitUsings>
    <ApplicationIcon>Assets\robot.ico</ApplicationIcon>
    <Version>1.0.0</Version>
    <AssemblyName>RobotController</AssemblyName>
    <RootNamespace>RobotUI</RootNamespace>
  </PropertyGroup>

  <!-- NuGet Packages -->
  <ItemGroup>
    <!-- MVVM -->
    <PackageReference Include="CommunityToolkit.Mvvm" Version="8.2.2" />

    <!-- 3D Visualization -->
    <PackageReference Include="HelixToolkit.Wpf" Version="2.25.0" />
    <PackageReference Include="HelixToolkit.Wpf.SharpDX" Version="2.25.0" />

    <!-- IPC -->
    <PackageReference Include="NetMQ" Version="4.0.1.13" />

    <!-- Logging -->
    <PackageReference Include="Serilog" Version="3.1.1" />
    <PackageReference Include="Serilog.Sinks.File" Version="5.0.0" />
    <PackageReference Include="Serilog.Sinks.Console" Version="5.0.1" />

    <!-- Configuration -->
    <PackageReference Include="YamlDotNet" Version="15.1.2" />

    <!-- DI -->
    <PackageReference Include="Microsoft.Extensions.DependencyInjection" Version="8.0.0" />
    <PackageReference Include="Microsoft.Extensions.Hosting" Version="8.0.0" />
  </ItemGroup>

  <!-- Post-build: Copy native DLLs -->
  <Target Name="CopyNativeDlls" AfterTargets="Build">
    <ItemGroup>
      <NativeDlls Include="$(SolutionDir)..\..\..\out\build\x64-release\bin\*.dll" />
    </ItemGroup>
    <Copy SourceFiles="@(NativeDlls)" DestinationFolder="$(OutputPath)" SkipUnchangedFiles="true" />
  </Target>

</Project>
```

### 5.3. App Configuration

**App.xaml.cs**:

```csharp
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;
using RobotUI.Services;
using RobotUI.ViewModels;
using Serilog;
using System.Windows;

namespace RobotUI;

public partial class App : Application
{
    private readonly IHost _host;

    public App()
    {
        // Configure Serilog
        Log.Logger = new LoggerConfiguration()
            .MinimumLevel.Debug()
            .WriteTo.Console()
            .WriteTo.File("logs/robotui-.log",
                rollingInterval: RollingInterval.Day,
                retainedFileCountLimit: 7)
            .CreateLogger();

        _host = Host.CreateDefaultBuilder()
            .UseSerilog()
            .ConfigureServices((context, services) =>
            {
                ConfigureServices(services);
            })
            .Build();
    }

    private void ConfigureServices(IServiceCollection services)
    {
        // Services
        services.AddSingleton<IConfigService, ConfigService>();
        services.AddSingleton<IIPCClient, IPCClient>();
        services.AddSingleton<IRobotModelService, RobotModelService>();

        // ViewModels
        services.AddTransient<MainViewModel>();
        services.AddTransient<JogViewModel>();
        services.AddTransient<ProgramViewModel>();
        services.AddTransient<ConfigViewModel>();

        // Views
        services.AddTransient<MainWindow>();
    }

    protected override async void OnStartup(StartupEventArgs e)
    {
        await _host.StartAsync();

        var mainWindow = _host.Services.GetRequiredService<MainWindow>();
        mainWindow.Show();

        base.OnStartup(e);
    }

    protected override async void OnExit(ExitEventArgs e)
    {
        Log.Information("Application shutting down");
        await _host.StopAsync();
        _host.Dispose();
        Log.CloseAndFlush();
        base.OnExit(e);
    }
}
```

---

## 6. Firmware Setup (PlatformIO)

### 6.1. PlatformIO Configuration

**src/firmware/platformio.ini**:

```ini
; PlatformIO Project Configuration File
; Robot Controller Firmware - Teensy 4.1 + grblHAL

[platformio]
default_envs = teensy41
src_dir = src
lib_dir = lib
include_dir = include

[env:teensy41]
platform = teensy
board = teensy41
framework = arduino

; Build settings
build_flags =
    -D GRBL_BUILD
    -D N_AXIS=6
    -D OVERRIDE_MY_MACHINE
    -D USB_SERIAL
    -D RX_BUFFER_SIZE=4096
    -D BLOCK_BUFFER_SIZE=128
    -D SEGMENT_BUFFER_SIZE=128
    -D ACCELERATION_TICKS_PER_SECOND=1000
    -Wall
    -Wextra
    -O2

; Upload settings
upload_protocol = teensy-cli
upload_speed = 115200

; Monitor settings
monitor_speed = 115200
monitor_filters =
    direct
    time

; Library dependencies
lib_deps =
    https://github.com/grblHAL/core.git
    https://github.com/grblHAL/Plugin_spindle.git

; Extra scripts
extra_scripts =
    pre:scripts/pre_build.py
    post:scripts/post_build.py
```

### 6.2. grblHAL Configuration

**src/firmware/include/my_machine.h**:

```c
/*
 * my_machine.h - Robot Controller 6-DOF Configuration
 * Override grblHAL defaults for 6-axis robot control
 */

#ifndef MY_MACHINE_H
#define MY_MACHINE_H

// ============================================================================
// Axis Configuration
// ============================================================================
#define N_AXIS 6                    // 6 DOF robot

#define X_AXIS 0                    // Joint 1 (Base)
#define Y_AXIS 1                    // Joint 2 (Shoulder)
#define Z_AXIS 2                    // Joint 3 (Elbow)
#define A_AXIS 3                    // Joint 4 (Wrist 1)
#define B_AXIS 4                    // Joint 5 (Wrist 2)
#define C_AXIS 5                    // Joint 6 (Wrist 3)

// ============================================================================
// Buffer Configuration - Critical for smooth motion
// ============================================================================
#define RX_BUFFER_SIZE 4096         // Serial receive buffer
#define BLOCK_BUFFER_SIZE 128       // Motion planner blocks
#define SEGMENT_BUFFER_SIZE 128     // Segment buffer

// ============================================================================
// Stepper Configuration
// ============================================================================
#define DEFAULT_STEPPING_INVERT_MASK 0
#define DEFAULT_DIRECTION_INVERT_MASK 0

// Steps per degree for each joint (calculate from motor + gearbox)
#define DEFAULT_X_STEPS_PER_MM 200.0  // Joint 1: steps/degree
#define DEFAULT_Y_STEPS_PER_MM 200.0  // Joint 2: steps/degree
#define DEFAULT_Z_STEPS_PER_MM 200.0  // Joint 3: steps/degree
#define DEFAULT_A_STEPS_PER_MM 200.0  // Joint 4: steps/degree
#define DEFAULT_B_STEPS_PER_MM 200.0  // Joint 5: steps/degree
#define DEFAULT_C_STEPS_PER_MM 200.0  // Joint 6: steps/degree

// ============================================================================
// Speed & Acceleration - Will be overridden by PC
// ============================================================================
#define DEFAULT_X_MAX_RATE 180.0      // deg/s
#define DEFAULT_Y_MAX_RATE 180.0
#define DEFAULT_Z_MAX_RATE 180.0
#define DEFAULT_A_MAX_RATE 360.0
#define DEFAULT_B_MAX_RATE 360.0
#define DEFAULT_C_MAX_RATE 360.0

#define DEFAULT_X_ACCELERATION 720.0  // deg/s²
#define DEFAULT_Y_ACCELERATION 720.0
#define DEFAULT_Z_ACCELERATION 720.0
#define DEFAULT_A_ACCELERATION 1440.0
#define DEFAULT_B_ACCELERATION 1440.0
#define DEFAULT_C_ACCELERATION 1440.0

// ============================================================================
// Planner Bypass - Let PC control trajectory
// ============================================================================
#define DEFAULT_JUNCTION_DEVIATION 100.0   // Very high = no blending
#define DEFAULT_ARC_TOLERANCE 0.001

// ============================================================================
// Safety
// ============================================================================
#define ENABLE_SAFETY_DOOR_INPUT_PIN
#define DEFAULT_SOFT_LIMIT_ENABLE 1
#define DEFAULT_HARD_LIMIT_ENABLE 1

// Joint limits (degrees)
#define DEFAULT_X_MAX_TRAVEL 360.0    // Joint 1: ±180°
#define DEFAULT_Y_MAX_TRAVEL 270.0    // Joint 2
#define DEFAULT_Z_MAX_TRAVEL 270.0    // Joint 3
#define DEFAULT_A_MAX_TRAVEL 360.0    // Joint 4: ±180°
#define DEFAULT_B_MAX_TRAVEL 240.0    // Joint 5
#define DEFAULT_C_MAX_TRAVEL 720.0    // Joint 6: ±360°

#endif // MY_MACHINE_H
```

---

## 7. Development Scripts

### 7.1. Setup Script

**scripts/setup_dev.ps1**:

```powershell
#Requires -Version 7.0
<#
.SYNOPSIS
    Setup development environment for Robot Controller project
.DESCRIPTION
    - Installs vcpkg if not present
    - Installs C++ dependencies
    - Restores .NET packages
    - Configures CMake
#>

param(
    [switch]$Force,
    [switch]$SkipVcpkg,
    [switch]$SkipDotnet
)

$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent $PSScriptRoot

Write-Host "=== Robot Controller Development Setup ===" -ForegroundColor Cyan

# -----------------------------------------------------------------------------
# 1. Check Prerequisites
# -----------------------------------------------------------------------------
Write-Host "`n[1/5] Checking prerequisites..." -ForegroundColor Yellow

$requirements = @{
    "cmake"   = "cmake --version"
    "ninja"   = "ninja --version"
    "git"     = "git --version"
    "dotnet"  = "dotnet --version"
}

foreach ($tool in $requirements.Keys) {
    try {
        Invoke-Expression $requirements[$tool] | Out-Null
        Write-Host "  ✓ $tool found" -ForegroundColor Green
    }
    catch {
        Write-Host "  ✗ $tool not found. Please install it." -ForegroundColor Red
        exit 1
    }
}

# -----------------------------------------------------------------------------
# 2. Setup vcpkg
# -----------------------------------------------------------------------------
if (-not $SkipVcpkg) {
    Write-Host "`n[2/5] Setting up vcpkg..." -ForegroundColor Yellow

    $vcpkgPath = Join-Path $ProjectRoot "tools/vcpkg"

    if (-not (Test-Path $vcpkgPath) -or $Force) {
        Write-Host "  Cloning vcpkg..."
        git clone https://github.com/Microsoft/vcpkg.git $vcpkgPath

        Push-Location $vcpkgPath
        .\bootstrap-vcpkg.bat
        Pop-Location
    }

    # Set environment variable
    $env:VCPKG_ROOT = $vcpkgPath
    [Environment]::SetEnvironmentVariable("VCPKG_ROOT", $vcpkgPath, "User")
    Write-Host "  ✓ VCPKG_ROOT set to $vcpkgPath" -ForegroundColor Green

    # Install dependencies
    Write-Host "  Installing C++ dependencies (this may take a while)..."
    Push-Location $ProjectRoot
    & "$vcpkgPath/vcpkg.exe" install --triplet x64-windows
    Pop-Location
    Write-Host "  ✓ Dependencies installed" -ForegroundColor Green
}

# -----------------------------------------------------------------------------
# 3. Configure CMake
# -----------------------------------------------------------------------------
Write-Host "`n[3/5] Configuring CMake..." -ForegroundColor Yellow

Push-Location $ProjectRoot
cmake --preset x64-debug
Pop-Location

Write-Host "  ✓ CMake configured" -ForegroundColor Green

# -----------------------------------------------------------------------------
# 4. Restore .NET packages
# -----------------------------------------------------------------------------
if (-not $SkipDotnet) {
    Write-Host "`n[4/5] Restoring .NET packages..." -ForegroundColor Yellow

    $slnPath = Join-Path $ProjectRoot "src/ui/RobotController.sln"
    if (Test-Path $slnPath) {
        dotnet restore $slnPath
        Write-Host "  ✓ .NET packages restored" -ForegroundColor Green
    }
    else {
        Write-Host "  ⚠ Solution not found at $slnPath" -ForegroundColor Yellow
    }
}

# -----------------------------------------------------------------------------
# 5. Final verification
# -----------------------------------------------------------------------------
Write-Host "`n[5/5] Verification..." -ForegroundColor Yellow

# Try a test build
Push-Location $ProjectRoot
$buildResult = cmake --build --preset x64-debug --target RobotCore 2>&1
Pop-Location

if ($LASTEXITCODE -eq 0) {
    Write-Host "  ✓ Test build successful" -ForegroundColor Green
}
else {
    Write-Host "  ✗ Test build failed. Check output above." -ForegroundColor Red
}

Write-Host "`n=== Setup Complete ===" -ForegroundColor Cyan
Write-Host "You can now:"
Write-Host "  1. Open VS2022: Robot_controller.sln"
Write-Host "  2. Build C++:   cmake --build --preset x64-debug"
Write-Host "  3. Build C#:    dotnet build src/ui/RobotController.sln"
```

### 7.2. Build All Script

**scripts/build_all.ps1**:

```powershell
#Requires -Version 7.0
<#
.SYNOPSIS
    Build entire Robot Controller project
.PARAMETER Config
    Build configuration: Debug, Release, RelWithDebInfo
.PARAMETER Clean
    Clean build directories before building
#>

param(
    [ValidateSet("Debug", "Release", "RelWithDebInfo")]
    [string]$Config = "Debug",

    [switch]$Clean,
    [switch]$SkipCpp,
    [switch]$SkipCSharp,
    [switch]$SkipFirmware
)

$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent $PSScriptRoot

$preset = switch ($Config) {
    "Debug"          { "x64-debug" }
    "Release"        { "x64-release" }
    "RelWithDebInfo" { "x64-relwithdebinfo" }
}

Write-Host "=== Building Robot Controller ($Config) ===" -ForegroundColor Cyan

# -----------------------------------------------------------------------------
# Clean if requested
# -----------------------------------------------------------------------------
if ($Clean) {
    Write-Host "`n[Clean] Removing build directories..." -ForegroundColor Yellow
    Remove-Item -Recurse -Force "$ProjectRoot/out/build" -ErrorAction SilentlyContinue
    Write-Host "  ✓ Cleaned" -ForegroundColor Green
}

# -----------------------------------------------------------------------------
# Build C++ Core
# -----------------------------------------------------------------------------
if (-not $SkipCpp) {
    Write-Host "`n[C++] Building Core Library..." -ForegroundColor Yellow

    Push-Location $ProjectRoot
    cmake --preset $preset
    cmake --build --preset $preset
    Pop-Location

    if ($LASTEXITCODE -eq 0) {
        Write-Host "  ✓ C++ build successful" -ForegroundColor Green
    }
    else {
        Write-Host "  ✗ C++ build failed" -ForegroundColor Red
        exit 1
    }
}

# -----------------------------------------------------------------------------
# Build C# UI
# -----------------------------------------------------------------------------
if (-not $SkipCSharp) {
    Write-Host "`n[C#] Building UI Application..." -ForegroundColor Yellow

    $slnPath = Join-Path $ProjectRoot "src/ui/RobotController.sln"
    $csharpConfig = if ($Config -eq "Debug") { "Debug" } else { "Release" }

    dotnet build $slnPath -c $csharpConfig

    if ($LASTEXITCODE -eq 0) {
        Write-Host "  ✓ C# build successful" -ForegroundColor Green
    }
    else {
        Write-Host "  ✗ C# build failed" -ForegroundColor Red
        exit 1
    }
}

# -----------------------------------------------------------------------------
# Build Firmware
# -----------------------------------------------------------------------------
if (-not $SkipFirmware) {
    Write-Host "`n[Firmware] Building Teensy firmware..." -ForegroundColor Yellow

    $firmwarePath = Join-Path $ProjectRoot "src/firmware"
    if (Test-Path $firmwarePath) {
        Push-Location $firmwarePath

        # Check if platformio is available
        if (Get-Command pio -ErrorAction SilentlyContinue) {
            pio run
            if ($LASTEXITCODE -eq 0) {
                Write-Host "  ✓ Firmware build successful" -ForegroundColor Green
            }
            else {
                Write-Host "  ✗ Firmware build failed" -ForegroundColor Red
            }
        }
        else {
            Write-Host "  ⚠ PlatformIO not found. Skipping firmware build." -ForegroundColor Yellow
        }

        Pop-Location
    }
    else {
        Write-Host "  ⚠ Firmware directory not found" -ForegroundColor Yellow
    }
}

# -----------------------------------------------------------------------------
# Summary
# -----------------------------------------------------------------------------
Write-Host "`n=== Build Complete ===" -ForegroundColor Cyan
Write-Host "Output directories:"
Write-Host "  C++ binaries: out/build/$preset/bin/"
Write-Host "  C# binaries:  src/ui/RobotUI/bin/$csharpConfig/"
Write-Host "  Firmware:     src/firmware/.pio/build/teensy41/"
```

---

## 8. Code Style & Formatting

### 8.1. C++ Clang-Format

**.clang-format**:

```yaml
---
Language: Cpp
BasedOnStyle: Google
IndentWidth: 4
TabWidth: 4
UseTab: Never
ColumnLimit: 100

# Alignment
AlignAfterOpenBracket: Align
AlignConsecutiveAssignments: false
AlignConsecutiveDeclarations: false
AlignOperands: true
AlignTrailingComments: true

# Braces
BreakBeforeBraces: Custom
BraceWrapping:
  AfterClass: true
  AfterControlStatement: false
  AfterEnum: false
  AfterFunction: true
  AfterNamespace: false
  AfterStruct: true
  BeforeCatch: false
  BeforeElse: false

# Indentation
AccessModifierOffset: -4
IndentCaseLabels: true
NamespaceIndentation: None

# Includes
IncludeBlocks: Regroup
IncludeCategories:
  - Regex: '^<robot/.*>'
    Priority: 1
  - Regex: '^<.*\.hpp>'
    Priority: 2
  - Regex: '^<.*>'
    Priority: 3
  - Regex: '.*'
    Priority: 4

# Other
AllowShortFunctionsOnASingleLine: Inline
AllowShortIfStatementsOnASingleLine: Never
AllowShortLoopsOnASingleLine: false
PointerAlignment: Left
SpaceAfterCStyleCast: false
SpaceBeforeParens: ControlStatements
...
```

### 8.2. EditorConfig

**.editorconfig**:

```ini
# EditorConfig - Robot Controller Project
root = true

[*]
indent_style = space
indent_size = 4
end_of_line = crlf
charset = utf-8
trim_trailing_whitespace = true
insert_final_newline = true

[*.md]
trim_trailing_whitespace = false

[*.{yml,yaml}]
indent_size = 2

[*.json]
indent_size = 2

[*.{cpp,hpp,h,c}]
indent_size = 4

[*.{cs,xaml}]
indent_size = 4
csharp_new_line_before_open_brace = all
csharp_new_line_before_else = true
csharp_new_line_before_catch = true
csharp_new_line_before_finally = true

[CMakeLists.txt]
indent_size = 4

[*.cmake]
indent_size = 4
```

---

## 9. Git Configuration

### 9.1. Git Ignore

**.gitignore**:

```gitignore
# ==============================================================================
# Build outputs
# ==============================================================================
out/
build/
bin/
obj/
*.exe
*.dll
*.so
*.dylib
*.lib
*.a
*.pdb
*.ilk

# ==============================================================================
# IDE files
# ==============================================================================
.vs/
.vscode/settings.json
.idea/
*.user
*.suo
*.sln.docstates

# CMake
CMakeCache.txt
CMakeFiles/
cmake_install.cmake
compile_commands.json
CTestTestfile.cmake
_deps/

# ==============================================================================
# Package managers
# ==============================================================================
# vcpkg (only keep manifest)
vcpkg_installed/
tools/vcpkg/

# NuGet
packages/
*.nupkg

# ==============================================================================
# PlatformIO
# ==============================================================================
.pio/
.pioenvs/
.piolibdeps/

# ==============================================================================
# Logs & temporary files
# ==============================================================================
logs/
*.log
*.tmp
*.temp
*.cache

# ==============================================================================
# OS files
# ==============================================================================
.DS_Store
Thumbs.db
desktop.ini

# ==============================================================================
# Secrets (NEVER commit these)
# ==============================================================================
*.env
.env.*
secrets/
credentials/
```

---

## 10. Tasks Breakdown

### 10.1. Task List

| ID | Task | Description | Status |
|----|------|-------------|--------|
| T01 | Create directory structure | Tạo cấu trúc thư mục theo spec | ⬜ |
| T02 | Setup CMake root | CMakeLists.txt + presets | ⬜ |
| T03 | Setup vcpkg | Clone và configure vcpkg | ⬜ |
| T04 | Create vcpkg.json | Dependencies manifest | ⬜ |
| T05 | Create Core CMake | src/core/CMakeLists.txt | ⬜ |
| T06 | Create C# solution | RobotController.sln + projects | ⬜ |
| T07 | Setup PlatformIO | platformio.ini + my_machine.h | ⬜ |
| T08 | Create dev scripts | setup_dev.ps1, build_all.ps1 | ⬜ |
| T09 | Configure formatting | .clang-format, .editorconfig | ⬜ |
| T10 | Test build | Verify full build works | ⬜ |

### 10.2. Verification Checklist

```powershell
# 1. CMake configuration works
cmake --preset x64-debug

# 2. C++ builds successfully
cmake --build --preset x64-debug

# 3. C# builds successfully
dotnet build src/ui/RobotController.sln

# 4. Tests run
ctest --preset x64-debug

# 5. Firmware builds (if PlatformIO installed)
cd src/firmware && pio run
```

---

## 11. References

### 11.1. Research Documents
- PROJECT BLUEPRINT: BỘ ĐIỀU KHIỂN ROBOT HÀN 6-DOF THƯƠNG MẠI.md

### 11.2. External Resources
| Resource | URL |
|----------|-----|
| CMake Documentation | https://cmake.org/documentation/ |
| vcpkg Documentation | https://vcpkg.io/en/docs/ |
| CMake Presets | https://cmake.org/cmake/help/latest/manual/cmake-presets.7.html |
| PlatformIO Docs | https://docs.platformio.org/ |
| .NET CLI | https://docs.microsoft.com/en-us/dotnet/core/tools/ |
| grblHAL | https://github.com/grblHAL |

### 11.3. Related Documents
| Document | Description |
|----------|-------------|
| [CORE_IPC.md](./CORE_IPC.md) | IPC layer implementation |
| [CORE_Config.md](./CORE_Config.md) | Configuration system |
| [PHASE_1_Foundation.md](../phases/PHASE_1_Foundation.md) | Phase 1 details |

---

*Document version: 1.0 | Last updated: 2026-02-01*
