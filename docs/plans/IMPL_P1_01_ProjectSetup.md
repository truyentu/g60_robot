# IMPL_P1_01: Project Setup & Build System

| Metadata      | Value                           |
|---------------|---------------------------------|
| Plan ID       | IMPL_P1_01                      |
| Covers Tasks  | P1-01, P1-02, P1-03             |
| Status        | DRAFT                           |
| Version       | 1.0                             |
| Created       | 2026-02-01                      |

---

## Required Reading (ĐỌC TRƯỚC KHI CODE)

| Priority | Document | Lý do |
|----------|----------|-------|
| P0 | `ressearch_doc_md/PROJECT BLUEPRINT_ BỘ ĐIỀU KHIỂN ROBOT HÀN 6-DOF THƯƠNG MẠI.md` | Hiểu architecture tổng quan, design decisions |

---

## Prerequisites

Trước khi bắt đầu, đảm bảo đã cài đặt:

| Tool | Version | Check Command | Download |
|------|---------|---------------|----------|
| Visual Studio 2022 | 17.x | `cl` | [VS Download](https://visualstudio.microsoft.com/) |
| CMake | 3.20+ | `cmake --version` | [CMake Download](https://cmake.org/download/) |
| .NET SDK | 8.0 | `dotnet --version` | [.NET Download](https://dotnet.microsoft.com/download) |
| Git | 2.x | `git --version` | [Git Download](https://git-scm.com/) |
| vcpkg | Latest | `vcpkg --version` | [vcpkg Guide](#step-12-install-vcpkg) |

### Verification Commands

```powershell
# Run in PowerShell to verify all tools
Write-Host "=== Checking Prerequisites ===" -ForegroundColor Cyan

# CMake
$cmake = cmake --version 2>&1
if ($LASTEXITCODE -eq 0) { Write-Host "[OK] CMake: $($cmake[0])" -ForegroundColor Green }
else { Write-Host "[MISSING] CMake" -ForegroundColor Red }

# .NET
$dotnet = dotnet --version 2>&1
if ($LASTEXITCODE -eq 0) { Write-Host "[OK] .NET SDK: $dotnet" -ForegroundColor Green }
else { Write-Host "[MISSING] .NET SDK" -ForegroundColor Red }

# Git
$git = git --version 2>&1
if ($LASTEXITCODE -eq 0) { Write-Host "[OK] Git: $git" -ForegroundColor Green }
else { Write-Host "[MISSING] Git" -ForegroundColor Red }

# MSVC (from VS Developer Command Prompt)
$cl = cl 2>&1
if ($cl -match "Microsoft") { Write-Host "[OK] MSVC Compiler found" -ForegroundColor Green }
else { Write-Host "[WARNING] MSVC not in PATH - use VS Developer Command Prompt" -ForegroundColor Yellow }
```

---

## PART A: Project Structure Setup (P1-01)

### Step 1: Create Root Directory Structure

**Working Directory:** `E:\DEV_CONTEXT_PROJECTs\Robot_controller`

```powershell
# Step 1.1: Create main directories
$root = "E:\DEV_CONTEXT_PROJECTs\Robot_controller"
Set-Location $root

# Source directories
New-Item -ItemType Directory -Force -Path "src\ui\RobotController.UI\Views"
New-Item -ItemType Directory -Force -Path "src\ui\RobotController.UI\ViewModels"
New-Item -ItemType Directory -Force -Path "src\ui\RobotController.UI\Services"
New-Item -ItemType Directory -Force -Path "src\ui\RobotController.UI\Models"
New-Item -ItemType Directory -Force -Path "src\ui\RobotController.Common\Messages"

New-Item -ItemType Directory -Force -Path "src\core\src\ipc"
New-Item -ItemType Directory -Force -Path "src\core\src\config"
New-Item -ItemType Directory -Force -Path "src\core\src\robot"
New-Item -ItemType Directory -Force -Path "src\core\src\logging"
New-Item -ItemType Directory -Force -Path "src\core\include\robot_controller"
New-Item -ItemType Directory -Force -Path "src\core\tests"

New-Item -ItemType Directory -Force -Path "src\firmware"

# Config & Resources
New-Item -ItemType Directory -Force -Path "config"
New-Item -ItemType Directory -Force -Path "resources\models"
New-Item -ItemType Directory -Force -Path "resources\icons"

# Scripts & Logs
New-Item -ItemType Directory -Force -Path "scripts"
New-Item -ItemType Directory -Force -Path "logs"

Write-Host "[OK] Directory structure created" -ForegroundColor Green
```

**Validation:**
```powershell
# Verify structure
tree /F src | Select-Object -First 50
```

**Expected Output:**
```
src
├── core
│   ├── include
│   │   └── robot_controller
│   ├── src
│   │   ├── config
│   │   ├── ipc
│   │   ├── logging
│   │   └── robot
│   └── tests
├── firmware
└── ui
    ├── RobotController.Common
    │   └── Messages
    └── RobotController.UI
        ├── Models
        ├── Services
        ├── ViewModels
        └── Views
```

---

### Step 2: Create .gitignore

**File:** `.gitignore`

```powershell
# Step 2.1: Create .gitignore
$gitignore = @"
# Build outputs
build/
out/
bin/
obj/
x64/
x86/
Debug/
Release/

# Visual Studio
.vs/
*.user
*.suo
*.userosscache
*.sln.docstates

# CMake
CMakeCache.txt
CMakeFiles/
cmake_install.cmake
Makefile
*.cmake

# .NET
*.dll
*.exe
*.pdb
project.lock.json

# Logs
logs/*.log
*.log

# IDE
.idea/
.vscode/
*.swp
*~

# Dependencies (managed by vcpkg/nuget)
packages/
vcpkg_installed/

# OS
.DS_Store
Thumbs.db

# Temporary
*.tmp
*.temp
"@

Set-Content -Path ".gitignore" -Value $gitignore -Encoding UTF8
Write-Host "[OK] .gitignore created" -ForegroundColor Green
```

---

### Step 3: Create README.md

**File:** `README.md`

```powershell
# Step 3.1: Create README.md
$readme = @"
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

``````powershell
cd src/core
cmake -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
``````

### C# UI

``````powershell
cd src/ui
dotnet restore
dotnet build
``````

## Project Structure

``````
Robot_controller/
├── src/
│   ├── ui/           # C# WPF Application
│   ├── core/         # C++ Core Logic
│   └── firmware/     # Teensy Firmware
├── config/           # Configuration files
├── resources/        # 3D models, icons
├── docs/             # Documentation
└── scripts/          # Build scripts
``````

## Documentation

See [docs/00_MASTER_ROADMAP.md](docs/00_MASTER_ROADMAP.md) for full documentation.

## License

Proprietary - All Rights Reserved
"@

Set-Content -Path "README.md" -Value $readme -Encoding UTF8
Write-Host "[OK] README.md created" -ForegroundColor Green
```

---

### Step 4: Initialize Git Repository

```powershell
# Step 4.1: Initialize git (if not already)
if (-not (Test-Path ".git")) {
    git init
    Write-Host "[OK] Git repository initialized" -ForegroundColor Green
} else {
    Write-Host "[SKIP] Git already initialized" -ForegroundColor Yellow
}

# Step 4.2: Initial commit
git add .gitignore README.md
git commit -m "Initial project setup

- Add directory structure
- Add .gitignore
- Add README.md

Co-Authored-By: Claude <noreply@anthropic.com>"
```

**Validation:**
```powershell
git status
git log --oneline -1
```

---

## PART B: C++ Build System - CMake (P1-02)

### Step 5: Install vcpkg (if not installed)

```powershell
# Step 5.1: Clone vcpkg (one-time setup)
$vcpkgPath = "C:\vcpkg"

if (-not (Test-Path $vcpkgPath)) {
    git clone https://github.com/Microsoft/vcpkg.git $vcpkgPath
    Set-Location $vcpkgPath
    .\bootstrap-vcpkg.bat

    # Add to PATH (run as Admin or add manually)
    [Environment]::SetEnvironmentVariable("PATH", "$env:PATH;$vcpkgPath", "User")

    Write-Host "[OK] vcpkg installed at $vcpkgPath" -ForegroundColor Green
} else {
    Write-Host "[SKIP] vcpkg already exists at $vcpkgPath" -ForegroundColor Yellow
}

# Return to project root
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller"
```

---

### Step 6: Install C++ Dependencies via vcpkg

```powershell
# Step 6.1: Install required packages
$vcpkg = "C:\vcpkg\vcpkg.exe"

# Core dependencies
& $vcpkg install spdlog:x64-windows
& $vcpkg install nlohmann-json:x64-windows
& $vcpkg install cppzmq:x64-windows
& $vcpkg install zeromq:x64-windows
& $vcpkg install yaml-cpp:x64-windows
& $vcpkg install gtest:x64-windows

Write-Host "[OK] All vcpkg packages installed" -ForegroundColor Green
```

**Validation:**
```powershell
& $vcpkg list
```

**Expected packages:**
```
nlohmann-json:x64-windows    3.11.x
spdlog:x64-windows           1.12.x
cppzmq:x64-windows           4.10.x
zeromq:x64-windows           4.3.x
yaml-cpp:x64-windows         0.8.x
gtest:x64-windows            1.14.x
```

---

### Step 7: Create CMakeLists.txt (Root)

**File:** `src/core/CMakeLists.txt`

```powershell
# Step 7.1: Create main CMakeLists.txt
$cmakeRoot = @"
cmake_minimum_required(VERSION 3.20)
project(RobotControllerCore VERSION 1.0.0 LANGUAGES CXX)

# ============================================================================
# Build Settings
# ============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands for IDE integration
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY `${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY `${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY `${CMAKE_BINARY_DIR}/lib)

# ============================================================================
# vcpkg Integration
# ============================================================================
if(DEFINED ENV{VCPKG_ROOT})
    set(CMAKE_TOOLCHAIN_FILE "`$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "Vcpkg toolchain file")
elseif(EXISTS "C:/vcpkg/scripts/buildsystems/vcpkg.cmake")
    set(CMAKE_TOOLCHAIN_FILE "C:/vcpkg/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "Vcpkg toolchain file")
endif()

# ============================================================================
# Find Dependencies
# ============================================================================
find_package(spdlog CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(cppzmq CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)
find_package(GTest CONFIG REQUIRED)

# ============================================================================
# Source Files
# ============================================================================
set(CORE_SOURCES
    src/main.cpp
    src/ipc/IpcServer.cpp
    src/config/ConfigManager.cpp
    src/robot/RobotModel.cpp
    src/logging/Logger.cpp
)

set(CORE_HEADERS
    include/robot_controller/core.hpp
    src/ipc/IpcServer.hpp
    src/ipc/Message.hpp
    src/config/ConfigManager.hpp
    src/config/RobotConfig.hpp
    src/robot/RobotModel.hpp
    src/robot/DHParameters.hpp
    src/logging/Logger.hpp
)

# ============================================================================
# Main Executable
# ============================================================================
add_executable(robot_core `${CORE_SOURCES} `${CORE_HEADERS})

target_include_directories(robot_core
    PRIVATE
        `${CMAKE_CURRENT_SOURCE_DIR}/src
        `${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(robot_core
    PRIVATE
        spdlog::spdlog
        nlohmann_json::nlohmann_json
        cppzmq
        yaml-cpp::yaml-cpp
)

# Windows-specific settings
if(WIN32)
    target_compile_definitions(robot_core PRIVATE
        _WIN32_WINNT=0x0A00  # Windows 10
        NOMINMAX             # Prevent min/max macro conflicts
    )
endif()

# ============================================================================
# Tests
# ============================================================================
enable_testing()

add_executable(robot_core_tests
    tests/test_main.cpp
    tests/test_config.cpp
    tests/test_ipc.cpp
    src/config/ConfigManager.cpp
    src/logging/Logger.cpp
)

target_include_directories(robot_core_tests
    PRIVATE
        `${CMAKE_CURRENT_SOURCE_DIR}/src
        `${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(robot_core_tests
    PRIVATE
        GTest::gtest
        GTest::gtest_main
        spdlog::spdlog
        nlohmann_json::nlohmann_json
        cppzmq
        yaml-cpp::yaml-cpp
)

include(GoogleTest)
gtest_discover_tests(robot_core_tests)

# ============================================================================
# Installation
# ============================================================================
install(TARGETS robot_core
    RUNTIME DESTINATION bin
)

install(DIRECTORY `${CMAKE_CURRENT_SOURCE_DIR}/../../config/
    DESTINATION config
)

# ============================================================================
# Summary
# ============================================================================
message(STATUS "")
message(STATUS "=== Robot Controller Core ===")
message(STATUS "Version:      `${PROJECT_VERSION}")
message(STATUS "C++ Standard: `${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type:   `${CMAKE_BUILD_TYPE}")
message(STATUS "")
"@

Set-Content -Path "src\core\CMakeLists.txt" -Value $cmakeRoot -Encoding UTF8
Write-Host "[OK] CMakeLists.txt created" -ForegroundColor Green
```

---

### Step 8: Create Placeholder Source Files (C++)

#### Step 8.1: main.cpp

**File:** `src/core/src/main.cpp`

```powershell
$mainCpp = @"
/**
 * @file main.cpp
 * @brief Robot Controller Core - Entry Point
 */

#include <iostream>
#include "logging/Logger.hpp"

int main(int argc, char* argv[]) {
    // Initialize logging
    robot_controller::Logger::init("../../logs/core.log", "debug");
    LOG_INFO("Robot Controller Core v1.0.0 starting...");

    LOG_INFO("Core initialized successfully");
    LOG_INFO("Press Ctrl+C to exit");

    // TODO: Initialize IPC Server
    // TODO: Initialize Config Manager
    // TODO: Main loop

    // Temporary: Keep running
    std::cout << "Robot Controller Core running. Press Enter to exit..." << std::endl;
    std::cin.get();

    LOG_INFO("Robot Controller Core shutting down");
    return 0;
}
"@

Set-Content -Path "src\core\src\main.cpp" -Value $mainCpp -Encoding UTF8
Write-Host "[OK] main.cpp created" -ForegroundColor Green
```

#### Step 8.2: Logger.hpp

**File:** `src/core/src/logging/Logger.hpp`

```powershell
$loggerHpp = @"
/**
 * @file Logger.hpp
 * @brief Logging framework wrapper using spdlog
 */

#pragma once

#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <memory>
#include <string>

namespace robot_controller {

class Logger {
public:
    /**
     * Initialize the logging system
     * @param log_file Path to log file
     * @param level Log level (trace, debug, info, warn, error)
     * @param max_size Maximum file size in bytes (default 10MB)
     * @param max_files Maximum number of rotated files
     */
    static void init(const std::string& log_file = "logs/core.log",
                     const std::string& level = "info",
                     size_t max_size = 10 * 1024 * 1024,
                     size_t max_files = 5);

    /**
     * Get the logger instance
     */
    static std::shared_ptr<spdlog::logger> get();

private:
    static std::shared_ptr<spdlog::logger> s_logger;
    static bool s_initialized;
};

} // namespace robot_controller

// Convenience macros
#define LOG_TRACE(...) ::robot_controller::Logger::get()->trace(__VA_ARGS__)
#define LOG_DEBUG(...) ::robot_controller::Logger::get()->debug(__VA_ARGS__)
#define LOG_INFO(...)  ::robot_controller::Logger::get()->info(__VA_ARGS__)
#define LOG_WARN(...)  ::robot_controller::Logger::get()->warn(__VA_ARGS__)
#define LOG_ERROR(...) ::robot_controller::Logger::get()->error(__VA_ARGS__)
"@

Set-Content -Path "src\core\src\logging\Logger.hpp" -Value $loggerHpp -Encoding UTF8
Write-Host "[OK] Logger.hpp created" -ForegroundColor Green
```

#### Step 8.3: Logger.cpp

**File:** `src/core/src/logging/Logger.cpp`

```powershell
$loggerCpp = @"
/**
 * @file Logger.cpp
 * @brief Logger implementation
 */

#include "Logger.hpp"
#include <vector>
#include <filesystem>

namespace robot_controller {

std::shared_ptr<spdlog::logger> Logger::s_logger = nullptr;
bool Logger::s_initialized = false;

void Logger::init(const std::string& log_file,
                  const std::string& level,
                  size_t max_size,
                  size_t max_files) {
    if (s_initialized) {
        return;
    }

    try {
        // Create logs directory if needed
        std::filesystem::path log_path(log_file);
        if (log_path.has_parent_path()) {
            std::filesystem::create_directories(log_path.parent_path());
        }

        // Create sinks
        std::vector<spdlog::sink_ptr> sinks;

        // Console sink (colored)
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%`$] [%s:%#] %v");
        sinks.push_back(console_sink);

        // File sink (rotating)
        auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            log_file, max_size, max_files);
        file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] [%s:%#] %v");
        sinks.push_back(file_sink);

        // Create logger
        s_logger = std::make_shared<spdlog::logger>("robot_core", sinks.begin(), sinks.end());

        // Set level
        if (level == "trace") s_logger->set_level(spdlog::level::trace);
        else if (level == "debug") s_logger->set_level(spdlog::level::debug);
        else if (level == "info") s_logger->set_level(spdlog::level::info);
        else if (level == "warn") s_logger->set_level(spdlog::level::warn);
        else if (level == "error") s_logger->set_level(spdlog::level::err);
        else s_logger->set_level(spdlog::level::info);

        // Flush on warn or above
        s_logger->flush_on(spdlog::level::warn);

        // Register as default
        spdlog::set_default_logger(s_logger);

        s_initialized = true;

    } catch (const spdlog::spdlog_ex& ex) {
        std::cerr << "Logger initialization failed: " << ex.what() << std::endl;
    }
}

std::shared_ptr<spdlog::logger> Logger::get() {
    if (!s_initialized) {
        init(); // Initialize with defaults
    }
    return s_logger;
}

} // namespace robot_controller
"@

Set-Content -Path "src\core\src\logging\Logger.cpp" -Value $loggerCpp -Encoding UTF8
Write-Host "[OK] Logger.cpp created" -ForegroundColor Green
```

#### Step 8.4: Placeholder files for other modules

```powershell
# IpcServer placeholder
$ipcServerHpp = @"
#pragma once
// TODO: Implement in IMPL_P1_02
namespace robot_controller {
class IpcServer {
public:
    void start() {}
    void stop() {}
};
}
"@
Set-Content -Path "src\core\src\ipc\IpcServer.hpp" -Value $ipcServerHpp -Encoding UTF8

$ipcServerCpp = @"
#include "IpcServer.hpp"
// TODO: Implement in IMPL_P1_02
"@
Set-Content -Path "src\core\src\ipc\IpcServer.cpp" -Value $ipcServerCpp -Encoding UTF8

# Message placeholder
$messageHpp = @"
#pragma once
// TODO: Implement in IMPL_P1_02
"@
Set-Content -Path "src\core\src\ipc\Message.hpp" -Value $messageHpp -Encoding UTF8

# ConfigManager placeholder
$configManagerHpp = @"
#pragma once
// TODO: Implement in IMPL_P1_03
namespace robot_controller {
class ConfigManager {
public:
    static ConfigManager& instance() {
        static ConfigManager instance;
        return instance;
    }
};
}
"@
Set-Content -Path "src\core\src\config\ConfigManager.hpp" -Value $configManagerHpp -Encoding UTF8

$configManagerCpp = @"
#include "ConfigManager.hpp"
// TODO: Implement in IMPL_P1_03
"@
Set-Content -Path "src\core\src\config\ConfigManager.cpp" -Value $configManagerCpp -Encoding UTF8

# RobotConfig placeholder
$robotConfigHpp = @"
#pragma once
// TODO: Implement in IMPL_P1_03
"@
Set-Content -Path "src\core\src\config\RobotConfig.hpp" -Value $robotConfigHpp -Encoding UTF8

# RobotModel placeholder
$robotModelHpp = @"
#pragma once
// TODO: Implement in IMPL_P1_04
namespace robot_controller {
class RobotModel {};
}
"@
Set-Content -Path "src\core\src\robot\RobotModel.hpp" -Value $robotModelHpp -Encoding UTF8

$robotModelCpp = @"
#include "RobotModel.hpp"
// TODO: Implement in IMPL_P1_04
"@
Set-Content -Path "src\core\src\robot\RobotModel.cpp" -Value $robotModelCpp -Encoding UTF8

# DHParameters placeholder
$dhParamsHpp = @"
#pragma once
// TODO: Implement in IMPL_P1_04
"@
Set-Content -Path "src\core\src\robot\DHParameters.hpp" -Value $dhParamsHpp -Encoding UTF8

# Core header
$coreHpp = @"
#pragma once
/**
 * @file core.hpp
 * @brief Main include file for Robot Controller Core
 */

#include \"../src/logging/Logger.hpp\"
#include \"../src/config/ConfigManager.hpp\"
#include \"../src/ipc/IpcServer.hpp\"
#include \"../src/robot/RobotModel.hpp\"
"@
Set-Content -Path "src\core\include\robot_controller\core.hpp" -Value $coreHpp -Encoding UTF8

Write-Host "[OK] All placeholder C++ files created" -ForegroundColor Green
```

---

### Step 9: Create Test Files

**File:** `src/core/tests/test_main.cpp`

```powershell
$testMainCpp = @"
/**
 * @file test_main.cpp
 * @brief Google Test main entry
 */

#include <gtest/gtest.h>

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
"@
Set-Content -Path "src\core\tests\test_main.cpp" -Value $testMainCpp -Encoding UTF8

$testConfigCpp = @"
/**
 * @file test_config.cpp
 * @brief Configuration tests
 */

#include <gtest/gtest.h>
#include \"config/ConfigManager.hpp\"

TEST(ConfigManager, SingletonInstance) {
    auto& instance1 = robot_controller::ConfigManager::instance();
    auto& instance2 = robot_controller::ConfigManager::instance();
    EXPECT_EQ(&instance1, &instance2);
}

// TODO: Add more tests in IMPL_P1_03
"@
Set-Content -Path "src\core\tests\test_config.cpp" -Value $testConfigCpp -Encoding UTF8

$testIpcCpp = @"
/**
 * @file test_ipc.cpp
 * @brief IPC tests
 */

#include <gtest/gtest.h>
#include \"ipc/IpcServer.hpp\"

TEST(IpcServer, CreateInstance) {
    robot_controller::IpcServer server;
    // Basic instantiation test
    SUCCEED();
}

// TODO: Add more tests in IMPL_P1_02
"@
Set-Content -Path "src\core\tests\test_ipc.cpp" -Value $testIpcCpp -Encoding UTF8

Write-Host "[OK] Test files created" -ForegroundColor Green
```

---

### Step 10: Build C++ Core

```powershell
# Step 10.1: Configure CMake
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller\src\core"

# Configure with vcpkg toolchain
cmake -B build -G "Visual Studio 17 2022" -A x64 `
    -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake"

# Check for errors
if ($LASTEXITCODE -ne 0) {
    Write-Host "[ERROR] CMake configure failed" -ForegroundColor Red
    exit 1
}
Write-Host "[OK] CMake configured successfully" -ForegroundColor Green
```

```powershell
# Step 10.2: Build
cmake --build build --config Release

if ($LASTEXITCODE -ne 0) {
    Write-Host "[ERROR] Build failed" -ForegroundColor Red
    exit 1
}
Write-Host "[OK] Build successful" -ForegroundColor Green
```

**Validation:**
```powershell
# Step 10.3: Verify executable exists
$exe = "src\core\build\bin\Release\robot_core.exe"
if (Test-Path $exe) {
    Write-Host "[OK] Executable found: $exe" -ForegroundColor Green
    & $exe
} else {
    Write-Host "[ERROR] Executable not found" -ForegroundColor Red
}
```

**Expected Output:**
```
[2026-02-01 ...] [info] Robot Controller Core v1.0.0 starting...
[2026-02-01 ...] [info] Core initialized successfully
[2026-02-01 ...] [info] Press Ctrl+C to exit
Robot Controller Core running. Press Enter to exit...
```

---

### Step 11: Run C++ Tests

```powershell
# Step 11.1: Run tests
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller\src\core"
cmake --build build --config Release --target robot_core_tests
ctest --test-dir build -C Release --output-on-failure

Write-Host "[OK] Tests completed" -ForegroundColor Green
```

**Expected Output:**
```
[==========] Running 2 tests from 2 test suites.
...
[  PASSED  ] 2 tests.
```

---

## PART C: C# Solution Setup (P1-03)

### Step 12: Create .NET Solution

```powershell
# Step 12.1: Navigate to UI folder
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller\src\ui"

# Step 12.2: Create solution
dotnet new sln -n RobotController

Write-Host "[OK] Solution created" -ForegroundColor Green
```

---

### Step 13: Create RobotController.Common Project

```powershell
# Step 13.1: Create class library
dotnet new classlib -n RobotController.Common -f net8.0
dotnet sln add RobotController.Common/RobotController.Common.csproj

# Step 13.2: Delete default Class1.cs
Remove-Item "RobotController.Common\Class1.cs" -ErrorAction SilentlyContinue

Write-Host "[OK] RobotController.Common project created" -ForegroundColor Green
```

**File:** `src/ui/RobotController.Common/RobotController.Common.csproj`

```powershell
$commonCsproj = @"
<Project Sdk="Microsoft.NET.Sdk">

  <PropertyGroup>
    <TargetFramework>net8.0</TargetFramework>
    <ImplicitUsings>enable</ImplicitUsings>
    <Nullable>enable</Nullable>
    <RootNamespace>RobotController.Common</RootNamespace>
  </PropertyGroup>

  <ItemGroup>
    <PackageReference Include="System.Text.Json" Version="8.0.0" />
  </ItemGroup>

</Project>
"@
Set-Content -Path "RobotController.Common\RobotController.Common.csproj" -Value $commonCsproj -Encoding UTF8
```

---

### Step 14: Create RobotController.UI Project (WPF)

```powershell
# Step 14.1: Create WPF project
dotnet new wpf -n RobotController.UI -f net8.0-windows
dotnet sln add RobotController.UI/RobotController.UI.csproj

# Step 14.2: Add reference to Common
dotnet add RobotController.UI/RobotController.UI.csproj reference RobotController.Common/RobotController.Common.csproj

Write-Host "[OK] RobotController.UI project created" -ForegroundColor Green
```

**File:** Update `src/ui/RobotController.UI/RobotController.UI.csproj`

```powershell
$uiCsproj = @"
<Project Sdk="Microsoft.NET.Sdk">

  <PropertyGroup>
    <OutputType>WinExe</OutputType>
    <TargetFramework>net8.0-windows</TargetFramework>
    <Nullable>enable</Nullable>
    <ImplicitUsings>enable</ImplicitUsings>
    <UseWPF>true</UseWPF>
    <ApplicationIcon>..\..\resources\icons\app.ico</ApplicationIcon>
    <RootNamespace>RobotController.UI</RootNamespace>
    <AssemblyName>RobotController</AssemblyName>
  </PropertyGroup>

  <ItemGroup>
    <!-- MVVM -->
    <PackageReference Include="CommunityToolkit.Mvvm" Version="8.2.2" />

    <!-- 3D Visualization -->
    <PackageReference Include="HelixToolkit.Wpf" Version="2.24.0" />

    <!-- IPC -->
    <PackageReference Include="NetMQ" Version="4.0.1.13" />

    <!-- Logging -->
    <PackageReference Include="Serilog" Version="3.1.1" />
    <PackageReference Include="Serilog.Sinks.Console" Version="5.0.1" />
    <PackageReference Include="Serilog.Sinks.File" Version="5.0.0" />

    <!-- DI -->
    <PackageReference Include="Microsoft.Extensions.Hosting" Version="8.0.0" />
  </ItemGroup>

  <ItemGroup>
    <ProjectReference Include="..\RobotController.Common\RobotController.Common.csproj" />
  </ItemGroup>

</Project>
"@
Set-Content -Path "RobotController.UI\RobotController.UI.csproj" -Value $uiCsproj -Encoding UTF8
```

---

### Step 15: Create Basic UI Files

#### Step 15.1: App.xaml

**File:** `src/ui/RobotController.UI/App.xaml`

```powershell
$appXaml = @"
<Application x:Class="RobotController.UI.App"
             xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
             xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml"
             StartupUri="Views/MainWindow.xaml">
    <Application.Resources>
        <!-- Global Resources -->
        <SolidColorBrush x:Key="PrimaryBackground" Color="#1E1E1E"/>
        <SolidColorBrush x:Key="SecondaryBackground" Color="#252526"/>
        <SolidColorBrush x:Key="BorderBrush" Color="#3F3F46"/>
        <SolidColorBrush x:Key="ForegroundBrush" Color="#CCCCCC"/>
        <SolidColorBrush x:Key="AccentBrush" Color="#007ACC"/>
    </Application.Resources>
</Application>
"@
Set-Content -Path "RobotController.UI\App.xaml" -Value $appXaml -Encoding UTF8
```

#### Step 15.2: App.xaml.cs

**File:** `src/ui/RobotController.UI/App.xaml.cs`

```powershell
$appXamlCs = @"
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;
using Serilog;
using System.Windows;

namespace RobotController.UI;

public partial class App : Application
{
    private IHost? _host;

    protected override async void OnStartup(StartupEventArgs e)
    {
        base.OnStartup(e);

        // Setup Serilog
        Log.Logger = new LoggerConfiguration()
            .MinimumLevel.Debug()
            .WriteTo.Console()
            .WriteTo.File("../../logs/ui.log",
                rollingInterval: RollingInterval.Day,
                retainedFileCountLimit: 7)
            .CreateLogger();

        Log.Information("Robot Controller UI starting...");

        // Build host with DI
        _host = Host.CreateDefaultBuilder()
            .UseSerilog()
            .ConfigureServices((context, services) =>
            {
                // TODO: Register services in IMPL_P1_02, IMPL_P1_03
                // services.AddSingleton<IIpcClientService, IpcClientService>();
                // services.AddSingleton<IConfigService, ConfigService>();

                // ViewModels
                services.AddSingleton<ViewModels.MainViewModel>();

                // Views
                services.AddSingleton<Views.MainWindow>();
            })
            .Build();

        await _host.StartAsync();

        // Show main window
        var mainWindow = _host.Services.GetRequiredService<Views.MainWindow>();
        mainWindow.Show();
    }

    protected override async void OnExit(ExitEventArgs e)
    {
        Log.Information("Robot Controller UI shutting down");

        if (_host != null)
        {
            await _host.StopAsync();
            _host.Dispose();
        }

        Log.CloseAndFlush();
        base.OnExit(e);
    }
}
"@
Set-Content -Path "RobotController.UI\App.xaml.cs" -Value $appXamlCs -Encoding UTF8
```

#### Step 15.3: MainWindow.xaml

**File:** `src/ui/RobotController.UI/Views/MainWindow.xaml`

```powershell
# Create Views folder structure
New-Item -ItemType Directory -Force -Path "RobotController.UI\Views"
New-Item -ItemType Directory -Force -Path "RobotController.UI\ViewModels"
New-Item -ItemType Directory -Force -Path "RobotController.UI\Services"
New-Item -ItemType Directory -Force -Path "RobotController.UI\Models"

# Delete default MainWindow if exists
Remove-Item "RobotController.UI\MainWindow.xaml" -ErrorAction SilentlyContinue
Remove-Item "RobotController.UI\MainWindow.xaml.cs" -ErrorAction SilentlyContinue

$mainWindowXaml = @"
<Window x:Class="RobotController.UI.Views.MainWindow"
        xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
        xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml"
        xmlns:d="http://schemas.microsoft.com/expression/blend/2008"
        xmlns:mc="http://schemas.openxmlformats.org/markup-compatibility/2006"
        xmlns:helix="http://helix-toolkit.org/wpf"
        xmlns:vm="clr-namespace:RobotController.UI.ViewModels"
        mc:Ignorable="d"
        Title="Robot Controller"
        Height="800" Width="1400"
        MinHeight="600" MinWidth="1000"
        Background="{StaticResource PrimaryBackground}"
        WindowStartupLocation="CenterScreen">

    <Window.DataContext>
        <vm:MainViewModel/>
    </Window.DataContext>

    <Grid>
        <Grid.RowDefinitions>
            <RowDefinition Height="Auto"/>  <!-- Menu -->
            <RowDefinition Height="Auto"/>  <!-- Toolbar -->
            <RowDefinition Height="*"/>     <!-- Content -->
            <RowDefinition Height="Auto"/>  <!-- Status Bar -->
        </Grid.RowDefinitions>

        <!-- Menu Bar -->
        <Menu Grid.Row="0" Background="{StaticResource SecondaryBackground}"
              Foreground="{StaticResource ForegroundBrush}">
            <MenuItem Header="_File">
                <MenuItem Header="_New Program"/>
                <MenuItem Header="_Open Program"/>
                <MenuItem Header="_Save Program"/>
                <Separator/>
                <MenuItem Header="E_xit" Click="MenuItem_Exit_Click"/>
            </MenuItem>
            <MenuItem Header="_Robot">
                <MenuItem Header="_Connect"/>
                <MenuItem Header="_Disconnect"/>
                <Separator/>
                <MenuItem Header="_Home All Axes"/>
            </MenuItem>
            <MenuItem Header="_View">
                <MenuItem Header="_3D Viewport" IsCheckable="True" IsChecked="True"/>
                <MenuItem Header="_Program Editor" IsCheckable="True"/>
                <MenuItem Header="_Diagnostics" IsCheckable="True"/>
            </MenuItem>
            <MenuItem Header="_Help">
                <MenuItem Header="_About"/>
            </MenuItem>
        </Menu>

        <!-- Toolbar -->
        <ToolBarTray Grid.Row="1" Background="{StaticResource SecondaryBackground}">
            <ToolBar Background="{StaticResource SecondaryBackground}">
                <Button Content="Connect" Margin="2"/>
                <Button Content="Home" Margin="2"/>
                <Separator/>
                <Button Content="Run" Margin="2"/>
                <Button Content="Pause" Margin="2"/>
                <Button Content="Stop" Margin="2"/>
            </ToolBar>
        </ToolBarTray>

        <!-- Main Content -->
        <Grid Grid.Row="2">
            <Grid.ColumnDefinitions>
                <ColumnDefinition Width="200"/>  <!-- Navigation -->
                <ColumnDefinition Width="*"/>    <!-- 3D Viewport -->
                <ColumnDefinition Width="250"/>  <!-- Properties -->
            </Grid.ColumnDefinitions>

            <!-- Navigation Panel -->
            <Border Grid.Column="0" Background="{StaticResource SecondaryBackground}"
                    BorderBrush="{StaticResource BorderBrush}" BorderThickness="0,0,1,0">
                <StackPanel Margin="5">
                    <TextBlock Text="Navigation" FontWeight="Bold"
                               Foreground="{StaticResource ForegroundBrush}" Margin="5"/>
                    <ListBox Background="Transparent" BorderThickness="0"
                             Foreground="{StaticResource ForegroundBrush}">
                        <ListBoxItem Content="Manual Jog" IsSelected="True"/>
                        <ListBoxItem Content="Program"/>
                        <ListBoxItem Content="I/O"/>
                        <ListBoxItem Content="Configuration"/>
                        <ListBoxItem Content="Diagnostics"/>
                    </ListBox>
                </StackPanel>
            </Border>

            <!-- 3D Viewport -->
            <helix:HelixViewport3D Grid.Column="1"
                                   x:Name="Viewport3D"
                                   Background="#1E1E1E"
                                   ShowCoordinateSystem="True"
                                   CoordinateSystemLabelForeground="White"
                                   ZoomExtentsWhenLoaded="True">

                <!-- Lighting -->
                <helix:DefaultLights/>

                <!-- Grid -->
                <helix:GridLinesVisual3D Width="2000" Length="2000"
                                         MajorDistance="100" MinorDistance="25"
                                         Thickness="0.5" Fill="Gray"/>

                <!-- Coordinate Axes -->
                <helix:CoordinateSystemVisual3D ArrowLengths="100"/>

                <!-- Robot Model Placeholder -->
                <ModelVisual3D x:Name="RobotModel">
                    <!-- TODO: Load robot model in IMPL_P1_04 -->
                    <ModelVisual3D.Content>
                        <GeometryModel3D>
                            <GeometryModel3D.Geometry>
                                <MeshGeometry3D Positions="-50,-50,0 50,-50,0 50,50,0 -50,50,0
                                                          -50,-50,100 50,-50,100 50,50,100 -50,50,100"
                                               TriangleIndices="0,1,2 0,2,3 4,6,5 4,7,6
                                                               0,4,5 0,5,1 2,6,7 2,7,3
                                                               0,3,7 0,7,4 1,5,6 1,6,2"/>
                            </GeometryModel3D.Geometry>
                            <GeometryModel3D.Material>
                                <DiffuseMaterial Brush="Orange"/>
                            </GeometryModel3D.Material>
                        </GeometryModel3D>
                    </ModelVisual3D.Content>
                </ModelVisual3D>

            </helix:HelixViewport3D>

            <!-- Properties Panel -->
            <Border Grid.Column="2" Background="{StaticResource SecondaryBackground}"
                    BorderBrush="{StaticResource BorderBrush}" BorderThickness="1,0,0,0">
                <ScrollViewer VerticalScrollBarVisibility="Auto">
                    <StackPanel Margin="10">
                        <TextBlock Text="Robot Status" FontWeight="Bold"
                                   Foreground="{StaticResource ForegroundBrush}" Margin="0,0,0,10"/>

                        <!-- Connection Status -->
                        <Border Background="#333333" CornerRadius="3" Padding="8" Margin="0,0,0,10">
                            <StackPanel>
                                <TextBlock Text="Connection" Foreground="Gray" FontSize="10"/>
                                <TextBlock Text="{Binding ConnectionStatus}"
                                           Foreground="{Binding ConnectionStatusColor}"
                                           FontWeight="Bold"/>
                            </StackPanel>
                        </Border>

                        <!-- Robot State -->
                        <Border Background="#333333" CornerRadius="3" Padding="8" Margin="0,0,0,10">
                            <StackPanel>
                                <TextBlock Text="State" Foreground="Gray" FontSize="10"/>
                                <TextBlock Text="{Binding RobotState}"
                                           Foreground="{StaticResource ForegroundBrush}"
                                           FontWeight="Bold"/>
                            </StackPanel>
                        </Border>

                        <!-- Joint Positions -->
                        <TextBlock Text="Joint Positions" FontWeight="Bold"
                                   Foreground="{StaticResource ForegroundBrush}" Margin="0,10,0,5"/>
                        <ItemsControl ItemsSource="{Binding JointPositions}">
                            <ItemsControl.ItemTemplate>
                                <DataTemplate>
                                    <Grid Margin="0,2">
                                        <Grid.ColumnDefinitions>
                                            <ColumnDefinition Width="40"/>
                                            <ColumnDefinition Width="*"/>
                                        </Grid.ColumnDefinitions>
                                        <TextBlock Text="{Binding Name}" Foreground="Gray"/>
                                        <TextBlock Grid.Column="1" Text="{Binding Value, StringFormat={}{0:F2}°}"
                                                   Foreground="{StaticResource ForegroundBrush}"
                                                   HorizontalAlignment="Right"/>
                                    </Grid>
                                </DataTemplate>
                            </ItemsControl.ItemTemplate>
                        </ItemsControl>

                        <!-- TCP Position -->
                        <TextBlock Text="TCP Position" FontWeight="Bold"
                                   Foreground="{StaticResource ForegroundBrush}" Margin="0,15,0,5"/>
                        <Grid>
                            <Grid.ColumnDefinitions>
                                <ColumnDefinition Width="*"/>
                                <ColumnDefinition Width="*"/>
                            </Grid.ColumnDefinitions>
                            <Grid.RowDefinitions>
                                <RowDefinition/>
                                <RowDefinition/>
                                <RowDefinition/>
                            </Grid.RowDefinitions>
                            <TextBlock Grid.Row="0" Grid.Column="0" Text="X:" Foreground="Gray"/>
                            <TextBlock Grid.Row="0" Grid.Column="1" Text="{Binding TcpX, StringFormat={}{0:F1} mm}"
                                       Foreground="{StaticResource ForegroundBrush}" HorizontalAlignment="Right"/>
                            <TextBlock Grid.Row="1" Grid.Column="0" Text="Y:" Foreground="Gray"/>
                            <TextBlock Grid.Row="1" Grid.Column="1" Text="{Binding TcpY, StringFormat={}{0:F1} mm}"
                                       Foreground="{StaticResource ForegroundBrush}" HorizontalAlignment="Right"/>
                            <TextBlock Grid.Row="2" Grid.Column="0" Text="Z:" Foreground="Gray"/>
                            <TextBlock Grid.Row="2" Grid.Column="1" Text="{Binding TcpZ, StringFormat={}{0:F1} mm}"
                                       Foreground="{StaticResource ForegroundBrush}" HorizontalAlignment="Right"/>
                        </Grid>
                    </StackPanel>
                </ScrollViewer>
            </Border>
        </Grid>

        <!-- Status Bar -->
        <StatusBar Grid.Row="3" Background="{StaticResource SecondaryBackground}">
            <StatusBarItem>
                <StackPanel Orientation="Horizontal">
                    <Ellipse Width="8" Height="8" Fill="{Binding ConnectionStatusColor}" Margin="0,0,5,0"/>
                    <TextBlock Text="{Binding ConnectionStatus}" Foreground="{StaticResource ForegroundBrush}"/>
                </StackPanel>
            </StatusBarItem>
            <Separator/>
            <StatusBarItem>
                <TextBlock Text="{Binding RobotState}" Foreground="{StaticResource ForegroundBrush}"/>
            </StatusBarItem>
            <Separator/>
            <StatusBarItem>
                <TextBlock Text="Robot Controller v1.0.0" Foreground="Gray"/>
            </StatusBarItem>
        </StatusBar>
    </Grid>
</Window>
"@
Set-Content -Path "RobotController.UI\Views\MainWindow.xaml" -Value $mainWindowXaml -Encoding UTF8
Write-Host "[OK] MainWindow.xaml created" -ForegroundColor Green
```

#### Step 15.4: MainWindow.xaml.cs

**File:** `src/ui/RobotController.UI/Views/MainWindow.xaml.cs`

```powershell
$mainWindowCs = @"
using System.Windows;

namespace RobotController.UI.Views;

public partial class MainWindow : Window
{
    public MainWindow()
    {
        InitializeComponent();
    }

    private void MenuItem_Exit_Click(object sender, RoutedEventArgs e)
    {
        Application.Current.Shutdown();
    }
}
"@
Set-Content -Path "RobotController.UI\Views\MainWindow.xaml.cs" -Value $mainWindowCs -Encoding UTF8
Write-Host "[OK] MainWindow.xaml.cs created" -ForegroundColor Green
```

#### Step 15.5: MainViewModel.cs

**File:** `src/ui/RobotController.UI/ViewModels/MainViewModel.cs`

```powershell
$mainViewModelCs = @"
using CommunityToolkit.Mvvm.ComponentModel;
using System.Collections.ObjectModel;
using System.Windows.Media;

namespace RobotController.UI.ViewModels;

public partial class MainViewModel : ObservableObject
{
    [ObservableProperty]
    private string _connectionStatus = "Disconnected";

    [ObservableProperty]
    private Brush _connectionStatusColor = Brushes.Red;

    [ObservableProperty]
    private string _robotState = "IDLE";

    [ObservableProperty]
    private double _tcpX = 500.0;

    [ObservableProperty]
    private double _tcpY = 0.0;

    [ObservableProperty]
    private double _tcpZ = 600.0;

    public ObservableCollection<JointPosition> JointPositions { get; } = new()
    {
        new JointPosition { Name = "J1", Value = 0.0 },
        new JointPosition { Name = "J2", Value = -45.0 },
        new JointPosition { Name = "J3", Value = 90.0 },
        new JointPosition { Name = "J4", Value = 0.0 },
        new JointPosition { Name = "J5", Value = 45.0 },
        new JointPosition { Name = "J6", Value = 0.0 },
    };

    public MainViewModel()
    {
        // TODO: Initialize IPC client in IMPL_P1_02
        // TODO: Subscribe to status updates
    }
}

public class JointPosition
{
    public string Name { get; set; } = string.Empty;
    public double Value { get; set; }
}
"@
Set-Content -Path "RobotController.UI\ViewModels\MainViewModel.cs" -Value $mainViewModelCs -Encoding UTF8
Write-Host "[OK] MainViewModel.cs created" -ForegroundColor Green
```

---

### Step 16: Build C# Solution

```powershell
# Step 16.1: Restore packages
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller\src\ui"
dotnet restore

if ($LASTEXITCODE -ne 0) {
    Write-Host "[ERROR] Package restore failed" -ForegroundColor Red
    exit 1
}
Write-Host "[OK] Packages restored" -ForegroundColor Green
```

```powershell
# Step 16.2: Build solution
dotnet build --configuration Release

if ($LASTEXITCODE -ne 0) {
    Write-Host "[ERROR] Build failed" -ForegroundColor Red
    exit 1
}
Write-Host "[OK] Build successful" -ForegroundColor Green
```

**Validation:**
```powershell
# Step 16.3: Run the application
dotnet run --project RobotController.UI --configuration Release
```

**Expected:** WPF window appears with:
- Dark theme
- 3D viewport with grid and orange cube
- Navigation panel
- Status bar showing "Disconnected"

---

### Step 17: Create Build Scripts

**File:** `scripts/build_all.ps1`

```powershell
$buildAllScript = @"
# build_all.ps1 - Build entire Robot Controller project

param(
    [string]`$Configuration = "Release"
)

`$ErrorActionPreference = "Stop"
`$root = Split-Path -Parent `$PSScriptRoot

Write-Host "=== Building Robot Controller ===" -ForegroundColor Cyan
Write-Host "Configuration: `$Configuration"
Write-Host ""

# Build C++ Core
Write-Host "--- Building C++ Core ---" -ForegroundColor Yellow
Set-Location "`$root\src\core"

if (-not (Test-Path "build")) {
    cmake -B build -G "Visual Studio 17 2022" -A x64 ``
        -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake"
}

cmake --build build --config `$Configuration
if (`$LASTEXITCODE -ne 0) { throw "C++ build failed" }
Write-Host "[OK] C++ Core built" -ForegroundColor Green

# Build C# UI
Write-Host ""
Write-Host "--- Building C# UI ---" -ForegroundColor Yellow
Set-Location "`$root\src\ui"

dotnet build --configuration `$Configuration
if (`$LASTEXITCODE -ne 0) { throw "C# build failed" }
Write-Host "[OK] C# UI built" -ForegroundColor Green

Write-Host ""
Write-Host "=== Build Complete ===" -ForegroundColor Cyan

Set-Location `$root
"@
Set-Content -Path "scripts\build_all.ps1" -Value $buildAllScript -Encoding UTF8

$buildCoreScript = @"
# build_core.ps1 - Build C++ Core only

param(
    [string]`$Configuration = "Release"
)

`$ErrorActionPreference = "Stop"
`$root = Split-Path -Parent `$PSScriptRoot

Set-Location "`$root\src\core"

if (-not (Test-Path "build")) {
    cmake -B build -G "Visual Studio 17 2022" -A x64 ``
        -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake"
}

cmake --build build --config `$Configuration

Set-Location `$root
"@
Set-Content -Path "scripts\build_core.ps1" -Value $buildCoreScript -Encoding UTF8

$buildUiScript = @"
# build_ui.ps1 - Build C# UI only

param(
    [string]`$Configuration = "Release"
)

`$ErrorActionPreference = "Stop"
`$root = Split-Path -Parent `$PSScriptRoot

Set-Location "`$root\src\ui"
dotnet build --configuration `$Configuration

Set-Location `$root
"@
Set-Content -Path "scripts\build_ui.ps1" -Value $buildUiScript -Encoding UTF8

Write-Host "[OK] Build scripts created" -ForegroundColor Green
```

---

## Step 18: Final Verification

### 18.1 Full Build Test

```powershell
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller"
.\scripts\build_all.ps1 -Configuration Release
```

### 18.2 Verification Checklist

```powershell
$root = "E:\DEV_CONTEXT_PROJECTs\Robot_controller"
Write-Host "=== Verification Checklist ===" -ForegroundColor Cyan

# Check C++ executable
$cppExe = "$root\src\core\build\bin\Release\robot_core.exe"
if (Test-Path $cppExe) {
    Write-Host "[OK] C++ executable: $cppExe" -ForegroundColor Green
} else {
    Write-Host "[FAIL] C++ executable not found" -ForegroundColor Red
}

# Check C# executable
$csExe = "$root\src\ui\RobotController.UI\bin\Release\net8.0-windows\RobotController.exe"
if (Test-Path $csExe) {
    Write-Host "[OK] C# executable: $csExe" -ForegroundColor Green
} else {
    Write-Host "[FAIL] C# executable not found" -ForegroundColor Red
}

# Check logs directory
if (Test-Path "$root\logs") {
    Write-Host "[OK] Logs directory exists" -ForegroundColor Green
} else {
    Write-Host "[FAIL] Logs directory missing" -ForegroundColor Red
}

# Check config directory
if (Test-Path "$root\config") {
    Write-Host "[OK] Config directory exists" -ForegroundColor Green
} else {
    Write-Host "[FAIL] Config directory missing" -ForegroundColor Red
}
```

---

## Step 19: Git Commit

```powershell
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller"

git add .
git commit -m "IMPL_P1_01: Project Setup & Build System complete

- Create project directory structure
- Setup CMake build for C++ Core
- Setup .NET 8 solution for C# UI
- Add spdlog logging framework
- Add basic WPF MainWindow with HelixToolkit 3D viewport
- Add build scripts

Tasks completed: P1-01, P1-02, P1-03

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Completion Checklist

| Item | Status |
|------|--------|
| Directory structure created | [ ] |
| .gitignore created | [ ] |
| README.md created | [ ] |
| Git initialized | [ ] |
| vcpkg dependencies installed | [ ] |
| CMakeLists.txt created | [ ] |
| C++ placeholder files created | [ ] |
| C++ builds successfully | [ ] |
| C++ tests pass | [ ] |
| .NET solution created | [ ] |
| RobotController.Common project created | [ ] |
| RobotController.UI project created | [ ] |
| NuGet packages restored | [ ] |
| C# builds successfully | [ ] |
| WPF window displays correctly | [ ] |
| 3D viewport shows grid | [ ] |
| Build scripts created | [ ] |
| Final verification passed | [ ] |
| Git commit created | [ ] |

---

## Troubleshooting

### Problem: CMake cannot find vcpkg packages

**Solution:**
```powershell
# Verify VCPKG_ROOT
$env:VCPKG_ROOT = "C:\vcpkg"

# Or specify explicitly in cmake command
cmake -B build -DCMAKE_TOOLCHAIN_FILE="C:\vcpkg\scripts\buildsystems\vcpkg.cmake"
```

### Problem: HelixToolkit not rendering

**Solution:**
- Ensure GPU drivers are updated
- Check if running in VM (software rendering may be needed)
- Try adding to App.xaml.cs:
```csharp
RenderOptions.ProcessRenderMode = RenderMode.SoftwareOnly;
```

### Problem: NetMQ assembly not found

**Solution:**
```powershell
dotnet restore --force
dotnet build --no-incremental
```

### Problem: spdlog header not found

**Solution:**
```powershell
# Reinstall spdlog
vcpkg remove spdlog:x64-windows
vcpkg install spdlog:x64-windows
```

---

## Next Steps

After completing IMPL_P1_01:
1. Update IMPLEMENTATION_PLAN_TRACKER.md → Mark WRITE-01 as Done
2. Proceed to **IMPL_P1_02: IPC Layer**

---

*Document Version: 1.0 | Created: 2026-02-01*
