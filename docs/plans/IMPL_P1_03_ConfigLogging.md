# IMPL_P1_03: Config & Logging

| Metadata      | Value                           |
|---------------|---------------------------------|
| Plan ID       | IMPL_P1_03                      |
| Covers Tasks  | P1-07, P1-08, P1-09, P1-10      |
| Status        | DRAFT                           |
| Version       | 1.0                             |
| Created       | 2026-02-01                      |
| Prerequisites | IMPL_P1_01, IMPL_P1_02 completed |

---

## Required Reading (ĐỌC TRƯỚC KHI CODE)

| Priority | Document | Lý do |
|----------|----------|-------|
| - | (Không cần - sử dụng standard libraries) | yaml-cpp, spdlog, Serilog đều có docs chuẩn |

---

## Prerequisites

Trước khi bắt đầu, đảm bảo:

| Requirement | Check |
|-------------|-------|
| IMPL_P1_01 completed | Project structure exists |
| IMPL_P1_02 completed | IPC layer working |
| vcpkg packages | yaml-cpp, nlohmann-json installed |

### Verification

```powershell
# Verify yaml-cpp installed
C:\vcpkg\vcpkg.exe list | Select-String "yaml"

# Expected:
# yaml-cpp:x64-windows    0.8.x
```

---

## Overview

### Configuration Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    CONFIGURATION SYSTEM                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   config/                                                        │
│   ├── robot_config.yaml      ─────► C++ ConfigManager            │
│   ├── system_config.yaml     ─────► C++ ConfigManager            │
│   └── ui_config.json         ─────► C# ConfigService             │
│                                                                  │
│   ┌─────────────────────┐         ┌─────────────────────┐       │
│   │  C++ ConfigManager  │         │  C# ConfigService   │       │
│   │                     │         │                     │       │
│   │  • RobotConfig      │   IPC   │  • UiConfig         │       │
│   │  • SystemConfig     │◄───────►│  • ConnectionConfig │       │
│   │  • Load/Save YAML   │         │  • Load/Save JSON   │       │
│   │                     │         │                     │       │
│   └─────────────────────┘         └─────────────────────┘       │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Logging Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      LOGGING SYSTEM                              │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   C++ Core (spdlog)                C# UI (Serilog)              │
│   ┌─────────────────────┐         ┌─────────────────────┐       │
│   │  Logger             │         │  Log                │       │
│   │  • Console (color)  │         │  • Console          │       │
│   │  • File (rotating)  │         │  • File (daily)     │       │
│   │  • Configurable     │         │  • Structured       │       │
│   └─────────────────────┘         └─────────────────────┘       │
│           │                                │                     │
│           ▼                                ▼                     │
│   logs/core.log                    logs/ui.log                  │
│   logs/core.log.1                  logs/ui-20260201.log         │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## PART A: Configuration System - C++ (P1-07)

### Step 1: Create Config Directory and Sample Files

```powershell
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller"

# Ensure config directory exists
New-Item -ItemType Directory -Force -Path "config"

Write-Host "[OK] Config directory ready" -ForegroundColor Green
```

---

### Step 2: Create robot_config.yaml

**File:** `config/robot_config.yaml`

```powershell
$robotConfigYaml = @"
# Robot Configuration
# Robot Controller v1.0
# Last Modified: 2026-02-01

robot:
  # Robot identification
  name: "Robot6DOF"
  type: "6-axis-articulated"
  manufacturer: "Custom"
  model: "RC-6DOF-01"

  # DH Parameters (Modified DH Convention)
  # Units: a, d in mm; alpha, theta_offset in degrees
  dh_parameters:
    # Joint 1: Base rotation
    - joint: 1
      a: 0
      alpha: 0
      d: 400
      theta_offset: 0

    # Joint 2: Shoulder
    - joint: 2
      a: 25
      alpha: -90
      d: 0
      theta_offset: -90

    # Joint 3: Elbow
    - joint: 3
      a: 560
      alpha: 0
      d: 0
      theta_offset: 0

    # Joint 4: Wrist 1
    - joint: 4
      a: 35
      alpha: -90
      d: 515
      theta_offset: 0

    # Joint 5: Wrist 2
    - joint: 5
      a: 0
      alpha: 90
      d: 0
      theta_offset: 0

    # Joint 6: Wrist 3 (Tool flange)
    - joint: 6
      a: 0
      alpha: -90
      d: 80
      theta_offset: 0

  # Joint limits
  # Units: min/max in degrees, velocity in deg/s, acceleration in deg/s²
  joint_limits:
    - joint: 1
      min: -170
      max: 170
      max_velocity: 120
      max_acceleration: 500

    - joint: 2
      min: -120
      max: 60
      max_velocity: 120
      max_acceleration: 500

    - joint: 3
      min: -80
      max: 165
      max_velocity: 120
      max_acceleration: 500

    - joint: 4
      min: -185
      max: 185
      max_velocity: 190
      max_acceleration: 800

    - joint: 5
      min: -120
      max: 120
      max_velocity: 190
      max_acceleration: 800

    - joint: 6
      min: -360
      max: 360
      max_velocity: 250
      max_acceleration: 1000

  # TCP (Tool Center Point) offset from flange
  # Units: x, y, z in mm; rx, ry, rz in degrees
  tcp_offset:
    x: 0
    y: 0
    z: 150
    rx: 0
    ry: 0
    rz: 0

  # Home position (joint angles in degrees)
  home_position:
    - 0      # J1
    - -45    # J2
    - 90     # J3
    - 0      # J4
    - 45     # J5
    - 0      # J6

  # Payload
  max_payload_kg: 6.0

  # Work envelope
  reach_mm: 1200
"@

Set-Content -Path "config\robot_config.yaml" -Value $robotConfigYaml -Encoding UTF8
Write-Host "[OK] robot_config.yaml created" -ForegroundColor Green
```

---

### Step 3: Create system_config.yaml

**File:** `config/system_config.yaml`

```powershell
$systemConfigYaml = @"
# System Configuration
# Robot Controller v1.0
# Last Modified: 2026-02-01

system:
  # Version info
  version: "1.0.0"

  # IPC Settings
  ipc:
    rep_port: 5555          # REQ/REP port
    pub_port: 5556          # PUB/SUB port
    bind_address: "*"       # Bind to all interfaces
    heartbeat_ms: 1000      # Heartbeat interval

  # Logging Settings
  logging:
    level: "debug"          # trace, debug, info, warn, error
    file: "logs/core.log"
    max_size_mb: 10         # Max file size before rotation
    max_files: 5            # Number of rotated files to keep
    console_enabled: true
    file_enabled: true

  # Control Loop Settings
  control:
    cycle_time_ms: 4        # Main control loop cycle (250 Hz)
    interpolation_hz: 1000  # Trajectory interpolation rate
    status_publish_hz: 10   # Status publish rate

  # Serial Communication (to Teensy)
  serial:
    port: "AUTO"            # COM port or AUTO for auto-detect
    baudrate: 115200
    timeout_ms: 100

  # Safety Settings
  safety:
    e_stop_enabled: true
    soft_limits_enabled: true
    collision_check_enabled: false  # Future feature
    max_speed_percent: 100
"@

Set-Content -Path "config\system_config.yaml" -Value $systemConfigYaml -Encoding UTF8
Write-Host "[OK] system_config.yaml created" -ForegroundColor Green
```

---

### Step 4: Create RobotConfig.hpp

**File:** `src/core/src/config/RobotConfig.hpp`

```powershell
$robotConfigHpp = @"
/**
 * @file RobotConfig.hpp
 * @brief Robot configuration data structures
 */

#pragma once

#include <string>
#include <vector>
#include <array>

namespace robot_controller {
namespace config {

/**
 * DH Parameter for a single joint
 */
struct DHParameter {
    int joint = 0;
    double a = 0.0;             // Link length (mm)
    double alpha = 0.0;         // Link twist (degrees)
    double d = 0.0;             // Link offset (mm)
    double theta_offset = 0.0;  // Joint angle offset (degrees)

    // Convert alpha to radians
    double alphaRad() const { return alpha * 3.14159265358979 / 180.0; }
    // Convert theta_offset to radians
    double thetaOffsetRad() const { return theta_offset * 3.14159265358979 / 180.0; }
};

/**
 * Joint limits for a single joint
 */
struct JointLimit {
    int joint = 0;
    double min = -180.0;            // Min angle (degrees)
    double max = 180.0;             // Max angle (degrees)
    double max_velocity = 100.0;    // Max velocity (deg/s)
    double max_acceleration = 500.0; // Max acceleration (deg/s²)

    // Convert to radians
    double minRad() const { return min * 3.14159265358979 / 180.0; }
    double maxRad() const { return max * 3.14159265358979 / 180.0; }
    double maxVelocityRad() const { return max_velocity * 3.14159265358979 / 180.0; }
    double maxAccelerationRad() const { return max_acceleration * 3.14159265358979 / 180.0; }
};

/**
 * TCP (Tool Center Point) offset
 */
struct TcpOffset {
    double x = 0.0;     // mm
    double y = 0.0;     // mm
    double z = 0.0;     // mm
    double rx = 0.0;    // degrees
    double ry = 0.0;    // degrees
    double rz = 0.0;    // degrees
};

/**
 * Complete robot configuration
 */
struct RobotConfig {
    // Identification
    std::string name = "Robot6DOF";
    std::string type = "6-axis-articulated";
    std::string manufacturer = "Custom";
    std::string model = "RC-6DOF-01";

    // Kinematics
    std::vector<DHParameter> dh_parameters;
    std::vector<JointLimit> joint_limits;
    TcpOffset tcp_offset;

    // Home position (degrees)
    std::array<double, 6> home_position = {0, 0, 0, 0, 0, 0};

    // Specifications
    double max_payload_kg = 6.0;
    double reach_mm = 1200.0;

    /**
     * Get number of joints (should always be 6)
     */
    size_t numJoints() const {
        return dh_parameters.size();
    }

    /**
     * Check if configuration is valid
     */
    bool isValid() const {
        return dh_parameters.size() == 6 && joint_limits.size() == 6;
    }
};

} // namespace config
} // namespace robot_controller
"@

Set-Content -Path "src\core\src\config\RobotConfig.hpp" -Value $robotConfigHpp -Encoding UTF8
Write-Host "[OK] RobotConfig.hpp created" -ForegroundColor Green
```

---

### Step 5: Create SystemConfig.hpp

**File:** `src/core/src/config/SystemConfig.hpp`

```powershell
$systemConfigHpp = @"
/**
 * @file SystemConfig.hpp
 * @brief System configuration data structures
 */

#pragma once

#include <string>

namespace robot_controller {
namespace config {

/**
 * IPC configuration
 */
struct IpcConfig {
    int rep_port = 5555;
    int pub_port = 5556;
    std::string bind_address = "*";
    int heartbeat_ms = 1000;
};

/**
 * Logging configuration
 */
struct LoggingConfig {
    std::string level = "info";
    std::string file = "logs/core.log";
    int max_size_mb = 10;
    int max_files = 5;
    bool console_enabled = true;
    bool file_enabled = true;
};

/**
 * Control loop configuration
 */
struct ControlConfig {
    int cycle_time_ms = 4;
    int interpolation_hz = 1000;
    int status_publish_hz = 10;
};

/**
 * Serial communication configuration
 */
struct SerialConfig {
    std::string port = "AUTO";
    int baudrate = 115200;
    int timeout_ms = 100;
};

/**
 * Safety configuration
 */
struct SafetyConfig {
    bool e_stop_enabled = true;
    bool soft_limits_enabled = true;
    bool collision_check_enabled = false;
    int max_speed_percent = 100;
};

/**
 * Complete system configuration
 */
struct SystemConfig {
    std::string version = "1.0.0";
    IpcConfig ipc;
    LoggingConfig logging;
    ControlConfig control;
    SerialConfig serial;
    SafetyConfig safety;
};

} // namespace config
} // namespace robot_controller
"@

Set-Content -Path "src\core\src\config\SystemConfig.hpp" -Value $systemConfigHpp -Encoding UTF8
Write-Host "[OK] SystemConfig.hpp created" -ForegroundColor Green
```

---

### Step 6: Create ConfigManager.hpp

**File:** `src/core/src/config/ConfigManager.hpp`

```powershell
$configManagerHpp = @"
/**
 * @file ConfigManager.hpp
 * @brief Configuration manager - loads and provides access to configuration
 */

#pragma once

#include <string>
#include <memory>
#include <mutex>
#include "RobotConfig.hpp"
#include "SystemConfig.hpp"

namespace robot_controller {
namespace config {

/**
 * Configuration Manager (Singleton)
 *
 * Manages loading and access to robot and system configuration.
 * Thread-safe for reading after initialization.
 */
class ConfigManager {
public:
    /**
     * Get singleton instance
     */
    static ConfigManager& instance();

    // Delete copy/move
    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;
    ConfigManager(ConfigManager&&) = delete;
    ConfigManager& operator=(ConfigManager&&) = delete;

    /**
     * Load robot configuration from YAML file
     * @param filepath Path to robot_config.yaml
     * @return true if loaded successfully
     */
    bool loadRobotConfig(const std::string& filepath);

    /**
     * Load system configuration from YAML file
     * @param filepath Path to system_config.yaml
     * @return true if loaded successfully
     */
    bool loadSystemConfig(const std::string& filepath);

    /**
     * Load all configuration files from a directory
     * @param config_dir Path to config directory
     * @return true if all configs loaded successfully
     */
    bool loadAll(const std::string& config_dir = "config");

    /**
     * Save robot configuration to YAML file
     * @param filepath Path to save
     * @return true if saved successfully
     */
    bool saveRobotConfig(const std::string& filepath) const;

    /**
     * Get robot configuration (const reference)
     */
    const RobotConfig& robotConfig() const { return m_robot_config; }

    /**
     * Get system configuration (const reference)
     */
    const SystemConfig& systemConfig() const { return m_system_config; }

    /**
     * Check if configuration is loaded and valid
     */
    bool isLoaded() const { return m_loaded; }

    /**
     * Get configuration as JSON (for IPC)
     */
    std::string robotConfigToJson() const;
    std::string systemConfigToJson() const;

private:
    ConfigManager() = default;
    ~ConfigManager() = default;

    RobotConfig m_robot_config;
    SystemConfig m_system_config;
    bool m_loaded = false;
    mutable std::mutex m_mutex;
};

} // namespace config
} // namespace robot_controller
"@

Set-Content -Path "src\core\src\config\ConfigManager.hpp" -Value $configManagerHpp -Encoding UTF8
Write-Host "[OK] ConfigManager.hpp created" -ForegroundColor Green
```

---

### Step 7: Create ConfigManager.cpp

**File:** `src/core/src/config/ConfigManager.cpp`

```powershell
$configManagerCpp = @"
/**
 * @file ConfigManager.cpp
 * @brief Configuration manager implementation
 */

#include "ConfigManager.hpp"
#include "../logging/Logger.hpp"
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <filesystem>

namespace robot_controller {
namespace config {

using json = nlohmann::json;
namespace fs = std::filesystem;

ConfigManager& ConfigManager::instance() {
    static ConfigManager instance;
    return instance;
}

bool ConfigManager::loadRobotConfig(const std::string& filepath) {
    std::lock_guard<std::mutex> lock(m_mutex);

    try {
        if (!fs::exists(filepath)) {
            LOG_ERROR("Robot config file not found: {}", filepath);
            return false;
        }

        LOG_INFO("Loading robot config from: {}", filepath);

        YAML::Node config = YAML::LoadFile(filepath);
        YAML::Node robot = config["robot"];

        if (!robot) {
            LOG_ERROR("Missing 'robot' section in config");
            return false;
        }

        // Load basic info
        m_robot_config.name = robot["name"].as<std::string>("Robot6DOF");
        m_robot_config.type = robot["type"].as<std::string>("6-axis-articulated");
        m_robot_config.manufacturer = robot["manufacturer"].as<std::string>("Custom");
        m_robot_config.model = robot["model"].as<std::string>("RC-6DOF-01");

        // Load DH parameters
        m_robot_config.dh_parameters.clear();
        if (robot["dh_parameters"]) {
            for (const auto& dh : robot["dh_parameters"]) {
                DHParameter param;
                param.joint = dh["joint"].as<int>(0);
                param.a = dh["a"].as<double>(0.0);
                param.alpha = dh["alpha"].as<double>(0.0);
                param.d = dh["d"].as<double>(0.0);
                param.theta_offset = dh["theta_offset"].as<double>(0.0);
                m_robot_config.dh_parameters.push_back(param);
            }
        }

        // Load joint limits
        m_robot_config.joint_limits.clear();
        if (robot["joint_limits"]) {
            for (const auto& jl : robot["joint_limits"]) {
                JointLimit limit;
                limit.joint = jl["joint"].as<int>(0);
                limit.min = jl["min"].as<double>(-180.0);
                limit.max = jl["max"].as<double>(180.0);
                limit.max_velocity = jl["max_velocity"].as<double>(100.0);
                limit.max_acceleration = jl["max_acceleration"].as<double>(500.0);
                m_robot_config.joint_limits.push_back(limit);
            }
        }

        // Load TCP offset
        if (robot["tcp_offset"]) {
            auto tcp = robot["tcp_offset"];
            m_robot_config.tcp_offset.x = tcp["x"].as<double>(0.0);
            m_robot_config.tcp_offset.y = tcp["y"].as<double>(0.0);
            m_robot_config.tcp_offset.z = tcp["z"].as<double>(0.0);
            m_robot_config.tcp_offset.rx = tcp["rx"].as<double>(0.0);
            m_robot_config.tcp_offset.ry = tcp["ry"].as<double>(0.0);
            m_robot_config.tcp_offset.rz = tcp["rz"].as<double>(0.0);
        }

        // Load home position
        if (robot["home_position"]) {
            auto home = robot["home_position"];
            for (size_t i = 0; i < 6 && i < home.size(); ++i) {
                m_robot_config.home_position[i] = home[i].as<double>(0.0);
            }
        }

        // Load specifications
        m_robot_config.max_payload_kg = robot["max_payload_kg"].as<double>(6.0);
        m_robot_config.reach_mm = robot["reach_mm"].as<double>(1200.0);

        // Validate
        if (!m_robot_config.isValid()) {
            LOG_ERROR("Robot config validation failed: expected 6 joints, got {} DH params and {} limits",
                      m_robot_config.dh_parameters.size(), m_robot_config.joint_limits.size());
            return false;
        }

        LOG_INFO("Robot config loaded: {} ({} joints)", m_robot_config.name, m_robot_config.numJoints());
        return true;

    } catch (const YAML::Exception& e) {
        LOG_ERROR("YAML parse error in robot config: {}", e.what());
        return false;
    } catch (const std::exception& e) {
        LOG_ERROR("Error loading robot config: {}", e.what());
        return false;
    }
}

bool ConfigManager::loadSystemConfig(const std::string& filepath) {
    std::lock_guard<std::mutex> lock(m_mutex);

    try {
        if (!fs::exists(filepath)) {
            LOG_ERROR("System config file not found: {}", filepath);
            return false;
        }

        LOG_INFO("Loading system config from: {}", filepath);

        YAML::Node config = YAML::LoadFile(filepath);
        YAML::Node system = config["system"];

        if (!system) {
            LOG_ERROR("Missing 'system' section in config");
            return false;
        }

        // Version
        m_system_config.version = system["version"].as<std::string>("1.0.0");

        // IPC settings
        if (system["ipc"]) {
            auto ipc = system["ipc"];
            m_system_config.ipc.rep_port = ipc["rep_port"].as<int>(5555);
            m_system_config.ipc.pub_port = ipc["pub_port"].as<int>(5556);
            m_system_config.ipc.bind_address = ipc["bind_address"].as<std::string>("*");
            m_system_config.ipc.heartbeat_ms = ipc["heartbeat_ms"].as<int>(1000);
        }

        // Logging settings
        if (system["logging"]) {
            auto logging = system["logging"];
            m_system_config.logging.level = logging["level"].as<std::string>("info");
            m_system_config.logging.file = logging["file"].as<std::string>("logs/core.log");
            m_system_config.logging.max_size_mb = logging["max_size_mb"].as<int>(10);
            m_system_config.logging.max_files = logging["max_files"].as<int>(5);
            m_system_config.logging.console_enabled = logging["console_enabled"].as<bool>(true);
            m_system_config.logging.file_enabled = logging["file_enabled"].as<bool>(true);
        }

        // Control settings
        if (system["control"]) {
            auto control = system["control"];
            m_system_config.control.cycle_time_ms = control["cycle_time_ms"].as<int>(4);
            m_system_config.control.interpolation_hz = control["interpolation_hz"].as<int>(1000);
            m_system_config.control.status_publish_hz = control["status_publish_hz"].as<int>(10);
        }

        // Serial settings
        if (system["serial"]) {
            auto serial = system["serial"];
            m_system_config.serial.port = serial["port"].as<std::string>("AUTO");
            m_system_config.serial.baudrate = serial["baudrate"].as<int>(115200);
            m_system_config.serial.timeout_ms = serial["timeout_ms"].as<int>(100);
        }

        // Safety settings
        if (system["safety"]) {
            auto safety = system["safety"];
            m_system_config.safety.e_stop_enabled = safety["e_stop_enabled"].as<bool>(true);
            m_system_config.safety.soft_limits_enabled = safety["soft_limits_enabled"].as<bool>(true);
            m_system_config.safety.collision_check_enabled = safety["collision_check_enabled"].as<bool>(false);
            m_system_config.safety.max_speed_percent = safety["max_speed_percent"].as<int>(100);
        }

        LOG_INFO("System config loaded: version {}", m_system_config.version);
        return true;

    } catch (const YAML::Exception& e) {
        LOG_ERROR("YAML parse error in system config: {}", e.what());
        return false;
    } catch (const std::exception& e) {
        LOG_ERROR("Error loading system config: {}", e.what());
        return false;
    }
}

bool ConfigManager::loadAll(const std::string& config_dir) {
    std::string robot_path = config_dir + "/robot_config.yaml";
    std::string system_path = config_dir + "/system_config.yaml";

    bool robot_ok = loadRobotConfig(robot_path);
    bool system_ok = loadSystemConfig(system_path);

    m_loaded = robot_ok && system_ok;
    return m_loaded;
}

bool ConfigManager::saveRobotConfig(const std::string& filepath) const {
    std::lock_guard<std::mutex> lock(m_mutex);

    try {
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "robot" << YAML::Value << YAML::BeginMap;

        // Basic info
        out << YAML::Key << "name" << YAML::Value << m_robot_config.name;
        out << YAML::Key << "type" << YAML::Value << m_robot_config.type;
        out << YAML::Key << "manufacturer" << YAML::Value << m_robot_config.manufacturer;
        out << YAML::Key << "model" << YAML::Value << m_robot_config.model;

        // DH Parameters
        out << YAML::Key << "dh_parameters" << YAML::Value << YAML::BeginSeq;
        for (const auto& dh : m_robot_config.dh_parameters) {
            out << YAML::BeginMap;
            out << YAML::Key << "joint" << YAML::Value << dh.joint;
            out << YAML::Key << "a" << YAML::Value << dh.a;
            out << YAML::Key << "alpha" << YAML::Value << dh.alpha;
            out << YAML::Key << "d" << YAML::Value << dh.d;
            out << YAML::Key << "theta_offset" << YAML::Value << dh.theta_offset;
            out << YAML::EndMap;
        }
        out << YAML::EndSeq;

        // Joint limits
        out << YAML::Key << "joint_limits" << YAML::Value << YAML::BeginSeq;
        for (const auto& jl : m_robot_config.joint_limits) {
            out << YAML::BeginMap;
            out << YAML::Key << "joint" << YAML::Value << jl.joint;
            out << YAML::Key << "min" << YAML::Value << jl.min;
            out << YAML::Key << "max" << YAML::Value << jl.max;
            out << YAML::Key << "max_velocity" << YAML::Value << jl.max_velocity;
            out << YAML::Key << "max_acceleration" << YAML::Value << jl.max_acceleration;
            out << YAML::EndMap;
        }
        out << YAML::EndSeq;

        // TCP offset
        out << YAML::Key << "tcp_offset" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "x" << YAML::Value << m_robot_config.tcp_offset.x;
        out << YAML::Key << "y" << YAML::Value << m_robot_config.tcp_offset.y;
        out << YAML::Key << "z" << YAML::Value << m_robot_config.tcp_offset.z;
        out << YAML::Key << "rx" << YAML::Value << m_robot_config.tcp_offset.rx;
        out << YAML::Key << "ry" << YAML::Value << m_robot_config.tcp_offset.ry;
        out << YAML::Key << "rz" << YAML::Value << m_robot_config.tcp_offset.rz;
        out << YAML::EndMap;

        // Home position
        out << YAML::Key << "home_position" << YAML::Value << YAML::Flow << YAML::BeginSeq;
        for (double angle : m_robot_config.home_position) {
            out << angle;
        }
        out << YAML::EndSeq;

        // Specifications
        out << YAML::Key << "max_payload_kg" << YAML::Value << m_robot_config.max_payload_kg;
        out << YAML::Key << "reach_mm" << YAML::Value << m_robot_config.reach_mm;

        out << YAML::EndMap;  // robot
        out << YAML::EndMap;  // root

        std::ofstream fout(filepath);
        fout << out.c_str();
        fout.close();

        LOG_INFO("Robot config saved to: {}", filepath);
        return true;

    } catch (const std::exception& e) {
        LOG_ERROR("Error saving robot config: {}", e.what());
        return false;
    }
}

std::string ConfigManager::robotConfigToJson() const {
    std::lock_guard<std::mutex> lock(m_mutex);

    json j;
    j["name"] = m_robot_config.name;
    j["type"] = m_robot_config.type;
    j["manufacturer"] = m_robot_config.manufacturer;
    j["model"] = m_robot_config.model;
    j["num_joints"] = m_robot_config.numJoints();
    j["max_payload_kg"] = m_robot_config.max_payload_kg;
    j["reach_mm"] = m_robot_config.reach_mm;

    // DH parameters
    j["dh_parameters"] = json::array();
    for (const auto& dh : m_robot_config.dh_parameters) {
        j["dh_parameters"].push_back({
            {"joint", dh.joint},
            {"a", dh.a},
            {"alpha", dh.alpha},
            {"d", dh.d},
            {"theta_offset", dh.theta_offset}
        });
    }

    // Joint limits
    j["joint_limits"] = json::array();
    for (const auto& jl : m_robot_config.joint_limits) {
        j["joint_limits"].push_back({
            {"joint", jl.joint},
            {"min", jl.min},
            {"max", jl.max},
            {"max_velocity", jl.max_velocity},
            {"max_acceleration", jl.max_acceleration}
        });
    }

    // TCP offset
    j["tcp_offset"] = {
        {"x", m_robot_config.tcp_offset.x},
        {"y", m_robot_config.tcp_offset.y},
        {"z", m_robot_config.tcp_offset.z},
        {"rx", m_robot_config.tcp_offset.rx},
        {"ry", m_robot_config.tcp_offset.ry},
        {"rz", m_robot_config.tcp_offset.rz}
    };

    // Home position
    j["home_position"] = m_robot_config.home_position;

    return j.dump();
}

std::string ConfigManager::systemConfigToJson() const {
    std::lock_guard<std::mutex> lock(m_mutex);

    json j;
    j["version"] = m_system_config.version;

    j["ipc"] = {
        {"rep_port", m_system_config.ipc.rep_port},
        {"pub_port", m_system_config.ipc.pub_port},
        {"bind_address", m_system_config.ipc.bind_address}
    };

    j["logging"] = {
        {"level", m_system_config.logging.level},
        {"file", m_system_config.logging.file}
    };

    j["control"] = {
        {"cycle_time_ms", m_system_config.control.cycle_time_ms},
        {"status_publish_hz", m_system_config.control.status_publish_hz}
    };

    j["safety"] = {
        {"e_stop_enabled", m_system_config.safety.e_stop_enabled},
        {"soft_limits_enabled", m_system_config.safety.soft_limits_enabled},
        {"max_speed_percent", m_system_config.safety.max_speed_percent}
    };

    return j.dump();
}

} // namespace config
} // namespace robot_controller
"@

Set-Content -Path "src\core\src\config\ConfigManager.cpp" -Value $configManagerCpp -Encoding UTF8
Write-Host "[OK] ConfigManager.cpp created" -ForegroundColor Green
```

---

### Step 8: Update main.cpp to use ConfigManager

**File:** `src/core/src/main.cpp`

```powershell
$mainCpp = @"
/**
 * @file main.cpp
 * @brief Robot Controller Core - Entry Point
 */

#include <iostream>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>

#include "logging/Logger.hpp"
#include "config/ConfigManager.hpp"
#include "ipc/IpcServer.hpp"
#include "ipc/Message.hpp"

using namespace robot_controller;
using namespace robot_controller::config;
using namespace robot_controller::ipc;

// Global flag for graceful shutdown
std::atomic<bool> g_running{true};

void signalHandler(int signal) {
    LOG_INFO("Received signal {}, shutting down...", signal);
    g_running = false;
}

int main(int argc, char* argv[]) {
    // Setup signal handlers
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // Determine config directory (relative to executable or from arg)
    std::string config_dir = "../../config";
    if (argc > 1) {
        config_dir = argv[1];
    }

    // Initialize logging (basic setup, will reconfigure after loading config)
    Logger::init("../../logs/core.log", "debug");

    LOG_INFO("========================================");
    LOG_INFO("Robot Controller Core v1.0.0");
    LOG_INFO("========================================");
    LOG_INFO("Config directory: {}", config_dir);

    // Load configuration
    auto& config = ConfigManager::instance();
    if (!config.loadAll(config_dir)) {
        LOG_ERROR("Failed to load configuration files");
        LOG_ERROR("Make sure robot_config.yaml and system_config.yaml exist in: {}", config_dir);
        return 1;
    }

    // Reconfigure logger based on loaded config
    const auto& logConfig = config.systemConfig().logging;
    Logger::init(
        "../../" + logConfig.file,
        logConfig.level,
        logConfig.max_size_mb * 1024 * 1024,
        logConfig.max_files
    );

    LOG_INFO("Configuration loaded successfully");
    LOG_INFO("Robot: {} ({})", config.robotConfig().name, config.robotConfig().model);
    LOG_INFO("Joints: {}", config.robotConfig().numJoints());

    // Get IPC config
    const auto& ipcConfig = config.systemConfig().ipc;
    std::string rep_addr = "tcp://" + ipcConfig.bind_address + ":" + std::to_string(ipcConfig.rep_port);
    std::string pub_addr = "tcp://" + ipcConfig.bind_address + ":" + std::to_string(ipcConfig.pub_port);

    // Create IPC server
    IpcServer server(rep_addr, pub_addr);

    // Register message handlers
    server.registerHandler(MessageType::GET_STATUS,
        [&config](const Message& request) -> nlohmann::json {
            LOG_DEBUG("Handling GET_STATUS request");
            const auto& robotConfig = config.robotConfig();
            return {
                {"state", "IDLE"},
                {"mode", "MANUAL"},
                {"joints", robotConfig.home_position},
                {"tcp_position", {500.0, 0.0, 600.0, 0.0, 180.0, 0.0}},
                {"homed", false},
                {"enabled", false},
                {"errors", nlohmann::json::array()}
            };
        });

    server.registerHandler(MessageType::GET_JOINT_POSITIONS,
        [&config](const Message& request) -> nlohmann::json {
            LOG_DEBUG("Handling GET_JOINT_POSITIONS request");
            const auto& robotConfig = config.robotConfig();
            return {
                {"joints", robotConfig.home_position},
                {"unit", "degrees"}
            };
        });

    server.registerHandler(MessageType::GET_CONFIG,
        [&config](const Message& request) -> nlohmann::json {
            LOG_DEBUG("Handling GET_CONFIG request");
            return nlohmann::json::parse(config.robotConfigToJson());
        });

    // Start server
    if (!server.start()) {
        LOG_ERROR("Failed to start IPC server");
        return 1;
    }

    LOG_INFO("IPC Server running on ports {} (REP) and {} (PUB)",
             ipcConfig.rep_port, ipcConfig.pub_port);
    LOG_INFO("Press Ctrl+C to exit");

    // Main loop - publish status periodically
    const auto& controlConfig = config.systemConfig().control;
    int status_interval_ms = 1000 / controlConfig.status_publish_hz;
    auto last_status = std::chrono::steady_clock::now();

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_status);

        if (elapsed.count() >= status_interval_ms) {
            // Publish status
            const auto& robotConfig = config.robotConfig();
            nlohmann::json status = {
                {"state", "IDLE"},
                {"mode", "MANUAL"},
                {"joints", robotConfig.home_position},
                {"tcp_position", {500.0, 0.0, 600.0, 0.0, 180.0, 0.0}},
                {"homed", false},
                {"enabled", false},
                {"errors", nlohmann::json::array()}
            };
            server.publishStatus(status);
            last_status = now;
        }

        // Sleep to prevent busy loop
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Shutdown
    LOG_INFO("Stopping server...");
    server.stop();

    LOG_INFO("Robot Controller Core shutdown complete");
    return 0;
}
"@

Set-Content -Path "src\core\src\main.cpp" -Value $mainCpp -Encoding UTF8
Write-Host "[OK] main.cpp updated with ConfigManager" -ForegroundColor Green
```

---

### Step 9: Update CMakeLists.txt

```powershell
# Read and update CMakeLists.txt
$cmakePath = "src\core\CMakeLists.txt"
$cmakeContent = Get-Content -Path $cmakePath -Raw

# Update CORE_HEADERS to include new config files
$oldHeaders = "src/ipc/MessageTypes.hpp"
$newHeaders = @"
src/ipc/MessageTypes.hpp
    src/config/SystemConfig.hpp
"@

$cmakeContent = $cmakeContent -replace [regex]::Escape($oldHeaders), $newHeaders
Set-Content -Path $cmakePath -Value $cmakeContent -Encoding UTF8

Write-Host "[OK] CMakeLists.txt updated" -ForegroundColor Green
```

---

### Step 10: Build C++ Core

```powershell
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller\src\core"

# Clean build
Remove-Item -Recurse -Force build -ErrorAction SilentlyContinue

# Configure
cmake -B build -G "Visual Studio 17 2022" -A x64 `
    -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake"

if ($LASTEXITCODE -ne 0) {
    Write-Host "[ERROR] CMake configure failed" -ForegroundColor Red
    exit 1
}

# Build
cmake --build build --config Release

if ($LASTEXITCODE -ne 0) {
    Write-Host "[ERROR] Build failed" -ForegroundColor Red
    exit 1
}

Write-Host "[OK] C++ Core built successfully" -ForegroundColor Green
```

**Validation:**
```powershell
# Run Core
.\build\bin\Release\robot_core.exe
```

**Expected Output:**
```
[info] ========================================
[info] Robot Controller Core v1.0.0
[info] ========================================
[info] Config directory: ../../config
[info] Loading robot config from: ../../config/robot_config.yaml
[info] Robot config loaded: Robot6DOF (6 joints)
[info] Loading system config from: ../../config/system_config.yaml
[info] System config loaded: version 1.0.0
[info] Configuration loaded successfully
[info] Robot: Robot6DOF (RC-6DOF-01)
[info] Joints: 6
[info] IPC Server running on ports 5555 (REP) and 5556 (PUB)
```

---

## PART B: Configuration System - C# (P1-08)

### Step 11: Create ui_config.json

**File:** `config/ui_config.json`

```powershell
$uiConfigJson = @"
{
  "ui": {
    "theme": "dark",
    "language": "en-US",
    "viewport": {
      "background_color": "#1E1E1E",
      "grid_visible": true,
      "grid_size": 100,
      "axis_visible": true,
      "show_tcp_marker": true
    },
    "panels": {
      "navigation_width": 200,
      "properties_width": 250,
      "show_properties": true
    }
  },
  "connection": {
    "core_address": "tcp://localhost",
    "rep_port": 5555,
    "pub_port": 5556,
    "auto_connect": true,
    "reconnect_interval_ms": 5000,
    "request_timeout_ms": 5000
  },
  "recent_files": [],
  "window": {
    "width": 1400,
    "height": 800,
    "left": 100,
    "top": 100,
    "maximized": false
  }
}
"@

Set-Content -Path "config\ui_config.json" -Value $uiConfigJson -Encoding UTF8
Write-Host "[OK] ui_config.json created" -ForegroundColor Green
```

---

### Step 12: Create UiConfig.cs

**File:** `src/ui/RobotController.Common/Config/UiConfig.cs`

```powershell
# Create Config directory
New-Item -ItemType Directory -Force -Path "src\ui\RobotController.Common\Config"

$uiConfigCs = @"
using System.Text.Json.Serialization;

namespace RobotController.Common.Config;

/// <summary>
/// Viewport configuration
/// </summary>
public class ViewportConfig
{
    [JsonPropertyName("background_color")]
    public string BackgroundColor { get; set; } = "#1E1E1E";

    [JsonPropertyName("grid_visible")]
    public bool GridVisible { get; set; } = true;

    [JsonPropertyName("grid_size")]
    public int GridSize { get; set; } = 100;

    [JsonPropertyName("axis_visible")]
    public bool AxisVisible { get; set; } = true;

    [JsonPropertyName("show_tcp_marker")]
    public bool ShowTcpMarker { get; set; } = true;
}

/// <summary>
/// Panel configuration
/// </summary>
public class PanelsConfig
{
    [JsonPropertyName("navigation_width")]
    public int NavigationWidth { get; set; } = 200;

    [JsonPropertyName("properties_width")]
    public int PropertiesWidth { get; set; } = 250;

    [JsonPropertyName("show_properties")]
    public bool ShowProperties { get; set; } = true;
}

/// <summary>
/// UI appearance settings
/// </summary>
public class UiSettings
{
    [JsonPropertyName("theme")]
    public string Theme { get; set; } = "dark";

    [JsonPropertyName("language")]
    public string Language { get; set; } = "en-US";

    [JsonPropertyName("viewport")]
    public ViewportConfig Viewport { get; set; } = new();

    [JsonPropertyName("panels")]
    public PanelsConfig Panels { get; set; } = new();
}

/// <summary>
/// Connection settings
/// </summary>
public class ConnectionConfig
{
    [JsonPropertyName("core_address")]
    public string CoreAddress { get; set; } = "tcp://localhost";

    [JsonPropertyName("rep_port")]
    public int RepPort { get; set; } = 5555;

    [JsonPropertyName("pub_port")]
    public int PubPort { get; set; } = 5556;

    [JsonPropertyName("auto_connect")]
    public bool AutoConnect { get; set; } = true;

    [JsonPropertyName("reconnect_interval_ms")]
    public int ReconnectIntervalMs { get; set; } = 5000;

    [JsonPropertyName("request_timeout_ms")]
    public int RequestTimeoutMs { get; set; } = 5000;

    /// <summary>
    /// Get full REP address
    /// </summary>
    public string RepAddress => $"{CoreAddress}:{RepPort}";

    /// <summary>
    /// Get full PUB address (for SUB socket)
    /// </summary>
    public string SubAddress => $"{CoreAddress}:{PubPort}";
}

/// <summary>
/// Window state
/// </summary>
public class WindowConfig
{
    [JsonPropertyName("width")]
    public int Width { get; set; } = 1400;

    [JsonPropertyName("height")]
    public int Height { get; set; } = 800;

    [JsonPropertyName("left")]
    public int Left { get; set; } = 100;

    [JsonPropertyName("top")]
    public int Top { get; set; } = 100;

    [JsonPropertyName("maximized")]
    public bool Maximized { get; set; } = false;
}

/// <summary>
/// Complete UI configuration
/// </summary>
public class UiConfig
{
    [JsonPropertyName("ui")]
    public UiSettings Ui { get; set; } = new();

    [JsonPropertyName("connection")]
    public ConnectionConfig Connection { get; set; } = new();

    [JsonPropertyName("recent_files")]
    public List<string> RecentFiles { get; set; } = new();

    [JsonPropertyName("window")]
    public WindowConfig Window { get; set; } = new();
}
"@

Set-Content -Path "src\ui\RobotController.Common\Config\UiConfig.cs" -Value $uiConfigCs -Encoding UTF8
Write-Host "[OK] UiConfig.cs created" -ForegroundColor Green
```

---

### Step 13: Create ConfigService.cs

**File:** `src/ui/RobotController.UI/Services/ConfigService.cs`

```powershell
$configServiceCs = @"
using System.Text.Json;
using RobotController.Common.Config;
using Serilog;

namespace RobotController.UI.Services;

/// <summary>
/// Interface for configuration service
/// </summary>
public interface IConfigService
{
    /// <summary>
    /// Current configuration
    /// </summary>
    UiConfig Config { get; }

    /// <summary>
    /// Load configuration from file
    /// </summary>
    bool Load(string? filePath = null);

    /// <summary>
    /// Save configuration to file
    /// </summary>
    bool Save(string? filePath = null);

    /// <summary>
    /// Reset to default configuration
    /// </summary>
    void Reset();

    /// <summary>
    /// Event raised when configuration changes
    /// </summary>
    event EventHandler<UiConfig>? ConfigChanged;
}

/// <summary>
/// Configuration service implementation
/// </summary>
public class ConfigService : IConfigService
{
    private readonly string _defaultPath;
    private UiConfig _config = new();
    private readonly JsonSerializerOptions _jsonOptions;

    public UiConfig Config => _config;

    public event EventHandler<UiConfig>? ConfigChanged;

    public ConfigService()
    {
        // Default path relative to executable
        _defaultPath = Path.Combine(
            AppDomain.CurrentDomain.BaseDirectory,
            "..", "..", "..", "..", "..", "config", "ui_config.json"
        );

        _jsonOptions = new JsonSerializerOptions
        {
            WriteIndented = true,
            PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
            ReadCommentHandling = JsonCommentHandling.Skip
        };

        // Try to load on construction
        Load();
    }

    public bool Load(string? filePath = null)
    {
        string path = filePath ?? _defaultPath;
        string fullPath = Path.GetFullPath(path);

        try
        {
            if (!File.Exists(fullPath))
            {
                Log.Warning("Config file not found: {Path}, using defaults", fullPath);
                _config = new UiConfig();
                return false;
            }

            Log.Information("Loading UI config from: {Path}", fullPath);

            string json = File.ReadAllText(fullPath);
            var config = JsonSerializer.Deserialize<UiConfig>(json, _jsonOptions);

            if (config != null)
            {
                _config = config;
                Log.Information("UI config loaded successfully");
                ConfigChanged?.Invoke(this, _config);
                return true;
            }
            else
            {
                Log.Warning("Failed to deserialize config, using defaults");
                _config = new UiConfig();
                return false;
            }
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Error loading config from {Path}", fullPath);
            _config = new UiConfig();
            return false;
        }
    }

    public bool Save(string? filePath = null)
    {
        string path = filePath ?? _defaultPath;
        string fullPath = Path.GetFullPath(path);

        try
        {
            Log.Information("Saving UI config to: {Path}", fullPath);

            // Ensure directory exists
            string? dir = Path.GetDirectoryName(fullPath);
            if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir))
            {
                Directory.CreateDirectory(dir);
            }

            string json = JsonSerializer.Serialize(_config, _jsonOptions);
            File.WriteAllText(fullPath, json);

            Log.Information("UI config saved successfully");
            return true;
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Error saving config to {Path}", fullPath);
            return false;
        }
    }

    public void Reset()
    {
        Log.Information("Resetting UI config to defaults");
        _config = new UiConfig();
        ConfigChanged?.Invoke(this, _config);
    }
}
"@

Set-Content -Path "src\ui\RobotController.UI\Services\ConfigService.cs" -Value $configServiceCs -Encoding UTF8
Write-Host "[OK] ConfigService.cs created" -ForegroundColor Green
```

---

### Step 14: Update IpcClientService to use ConnectionConfig

**File:** Update `src/ui/RobotController.UI/Services/IpcClientService.cs`

```powershell
# Read and update IpcClientService
$ipcClientPath = "src\ui\RobotController.UI\Services\IpcClientService.cs"
$ipcClientContent = Get-Content -Path $ipcClientPath -Raw

# Add using statement for Config
$oldUsing = "using RobotController.Common.Messages;"
$newUsing = @"
using RobotController.Common.Config;
using RobotController.Common.Messages;
"@
$ipcClientContent = $ipcClientContent -replace [regex]::Escape($oldUsing), $newUsing

Set-Content -Path $ipcClientPath -Value $ipcClientContent -Encoding UTF8
Write-Host "[OK] IpcClientService.cs updated" -ForegroundColor Green
```

---

### Step 15: Update App.xaml.cs to use ConfigService

**File:** `src/ui/RobotController.UI/App.xaml.cs`

```powershell
$appXamlCs = @"
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;
using RobotController.UI.Services;
using RobotController.UI.ViewModels;
using RobotController.UI.Views;
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
                retainedFileCountLimit: 7,
                outputTemplate: "{Timestamp:yyyy-MM-dd HH:mm:ss.fff} [{Level:u3}] {Message:lj}{NewLine}{Exception}")
            .CreateLogger();

        Log.Information("========================================");
        Log.Information("Robot Controller UI starting...");
        Log.Information("========================================");

        // Build host with DI
        _host = Host.CreateDefaultBuilder()
            .UseSerilog()
            .ConfigureServices((context, services) =>
            {
                // Configuration (load first)
                services.AddSingleton<IConfigService, ConfigService>();

                // IPC Client
                services.AddSingleton<IIpcClientService, IpcClientService>();

                // ViewModels
                services.AddSingleton<MainViewModel>();

                // Views
                services.AddSingleton<MainWindow>();
            })
            .Build();

        await _host.StartAsync();

        // Get services
        var configService = _host.Services.GetRequiredService<IConfigService>();
        var mainWindow = _host.Services.GetRequiredService<MainWindow>();
        var viewModel = _host.Services.GetRequiredService<MainViewModel>();

        // Apply window settings from config
        var windowConfig = configService.Config.Window;
        if (windowConfig.Width > 0) mainWindow.Width = windowConfig.Width;
        if (windowConfig.Height > 0) mainWindow.Height = windowConfig.Height;
        if (windowConfig.Left >= 0) mainWindow.Left = windowConfig.Left;
        if (windowConfig.Top >= 0) mainWindow.Top = windowConfig.Top;
        if (windowConfig.Maximized) mainWindow.WindowState = WindowState.Maximized;

        mainWindow.DataContext = viewModel;
        mainWindow.Closing += MainWindow_Closing;
        mainWindow.Show();

        Log.Information("UI initialized with theme: {Theme}", configService.Config.Ui.Theme);
    }

    private void MainWindow_Closing(object? sender, System.ComponentModel.CancelEventArgs e)
    {
        if (sender is MainWindow window && _host != null)
        {
            // Save window state
            var configService = _host.Services.GetService<IConfigService>();
            if (configService != null)
            {
                configService.Config.Window.Width = (int)window.Width;
                configService.Config.Window.Height = (int)window.Height;
                configService.Config.Window.Left = (int)window.Left;
                configService.Config.Window.Top = (int)window.Top;
                configService.Config.Window.Maximized = window.WindowState == WindowState.Maximized;
                configService.Save();
            }
        }
    }

    protected override async void OnExit(ExitEventArgs e)
    {
        Log.Information("Robot Controller UI shutting down");

        if (_host != null)
        {
            // Dispose ViewModel (which disposes IPC client)
            var viewModel = _host.Services.GetService<MainViewModel>();
            viewModel?.Dispose();

            await _host.StopAsync();
            _host.Dispose();
        }

        Log.CloseAndFlush();
        base.OnExit(e);
    }
}
"@

Set-Content -Path "src\ui\RobotController.UI\App.xaml.cs" -Value $appXamlCs -Encoding UTF8
Write-Host "[OK] App.xaml.cs updated with ConfigService" -ForegroundColor Green
```

---

### Step 16: Update MainViewModel to use ConfigService

**File:** Update `src/ui/RobotController.UI/ViewModels/MainViewModel.cs`

```powershell
$mainViewModelCs = @"
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Common.Config;
using RobotController.Common.Messages;
using RobotController.UI.Services;
using Serilog;
using System.Collections.ObjectModel;
using System.Windows.Media;
using System.Windows.Threading;

namespace RobotController.UI.ViewModels;

public partial class MainViewModel : ObservableObject, IDisposable
{
    private readonly IIpcClientService _ipcClient;
    private readonly IConfigService _configService;
    private readonly Dispatcher _dispatcher;
    private bool _disposed;

    [ObservableProperty]
    private string _connectionStatus = "Disconnected";

    [ObservableProperty]
    private Brush _connectionStatusColor = Brushes.Red;

    [ObservableProperty]
    private string _robotState = "UNKNOWN";

    [ObservableProperty]
    private string _robotMode = "MANUAL";

    [ObservableProperty]
    private bool _isHomed;

    [ObservableProperty]
    private bool _isEnabled;

    [ObservableProperty]
    private double _tcpX;

    [ObservableProperty]
    private double _tcpY;

    [ObservableProperty]
    private double _tcpZ;

    [ObservableProperty]
    private double _tcpRx;

    [ObservableProperty]
    private double _tcpRy;

    [ObservableProperty]
    private double _tcpRz;

    [ObservableProperty]
    private string _coreVersion = "";

    [ObservableProperty]
    private long _coreUptime;

    [ObservableProperty]
    private string _robotName = "";

    [ObservableProperty]
    private string _robotModel = "";

    public ObservableCollection<JointPosition> JointPositions { get; } = new()
    {
        new JointPosition { Name = "J1", Value = 0.0 },
        new JointPosition { Name = "J2", Value = 0.0 },
        new JointPosition { Name = "J3", Value = 0.0 },
        new JointPosition { Name = "J4", Value = 0.0 },
        new JointPosition { Name = "J5", Value = 0.0 },
        new JointPosition { Name = "J6", Value = 0.0 },
    };

    /// <summary>
    /// Design-time constructor
    /// </summary>
    public MainViewModel() : this(new IpcClientService(), new ConfigService())
    {
    }

    /// <summary>
    /// Runtime constructor with DI
    /// </summary>
    public MainViewModel(IIpcClientService ipcClient, IConfigService configService)
    {
        _ipcClient = ipcClient;
        _configService = configService;
        _dispatcher = Dispatcher.CurrentDispatcher;

        // Subscribe to events
        _ipcClient.StatusReceived += OnStatusReceived;
        _ipcClient.ConnectionStateChanged += OnConnectionStateChanged;
        _ipcClient.ErrorOccurred += OnErrorOccurred;

        // Auto-connect if configured
        if (_configService.Config.Connection.AutoConnect)
        {
            _ = ConnectAsync();
        }
    }

    [RelayCommand]
    private async Task ConnectAsync()
    {
        if (_ipcClient.IsConnected)
        {
            Log.Information("Already connected");
            return;
        }

        ConnectionStatus = "Connecting...";
        ConnectionStatusColor = Brushes.Yellow;

        var connConfig = _configService.Config.Connection;
        Log.Information("Connecting to {Address}...", connConfig.RepAddress);

        bool success = await _ipcClient.ConnectAsync(
            connConfig.RepAddress,
            connConfig.SubAddress
        );

        if (success)
        {
            // Get version info
            var pong = await _ipcClient.PingAsync();
            if (pong != null)
            {
                CoreVersion = pong.CoreVersion;
                CoreUptime = pong.UptimeMs;
            }

            // Get initial status
            var status = await _ipcClient.GetStatusAsync();
            if (status != null)
            {
                UpdateStatus(status);
            }
        }
    }

    [RelayCommand]
    private void Disconnect()
    {
        _ipcClient.Disconnect();
    }

    [RelayCommand]
    private async Task RefreshStatusAsync()
    {
        if (!_ipcClient.IsConnected)
        {
            return;
        }

        var status = await _ipcClient.GetStatusAsync();
        if (status != null)
        {
            UpdateStatus(status);
        }
    }

    private void OnStatusReceived(object? sender, StatusPayload status)
    {
        _dispatcher.InvokeAsync(() => UpdateStatus(status));
    }

    private void OnConnectionStateChanged(object? sender, bool isConnected)
    {
        _dispatcher.InvokeAsync(() =>
        {
            if (isConnected)
            {
                ConnectionStatus = "Connected";
                ConnectionStatusColor = Brushes.LimeGreen;
            }
            else
            {
                ConnectionStatus = "Disconnected";
                ConnectionStatusColor = Brushes.Red;
                RobotState = "UNKNOWN";
            }
        });
    }

    private void OnErrorOccurred(object? sender, string error)
    {
        Log.Error("IPC Error: {Error}", error);
    }

    private void UpdateStatus(StatusPayload status)
    {
        RobotState = status.State;
        RobotMode = status.Mode;
        IsHomed = status.Homed;
        IsEnabled = status.Enabled;

        // Update joint positions
        if (status.Joints.Length >= 6)
        {
            for (int i = 0; i < 6; i++)
            {
                JointPositions[i].Value = status.Joints[i];
            }
            OnPropertyChanged(nameof(JointPositions));
        }

        // Update TCP position
        if (status.TcpPosition.Length >= 6)
        {
            TcpX = status.TcpPosition[0];
            TcpY = status.TcpPosition[1];
            TcpZ = status.TcpPosition[2];
            TcpRx = status.TcpPosition[3];
            TcpRy = status.TcpPosition[4];
            TcpRz = status.TcpPosition[5];
        }
    }

    public void Dispose()
    {
        if (_disposed)
        {
            return;
        }

        _ipcClient.StatusReceived -= OnStatusReceived;
        _ipcClient.ConnectionStateChanged -= OnConnectionStateChanged;
        _ipcClient.ErrorOccurred -= OnErrorOccurred;
        _ipcClient.Dispose();

        _disposed = true;
        GC.SuppressFinalize(this);
    }
}

public class JointPosition : ObservableObject
{
    private string _name = string.Empty;
    private double _value;

    public string Name
    {
        get => _name;
        set => SetProperty(ref _name, value);
    }

    public double Value
    {
        get => _value;
        set => SetProperty(ref _value, value);
    }
}
"@

Set-Content -Path "src\ui\RobotController.UI\ViewModels\MainViewModel.cs" -Value $mainViewModelCs -Encoding UTF8
Write-Host "[OK] MainViewModel.cs updated with ConfigService" -ForegroundColor Green
```

---

### Step 17: Build C# Solution

```powershell
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller\src\ui"

# Restore and build
dotnet restore
dotnet build --configuration Release

if ($LASTEXITCODE -ne 0) {
    Write-Host "[ERROR] C# build failed" -ForegroundColor Red
    exit 1
}

Write-Host "[OK] C# UI built successfully" -ForegroundColor Green
```

---

## PART C: Enhanced Logging (P1-09, P1-10)

### Step 18: Enhanced Logger.hpp (already created in IMPL_P1_01, verify)

The Logger was already created. Let's add a log level change method.

**File:** Update `src/core/src/logging/Logger.hpp`

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

    /**
     * Set log level at runtime
     */
    static void setLevel(const std::string& level);

    /**
     * Flush all pending log messages
     */
    static void flush();

private:
    static std::shared_ptr<spdlog::logger> s_logger;
    static bool s_initialized;
};

} // namespace robot_controller

// Convenience macros with source location
#define LOG_TRACE(...) SPDLOG_LOGGER_TRACE(::robot_controller::Logger::get(), __VA_ARGS__)
#define LOG_DEBUG(...) SPDLOG_LOGGER_DEBUG(::robot_controller::Logger::get(), __VA_ARGS__)
#define LOG_INFO(...)  SPDLOG_LOGGER_INFO(::robot_controller::Logger::get(), __VA_ARGS__)
#define LOG_WARN(...)  SPDLOG_LOGGER_WARN(::robot_controller::Logger::get(), __VA_ARGS__)
#define LOG_ERROR(...) SPDLOG_LOGGER_ERROR(::robot_controller::Logger::get(), __VA_ARGS__)
"@

Set-Content -Path "src\core\src\logging\Logger.hpp" -Value $loggerHpp -Encoding UTF8
Write-Host "[OK] Logger.hpp updated" -ForegroundColor Green
```

---

### Step 19: Enhanced Logger.cpp

**File:** Update `src/core/src/logging/Logger.cpp`

```powershell
$loggerCpp = @"
/**
 * @file Logger.cpp
 * @brief Logger implementation
 */

#include "Logger.hpp"
#include <vector>
#include <filesystem>
#include <iostream>

namespace robot_controller {

std::shared_ptr<spdlog::logger> Logger::s_logger = nullptr;
bool Logger::s_initialized = false;

void Logger::init(const std::string& log_file,
                  const std::string& level,
                  size_t max_size,
                  size_t max_files) {
    // Allow re-initialization with new settings
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
        console_sink->set_pattern("%^[%Y-%m-%d %H:%M:%S.%e] [%l]%$ %v");
        sinks.push_back(console_sink);

        // File sink (rotating)
        auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            log_file, max_size, max_files);
        file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] [%s:%#] %v");
        sinks.push_back(file_sink);

        // Create or replace logger
        if (s_logger) {
            spdlog::drop("robot_core");
        }

        s_logger = std::make_shared<spdlog::logger>("robot_core", sinks.begin(), sinks.end());

        // Set level
        setLevel(level);

        // Flush on warn or above
        s_logger->flush_on(spdlog::level::warn);

        // Register as default
        spdlog::set_default_logger(s_logger);
        spdlog::set_pattern("%^[%Y-%m-%d %H:%M:%S.%e] [%l]%$ %v");

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

void Logger::setLevel(const std::string& level) {
    if (!s_logger) return;

    if (level == "trace")      s_logger->set_level(spdlog::level::trace);
    else if (level == "debug") s_logger->set_level(spdlog::level::debug);
    else if (level == "info")  s_logger->set_level(spdlog::level::info);
    else if (level == "warn")  s_logger->set_level(spdlog::level::warn);
    else if (level == "error") s_logger->set_level(spdlog::level::err);
    else                       s_logger->set_level(spdlog::level::info);
}

void Logger::flush() {
    if (s_logger) {
        s_logger->flush();
    }
}

} // namespace robot_controller
"@

Set-Content -Path "src\core\src\logging\Logger.cpp" -Value $loggerCpp -Encoding UTF8
Write-Host "[OK] Logger.cpp updated" -ForegroundColor Green
```

---

### Step 20: Add Config Tests

**File:** `src/core/tests/test_config.cpp`

```powershell
$testConfigCpp = @"
/**
 * @file test_config.cpp
 * @brief Configuration tests
 */

#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include "config/ConfigManager.hpp"
#include "logging/Logger.hpp"

using namespace robot_controller;
using namespace robot_controller::config;

class ConfigTest : public ::testing::Test {
protected:
    void SetUp() override {
        Logger::init("test_config.log", "debug");

        // Create temp config directory
        test_dir = std::filesystem::temp_directory_path() / "robot_controller_test";
        std::filesystem::create_directories(test_dir);
    }

    void TearDown() override {
        // Cleanup
        std::filesystem::remove_all(test_dir);
    }

    std::filesystem::path test_dir;

    void createTestRobotConfig() {
        std::ofstream f(test_dir / "robot_config.yaml");
        f << R"(
robot:
  name: "TestRobot"
  type: "6-axis"
  manufacturer: "Test"
  model: "T-100"
  dh_parameters:
    - { joint: 1, a: 0, alpha: 0, d: 400, theta_offset: 0 }
    - { joint: 2, a: 25, alpha: -90, d: 0, theta_offset: -90 }
    - { joint: 3, a: 560, alpha: 0, d: 0, theta_offset: 0 }
    - { joint: 4, a: 35, alpha: -90, d: 515, theta_offset: 0 }
    - { joint: 5, a: 0, alpha: 90, d: 0, theta_offset: 0 }
    - { joint: 6, a: 0, alpha: -90, d: 80, theta_offset: 0 }
  joint_limits:
    - { joint: 1, min: -170, max: 170, max_velocity: 120, max_acceleration: 500 }
    - { joint: 2, min: -120, max: 60, max_velocity: 120, max_acceleration: 500 }
    - { joint: 3, min: -80, max: 165, max_velocity: 120, max_acceleration: 500 }
    - { joint: 4, min: -185, max: 185, max_velocity: 190, max_acceleration: 800 }
    - { joint: 5, min: -120, max: 120, max_velocity: 190, max_acceleration: 800 }
    - { joint: 6, min: -360, max: 360, max_velocity: 250, max_acceleration: 1000 }
  tcp_offset:
    x: 0
    y: 0
    z: 150
    rx: 0
    ry: 0
    rz: 0
  home_position: [0, -45, 90, 0, 45, 0]
  max_payload_kg: 6.0
  reach_mm: 1200
)";
        f.close();
    }

    void createTestSystemConfig() {
        std::ofstream f(test_dir / "system_config.yaml");
        f << R"(
system:
  version: "1.0.0"
  ipc:
    rep_port: 5555
    pub_port: 5556
    bind_address: "*"
  logging:
    level: "debug"
    file: "logs/test.log"
    max_size_mb: 10
    max_files: 5
  control:
    cycle_time_ms: 4
    interpolation_hz: 1000
    status_publish_hz: 10
  serial:
    port: "COM3"
    baudrate: 115200
  safety:
    e_stop_enabled: true
    soft_limits_enabled: true
)";
        f.close();
    }
};

TEST_F(ConfigTest, SingletonInstance) {
    auto& instance1 = ConfigManager::instance();
    auto& instance2 = ConfigManager::instance();
    EXPECT_EQ(&instance1, &instance2);
}

TEST_F(ConfigTest, LoadRobotConfig) {
    createTestRobotConfig();

    auto& config = ConfigManager::instance();
    bool result = config.loadRobotConfig((test_dir / "robot_config.yaml").string());

    EXPECT_TRUE(result);
    EXPECT_EQ(config.robotConfig().name, "TestRobot");
    EXPECT_EQ(config.robotConfig().numJoints(), 6);
}

TEST_F(ConfigTest, LoadSystemConfig) {
    createTestSystemConfig();

    auto& config = ConfigManager::instance();
    bool result = config.loadSystemConfig((test_dir / "system_config.yaml").string());

    EXPECT_TRUE(result);
    EXPECT_EQ(config.systemConfig().version, "1.0.0");
    EXPECT_EQ(config.systemConfig().ipc.rep_port, 5555);
}

TEST_F(ConfigTest, LoadAll) {
    createTestRobotConfig();
    createTestSystemConfig();

    auto& config = ConfigManager::instance();
    bool result = config.loadAll(test_dir.string());

    EXPECT_TRUE(result);
    EXPECT_TRUE(config.isLoaded());
}

TEST_F(ConfigTest, DHParameters) {
    createTestRobotConfig();

    auto& config = ConfigManager::instance();
    config.loadRobotConfig((test_dir / "robot_config.yaml").string());

    EXPECT_EQ(config.robotConfig().dh_parameters.size(), 6);
    EXPECT_EQ(config.robotConfig().dh_parameters[0].d, 400.0);
    EXPECT_EQ(config.robotConfig().dh_parameters[1].alpha, -90.0);
}

TEST_F(ConfigTest, JointLimits) {
    createTestRobotConfig();

    auto& config = ConfigManager::instance();
    config.loadRobotConfig((test_dir / "robot_config.yaml").string());

    EXPECT_EQ(config.robotConfig().joint_limits.size(), 6);
    EXPECT_EQ(config.robotConfig().joint_limits[0].min, -170.0);
    EXPECT_EQ(config.robotConfig().joint_limits[0].max, 170.0);
}

TEST_F(ConfigTest, RobotConfigToJson) {
    createTestRobotConfig();

    auto& config = ConfigManager::instance();
    config.loadRobotConfig((test_dir / "robot_config.yaml").string());

    std::string json = config.robotConfigToJson();
    EXPECT_FALSE(json.empty());
    EXPECT_NE(json.find("TestRobot"), std::string::npos);
}

TEST_F(ConfigTest, MissingFileReturnsError) {
    auto& config = ConfigManager::instance();
    bool result = config.loadRobotConfig("nonexistent_file.yaml");
    EXPECT_FALSE(result);
}
"@

Set-Content -Path "src\core\tests\test_config.cpp" -Value $testConfigCpp -Encoding UTF8
Write-Host "[OK] test_config.cpp updated" -ForegroundColor Green
```

---

## Step 21: Final Build and Test

### 21.1 Build C++ Core

```powershell
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller\src\core"

# Clean and rebuild
Remove-Item -Recurse -Force build -ErrorAction SilentlyContinue
cmake -B build -G "Visual Studio 17 2022" -A x64 `
    -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake"

cmake --build build --config Release

if ($LASTEXITCODE -ne 0) {
    Write-Host "[ERROR] C++ build failed" -ForegroundColor Red
    exit 1
}
Write-Host "[OK] C++ Core built" -ForegroundColor Green
```

### 21.2 Run C++ Tests

```powershell
cmake --build build --config Release --target robot_core_tests
ctest --test-dir build -C Release --output-on-failure
```

### 21.3 Build C# UI

```powershell
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller\src\ui"
dotnet build --configuration Release
```

### 21.4 Integration Test

```powershell
# Terminal 1: Start Core
.\src\core\build\bin\Release\robot_core.exe

# Terminal 2: Start UI
dotnet run --project src\ui\RobotController.UI --configuration Release
```

### 21.5 Verification Checklist

| Test | Expected | Pass |
|------|----------|------|
| Core loads robot_config.yaml | Logs show "Robot config loaded: Robot6DOF" | [ ] |
| Core loads system_config.yaml | Logs show "System config loaded" | [ ] |
| Core uses config IPC ports | Uses ports from config (5555, 5556) | [ ] |
| UI loads ui_config.json | Logs show theme loading | [ ] |
| UI auto-connects | Uses connection config | [ ] |
| UI window size | Remembers window size on restart | [ ] |
| Config tests pass | All tests green | [ ] |

---

## Step 22: Git Commit

```powershell
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller"

git add .
git commit -m "IMPL_P1_03: Config & Logging complete

Config System:
- Add robot_config.yaml (DH params, joint limits, TCP offset)
- Add system_config.yaml (IPC, logging, control settings)
- Add ui_config.json (UI theme, connection, window state)
- Implement C++ ConfigManager with YAML-CPP
- Implement C# ConfigService with JSON
- Add config tests

Logging:
- Enhanced spdlog setup with runtime level change
- Serilog setup with daily rotation
- Configurable log levels from config files

Integration:
- Core uses config for IPC ports and logging
- UI uses config for connection and window state
- Window position/size saved on exit

Tasks completed: P1-07, P1-08, P1-09, P1-10

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Completion Checklist

| Item | Status |
|------|--------|
| robot_config.yaml created | [ ] |
| system_config.yaml created | [ ] |
| ui_config.json created | [ ] |
| RobotConfig.hpp created | [ ] |
| SystemConfig.hpp created | [ ] |
| ConfigManager.hpp created | [ ] |
| ConfigManager.cpp created | [ ] |
| UiConfig.cs created | [ ] |
| ConfigService.cs created | [ ] |
| Logger enhanced | [ ] |
| main.cpp uses ConfigManager | [ ] |
| App.xaml.cs uses ConfigService | [ ] |
| MainViewModel uses config | [ ] |
| C++ builds successfully | [ ] |
| C++ tests pass | [ ] |
| C# builds successfully | [ ] |
| Integration test passes | [ ] |
| Git commit created | [ ] |

---

## Troubleshooting

### Problem: yaml-cpp not found

**Solution:**
```powershell
vcpkg install yaml-cpp:x64-windows
# Rebuild with vcpkg toolchain
```

### Problem: Config file not found at runtime

**Solution:**
- Check working directory relative path
- Use absolute path or adjust relative path
- Core expects `../../config/` from `build/bin/Release/`

### Problem: JSON parse error in C#

**Solution:**
- Check JSON syntax (no trailing commas)
- Ensure property names match C# class

### Problem: Window position not saved

**Solution:**
- Ensure MainWindow_Closing event is subscribed
- Check config file write permissions
- Verify ConfigService.Save() is called

---

## Next Steps

After completing IMPL_P1_03:
1. Update IMPLEMENTATION_PLAN_TRACKER.md → Mark WRITE-03 as Done
2. Proceed to **IMPL_P1_04: HMI & 3D Visualization**

---

*Document Version: 1.0 | Created: 2026-02-01*
