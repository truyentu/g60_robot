# CORE MODULE: Configuration System

## Document Info
| Item | Value |
|------|-------|
| **Module** | Configuration System |
| **Layer** | Core Infrastructure |
| **Format** | YAML |
| **Libraries** | yaml-cpp (C++), YamlDotNet (C#) |
| **Version** | 1.0 |
| **Last Updated** | 2026-02-01 |

---

## 1. Overview

### 1.1. Purpose
Module Configuration System cung cấp cơ chế quản lý cấu hình tập trung cho toàn bộ hệ thống điều khiển robot. Module hỗ trợ:
- Load/Save configuration từ YAML files
- Runtime parameter updates
- Validation và schema checking
- Multi-layer configuration (defaults → machine → user)

### 1.2. Key Features
- **YAML Format**: Human-readable, easy to edit
- **Hierarchical Config**: Machine → Application → User overrides
- **Type Safety**: Strong typing với validation
- **Hot Reload**: Update parameters without restart (where safe)
- **Version Control Friendly**: Text-based, diff-able

### 1.3. Configuration Hierarchy

```
┌─────────────────────────────────────────────────────────────────┐
│                 CONFIGURATION HIERARCHY                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  Layer 1: DEFAULT CONFIG (Compiled-in)                     │ │
│  │  ══════════════════════════════════════                    │ │
│  │  • Hardcoded safe defaults                                 │ │
│  │  • Used if no config files found                           │ │
│  │  • Cannot be changed without recompilation                 │ │
│  └────────────────────────────────────────────────────────────┘ │
│                              │                                   │
│                              ▼ Override                          │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  Layer 2: MACHINE CONFIG (robot_config.yaml)               │ │
│  │  ═══════════════════════════════════════════               │ │
│  │  • Robot-specific parameters (DH, limits)                  │ │
│  │  • Hardware configuration                                  │ │
│  │  • Calibration data                                        │ │
│  │  • Rarely changed after commissioning                      │ │
│  └────────────────────────────────────────────────────────────┘ │
│                              │                                   │
│                              ▼ Override                          │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  Layer 3: APPLICATION CONFIG (app_config.yaml)             │ │
│  │  ═════════════════════════════════════════════             │ │
│  │  • IPC ports, logging settings                             │ │
│  │  • UI preferences                                          │ │
│  │  • Feature toggles                                         │ │
│  └────────────────────────────────────────────────────────────┘ │
│                              │                                   │
│                              ▼ Override                          │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  Layer 4: USER CONFIG (user_config.yaml)                   │ │
│  │  ═══════════════════════════════════════                   │ │
│  │  • User preferences                                        │ │
│  │  • Jog speed presets                                       │ │
│  │  • Last used settings                                      │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
│  RESULT: Final merged configuration                             │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Configuration Files

### 2.1. File Structure

```
┌─────────────────────────────────────────────────────────────────┐
│                    CONFIG DIRECTORY STRUCTURE                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  📁 config/                                                      │
│  │                                                               │
│  ├── 📄 robot_config.yaml        ← Robot/Machine parameters     │
│  │                                                               │
│  ├── 📄 app_config.yaml          ← Application settings         │
│  │                                                               │
│  ├── 📄 user_config.yaml         ← User preferences             │
│  │                                                               │
│  ├── 📁 robots/                  ← Robot model definitions      │
│  │   ├── 📄 puma560.yaml                                        │
│  │   ├── 📄 custom_6dof.yaml                                    │
│  │   └── 📄 my_robot.yaml                                       │
│  │                                                               │
│  ├── 📁 tools/                   ← Tool definitions             │
│  │   ├── 📄 welding_torch.yaml                                  │
│  │   ├── 📄 gripper.yaml                                        │
│  │   └── 📄 laser_sensor.yaml                                   │
│  │                                                               │
│  ├── 📁 frames/                  ← Work frames/Fixtures         │
│  │   ├── 📄 table_1.yaml                                        │
│  │   └── 📄 positioner.yaml                                     │
│  │                                                               │
│  └── 📁 welding/                 ← Welding parameters           │
│      ├── 📄 mild_steel.yaml                                     │
│      ├── 📄 stainless.yaml                                      │
│      └── 📄 aluminum.yaml                                       │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2. Robot Configuration (robot_config.yaml)

```yaml
# robot_config.yaml - Machine-specific configuration
# This file contains robot-specific parameters that rarely change

# Configuration metadata
config:
  version: "1.0"
  created: "2026-02-01"
  modified: "2026-02-01"
  description: "6-DOF Welding Robot Configuration"

# Robot model selection
robot:
  model: "custom_6dof"
  name: "WeldBot-01"
  serial_number: "WB-2026-001"

  # Load detailed model from separate file
  model_file: "robots/custom_6dof.yaml"

# Kinematics parameters (DH Convention: Modified DH)
kinematics:
  convention: "modified_dh"  # "standard_dh" or "modified_dh"

  # DH Parameters: [alpha, a, d, theta_offset]
  # Units: angles in radians, lengths in meters
  dh_parameters:
    - joint: 1
      alpha: 0.0
      a: 0.0
      d: 0.290       # Base height
      theta_offset: 0.0

    - joint: 2
      alpha: -1.5708  # -90°
      a: 0.0
      d: 0.0
      theta_offset: -1.5708  # -90° offset

    - joint: 3
      alpha: 0.0
      a: 0.270       # Upper arm length
      d: 0.0
      theta_offset: 0.0

    - joint: 4
      alpha: -1.5708  # -90°
      a: 0.070       # Elbow offset
      d: 0.302       # Forearm length
      theta_offset: 0.0

    - joint: 5
      alpha: 1.5708   # 90°
      a: 0.0
      d: 0.0
      theta_offset: 0.0

    - joint: 6
      alpha: -1.5708  # -90°
      a: 0.0
      d: 0.072       # Wrist to flange
      theta_offset: 0.0

  # Robot type for IK solver selection
  robot_type: "puma_like"  # "puma_like", "generic", "spherical_wrist"

  # Spherical wrist center offset (for analytical IK)
  wrist_offset: 0.072  # meters

# Joint limits
joints:
  # Per-joint configuration
  - id: 1
    name: "Base"
    type: "revolute"
    # Position limits
    min_position: -3.14159    # -180°
    max_position: 3.14159     # +180°
    # Velocity limits
    max_velocity: 3.14159     # 180°/s
    max_acceleration: 10.0    # rad/s²
    max_jerk: 100.0           # rad/s³
    # Home position
    home_position: 0.0
    home_direction: "negative"
    # Gear ratio (motor to joint)
    gear_ratio: 100.0
    # Encoder
    encoder_resolution: 131072  # counts per motor revolution

  - id: 2
    name: "Shoulder"
    type: "revolute"
    min_position: -2.0944     # -120°
    max_position: 2.0944      # +120°
    max_velocity: 2.618       # 150°/s
    max_acceleration: 8.0
    max_jerk: 80.0
    home_position: 0.0
    home_direction: "negative"
    gear_ratio: 120.0
    encoder_resolution: 131072

  - id: 3
    name: "Elbow"
    type: "revolute"
    min_position: -2.618      # -150°
    max_position: 2.618       # +150°
    max_velocity: 2.618       # 150°/s
    max_acceleration: 10.0
    max_jerk: 100.0
    home_position: 0.0
    home_direction: "positive"
    gear_ratio: 100.0
    encoder_resolution: 131072

  - id: 4
    name: "Wrist1"
    type: "revolute"
    min_position: -6.2832     # -360°
    max_position: 6.2832      # +360°
    max_velocity: 6.2832      # 360°/s
    max_acceleration: 20.0
    max_jerk: 200.0
    home_position: 0.0
    home_direction: "negative"
    gear_ratio: 50.0
    encoder_resolution: 65536

  - id: 5
    name: "Wrist2"
    type: "revolute"
    min_position: -2.0944     # -120°
    max_position: 2.0944      # +120°
    max_velocity: 6.2832      # 360°/s
    max_acceleration: 20.0
    max_jerk: 200.0
    home_position: 0.0
    home_direction: "negative"
    gear_ratio: 50.0
    encoder_resolution: 65536

  - id: 6
    name: "Wrist3"
    type: "revolute"
    min_position: -6.2832     # -360°
    max_position: 6.2832      # +360°
    max_velocity: 6.2832      # 360°/s
    max_acceleration: 20.0
    max_jerk: 200.0
    home_position: 0.0
    home_direction: "negative"
    gear_ratio: 50.0
    encoder_resolution: 65536

# Cartesian limits
cartesian:
  # Workspace limits (bounding box)
  workspace:
    x_min: -1.0
    x_max: 1.0
    y_min: -1.0
    y_max: 1.0
    z_min: 0.0
    z_max: 1.2

  # TCP velocity limits
  max_linear_velocity: 2.0      # m/s
  max_angular_velocity: 6.2832  # rad/s (360°/s)

  # TCP acceleration limits
  max_linear_acceleration: 10.0   # m/s²
  max_angular_acceleration: 30.0  # rad/s²

# Firmware/Hardware interface
hardware:
  # grblHAL configuration
  grblhal:
    port: "COM3"              # Serial port
    baud_rate: 2000000
    rx_buffer_size: 4096
    block_buffer_size: 256

  # Steps per unit (for grblHAL)
  steps_per_unit:
    - 88.889   # J1: steps per degree
    - 88.889   # J2
    - 88.889   # J3
    - 88.889   # J4
    - 88.889   # J5
    - 88.889   # J6

  # Homing sequence (order matters for safety!)
  homing:
    enabled: true
    sequence:
      - axes: [5, 6]      # Wrist joints first
        seek_rate: 500    # deg/min
        feed_rate: 50     # deg/min
      - axes: [4]
        seek_rate: 500
        feed_rate: 50
      - axes: [2, 3]      # Arm joints
        seek_rate: 300
        feed_rate: 30
      - axes: [1]         # Base last
        seek_rate: 300
        feed_rate: 30

# Safety configuration
safety:
  # E-Stop
  estop:
    dual_channel: true
    discrepancy_time_ms: 50

  # Speed limits by mode
  speed_limits:
    T1_max_tcp_speed: 0.250     # 250 mm/s (ISO 10218-1)
    T2_max_tcp_speed: 1.0       # Full speed verification
    AUTO_max_tcp_speed: 2.0     # Production speed

  # Soft limits margin (inside joint limits)
  soft_limit_margin: 0.0175     # 1 degree margin

  # Following error threshold
  following_error_threshold: 0.01  # radians

# Calibration data (updated during calibration)
calibration:
  last_calibrated: "2026-02-01"
  calibrated_by: "Engineer"

  # Joint zero offsets (encoder counts at mechanical zero)
  zero_offsets: [0, 0, 0, 0, 0, 0]

  # TCP calibration (tool center point)
  tcp:
    x: 0.0
    y: 0.0
    z: 0.150    # 150mm tool length
    rx: 0.0
    ry: 0.0
    rz: 0.0
```

### 2.3. Application Configuration (app_config.yaml)

```yaml
# app_config.yaml - Application settings

# Logging configuration
logging:
  level: "info"           # trace, debug, info, warn, error
  console_enabled: true
  file_enabled: true
  file_path: "logs/robot_controller.log"
  max_file_size_mb: 100
  max_files: 10

  # Per-module logging levels
  modules:
    ipc: "info"
    motion: "debug"
    safety: "warn"
    welding: "info"

# IPC configuration
ipc:
  command_port: 5555
  status_port: 5556
  event_port: 5557
  status_publish_rate_hz: 100

# Trajectory generator
trajectory:
  cycle_time_ms: 1.0        # 1kHz control loop

  # Default motion limits (can be overridden per-move)
  default_velocity_percent: 50
  default_acceleration_percent: 50

  # Blending/Zone settings
  default_zone_radius: 0.010   # 10mm blend radius

# State manager
state_manager:
  arming_timeout_ms: 5000
  servo_ready_timeout_ms: 2000

# Simulation mode
simulation:
  enabled: false
  virtual_cycle_time_ms: 1.0

# UI settings
ui:
  language: "vi"            # vi, en
  theme: "dark"

  # 3D view settings
  view_3d:
    show_coordinate_frames: true
    show_joint_limits: true
    show_workspace: false
    trail_length: 1000      # points

  # Jog panel defaults
  jog:
    default_speed_percent: 10
    speed_presets: [1, 5, 10, 25, 50, 100]
    step_sizes_mm: [0.1, 1.0, 10.0, 100.0]
    step_sizes_deg: [0.1, 1.0, 5.0, 15.0]

# Welding configuration
welding:
  # Default welding parameters
  default_travel_speed: 0.010   # 10 mm/s
  default_wire_feed: 5.0        # m/min
  default_voltage: 22.0         # V

  # Gas timing
  pre_flow_time_s: 0.5
  post_flow_time_s: 1.0

  # Arc timing
  ignition_time_s: 0.3
  crater_fill_time_s: 0.5
  burnback_time_s: 0.1

# Data recording
recording:
  enabled: false
  path: "recordings/"
  record_joints: true
  record_tcp: true
  record_welding: true
  sample_rate_hz: 100
```

### 2.4. Tool Definition (tools/welding_torch.yaml)

```yaml
# tools/welding_torch.yaml - Welding torch tool definition

tool:
  name: "MIG Torch 500A"
  type: "welding_torch"

  # Tool Center Point (TCP) relative to flange
  tcp:
    x: 0.0
    y: 0.0
    z: 0.180      # 180mm from flange to wire tip
    rx: 0.0
    ry: 0.0       # Torch angle (0 = straight)
    rz: 0.0

  # Tool mass properties (for dynamics)
  mass: 2.5       # kg
  center_of_mass:
    x: 0.0
    y: 0.0
    z: 0.090

  # Collision geometry (simplified cylinder)
  collision:
    type: "cylinder"
    radius: 0.040   # 40mm
    length: 0.200   # 200mm

  # Welding-specific parameters
  welding:
    process: "GMAW"         # GMAW, GTAW, etc.
    max_current: 500        # Amps
    wire_diameter: 1.2      # mm
    contact_tip_to_work: 15 # mm (CTWD)

  # Sensor offset (if laser sensor mounted)
  sensor_offset:
    x: 0.050      # 50mm ahead
    y: 0.0
    z: 0.030      # 30mm above
    rx: 0.0
    ry: -0.2618   # -15° look-ahead angle
    rz: 0.0
```

---

## 3. Task Breakdown

### 3.1. Task List

| ID | Task | Description | Priority | Dependencies |
|----|------|-------------|----------|--------------|
| **T-01** | Config Schema | Định nghĩa cấu trúc dữ liệu config | P0 | None |
| **T-02** | YAML Parser (C++) | yaml-cpp integration | P0 | T-01 |
| **T-03** | YAML Parser (C#) | YamlDotNet integration | P0 | T-01 |
| **T-04** | Config Loader | Load và merge config files | P0 | T-02 |
| **T-05** | Config Validator | Validate config values | P0 | T-01 |
| **T-06** | Default Values | Compiled-in defaults | P0 | T-01 |
| **T-07** | Config Manager | Central config access | P0 | T-04, T-05 |
| **T-08** | Hot Reload | Runtime config update | P1 | T-07 |
| **T-09** | Config Writer | Save config changes | P1 | T-02 |
| **T-10** | Robot Loader | Load robot model | P0 | T-04 |
| **T-11** | Tool Loader | Load tool definitions | P0 | T-04 |
| **T-12** | Frame Loader | Load work frames | P1 | T-04 |
| **T-13** | Config UI | Settings editor | P2 | T-07 |
| **T-14** | Unit Tests | Config loading tests | P0 | T-07 |

---

## 4. Implementation

### 4.1. Configuration Data Structures (C++)

```cpp
// config_types.hpp

#pragma once
#include <string>
#include <vector>
#include <array>
#include <optional>
#include <Eigen/Dense>

namespace robot_controller {
namespace config {

constexpr size_t NUM_JOINTS = 6;

/**
 * @brief DH Parameter for one joint
 */
struct DHParameter {
    int joint_id;
    double alpha;        // Link twist [rad]
    double a;            // Link length [m]
    double d;            // Link offset [m]
    double theta_offset; // Joint angle offset [rad]
};

/**
 * @brief Joint configuration
 */
struct JointConfig {
    int id;
    std::string name;
    std::string type;    // "revolute" or "prismatic"

    // Position limits [rad or m]
    double min_position;
    double max_position;

    // Dynamic limits
    double max_velocity;      // [rad/s or m/s]
    double max_acceleration;  // [rad/s² or m/s²]
    double max_jerk;          // [rad/s³ or m/s³]

    // Homing
    double home_position;
    std::string home_direction;

    // Motor parameters
    double gear_ratio;
    int encoder_resolution;
};

/**
 * @brief Kinematics configuration
 */
struct KinematicsConfig {
    std::string convention;  // "standard_dh" or "modified_dh"
    std::string robot_type;  // "puma_like", "generic"
    std::vector<DHParameter> dh_parameters;
    double wrist_offset;
};

/**
 * @brief Cartesian limits
 */
struct CartesianConfig {
    // Workspace bounding box [m]
    double x_min, x_max;
    double y_min, y_max;
    double z_min, z_max;

    // TCP limits
    double max_linear_velocity;     // [m/s]
    double max_angular_velocity;    // [rad/s]
    double max_linear_acceleration; // [m/s²]
    double max_angular_acceleration;// [rad/s²]
};

/**
 * @brief grblHAL hardware configuration
 */
struct GrblHALConfig {
    std::string port;
    int baud_rate;
    int rx_buffer_size;
    int block_buffer_size;
    std::array<double, NUM_JOINTS> steps_per_unit;
};

/**
 * @brief Homing step configuration
 */
struct HomingStep {
    std::vector<int> axes;
    double seek_rate;
    double feed_rate;
};

/**
 * @brief Safety configuration
 */
struct SafetyConfig {
    bool dual_channel_estop;
    int discrepancy_time_ms;

    double T1_max_tcp_speed;
    double T2_max_tcp_speed;
    double AUTO_max_tcp_speed;

    double soft_limit_margin;
    double following_error_threshold;
};

/**
 * @brief Tool Center Point
 */
struct TCPConfig {
    double x, y, z;
    double rx, ry, rz;

    Eigen::Isometry3d toTransform() const {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.translation() = Eigen::Vector3d(x, y, z);
        T.linear() = (Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX())).toRotationMatrix();
        return T;
    }
};

/**
 * @brief Tool definition
 */
struct ToolConfig {
    std::string name;
    std::string type;
    TCPConfig tcp;
    double mass;
    Eigen::Vector3d center_of_mass;

    // Optional sensor offset
    std::optional<TCPConfig> sensor_offset;
};

/**
 * @brief Calibration data
 */
struct CalibrationConfig {
    std::string last_calibrated;
    std::string calibrated_by;
    std::array<double, NUM_JOINTS> zero_offsets;
    TCPConfig tcp;
};

/**
 * @brief IPC configuration
 */
struct IPCConfig {
    int command_port;
    int status_port;
    int event_port;
    int status_publish_rate_hz;
};

/**
 * @brief Trajectory configuration
 */
struct TrajectoryConfig {
    double cycle_time_ms;
    double default_velocity_percent;
    double default_acceleration_percent;
    double default_zone_radius;
};

/**
 * @brief Logging configuration
 */
struct LoggingConfig {
    std::string level;
    bool console_enabled;
    bool file_enabled;
    std::string file_path;
    int max_file_size_mb;
    int max_files;
};

/**
 * @brief Welding configuration
 */
struct WeldingConfig {
    double default_travel_speed;
    double default_wire_feed;
    double default_voltage;
    double pre_flow_time_s;
    double post_flow_time_s;
    double ignition_time_s;
    double crater_fill_time_s;
    double burnback_time_s;
};

/**
 * @brief Complete robot configuration
 */
struct RobotConfig {
    // Metadata
    std::string config_version;
    std::string robot_name;
    std::string serial_number;

    // Core configuration
    KinematicsConfig kinematics;
    std::vector<JointConfig> joints;
    CartesianConfig cartesian;

    // Hardware
    GrblHALConfig grblhal;
    std::vector<HomingStep> homing_sequence;

    // Safety
    SafetyConfig safety;

    // Calibration
    CalibrationConfig calibration;
};

/**
 * @brief Complete application configuration
 */
struct AppConfig {
    LoggingConfig logging;
    IPCConfig ipc;
    TrajectoryConfig trajectory;
    WeldingConfig welding;
    bool simulation_enabled;
};

} // namespace config
} // namespace robot_controller
```

### 4.2. Configuration Manager (C++)

```cpp
// config_manager.hpp

#pragma once
#include "config_types.hpp"
#include <yaml-cpp/yaml.h>
#include <string>
#include <memory>
#include <mutex>
#include <functional>

namespace robot_controller {
namespace config {

/**
 * @brief Configuration change callback
 */
using ConfigChangeCallback = std::function<void(const std::string& section)>;

/**
 * @brief Central configuration manager
 *
 * Handles loading, merging, validation, and access to all configuration.
 * Thread-safe for concurrent read access.
 */
class ConfigManager {
public:
    /**
     * @brief Get singleton instance
     */
    static ConfigManager& getInstance();

    /**
     * @brief Initialize with config directory
     * @param config_dir Path to config directory
     * @return true if loading successful
     */
    bool initialize(const std::string& config_dir);

    /**
     * @brief Reload configuration from files
     */
    bool reload();

    /**
     * @brief Get robot configuration (read-only)
     */
    const RobotConfig& getRobotConfig() const;

    /**
     * @brief Get application configuration (read-only)
     */
    const AppConfig& getAppConfig() const;

    /**
     * @brief Get active tool configuration
     */
    const ToolConfig& getToolConfig() const;

    /**
     * @brief Load a specific tool
     * @param tool_name Tool file name (without path)
     */
    bool loadTool(const std::string& tool_name);

    /**
     * @brief Get configuration value by path
     * @param path Dot-separated path (e.g., "robot.joints[0].max_velocity")
     * @return YAML node at path
     */
    YAML::Node getValueByPath(const std::string& path) const;

    /**
     * @brief Set configuration value (for runtime updates)
     * @param path Configuration path
     * @param value New value
     * @return true if update successful
     */
    template<typename T>
    bool setValue(const std::string& path, const T& value);

    /**
     * @brief Save current configuration to file
     * @param section Which section to save ("robot", "app", "user")
     */
    bool save(const std::string& section);

    /**
     * @brief Register callback for configuration changes
     */
    void onConfigChange(ConfigChangeCallback callback);

    /**
     * @brief Get config directory path
     */
    std::string getConfigDir() const { return config_dir_; }

    /**
     * @brief Validate current configuration
     * @return Vector of validation errors (empty if valid)
     */
    std::vector<std::string> validate() const;

private:
    ConfigManager() = default;
    ~ConfigManager() = default;
    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;

    // Configuration storage
    RobotConfig robot_config_;
    AppConfig app_config_;
    ToolConfig active_tool_;

    // Raw YAML nodes for dynamic access
    YAML::Node robot_yaml_;
    YAML::Node app_yaml_;
    YAML::Node user_yaml_;

    // Paths
    std::string config_dir_;

    // Thread safety
    mutable std::mutex config_mutex_;

    // Callbacks
    std::vector<ConfigChangeCallback> change_callbacks_;

    // Internal methods
    bool loadRobotConfig(const std::string& path);
    bool loadAppConfig(const std::string& path);
    bool loadUserConfig(const std::string& path);
    void mergeConfigs();
    void applyDefaults();
    void notifyChange(const std::string& section);

    // Parsing helpers
    RobotConfig parseRobotConfig(const YAML::Node& node);
    AppConfig parseAppConfig(const YAML::Node& node);
    ToolConfig parseToolConfig(const YAML::Node& node);
    JointConfig parseJointConfig(const YAML::Node& node);
    DHParameter parseDHParameter(const YAML::Node& node);
};

} // namespace config
} // namespace robot_controller
```

### 4.3. ConfigManager Implementation

```cpp
// config_manager.cpp

#include "config_manager.hpp"
#include <spdlog/spdlog.h>
#include <fstream>
#include <filesystem>

namespace fs = std::filesystem;

namespace robot_controller {
namespace config {

ConfigManager& ConfigManager::getInstance() {
    static ConfigManager instance;
    return instance;
}

bool ConfigManager::initialize(const std::string& config_dir) {
    std::lock_guard<std::mutex> lock(config_mutex_);

    config_dir_ = config_dir;

    // Check directory exists
    if (!fs::exists(config_dir)) {
        spdlog::error("Config directory not found: {}", config_dir);
        return false;
    }

    // Apply compiled defaults first
    applyDefaults();

    // Load configuration files
    std::string robot_path = config_dir + "/robot_config.yaml";
    std::string app_path = config_dir + "/app_config.yaml";
    std::string user_path = config_dir + "/user_config.yaml";

    if (fs::exists(robot_path)) {
        if (!loadRobotConfig(robot_path)) {
            spdlog::error("Failed to load robot config");
            return false;
        }
    } else {
        spdlog::warn("Robot config not found, using defaults");
    }

    if (fs::exists(app_path)) {
        loadAppConfig(app_path);
    }

    if (fs::exists(user_path)) {
        loadUserConfig(user_path);
    }

    // Merge configurations (user overrides app overrides robot)
    mergeConfigs();

    // Validate
    auto errors = validate();
    if (!errors.empty()) {
        for (const auto& err : errors) {
            spdlog::error("Config validation: {}", err);
        }
        return false;
    }

    spdlog::info("Configuration loaded successfully from {}", config_dir);
    return true;
}

bool ConfigManager::reload() {
    spdlog::info("Reloading configuration...");
    return initialize(config_dir_);
}

const RobotConfig& ConfigManager::getRobotConfig() const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    return robot_config_;
}

const AppConfig& ConfigManager::getAppConfig() const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    return app_config_;
}

const ToolConfig& ConfigManager::getToolConfig() const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    return active_tool_;
}

bool ConfigManager::loadTool(const std::string& tool_name) {
    std::string tool_path = config_dir_ + "/tools/" + tool_name;

    if (!fs::exists(tool_path)) {
        spdlog::error("Tool file not found: {}", tool_path);
        return false;
    }

    try {
        YAML::Node tool_yaml = YAML::LoadFile(tool_path);
        std::lock_guard<std::mutex> lock(config_mutex_);
        active_tool_ = parseToolConfig(tool_yaml["tool"]);
        spdlog::info("Loaded tool: {}", active_tool_.name);
        notifyChange("tool");
        return true;
    } catch (const YAML::Exception& e) {
        spdlog::error("Failed to parse tool file: {}", e.what());
        return false;
    }
}

bool ConfigManager::loadRobotConfig(const std::string& path) {
    try {
        robot_yaml_ = YAML::LoadFile(path);
        robot_config_ = parseRobotConfig(robot_yaml_);
        return true;
    } catch (const YAML::Exception& e) {
        spdlog::error("YAML parse error in {}: {}", path, e.what());
        return false;
    }
}

bool ConfigManager::loadAppConfig(const std::string& path) {
    try {
        app_yaml_ = YAML::LoadFile(path);
        app_config_ = parseAppConfig(app_yaml_);
        return true;
    } catch (const YAML::Exception& e) {
        spdlog::error("YAML parse error in {}: {}", path, e.what());
        return false;
    }
}

bool ConfigManager::loadUserConfig(const std::string& path) {
    try {
        user_yaml_ = YAML::LoadFile(path);
        return true;
    } catch (const YAML::Exception& e) {
        spdlog::warn("Failed to load user config: {}", e.what());
        return false;
    }
}

void ConfigManager::applyDefaults() {
    // Robot defaults
    robot_config_.config_version = "1.0";
    robot_config_.robot_name = "Default Robot";

    // Default joint limits
    for (int i = 0; i < NUM_JOINTS; i++) {
        JointConfig joint;
        joint.id = i + 1;
        joint.name = "Joint " + std::to_string(i + 1);
        joint.type = "revolute";
        joint.min_position = -3.14159;
        joint.max_position = 3.14159;
        joint.max_velocity = 3.14159;
        joint.max_acceleration = 10.0;
        joint.max_jerk = 100.0;
        joint.home_position = 0.0;
        joint.home_direction = "negative";
        joint.gear_ratio = 100.0;
        joint.encoder_resolution = 131072;
        robot_config_.joints.push_back(joint);
    }

    // Safety defaults
    robot_config_.safety.dual_channel_estop = true;
    robot_config_.safety.discrepancy_time_ms = 50;
    robot_config_.safety.T1_max_tcp_speed = 0.250;
    robot_config_.safety.T2_max_tcp_speed = 1.0;
    robot_config_.safety.AUTO_max_tcp_speed = 2.0;
    robot_config_.safety.soft_limit_margin = 0.0175;
    robot_config_.safety.following_error_threshold = 0.01;

    // App defaults
    app_config_.logging.level = "info";
    app_config_.logging.console_enabled = true;
    app_config_.logging.file_enabled = true;
    app_config_.ipc.command_port = 5555;
    app_config_.ipc.status_port = 5556;
    app_config_.ipc.event_port = 5557;
    app_config_.ipc.status_publish_rate_hz = 100;
    app_config_.trajectory.cycle_time_ms = 1.0;
    app_config_.simulation_enabled = false;

    // Default tool
    active_tool_.name = "No Tool";
    active_tool_.type = "none";
    active_tool_.tcp = {0, 0, 0, 0, 0, 0};
    active_tool_.mass = 0;
}

RobotConfig ConfigManager::parseRobotConfig(const YAML::Node& node) {
    RobotConfig config = robot_config_;  // Start with defaults

    if (node["config"]["version"]) {
        config.config_version = node["config"]["version"].as<std::string>();
    }

    if (node["robot"]) {
        auto robot = node["robot"];
        if (robot["name"]) config.robot_name = robot["name"].as<std::string>();
        if (robot["serial_number"]) config.serial_number = robot["serial_number"].as<std::string>();
    }

    // Parse kinematics
    if (node["kinematics"]) {
        auto kin = node["kinematics"];
        config.kinematics.convention = kin["convention"].as<std::string>("modified_dh");
        config.kinematics.robot_type = kin["robot_type"].as<std::string>("puma_like");
        config.kinematics.wrist_offset = kin["wrist_offset"].as<double>(0.0);

        if (kin["dh_parameters"]) {
            config.kinematics.dh_parameters.clear();
            for (const auto& dh : kin["dh_parameters"]) {
                config.kinematics.dh_parameters.push_back(parseDHParameter(dh));
            }
        }
    }

    // Parse joints
    if (node["joints"]) {
        config.joints.clear();
        for (const auto& joint : node["joints"]) {
            config.joints.push_back(parseJointConfig(joint));
        }
    }

    // Parse cartesian
    if (node["cartesian"]) {
        auto cart = node["cartesian"];
        if (cart["workspace"]) {
            auto ws = cart["workspace"];
            config.cartesian.x_min = ws["x_min"].as<double>(-1.0);
            config.cartesian.x_max = ws["x_max"].as<double>(1.0);
            config.cartesian.y_min = ws["y_min"].as<double>(-1.0);
            config.cartesian.y_max = ws["y_max"].as<double>(1.0);
            config.cartesian.z_min = ws["z_min"].as<double>(0.0);
            config.cartesian.z_max = ws["z_max"].as<double>(1.2);
        }
        config.cartesian.max_linear_velocity = cart["max_linear_velocity"].as<double>(2.0);
        config.cartesian.max_angular_velocity = cart["max_angular_velocity"].as<double>(6.28);
    }

    // Parse hardware
    if (node["hardware"]["grblhal"]) {
        auto grbl = node["hardware"]["grblhal"];
        config.grblhal.port = grbl["port"].as<std::string>("COM3");
        config.grblhal.baud_rate = grbl["baud_rate"].as<int>(2000000);
        config.grblhal.rx_buffer_size = grbl["rx_buffer_size"].as<int>(4096);
        config.grblhal.block_buffer_size = grbl["block_buffer_size"].as<int>(256);
    }

    // Parse safety
    if (node["safety"]) {
        auto safety = node["safety"];
        if (safety["estop"]) {
            config.safety.dual_channel_estop = safety["estop"]["dual_channel"].as<bool>(true);
            config.safety.discrepancy_time_ms = safety["estop"]["discrepancy_time_ms"].as<int>(50);
        }
        if (safety["speed_limits"]) {
            auto limits = safety["speed_limits"];
            config.safety.T1_max_tcp_speed = limits["T1_max_tcp_speed"].as<double>(0.250);
            config.safety.T2_max_tcp_speed = limits["T2_max_tcp_speed"].as<double>(1.0);
            config.safety.AUTO_max_tcp_speed = limits["AUTO_max_tcp_speed"].as<double>(2.0);
        }
    }

    // Parse calibration
    if (node["calibration"]) {
        auto cal = node["calibration"];
        config.calibration.last_calibrated = cal["last_calibrated"].as<std::string>("");
        config.calibration.calibrated_by = cal["calibrated_by"].as<std::string>("");

        if (cal["zero_offsets"]) {
            auto offsets = cal["zero_offsets"].as<std::vector<double>>();
            for (size_t i = 0; i < std::min(offsets.size(), (size_t)NUM_JOINTS); i++) {
                config.calibration.zero_offsets[i] = offsets[i];
            }
        }

        if (cal["tcp"]) {
            auto tcp = cal["tcp"];
            config.calibration.tcp.x = tcp["x"].as<double>(0);
            config.calibration.tcp.y = tcp["y"].as<double>(0);
            config.calibration.tcp.z = tcp["z"].as<double>(0);
            config.calibration.tcp.rx = tcp["rx"].as<double>(0);
            config.calibration.tcp.ry = tcp["ry"].as<double>(0);
            config.calibration.tcp.rz = tcp["rz"].as<double>(0);
        }
    }

    return config;
}

JointConfig ConfigManager::parseJointConfig(const YAML::Node& node) {
    JointConfig joint;
    joint.id = node["id"].as<int>();
    joint.name = node["name"].as<std::string>("Joint");
    joint.type = node["type"].as<std::string>("revolute");
    joint.min_position = node["min_position"].as<double>(-3.14159);
    joint.max_position = node["max_position"].as<double>(3.14159);
    joint.max_velocity = node["max_velocity"].as<double>(3.14159);
    joint.max_acceleration = node["max_acceleration"].as<double>(10.0);
    joint.max_jerk = node["max_jerk"].as<double>(100.0);
    joint.home_position = node["home_position"].as<double>(0.0);
    joint.home_direction = node["home_direction"].as<std::string>("negative");
    joint.gear_ratio = node["gear_ratio"].as<double>(100.0);
    joint.encoder_resolution = node["encoder_resolution"].as<int>(131072);
    return joint;
}

DHParameter ConfigManager::parseDHParameter(const YAML::Node& node) {
    DHParameter dh;
    dh.joint_id = node["joint"].as<int>();
    dh.alpha = node["alpha"].as<double>(0);
    dh.a = node["a"].as<double>(0);
    dh.d = node["d"].as<double>(0);
    dh.theta_offset = node["theta_offset"].as<double>(0);
    return dh;
}

AppConfig ConfigManager::parseAppConfig(const YAML::Node& node) {
    AppConfig config = app_config_;  // Start with defaults

    if (node["logging"]) {
        auto log = node["logging"];
        config.logging.level = log["level"].as<std::string>("info");
        config.logging.console_enabled = log["console_enabled"].as<bool>(true);
        config.logging.file_enabled = log["file_enabled"].as<bool>(true);
        config.logging.file_path = log["file_path"].as<std::string>("logs/robot.log");
        config.logging.max_file_size_mb = log["max_file_size_mb"].as<int>(100);
        config.logging.max_files = log["max_files"].as<int>(10);
    }

    if (node["ipc"]) {
        auto ipc = node["ipc"];
        config.ipc.command_port = ipc["command_port"].as<int>(5555);
        config.ipc.status_port = ipc["status_port"].as<int>(5556);
        config.ipc.event_port = ipc["event_port"].as<int>(5557);
        config.ipc.status_publish_rate_hz = ipc["status_publish_rate_hz"].as<int>(100);
    }

    if (node["trajectory"]) {
        auto traj = node["trajectory"];
        config.trajectory.cycle_time_ms = traj["cycle_time_ms"].as<double>(1.0);
        config.trajectory.default_velocity_percent = traj["default_velocity_percent"].as<double>(50);
        config.trajectory.default_acceleration_percent = traj["default_acceleration_percent"].as<double>(50);
        config.trajectory.default_zone_radius = traj["default_zone_radius"].as<double>(0.010);
    }

    if (node["welding"]) {
        auto weld = node["welding"];
        config.welding.default_travel_speed = weld["default_travel_speed"].as<double>(0.010);
        config.welding.pre_flow_time_s = weld["pre_flow_time_s"].as<double>(0.5);
        config.welding.post_flow_time_s = weld["post_flow_time_s"].as<double>(1.0);
    }

    if (node["simulation"]) {
        config.simulation_enabled = node["simulation"]["enabled"].as<bool>(false);
    }

    return config;
}

ToolConfig ConfigManager::parseToolConfig(const YAML::Node& node) {
    ToolConfig tool;
    tool.name = node["name"].as<std::string>("Unknown Tool");
    tool.type = node["type"].as<std::string>("generic");

    if (node["tcp"]) {
        auto tcp = node["tcp"];
        tool.tcp.x = tcp["x"].as<double>(0);
        tool.tcp.y = tcp["y"].as<double>(0);
        tool.tcp.z = tcp["z"].as<double>(0);
        tool.tcp.rx = tcp["rx"].as<double>(0);
        tool.tcp.ry = tcp["ry"].as<double>(0);
        tool.tcp.rz = tcp["rz"].as<double>(0);
    }

    tool.mass = node["mass"].as<double>(0);

    if (node["center_of_mass"]) {
        auto com = node["center_of_mass"];
        tool.center_of_mass = Eigen::Vector3d(
            com["x"].as<double>(0),
            com["y"].as<double>(0),
            com["z"].as<double>(0)
        );
    }

    if (node["sensor_offset"]) {
        auto sensor = node["sensor_offset"];
        TCPConfig offset;
        offset.x = sensor["x"].as<double>(0);
        offset.y = sensor["y"].as<double>(0);
        offset.z = sensor["z"].as<double>(0);
        offset.rx = sensor["rx"].as<double>(0);
        offset.ry = sensor["ry"].as<double>(0);
        offset.rz = sensor["rz"].as<double>(0);
        tool.sensor_offset = offset;
    }

    return tool;
}

void ConfigManager::mergeConfigs() {
    // User config can override app config values
    // Implementation depends on which values should be overridable
}

std::vector<std::string> ConfigManager::validate() const {
    std::vector<std::string> errors;

    // Check joint count
    if (robot_config_.joints.size() != NUM_JOINTS) {
        errors.push_back("Expected " + std::to_string(NUM_JOINTS) + " joints, got " +
                         std::to_string(robot_config_.joints.size()));
    }

    // Validate joint limits
    for (const auto& joint : robot_config_.joints) {
        if (joint.min_position >= joint.max_position) {
            errors.push_back("Joint " + std::to_string(joint.id) +
                             ": min_position >= max_position");
        }
        if (joint.max_velocity <= 0) {
            errors.push_back("Joint " + std::to_string(joint.id) +
                             ": max_velocity must be positive");
        }
    }

    // Validate DH parameters count
    if (robot_config_.kinematics.dh_parameters.size() != NUM_JOINTS) {
        errors.push_back("Expected " + std::to_string(NUM_JOINTS) +
                         " DH parameters, got " +
                         std::to_string(robot_config_.kinematics.dh_parameters.size()));
    }

    // Validate IPC ports
    if (app_config_.ipc.command_port == app_config_.ipc.status_port ||
        app_config_.ipc.command_port == app_config_.ipc.event_port ||
        app_config_.ipc.status_port == app_config_.ipc.event_port) {
        errors.push_back("IPC ports must be unique");
    }

    // Validate safety limits
    if (robot_config_.safety.T1_max_tcp_speed > 0.250) {
        errors.push_back("T1 speed limit exceeds ISO 10218-1 maximum of 250 mm/s");
    }

    return errors;
}

void ConfigManager::onConfigChange(ConfigChangeCallback callback) {
    change_callbacks_.push_back(callback);
}

void ConfigManager::notifyChange(const std::string& section) {
    for (const auto& callback : change_callbacks_) {
        callback(section);
    }
}

bool ConfigManager::save(const std::string& section) {
    std::lock_guard<std::mutex> lock(config_mutex_);

    std::string path;
    YAML::Node* node = nullptr;

    if (section == "robot") {
        path = config_dir_ + "/robot_config.yaml";
        node = &robot_yaml_;
    } else if (section == "app") {
        path = config_dir_ + "/app_config.yaml";
        node = &app_yaml_;
    } else if (section == "user") {
        path = config_dir_ + "/user_config.yaml";
        node = &user_yaml_;
    } else {
        spdlog::error("Unknown config section: {}", section);
        return false;
    }

    try {
        std::ofstream fout(path);
        fout << *node;
        spdlog::info("Saved configuration to {}", path);
        return true;
    } catch (const std::exception& e) {
        spdlog::error("Failed to save config: {}", e.what());
        return false;
    }
}

} // namespace config
} // namespace robot_controller
```

---

## 5. C# Configuration Client

```csharp
// ConfigManager.cs

using System;
using System.Collections.Generic;
using System.IO;
using YamlDotNet.Serialization;
using YamlDotNet.Serialization.NamingConventions;

namespace RobotController.Config
{
    /// <summary>
    /// Configuration manager for C# UI
    /// </summary>
    public class ConfigManager
    {
        private static ConfigManager _instance;
        private static readonly object _lock = new object();

        public RobotConfig RobotConfig { get; private set; }
        public AppConfig AppConfig { get; private set; }
        public ToolConfig ActiveTool { get; private set; }

        public string ConfigDirectory { get; private set; }

        public event EventHandler<string> ConfigChanged;

        private ConfigManager() { }

        public static ConfigManager Instance
        {
            get
            {
                lock (_lock)
                {
                    return _instance ??= new ConfigManager();
                }
            }
        }

        public bool Initialize(string configDir)
        {
            ConfigDirectory = configDir;

            if (!Directory.Exists(configDir))
            {
                throw new DirectoryNotFoundException($"Config directory not found: {configDir}");
            }

            // Apply defaults
            ApplyDefaults();

            // Load configuration files
            var robotPath = Path.Combine(configDir, "robot_config.yaml");
            var appPath = Path.Combine(configDir, "app_config.yaml");

            if (File.Exists(robotPath))
            {
                LoadRobotConfig(robotPath);
            }

            if (File.Exists(appPath))
            {
                LoadAppConfig(appPath);
            }

            // Validate
            var errors = Validate();
            if (errors.Count > 0)
            {
                foreach (var error in errors)
                {
                    Console.WriteLine($"Config error: {error}");
                }
                return false;
            }

            return true;
        }

        private void ApplyDefaults()
        {
            RobotConfig = new RobotConfig
            {
                ConfigVersion = "1.0",
                RobotName = "Default Robot",
                Joints = new List<JointConfig>()
            };

            for (int i = 0; i < 6; i++)
            {
                RobotConfig.Joints.Add(new JointConfig
                {
                    Id = i + 1,
                    Name = $"Joint {i + 1}",
                    MinPosition = -Math.PI,
                    MaxPosition = Math.PI,
                    MaxVelocity = Math.PI,
                    MaxAcceleration = 10.0
                });
            }

            AppConfig = new AppConfig
            {
                Logging = new LoggingConfig
                {
                    Level = "info",
                    ConsoleEnabled = true,
                    FileEnabled = true
                },
                IPC = new IPCConfig
                {
                    CommandPort = 5555,
                    StatusPort = 5556,
                    EventPort = 5557,
                    StatusPublishRateHz = 100
                }
            };

            ActiveTool = new ToolConfig
            {
                Name = "No Tool",
                Type = "none"
            };
        }

        private void LoadRobotConfig(string path)
        {
            var yaml = File.ReadAllText(path);
            var deserializer = new DeserializerBuilder()
                .WithNamingConvention(UnderscoredNamingConvention.Instance)
                .Build();

            var doc = deserializer.Deserialize<Dictionary<string, object>>(yaml);
            // Parse robot config from dictionary
            // (Full implementation would map to RobotConfig class)
        }

        private void LoadAppConfig(string path)
        {
            var yaml = File.ReadAllText(path);
            var deserializer = new DeserializerBuilder()
                .WithNamingConvention(UnderscoredNamingConvention.Instance)
                .Build();

            AppConfig = deserializer.Deserialize<AppConfig>(yaml);
        }

        public bool LoadTool(string toolName)
        {
            var toolPath = Path.Combine(ConfigDirectory, "tools", toolName);

            if (!File.Exists(toolPath))
            {
                return false;
            }

            var yaml = File.ReadAllText(toolPath);
            var deserializer = new DeserializerBuilder()
                .WithNamingConvention(UnderscoredNamingConvention.Instance)
                .Build();

            var doc = deserializer.Deserialize<Dictionary<string, ToolConfig>>(yaml);
            if (doc.TryGetValue("tool", out var tool))
            {
                ActiveTool = tool;
                ConfigChanged?.Invoke(this, "tool");
                return true;
            }

            return false;
        }

        public List<string> Validate()
        {
            var errors = new List<string>();

            if (RobotConfig.Joints.Count != 6)
            {
                errors.Add($"Expected 6 joints, got {RobotConfig.Joints.Count}");
            }

            foreach (var joint in RobotConfig.Joints)
            {
                if (joint.MinPosition >= joint.MaxPosition)
                {
                    errors.Add($"Joint {joint.Id}: min_position >= max_position");
                }
            }

            return errors;
        }

        public void Save(string section)
        {
            var serializer = new SerializerBuilder()
                .WithNamingConvention(UnderscoredNamingConvention.Instance)
                .Build();

            string path = section switch
            {
                "robot" => Path.Combine(ConfigDirectory, "robot_config.yaml"),
                "app" => Path.Combine(ConfigDirectory, "app_config.yaml"),
                "user" => Path.Combine(ConfigDirectory, "user_config.yaml"),
                _ => throw new ArgumentException($"Unknown section: {section}")
            };

            object config = section switch
            {
                "robot" => RobotConfig,
                "app" => AppConfig,
                _ => throw new ArgumentException($"Unknown section: {section}")
            };

            var yaml = serializer.Serialize(config);
            File.WriteAllText(path, yaml);
        }
    }

    #region Config Classes

    public class RobotConfig
    {
        public string ConfigVersion { get; set; }
        public string RobotName { get; set; }
        public string SerialNumber { get; set; }
        public KinematicsConfig Kinematics { get; set; }
        public List<JointConfig> Joints { get; set; }
        public CartesianConfig Cartesian { get; set; }
        public SafetyConfig Safety { get; set; }
    }

    public class KinematicsConfig
    {
        public string Convention { get; set; }
        public string RobotType { get; set; }
        public List<DHParameter> DHParameters { get; set; }
        public double WristOffset { get; set; }
    }

    public class DHParameter
    {
        public int Joint { get; set; }
        public double Alpha { get; set; }
        public double A { get; set; }
        public double D { get; set; }
        public double ThetaOffset { get; set; }
    }

    public class JointConfig
    {
        public int Id { get; set; }
        public string Name { get; set; }
        public string Type { get; set; }
        public double MinPosition { get; set; }
        public double MaxPosition { get; set; }
        public double MaxVelocity { get; set; }
        public double MaxAcceleration { get; set; }
        public double MaxJerk { get; set; }
        public double HomePosition { get; set; }
        public string HomeDirection { get; set; }
        public double GearRatio { get; set; }
        public int EncoderResolution { get; set; }
    }

    public class CartesianConfig
    {
        public double XMin { get; set; }
        public double XMax { get; set; }
        public double YMin { get; set; }
        public double YMax { get; set; }
        public double ZMin { get; set; }
        public double ZMax { get; set; }
        public double MaxLinearVelocity { get; set; }
        public double MaxAngularVelocity { get; set; }
    }

    public class SafetyConfig
    {
        public bool DualChannelEstop { get; set; }
        public int DiscrepancyTimeMs { get; set; }
        public double T1MaxTcpSpeed { get; set; }
        public double T2MaxTcpSpeed { get; set; }
        public double AutoMaxTcpSpeed { get; set; }
    }

    public class AppConfig
    {
        public LoggingConfig Logging { get; set; }
        public IPCConfig IPC { get; set; }
        public TrajectoryConfig Trajectory { get; set; }
        public WeldingConfig Welding { get; set; }
        public bool SimulationEnabled { get; set; }
    }

    public class LoggingConfig
    {
        public string Level { get; set; }
        public bool ConsoleEnabled { get; set; }
        public bool FileEnabled { get; set; }
        public string FilePath { get; set; }
    }

    public class IPCConfig
    {
        public int CommandPort { get; set; }
        public int StatusPort { get; set; }
        public int EventPort { get; set; }
        public int StatusPublishRateHz { get; set; }
    }

    public class TrajectoryConfig
    {
        public double CycleTimeMs { get; set; }
        public double DefaultVelocityPercent { get; set; }
        public double DefaultAccelerationPercent { get; set; }
    }

    public class WeldingConfig
    {
        public double DefaultTravelSpeed { get; set; }
        public double PreFlowTimeS { get; set; }
        public double PostFlowTimeS { get; set; }
    }

    public class ToolConfig
    {
        public string Name { get; set; }
        public string Type { get; set; }
        public TCPConfig TCP { get; set; }
        public double Mass { get; set; }
    }

    public class TCPConfig
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public double Rx { get; set; }
        public double Ry { get; set; }
        public double Rz { get; set; }
    }

    #endregion
}
```

---

## 6. Testing

### 6.1. Unit Tests

```cpp
// test_config.cpp

#include <gtest/gtest.h>
#include "config/config_manager.hpp"
#include <filesystem>

namespace fs = std::filesystem;
using namespace robot_controller::config;

class ConfigManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create temp config directory
        test_dir_ = fs::temp_directory_path() / "robot_config_test";
        fs::create_directories(test_dir_);

        // Create minimal robot config
        std::ofstream robot_file(test_dir_ / "robot_config.yaml");
        robot_file << R"(
config:
  version: "1.0"
robot:
  name: "Test Robot"
kinematics:
  convention: "modified_dh"
  robot_type: "puma_like"
  wrist_offset: 0.072
  dh_parameters:
    - joint: 1
      alpha: 0.0
      a: 0.0
      d: 0.290
      theta_offset: 0.0
    - joint: 2
      alpha: -1.5708
      a: 0.0
      d: 0.0
      theta_offset: 0.0
    - joint: 3
      alpha: 0.0
      a: 0.270
      d: 0.0
      theta_offset: 0.0
    - joint: 4
      alpha: -1.5708
      a: 0.070
      d: 0.302
      theta_offset: 0.0
    - joint: 5
      alpha: 1.5708
      a: 0.0
      d: 0.0
      theta_offset: 0.0
    - joint: 6
      alpha: -1.5708
      a: 0.0
      d: 0.072
      theta_offset: 0.0
joints:
  - id: 1
    name: "Base"
    type: "revolute"
    min_position: -3.14159
    max_position: 3.14159
    max_velocity: 3.14159
    max_acceleration: 10.0
    max_jerk: 100.0
  - id: 2
    name: "Shoulder"
    type: "revolute"
    min_position: -2.0944
    max_position: 2.0944
    max_velocity: 2.618
    max_acceleration: 8.0
    max_jerk: 80.0
  - id: 3
    name: "Elbow"
    type: "revolute"
    min_position: -2.618
    max_position: 2.618
    max_velocity: 2.618
    max_acceleration: 10.0
    max_jerk: 100.0
  - id: 4
    name: "Wrist1"
    type: "revolute"
    min_position: -6.2832
    max_position: 6.2832
    max_velocity: 6.2832
    max_acceleration: 20.0
    max_jerk: 200.0
  - id: 5
    name: "Wrist2"
    type: "revolute"
    min_position: -2.0944
    max_position: 2.0944
    max_velocity: 6.2832
    max_acceleration: 20.0
    max_jerk: 200.0
  - id: 6
    name: "Wrist3"
    type: "revolute"
    min_position: -6.2832
    max_position: 6.2832
    max_velocity: 6.2832
    max_acceleration: 20.0
    max_jerk: 200.0
)";
        robot_file.close();
    }

    void TearDown() override {
        fs::remove_all(test_dir_);
    }

    fs::path test_dir_;
};

TEST_F(ConfigManagerTest, LoadsRobotConfig) {
    auto& cm = ConfigManager::getInstance();
    EXPECT_TRUE(cm.initialize(test_dir_.string()));

    const auto& config = cm.getRobotConfig();
    EXPECT_EQ(config.robot_name, "Test Robot");
    EXPECT_EQ(config.joints.size(), 6);
}

TEST_F(ConfigManagerTest, ParsesDHParameters) {
    auto& cm = ConfigManager::getInstance();
    EXPECT_TRUE(cm.initialize(test_dir_.string()));

    const auto& config = cm.getRobotConfig();
    EXPECT_EQ(config.kinematics.dh_parameters.size(), 6);

    // Check first DH parameter
    EXPECT_EQ(config.kinematics.dh_parameters[0].joint_id, 1);
    EXPECT_DOUBLE_EQ(config.kinematics.dh_parameters[0].d, 0.290);
}

TEST_F(ConfigManagerTest, ValidatesJointLimits) {
    // Create config with invalid limits
    std::ofstream bad_file(test_dir_ / "robot_config.yaml");
    bad_file << R"(
config:
  version: "1.0"
joints:
  - id: 1
    min_position: 1.0
    max_position: 0.0
)";
    bad_file.close();

    auto& cm = ConfigManager::getInstance();
    EXPECT_FALSE(cm.initialize(test_dir_.string()));
}

TEST_F(ConfigManagerTest, AppliesDefaults) {
    // Empty config should get defaults
    fs::remove(test_dir_ / "robot_config.yaml");

    auto& cm = ConfigManager::getInstance();
    EXPECT_TRUE(cm.initialize(test_dir_.string()));

    const auto& config = cm.getRobotConfig();
    EXPECT_EQ(config.safety.T1_max_tcp_speed, 0.250);
}
```

---

## 7. References

### 7.1. External Resources
| Resource | URL |
|----------|-----|
| yaml-cpp | https://github.com/jbeder/yaml-cpp |
| YamlDotNet | https://github.com/aaubry/YamlDotNet |
| YAML Spec | https://yaml.org/spec/1.2/ |

---

## APPENDIX

### A. CMake Configuration

```cmake
# CMakeLists.txt for config module

find_package(yaml-cpp REQUIRED)

add_library(config
    src/config/config_manager.cpp
    src/config/config_types.cpp
)

target_link_libraries(config
    PUBLIC
        yaml-cpp
        Eigen3::Eigen
    PRIVATE
        spdlog::spdlog
)
```

### B. NuGet Packages (C#)

```xml
<PackageReference Include="YamlDotNet" Version="13.0.*" />
```

### C. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-02-01 | Initial version |

---

*Document generated as part of Robot Controller development project.*
