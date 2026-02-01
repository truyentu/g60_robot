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
