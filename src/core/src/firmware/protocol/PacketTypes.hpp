/**
 * @file PacketTypes.hpp
 * @brief V2 Binary Protocol — Command/Response type enums and system states
 *
 * Shared definitions between PC (C++ Core) and STM32 firmware.
 * All enums match V2 Architecture Document Section 3.3-3.5.
 */

#pragma once

#include <cstdint>
#include <string>

namespace robot_controller {
namespace firmware {
namespace protocol {

// ============================================================================
// Command Types (PC -> STM32)
// ============================================================================

enum class CommandType : uint8_t {
    // Motion
    CMD_PVT_POINT       = 0x10,
    CMD_PVT_BATCH       = 0x11,
    CMD_MOVE_ABSOLUTE   = 0x12,
    CMD_STOP_MOTION     = 0x13,

    // Jog
    CMD_JOG_START       = 0x20,
    CMD_JOG_STOP        = 0x21,

    // Homing
    CMD_HOME_START      = 0x30,
    CMD_HOME_SET_PARAMS = 0x31,
    CMD_HOME_STOP       = 0x32,

    // Drive Control
    CMD_ENABLE_DRIVES   = 0x40,
    CMD_DISABLE_DRIVES  = 0x41,
    CMD_RESET_ALARM     = 0x42,

    // Configuration
    CMD_SET_AXIS_PARAMS = 0x50,
    CMD_SET_ACCEL       = 0x51,

    // I/O
    CMD_SET_OUTPUT      = 0x60,
    CMD_SET_OUTPUTS_BATCH = 0x61,

    // System
    CMD_HEARTBEAT       = 0xF0,
    CMD_GET_VERSION     = 0xF1,
    CMD_E_STOP          = 0xFF,
};

// ============================================================================
// Response Types (STM32 -> PC)
// ============================================================================

enum class ResponseType : uint8_t {
    RSP_STATUS          = 0x80,
    RSP_ALARM           = 0x81,
    RSP_HOME_COMPLETE   = 0x82,
    RSP_LIMIT_HIT       = 0x83,
    RSP_MOTION_COMPLETE = 0x84,
    RSP_BUFFER_LOW      = 0x85,
    RSP_ACK             = 0x8A,
    RSP_NACK            = 0x8B,
    RSP_VERSION         = 0x8C,
    RSP_ERROR           = 0x8F,
};

// ============================================================================
// System States
// ============================================================================

enum class SystemState : uint8_t {
    STATE_INIT          = 0x00,
    STATE_IDLE          = 0x01,
    STATE_MOVING        = 0x02,
    STATE_JOGGING       = 0x03,
    STATE_HOMING        = 0x04,
    STATE_HOLD          = 0x05,
    STATE_ALARM         = 0x06,
    STATE_ESTOP         = 0x07,
    STATE_DISABLED      = 0x08,
    STATE_ERROR         = 0x0F,
};

// ============================================================================
// Alarm Codes
// ============================================================================

enum class AlarmCode : uint8_t {
    ALARM_NONE              = 0x00,
    ALARM_HARD_LIMIT        = 0x01,
    ALARM_SOFT_LIMIT        = 0x02,
    ALARM_FOLLOWING_ERROR   = 0x03,
    ALARM_OVERCURRENT       = 0x04,
    ALARM_ENCODER_FAULT     = 0x05,
    ALARM_MOTOR_FAULT       = 0x06,
    ALARM_COMM_TIMEOUT      = 0x07,
    ALARM_WATCHDOG          = 0x08,
    ALARM_ESTOP             = 0x09,
    ALARM_HOMING_FAIL       = 0x0A,
};

// ============================================================================
// String Conversions
// ============================================================================

inline std::string systemStateToString(SystemState state) {
    switch (state) {
        case SystemState::STATE_INIT:     return "Init";
        case SystemState::STATE_IDLE:     return "Idle";
        case SystemState::STATE_MOVING:   return "Moving";
        case SystemState::STATE_JOGGING:  return "Jogging";
        case SystemState::STATE_HOMING:   return "Homing";
        case SystemState::STATE_HOLD:     return "Hold";
        case SystemState::STATE_ALARM:    return "Alarm";
        case SystemState::STATE_ESTOP:    return "EStop";
        case SystemState::STATE_DISABLED: return "Disabled";
        case SystemState::STATE_ERROR:    return "Error";
        default:                          return "Unknown";
    }
}

inline std::string commandTypeToString(CommandType type) {
    switch (type) {
        case CommandType::CMD_PVT_POINT:       return "CMD_PVT_POINT";
        case CommandType::CMD_PVT_BATCH:       return "CMD_PVT_BATCH";
        case CommandType::CMD_MOVE_ABSOLUTE:   return "CMD_MOVE_ABSOLUTE";
        case CommandType::CMD_STOP_MOTION:     return "CMD_STOP_MOTION";
        case CommandType::CMD_JOG_START:       return "CMD_JOG_START";
        case CommandType::CMD_JOG_STOP:        return "CMD_JOG_STOP";
        case CommandType::CMD_HOME_START:      return "CMD_HOME_START";
        case CommandType::CMD_HOME_SET_PARAMS: return "CMD_HOME_SET_PARAMS";
        case CommandType::CMD_HOME_STOP:       return "CMD_HOME_STOP";
        case CommandType::CMD_ENABLE_DRIVES:   return "CMD_ENABLE_DRIVES";
        case CommandType::CMD_DISABLE_DRIVES:  return "CMD_DISABLE_DRIVES";
        case CommandType::CMD_RESET_ALARM:     return "CMD_RESET_ALARM";
        case CommandType::CMD_SET_AXIS_PARAMS: return "CMD_SET_AXIS_PARAMS";
        case CommandType::CMD_SET_ACCEL:       return "CMD_SET_ACCEL";
        case CommandType::CMD_SET_OUTPUT:       return "CMD_SET_OUTPUT";
        case CommandType::CMD_SET_OUTPUTS_BATCH: return "CMD_SET_OUTPUTS_BATCH";
        case CommandType::CMD_HEARTBEAT:       return "CMD_HEARTBEAT";
        case CommandType::CMD_GET_VERSION:     return "CMD_GET_VERSION";
        case CommandType::CMD_E_STOP:          return "CMD_E_STOP";
        default:                               return "UNKNOWN_CMD";
    }
}

} // namespace protocol
} // namespace firmware
} // namespace robot_controller
