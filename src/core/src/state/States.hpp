/**
 * @file States.hpp
 * @brief Robot state and mode definitions
 */

#pragma once

#include <string>
#include <cstdint>

namespace robot_controller {
namespace state {

/**
 * Robot operational states
 */
enum class RobotState : uint8_t {
    INIT = 0,           // Initializing
    IDLE,               // Idle, not enabled
    HOMING,             // Homing in progress
    READY,              // Ready to move (enabled, homed)
    MOVING,             // Executing motion (jog, move)
    EXECUTING,          // Executing program
    PAUSED,             // Program paused
    ERROR,              // Error state (recoverable)
    ESTOP               // Emergency stop (requires reset)
};

/**
 * Robot operation modes
 */
enum class RobotMode : uint8_t {
    MANUAL = 0,         // Manual jog mode
    AUTO,               // Automatic program execution
    T1,                 // Teach mode 1 (reduced speed, 250mm/s max)
    T2                  // Teach mode 2 (full speed, requires enabling device)
};

/**
 * Safety states
 */
enum class SafetyState : uint8_t {
    NORMAL = 0,         // Normal operation
    PROTECTIVE_STOP,    // Protective stop (safety door, etc.)
    EMERGENCY_STOP      // Emergency stop pressed
};

/**
 * Motion types
 */
enum class MotionType : uint8_t {
    NONE = 0,
    JOG_JOINT,          // Joint space jog
    JOG_CARTESIAN,      // Cartesian space jog
    MOVE_JOINT,         // Joint interpolated move
    MOVE_LINEAR,        // Linear interpolated move
    MOVE_CIRCULAR       // Circular interpolated move
};

/**
 * State transition events
 */
enum class StateEvent : uint8_t {
    // System events
    INIT_COMPLETE = 0,
    ENABLE_REQUEST,
    DISABLE_REQUEST,
    RESET_REQUEST,

    // Homing events
    HOME_REQUEST,
    HOME_COMPLETE,
    HOME_ERROR,

    // Motion events
    MOTION_START,
    MOTION_COMPLETE,
    MOTION_ERROR,

    // Program events
    PROGRAM_START,
    PROGRAM_PAUSE,
    PROGRAM_RESUME,
    PROGRAM_STOP,
    PROGRAM_COMPLETE,
    PROGRAM_ERROR,

    // Error events
    ERROR_OCCURRED,
    ERROR_CLEARED,

    // Safety events
    ESTOP_PRESSED,
    ESTOP_RELEASED,
    SAFETY_DOOR_OPEN,
    SAFETY_DOOR_CLOSED
};

/**
 * Error codes
 */
enum class ErrorCode : uint16_t {
    NONE = 0,

    // Communication errors (100-199)
    COMM_TIMEOUT = 100,
    COMM_CRC_ERROR,
    COMM_DISCONNECTED,

    // Motion errors (200-299)
    MOTION_LIMIT_EXCEEDED = 200,
    MOTION_FOLLOWING_ERROR,
    MOTION_COLLISION,
    MOTION_SINGULARITY,
    MOTION_IK_FAILED,

    // Safety errors (300-399)
    SAFETY_ESTOP = 300,
    SAFETY_DOOR_OPEN,
    SAFETY_OVERLOAD,
    SAFETY_OVERHEAT,

    // Hardware errors (400-499)
    HW_DRIVER_FAULT = 400,
    HW_ENCODER_ERROR,
    HW_BRAKE_FAULT,
    HW_POWER_FAULT,

    // Configuration errors (500-599)
    CONFIG_INVALID = 500,
    CONFIG_MISSING,

    // General errors (900-999)
    UNKNOWN_ERROR = 999
};

// String conversion functions
inline std::string toString(RobotState state) {
    switch (state) {
        case RobotState::INIT:      return "INIT";
        case RobotState::IDLE:      return "IDLE";
        case RobotState::HOMING:    return "HOMING";
        case RobotState::READY:     return "READY";
        case RobotState::MOVING:    return "MOVING";
        case RobotState::EXECUTING: return "EXECUTING";
        case RobotState::PAUSED:    return "PAUSED";
        case RobotState::ERROR:     return "ERROR";
        case RobotState::ESTOP:     return "ESTOP";
        default:                    return "UNKNOWN";
    }
}

inline std::string toString(RobotMode mode) {
    switch (mode) {
        case RobotMode::MANUAL: return "MANUAL";
        case RobotMode::AUTO:   return "AUTO";
        case RobotMode::T1:     return "T1";
        case RobotMode::T2:     return "T2";
        default:                return "UNKNOWN";
    }
}

inline std::string toString(SafetyState state) {
    switch (state) {
        case SafetyState::NORMAL:          return "NORMAL";
        case SafetyState::PROTECTIVE_STOP: return "PROTECTIVE_STOP";
        case SafetyState::EMERGENCY_STOP:  return "EMERGENCY_STOP";
        default:                           return "UNKNOWN";
    }
}

inline std::string toString(StateEvent event) {
    switch (event) {
        case StateEvent::INIT_COMPLETE:      return "INIT_COMPLETE";
        case StateEvent::ENABLE_REQUEST:     return "ENABLE_REQUEST";
        case StateEvent::DISABLE_REQUEST:    return "DISABLE_REQUEST";
        case StateEvent::RESET_REQUEST:      return "RESET_REQUEST";
        case StateEvent::HOME_REQUEST:       return "HOME_REQUEST";
        case StateEvent::HOME_COMPLETE:      return "HOME_COMPLETE";
        case StateEvent::HOME_ERROR:         return "HOME_ERROR";
        case StateEvent::MOTION_START:       return "MOTION_START";
        case StateEvent::MOTION_COMPLETE:    return "MOTION_COMPLETE";
        case StateEvent::MOTION_ERROR:       return "MOTION_ERROR";
        case StateEvent::PROGRAM_START:      return "PROGRAM_START";
        case StateEvent::PROGRAM_PAUSE:      return "PROGRAM_PAUSE";
        case StateEvent::PROGRAM_RESUME:     return "PROGRAM_RESUME";
        case StateEvent::PROGRAM_STOP:       return "PROGRAM_STOP";
        case StateEvent::PROGRAM_COMPLETE:   return "PROGRAM_COMPLETE";
        case StateEvent::PROGRAM_ERROR:      return "PROGRAM_ERROR";
        case StateEvent::ERROR_OCCURRED:     return "ERROR_OCCURRED";
        case StateEvent::ERROR_CLEARED:      return "ERROR_CLEARED";
        case StateEvent::ESTOP_PRESSED:      return "ESTOP_PRESSED";
        case StateEvent::ESTOP_RELEASED:     return "ESTOP_RELEASED";
        case StateEvent::SAFETY_DOOR_OPEN:   return "SAFETY_DOOR_OPEN";
        case StateEvent::SAFETY_DOOR_CLOSED: return "SAFETY_DOOR_CLOSED";
        default:                             return "UNKNOWN";
    }
}

inline std::string toString(ErrorCode code) {
    switch (code) {
        case ErrorCode::NONE:                  return "NONE";
        case ErrorCode::COMM_TIMEOUT:          return "COMM_TIMEOUT";
        case ErrorCode::COMM_CRC_ERROR:        return "COMM_CRC_ERROR";
        case ErrorCode::COMM_DISCONNECTED:     return "COMM_DISCONNECTED";
        case ErrorCode::MOTION_LIMIT_EXCEEDED: return "MOTION_LIMIT_EXCEEDED";
        case ErrorCode::MOTION_FOLLOWING_ERROR:return "MOTION_FOLLOWING_ERROR";
        case ErrorCode::MOTION_COLLISION:      return "MOTION_COLLISION";
        case ErrorCode::MOTION_SINGULARITY:    return "MOTION_SINGULARITY";
        case ErrorCode::MOTION_IK_FAILED:      return "MOTION_IK_FAILED";
        case ErrorCode::SAFETY_ESTOP:          return "SAFETY_ESTOP";
        case ErrorCode::SAFETY_DOOR_OPEN:      return "SAFETY_DOOR_OPEN";
        case ErrorCode::SAFETY_OVERLOAD:       return "SAFETY_OVERLOAD";
        case ErrorCode::SAFETY_OVERHEAT:       return "SAFETY_OVERHEAT";
        case ErrorCode::HW_DRIVER_FAULT:       return "HW_DRIVER_FAULT";
        case ErrorCode::HW_ENCODER_ERROR:      return "HW_ENCODER_ERROR";
        case ErrorCode::HW_BRAKE_FAULT:        return "HW_BRAKE_FAULT";
        case ErrorCode::HW_POWER_FAULT:        return "HW_POWER_FAULT";
        case ErrorCode::CONFIG_INVALID:        return "CONFIG_INVALID";
        case ErrorCode::CONFIG_MISSING:        return "CONFIG_MISSING";
        default:                               return "UNKNOWN_ERROR";
    }
}

} // namespace state
} // namespace robot_controller
