/**
 * @file HomingTypes.hpp
 * @brief Type definitions for robot homing/mastering system
 */

#pragma once

#include <array>
#include <functional>
#include <string>
#include <vector>

namespace robot_controller {
namespace homing {

/**
 * @brief Homing method enumeration
 */
enum class HomingMethod {
    LIMIT_SWITCH,      // Use hardware limit switch
    INDEX_PULSE,       // Use encoder index pulse
    MANUAL,            // Manual positioning by operator
    ABSOLUTE_ENCODER   // Absolute encoder (no homing needed, just validation)
};

/**
 * @brief Homing state for a joint
 */
enum class HomingState {
    NOT_HOMED,          // Joint has not been homed
    HOMING_IN_PROGRESS, // Homing sequence is running
    HOMED,              // Joint is successfully homed
    HOMING_ERROR        // Homing failed
};

/**
 * @brief Internal homing sequence state
 */
enum class HomingSequenceState {
    IDLE,               // Not homing
    MOVING_TO_SWITCH,   // Moving toward limit switch (fast)
    BACKING_OFF,        // Backing off from switch
    MOVING_SLOW,        // Moving slowly to switch for precision
    SETTING_ZERO,       // Setting position to zero
    COMPLETE,           // Homing complete
    ERROR               // Error occurred
};

/**
 * @brief Homing direction
 */
enum class HomingDirection {
    POSITIVE,   // Home in positive direction
    NEGATIVE    // Home in negative direction
};

/**
 * @brief Configuration for homing a single joint
 */
struct HomingConfig {
    HomingMethod method = HomingMethod::LIMIT_SWITCH;
    HomingDirection direction = HomingDirection::NEGATIVE;

    double searchVelocity = 20.0;     // deg/s - fast search velocity
    double locateVelocity = 5.0;      // deg/s - slow locate velocity
    double backoffDistance = 5.0;     // degrees to back off from switch
    double acceleration = 50.0;       // deg/s^2

    int limitSwitchInput = -1;        // Digital input for limit switch (-1 = none)
    double indexPulseOffset = 0.0;    // Offset from index pulse to zero position
    double homeOffset = 0.0;          // Offset from switch/index to zero position

    double timeout = 60.0;            // Timeout in seconds
};

/**
 * @brief Status of homing for a single joint
 */
struct JointHomingStatus {
    int jointIndex = 0;
    HomingState state = HomingState::NOT_HOMED;
    HomingSequenceState sequenceState = HomingSequenceState::IDLE;
    double progress = 0.0;            // 0.0 - 1.0 progress through sequence
    std::string errorMessage;
    bool limitSwitchActive = false;
    double currentPosition = 0.0;     // Current position in degrees

    // Serialization helpers
    std::string stateToString() const {
        switch (state) {
            case HomingState::NOT_HOMED: return "NOT_HOMED";
            case HomingState::HOMING_IN_PROGRESS: return "HOMING_IN_PROGRESS";
            case HomingState::HOMED: return "HOMED";
            case HomingState::HOMING_ERROR: return "HOMING_ERROR";
            default: return "UNKNOWN";
        }
    }

    static HomingState stringToState(const std::string& str) {
        if (str == "NOT_HOMED") return HomingState::NOT_HOMED;
        if (str == "HOMING_IN_PROGRESS") return HomingState::HOMING_IN_PROGRESS;
        if (str == "HOMED") return HomingState::HOMED;
        if (str == "HOMING_ERROR") return HomingState::HOMING_ERROR;
        return HomingState::NOT_HOMED;
    }
};

/**
 * @brief Overall homing system status
 */
struct HomingSystemStatus {
    std::vector<JointHomingStatus> joints;
    bool allHomed = false;
    bool anyHoming = false;
    bool anyError = false;
    int totalJoints = 6;
    int homedCount = 0;
};

/**
 * @brief Callback type for homing state changes
 */
using HomingStateCallback = std::function<void(int jointIndex, HomingState newState)>;

/**
 * @brief Convert HomingMethod to string
 */
inline std::string homingMethodToString(HomingMethod method) {
    switch (method) {
        case HomingMethod::LIMIT_SWITCH: return "LIMIT_SWITCH";
        case HomingMethod::INDEX_PULSE: return "INDEX_PULSE";
        case HomingMethod::MANUAL: return "MANUAL";
        case HomingMethod::ABSOLUTE_ENCODER: return "ABSOLUTE_ENCODER";
        default: return "UNKNOWN";
    }
}

/**
 * @brief Convert string to HomingMethod
 */
inline HomingMethod stringToHomingMethod(const std::string& str) {
    if (str == "LIMIT_SWITCH") return HomingMethod::LIMIT_SWITCH;
    if (str == "INDEX_PULSE") return HomingMethod::INDEX_PULSE;
    if (str == "MANUAL") return HomingMethod::MANUAL;
    if (str == "ABSOLUTE_ENCODER") return HomingMethod::ABSOLUTE_ENCODER;
    return HomingMethod::LIMIT_SWITCH;
}

} // namespace homing
} // namespace robot_controller
