#pragma once

#include <string>
#include <map>

namespace robot_controller {
namespace mode {

/**
 * @brief KUKA-inspired operation modes
 *
 * Each mode has specific velocity limits and safety requirements:
 * - MANUAL (T1): Teaching mode, max 250mm/s, deadman required
 * - TEST (T2): Test mode, full speed, operator supervision
 * - AUTO: Automatic mode, full speed, safety interlocks required
 * - REMOTE: External control via PLC, full safety system
 */
enum class OperationMode {
    MANUAL,     // T1 - Teaching mode
    TEST,       // T2 - Test mode
    AUTO,       // Automatic operation
    REMOTE      // External PLC control
};

/**
 * @brief Configuration parameters for each operation mode
 */
struct OperationModeConfig {
    double maxLinearVelocity;      // Maximum TCP velocity in mm/s
    double maxJointVelocity;       // Maximum joint velocity in deg/s
    bool requireDeadman;           // Deadman switch must be held
    bool requireSafetyFence;       // Safety fence must be closed
    bool allowExternalControl;     // PLC can control robot
    bool reducedWorkspace;         // Use reduced workspace limits
};

/**
 * @brief Requirements that must be met for mode transition
 */
struct ModeTransitionRequirement {
    bool robotStopped;             // Robot must be stationary
    bool safetyFenceClosed;        // Safety fence must be closed
    bool deadmanReleased;          // Deadman switch must be released
    bool noActiveAlarms;           // No active alarms/errors
    bool homingComplete;           // Homing must be completed
};

/**
 * @brief String conversion for OperationMode
 */
inline std::string operationModeToString(OperationMode mode) {
    switch (mode) {
        case OperationMode::MANUAL: return "MANUAL";
        case OperationMode::TEST:   return "TEST";
        case OperationMode::AUTO:   return "AUTO";
        case OperationMode::REMOTE: return "REMOTE";
        default: return "UNKNOWN";
    }
}

/**
 * @brief Parse OperationMode from string
 */
inline OperationMode operationModeFromString(const std::string& str) {
    if (str == "MANUAL") return OperationMode::MANUAL;
    if (str == "TEST")   return OperationMode::TEST;
    if (str == "AUTO")   return OperationMode::AUTO;
    if (str == "REMOTE") return OperationMode::REMOTE;
    return OperationMode::MANUAL; // Default to safest mode
}

/**
 * @brief Mode configuration constants (KUKA-inspired limits)
 *
 * MANUAL: Reduced speed for safe teaching
 * TEST: Full speed but with operator supervision
 * AUTO: Full speed with automatic safety system
 * REMOTE: Full speed with PLC control
 */
const std::map<OperationMode, OperationModeConfig> MODE_CONFIGS = {
    {OperationMode::MANUAL, {
        250.0,      // maxLinearVelocity: 250 mm/s (T1 limit)
        30.0,       // maxJointVelocity: 30 deg/s
        true,       // requireDeadman
        false,      // requireSafetyFence
        false,      // allowExternalControl
        false       // reducedWorkspace
    }},
    {OperationMode::TEST, {
        2000.0,     // maxLinearVelocity: 2000 mm/s
        180.0,      // maxJointVelocity: 180 deg/s
        false,      // requireDeadman
        false,      // requireSafetyFence
        false,      // allowExternalControl
        true        // reducedWorkspace (test with limits)
    }},
    {OperationMode::AUTO, {
        2000.0,     // maxLinearVelocity: 2000 mm/s
        180.0,      // maxJointVelocity: 180 deg/s
        false,      // requireDeadman
        true,       // requireSafetyFence
        false,      // allowExternalControl
        false       // reducedWorkspace
    }},
    {OperationMode::REMOTE, {
        2000.0,     // maxLinearVelocity: 2000 mm/s
        180.0,      // maxJointVelocity: 180 deg/s
        false,      // requireDeadman
        true,       // requireSafetyFence
        true,       // allowExternalControl
        false       // reducedWorkspace
    }}
};

} // namespace mode
} // namespace robot_controller
