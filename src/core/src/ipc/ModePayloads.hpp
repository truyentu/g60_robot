#pragma once

#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace robot_controller {
namespace ipc {

/**
 * Response for GET_OPERATION_MODE
 */
struct GetOperationModeResponse {
    std::string mode;               // "MANUAL", "TEST", "AUTO", "REMOTE"
    double maxLinearVelocity;       // mm/s
    double maxJointVelocity;        // deg/s
    bool deadmanRequired;
    bool safetyFenceRequired;
    bool externalControlAllowed;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(GetOperationModeResponse,
        mode, maxLinearVelocity, maxJointVelocity,
        deadmanRequired, safetyFenceRequired, externalControlAllowed)
};

/**
 * Request for SET_OPERATION_MODE
 */
struct SetOperationModeRequest {
    std::string mode;               // Target mode

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(SetOperationModeRequest, mode)
};

/**
 * Response for SET_OPERATION_MODE
 */
struct SetOperationModeResponse {
    bool success;
    std::string newMode;
    std::string error;
    std::vector<std::string> missingRequirements;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(SetOperationModeResponse,
        success, newMode, error, missingRequirements)
};

/**
 * Request for GET_MODE_REQUIREMENTS
 */
struct GetModeRequirementsRequest {
    std::string targetMode;         // Mode to check requirements for

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(GetModeRequirementsRequest, targetMode)
};

/**
 * Response for GET_MODE_REQUIREMENTS
 */
struct GetModeRequirementsResponse {
    std::string targetMode;
    bool robotStopped;
    bool safetyFenceClosed;
    bool deadmanReleased;
    bool noActiveAlarms;
    bool homingComplete;
    std::vector<std::string> missingItems;
    bool canTransition;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(GetModeRequirementsResponse,
        targetMode, robotStopped, safetyFenceClosed, deadmanReleased,
        noActiveAlarms, homingComplete, missingItems, canTransition)
};

/**
 * Event for OPERATION_MODE_CHANGED
 */
struct OperationModeChangedEvent {
    std::string previousMode;
    std::string newMode;
    double maxLinearVelocity;
    double maxJointVelocity;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(OperationModeChangedEvent,
        previousMode, newMode, maxLinearVelocity, maxJointVelocity)
};

} // namespace ipc
} // namespace robot_controller
