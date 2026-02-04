/**
 * @file HomingPayloads.hpp
 * @brief IPC payload structures for homing operations
 */

#pragma once

#include "../homing/HomingTypes.hpp"
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace robot_controller {
namespace ipc {

/**
 * @brief Request to start homing
 */
struct StartHomingRequest {
    int jointIndex = -1;        // -1 = all joints
    std::string method = "LIMIT_SWITCH";

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(StartHomingRequest, jointIndex, method)
};

/**
 * @brief Response for start homing request
 */
struct StartHomingResponse {
    bool success = false;
    std::string error;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(StartHomingResponse, success, error)
};

/**
 * @brief Request to stop homing
 */
struct StopHomingRequest {
    int jointIndex = -1;        // -1 = all joints

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(StopHomingRequest, jointIndex)
};

/**
 * @brief Response for stop homing request
 */
struct StopHomingResponse {
    bool success = false;
    std::string error;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(StopHomingResponse, success, error)
};

/**
 * @brief Status of a single joint's homing
 */
struct JointHomingStatusPayload {
    int jointIndex = 0;
    std::string state = "NOT_HOMED";
    double progress = 0.0;
    std::string errorMessage;
    bool limitSwitchActive = false;
    double currentPosition = 0.0;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(JointHomingStatusPayload,
        jointIndex, state, progress, errorMessage, limitSwitchActive, currentPosition)

    // Convert from internal type
    static JointHomingStatusPayload fromInternal(const homing::JointHomingStatus& status) {
        JointHomingStatusPayload payload;
        payload.jointIndex = status.jointIndex;
        payload.state = status.stateToString();
        payload.progress = status.progress;
        payload.errorMessage = status.errorMessage;
        payload.limitSwitchActive = status.limitSwitchActive;
        payload.currentPosition = status.currentPosition;
        return payload;
    }
};

/**
 * @brief Response for get homing state request
 */
struct HomingStateResponse {
    std::vector<JointHomingStatusPayload> joints;
    bool allHomed = false;
    bool anyHoming = false;
    bool anyError = false;
    int totalJoints = 6;
    int homedCount = 0;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(HomingStateResponse,
        joints, allHomed, anyHoming, anyError, totalJoints, homedCount)

    // Convert from internal type
    static HomingStateResponse fromInternal(const homing::HomingSystemStatus& status) {
        HomingStateResponse response;
        response.allHomed = status.allHomed;
        response.anyHoming = status.anyHoming;
        response.anyError = status.anyError;
        response.totalJoints = status.totalJoints;
        response.homedCount = status.homedCount;

        for (const auto& joint : status.joints) {
            response.joints.push_back(JointHomingStatusPayload::fromInternal(joint));
        }

        return response;
    }
};

/**
 * @brief Event published when homing state changes
 */
struct HomingStateChangedEvent {
    int jointIndex = 0;
    std::string previousState;
    std::string newState;
    std::string errorMessage;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(HomingStateChangedEvent,
        jointIndex, previousState, newState, errorMessage)
};

} // namespace ipc
} // namespace robot_controller
