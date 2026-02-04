/**
 * @file OverridePayloads.hpp
 * @brief IPC payloads for override control
 */

#pragma once

#include <nlohmann/json.hpp>

namespace robot_controller {
namespace ipc {

/**
 * SET_OVERRIDE request
 */
struct SetOverrideRequest {
    int programOverride = -1;  // -1 = don't change
    int jogOverride = -1;
    int manualOverride = -1;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE_WITH_DEFAULT(SetOverrideRequest,
        programOverride, jogOverride, manualOverride)
};

/**
 * SET_OVERRIDE response
 */
struct SetOverrideResponse {
    bool success = false;
    std::string error;
    int programOverride = 100;
    int jogOverride = 100;
    int manualOverride = 100;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE_WITH_DEFAULT(SetOverrideResponse,
        success, error, programOverride, jogOverride, manualOverride)
};

/**
 * GET_OVERRIDE response
 */
struct GetOverrideResponse {
    int programOverride = 100;
    int jogOverride = 100;
    int manualOverride = 100;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE_WITH_DEFAULT(GetOverrideResponse,
        programOverride, jogOverride, manualOverride)
};

/**
 * OVERRIDE_CHANGED event
 */
struct OverrideChangedEvent {
    int programOverride = 100;
    int jogOverride = 100;
    int manualOverride = 100;
    std::string changedBy;  // "user", "safety", "program"

    NLOHMANN_DEFINE_TYPE_INTRUSIVE_WITH_DEFAULT(OverrideChangedEvent,
        programOverride, jogOverride, manualOverride, changedBy)
};

} // namespace ipc
} // namespace robot_controller
