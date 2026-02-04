/**
 * @file CatalogPayloads.hpp
 * @brief IPC Payload structures for Robot Catalog operations
 */

#pragma once

#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace robot_controller {
namespace ipc {

/**
 * Summary of a robot model (for catalog listing)
 */
struct RobotModelSummary {
    std::string id;
    std::string name;
    std::string manufacturer;
    int dof = 6;
    double maxPayloadKg = 0.0;
    double reachMm = 0.0;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(RobotModelSummary,
        id, name, manufacturer, dof, maxPayloadKg, reachMm)
};

/**
 * Response for GET_ROBOT_CATALOG request
 */
struct GetRobotCatalogResponse {
    std::vector<RobotModelSummary> models;
    std::string activeModelId;
    std::string activeInstanceId;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(GetRobotCatalogResponse,
        models, activeModelId, activeInstanceId)
};

/**
 * Request to select a robot model
 */
struct SelectRobotModelRequest {
    std::string modelId;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(SelectRobotModelRequest, modelId)
};

/**
 * Response for SELECT_ROBOT_MODEL request
 */
struct SelectRobotModelResponse {
    bool success = false;
    std::string error;
    std::string modelId;
    std::string modelName;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(SelectRobotModelResponse,
        success, error, modelId, modelName)
};

/**
 * Response for GET_ACTIVE_ROBOT request
 */
struct GetActiveRobotResponse {
    std::string modelId;
    std::string modelName;
    std::string instanceId;
    std::string manufacturer;
    int dof = 6;
    double maxPayloadKg = 0.0;
    double reachMm = 0.0;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(GetActiveRobotResponse,
        modelId, modelName, instanceId, manufacturer, dof, maxPayloadKg, reachMm)
};

/**
 * Event published when robot config changes
 */
struct RobotConfigChangedEvent {
    std::string modelId;
    std::string modelName;
    std::string instanceId;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(RobotConfigChangedEvent,
        modelId, modelName, instanceId)
};

} // namespace ipc
} // namespace robot_controller
