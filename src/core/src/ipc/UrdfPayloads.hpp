/**
 * @file UrdfPayloads.hpp
 * @brief IPC Payload types for URDF import operations
 *
 * Part of Phase 8: Virtual Simulation - Auto Import Feature
 */

#pragma once

#include "../config/UrdfParser.hpp"
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace robot_controller {
namespace ipc {

/**
 * Convert UrdfJoint to JSON
 */
inline nlohmann::json urdfJointToJson(const config::UrdfJoint& joint) {
    return nlohmann::json{
        {"name", joint.name},
        {"type", joint.type},
        {"parent_link", joint.parent_link},
        {"child_link", joint.child_link},
        {"origin_xyz", {
            config::UrdfParser::metersToMm(joint.origin_xyz[0]),
            config::UrdfParser::metersToMm(joint.origin_xyz[1]),
            config::UrdfParser::metersToMm(joint.origin_xyz[2])
        }},
        {"origin_rpy", {joint.origin_rpy[0], joint.origin_rpy[1], joint.origin_rpy[2]}},
        {"axis", {joint.axis[0], joint.axis[1], joint.axis[2]}},
        {"limit_lower_deg", config::UrdfParser::radiansToDegrees(joint.limit_lower)},
        {"limit_upper_deg", config::UrdfParser::radiansToDegrees(joint.limit_upper)},
        {"limit_velocity_deg", config::UrdfParser::radiansToDegrees(joint.limit_velocity)}
    };
}

/**
 * Convert UrdfLink to JSON
 */
inline nlohmann::json urdfLinkToJson(const config::UrdfLink& link) {
    return nlohmann::json{
        {"name", link.name},
        {"visual_mesh", link.visual_mesh},
        {"collision_mesh", link.collision_mesh}
    };
}

/**
 * Convert UrdfModel to JSON
 */
inline nlohmann::json urdfModelToJson(const config::UrdfModel& model) {
    nlohmann::json links = nlohmann::json::array();
    for (const auto& link : model.links) {
        links.push_back(urdfLinkToJson(link));
    }

    nlohmann::json joints = nlohmann::json::array();
    for (const auto& joint : model.joints) {
        joints.push_back(urdfJointToJson(joint));
    }

    return nlohmann::json{
        {"name", model.name},
        {"base_link", model.base_link_name},
        {"joint_order", model.joint_order},
        {"links", links},
        {"joints", joints}
    };
}

/**
 * Request for PARSE_URDF
 */
struct ParseUrdfRequest {
    std::string urdf_content;  // URDF XML content (or file path)
    bool is_file_path = false; // If true, urdf_content is a file path
};

inline nlohmann::json parseUrdfRequestToJson(const ParseUrdfRequest& req) {
    return nlohmann::json{
        {"urdf_content", req.urdf_content},
        {"is_file_path", req.is_file_path}
    };
}

inline ParseUrdfRequest jsonToParseUrdfRequest(const nlohmann::json& j) {
    ParseUrdfRequest req;
    req.urdf_content = j.value("urdf_content", "");
    req.is_file_path = j.value("is_file_path", false);
    return req;
}

/**
 * Response for PARSE_URDF
 */
struct ParseUrdfResponse {
    bool success = false;
    std::string error;
    config::UrdfModel model;
};

inline nlohmann::json parseUrdfResponseToJson(const ParseUrdfResponse& resp) {
    nlohmann::json j = {
        {"success", resp.success},
        {"error", resp.error}
    };

    if (resp.success) {
        j["model"] = urdfModelToJson(resp.model);
    }

    return j;
}

/**
 * Request for GENERATE_ROBOT_YAML
 */
struct GenerateRobotYamlRequest {
    std::string urdf_content;
    bool is_file_path = false;
    std::string robot_name;
    std::string manufacturer;
    std::string output_path;  // Where to save robot.yaml
};

inline GenerateRobotYamlRequest jsonToGenerateYamlRequest(const nlohmann::json& j) {
    GenerateRobotYamlRequest req;
    req.urdf_content = j.value("urdf_content", "");
    req.is_file_path = j.value("is_file_path", false);
    req.robot_name = j.value("robot_name", "");
    req.manufacturer = j.value("manufacturer", "Unknown");
    req.output_path = j.value("output_path", "");
    return req;
}

/**
 * Response for GENERATE_ROBOT_YAML
 */
struct GenerateRobotYamlResponse {
    bool success = false;
    std::string error;
    std::string yaml_content;
    std::string saved_path;
};

inline nlohmann::json generateYamlResponseToJson(const GenerateRobotYamlResponse& resp) {
    return nlohmann::json{
        {"success", resp.success},
        {"error", resp.error},
        {"yaml_content", resp.yaml_content},
        {"saved_path", resp.saved_path}
    };
}

} // namespace ipc
} // namespace robot_controller
