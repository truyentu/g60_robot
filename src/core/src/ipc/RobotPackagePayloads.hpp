/**
 * @file RobotPackagePayloads.hpp
 * @brief IPC Payload types for Robot Package operations
 *
 * Part of Phase 8: Virtual Simulation (IMPL_P8_01)
 */

#pragma once

#include "../config/RobotPackageSchema.hpp"
#include <nlohmann/json.hpp>
#include <vector>
#include <string>

namespace robot_controller {
namespace ipc {

/**
 * Convert RobotPackageInfo to JSON
 */
inline nlohmann::json packageInfoToJson(const config::RobotPackageInfo& info) {
    return nlohmann::json{
        {"id", info.id},
        {"name", info.name},
        {"manufacturer", info.manufacturer},
        {"model_type", info.model_type},
        {"payload_kg", info.payload_kg},
        {"reach_mm", info.reach_mm},
        {"dof", info.dof},
        {"has_meshes", info.has_meshes},
        {"thumbnail_path", info.thumbnail_path}
    };
}

/**
 * Convert JSON to RobotPackageInfo
 */
inline config::RobotPackageInfo jsonToPackageInfo(const nlohmann::json& j) {
    config::RobotPackageInfo info;
    info.id = j.value("id", "");
    info.name = j.value("name", "");
    info.manufacturer = j.value("manufacturer", "");
    info.model_type = j.value("model_type", "");
    info.payload_kg = j.value("payload_kg", 0.0);
    info.reach_mm = j.value("reach_mm", 0.0);
    info.dof = j.value("dof", 6);
    info.has_meshes = j.value("has_meshes", false);
    info.thumbnail_path = j.value("thumbnail_path", "");
    return info;
}

/**
 * Convert JointMeshInfo to JSON
 */
inline nlohmann::json jointMeshToJson(const config::JointMeshInfo& mesh) {
    return nlohmann::json{
        {"visual_mesh", mesh.visual_mesh},
        {"collision_mesh", mesh.collision_mesh},
        {"origin", mesh.origin}
    };
}

/**
 * Convert JointDefinition to JSON
 */
inline nlohmann::json jointDefToJson(const config::JointDefinition& joint) {
    return nlohmann::json{
        {"name", joint.name},
        {"type", joint.type},
        {"dh_a", joint.dh_a},
        {"dh_alpha", joint.dh_alpha},
        {"dh_d", joint.dh_d},
        {"dh_theta_offset", joint.dh_theta_offset},
        {"limit_min", joint.limit_min},
        {"limit_max", joint.limit_max},
        {"velocity_max", joint.velocity_max},
        {"acceleration_max", joint.acceleration_max},
        {"mesh", jointMeshToJson(joint.mesh)}
    };
}

/**
 * Convert RobotPackage to JSON
 */
inline nlohmann::json packageToJson(const config::RobotPackage& pkg) {
    nlohmann::json joints = nlohmann::json::array();
    for (const auto& joint : pkg.joints) {
        joints.push_back(jointDefToJson(joint));
    }

    return nlohmann::json{
        {"name", pkg.name},
        {"id", pkg.id},
        {"manufacturer", pkg.manufacturer},
        {"model_type", pkg.model_type},
        {"payload_kg", pkg.payload_kg},
        {"reach_mm", pkg.reach_mm},
        {"dh_convention", pkg.dh_convention},
        {"joints", joints},
        {"base_mesh", pkg.base_mesh},
        {"base_origin", pkg.base_origin},
        {"flange_offset", pkg.flange_offset},
        {"home_position", pkg.home_position},
        {"package_path", pkg.package_path}
    };
}

/**
 * Convert JSON to JointMeshInfo
 */
inline config::JointMeshInfo jsonToJointMesh(const nlohmann::json& j) {
    config::JointMeshInfo mesh;
    mesh.visual_mesh = j.value("visual_mesh", "");
    mesh.collision_mesh = j.value("collision_mesh", "");

    if (j.contains("origin") && j["origin"].is_array() && j["origin"].size() >= 3) {
        mesh.origin = {
            j["origin"][0].get<double>(),
            j["origin"][1].get<double>(),
            j["origin"][2].get<double>()
        };
    }

    return mesh;
}

/**
 * Convert JSON to JointDefinition
 */
inline config::JointDefinition jsonToJointDef(const nlohmann::json& j) {
    config::JointDefinition joint;
    joint.name = j.value("name", "");
    joint.type = j.value("type", "revolute");
    joint.dh_a = j.value("dh_a", 0.0);
    joint.dh_alpha = j.value("dh_alpha", 0.0);
    joint.dh_d = j.value("dh_d", 0.0);
    joint.dh_theta_offset = j.value("dh_theta_offset", 0.0);
    joint.limit_min = j.value("limit_min", -180.0);
    joint.limit_max = j.value("limit_max", 180.0);
    joint.velocity_max = j.value("velocity_max", 180.0);
    joint.acceleration_max = j.value("acceleration_max", 360.0);

    if (j.contains("mesh")) {
        joint.mesh = jsonToJointMesh(j["mesh"]);
    }

    return joint;
}

/**
 * Convert JSON to RobotPackage
 */
inline config::RobotPackage jsonToPackage(const nlohmann::json& j) {
    config::RobotPackage pkg;
    pkg.name = j.value("name", "");
    pkg.id = j.value("id", "");
    pkg.manufacturer = j.value("manufacturer", "");
    pkg.model_type = j.value("model_type", "");
    pkg.payload_kg = j.value("payload_kg", 0.0);
    pkg.reach_mm = j.value("reach_mm", 0.0);
    pkg.dh_convention = j.value("dh_convention", "modified_dh");

    if (j.contains("joints") && j["joints"].is_array()) {
        for (const auto& jj : j["joints"]) {
            pkg.joints.push_back(jsonToJointDef(jj));
        }
    }

    pkg.base_mesh = j.value("base_mesh", "");

    if (j.contains("base_origin") && j["base_origin"].is_array() && j["base_origin"].size() >= 6) {
        for (size_t i = 0; i < 6; ++i) {
            pkg.base_origin[i] = j["base_origin"][i].get<double>();
        }
    }

    if (j.contains("flange_offset") && j["flange_offset"].is_array() && j["flange_offset"].size() >= 3) {
        for (size_t i = 0; i < 3; ++i) {
            pkg.flange_offset[i] = j["flange_offset"][i].get<double>();
        }
    }

    if (j.contains("home_position") && j["home_position"].is_array()) {
        for (const auto& val : j["home_position"]) {
            pkg.home_position.push_back(val.get<double>());
        }
    }

    pkg.package_path = j.value("package_path", "");

    return pkg;
}

} // namespace ipc
} // namespace robot_controller
