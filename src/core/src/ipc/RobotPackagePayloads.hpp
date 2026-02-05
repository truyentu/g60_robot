/**
 * @file RobotPackagePayloads.hpp
 * @brief IPC Payload types for Robot Package operations
 *
 * Part of Phase 8: Virtual Simulation (IMPL_P8_01)
 *
 * IMPORTANT: Implementations moved to RobotPackagePayloads.cpp to ensure proper recompilation.
 * Previously, inline functions were cached by compiler and not recompiled even after changes.
 */

#pragma once

#include "../config/RobotPackageSchema.hpp"
#include <nlohmann/json.hpp>
#include <vector>
#include <string>

namespace robot_controller {
namespace ipc {

// Function declarations - implementations in RobotPackagePayloads.cpp

/**
 * Convert RobotPackageInfo to JSON
 */
nlohmann::json packageInfoToJson(const config::RobotPackageInfo& info);

/**
 * Convert JSON to RobotPackageInfo
 */
config::RobotPackageInfo jsonToPackageInfo(const nlohmann::json& j);

/**
 * Convert JointMeshInfo to JSON
 */
nlohmann::json jointMeshToJson(const config::JointMeshInfo& mesh);

/**
 * Convert JointDefinition to JSON
 * NOW PROPERLY SERIALIZES URDF FIELDS!
 */
nlohmann::json jointDefToJson(const config::JointDefinition& joint);

/**
 * Convert RobotPackage to JSON
 */
nlohmann::json packageToJson(const config::RobotPackage& pkg);

/**
 * Convert JSON to JointMeshInfo
 */
config::JointMeshInfo jsonToJointMesh(const nlohmann::json& j);

/**
 * Convert JSON to JointDefinition
 */
config::JointDefinition jsonToJointDef(const nlohmann::json& j);

/**
 * Convert JSON to RobotPackage
 */
config::RobotPackage jsonToPackage(const nlohmann::json& j);

} // namespace ipc
} // namespace robot_controller
