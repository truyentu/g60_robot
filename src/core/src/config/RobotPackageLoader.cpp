/**
 * @file RobotPackageLoader.cpp
 * @brief Robot Package Loader implementation
 *
 * Part of Phase 8: Virtual Simulation (IMPL_P8_01)
 */

#include "RobotPackageLoader.hpp"
#include "../logging/Logger.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sstream>

namespace robot_controller {
namespace config {

// Default library path - will be set during initialization
std::filesystem::path RobotPackageLoader::s_libraryPath = "";

void RobotPackageLoader::setLibraryPath(const std::filesystem::path& path) {
    s_libraryPath = path;
    LOG_INFO("Robot library path set to: {}", path.string());
}

std::filesystem::path RobotPackageLoader::getLibraryPath() {
    return s_libraryPath;
}

std::optional<RobotPackage> RobotPackageLoader::loadFromDirectory(
    const std::filesystem::path& packagePath)
{
    // Check robot.yaml exists
    auto yamlPath = packagePath / "robot.yaml";
    if (!std::filesystem::exists(yamlPath)) {
        LOG_ERROR("robot.yaml not found in {}", packagePath.string());
        return std::nullopt;
    }

    RobotPackage package;
    package.package_path = packagePath.string();

    if (!parseRobotYaml(yamlPath, package)) {
        LOG_ERROR("Failed to parse robot.yaml in {}", packagePath.string());
        return std::nullopt;
    }

    // Validate
    auto error = validate(package);
    if (!error.empty()) {
        LOG_ERROR("Package validation failed: {}", error);
        return std::nullopt;
    }

    LOG_INFO("Loaded robot package: {} from {}", package.name, packagePath.string());
    return package;
}

std::optional<RobotPackage> RobotPackageLoader::loadFromZip(
    const std::filesystem::path& zipPath)
{
    // TODO: Implement ZIP extraction using minizip or similar
    LOG_WARN("ZIP loading not yet implemented: {}", zipPath.string());
    return std::nullopt;
}

std::string RobotPackageLoader::validate(const RobotPackage& package) {
    if (package.name.empty()) {
        return "Package name is empty";
    }

    if (package.joints.empty()) {
        return "Package has no joints defined";
    }

    // Check joint count
    if (package.joints.size() > 8) {
        return "Too many joints (max 8)";
    }

    // Validate each joint
    for (const auto& joint : package.joints) {
        if (joint.name.empty()) {
            return "Joint name is empty";
        }

        if (joint.type != "revolute" && joint.type != "prismatic") {
            return "Invalid joint type: " + joint.type;
        }

        if (joint.limit_min >= joint.limit_max) {
            return "Invalid limits for joint " + joint.name;
        }

        if (joint.velocity_max <= 0) {
            return "Invalid velocity limit for joint " + joint.name;
        }
    }

    // Validate home position
    if (!package.home_position.empty() &&
        package.home_position.size() != package.joints.size()) {
        return "Home position size mismatch with number of joints";
    }

    return ""; // Valid
}

std::vector<RobotPackageInfo> RobotPackageLoader::getBuiltInPackages() {
    std::vector<RobotPackageInfo> packages;

    if (s_libraryPath.empty() || !std::filesystem::exists(s_libraryPath)) {
        LOG_WARN("Robot library path not set or doesn't exist");
        return packages;
    }

    // Iterate through subdirectories
    for (const auto& entry : std::filesystem::directory_iterator(s_libraryPath)) {
        if (!entry.is_directory()) continue;

        auto robotYaml = entry.path() / "robot.yaml";
        if (!std::filesystem::exists(robotYaml)) continue;

        try {
            YAML::Node config = YAML::LoadFile(robotYaml.string());

            RobotPackageInfo info;
            info.id = entry.path().filename().string();
            info.name = config["name"].as<std::string>("");
            info.manufacturer = config["manufacturer"].as<std::string>("");
            info.model_type = config["type"].as<std::string>("");
            info.payload_kg = config["payload_kg"].as<double>(0);
            info.reach_mm = config["reach_mm"].as<double>(0);

            // Count joints
            if (config["kinematics"] && config["kinematics"]["joints"]) {
                info.dof = config["kinematics"]["joints"].size();
            }

            // Check for meshes
            auto meshesDir = entry.path() / "meshes";
            info.has_meshes = std::filesystem::exists(meshesDir);

            // Check for thumbnail
            auto thumbPath = entry.path() / "thumbnail.png";
            if (std::filesystem::exists(thumbPath)) {
                info.thumbnail_path = thumbPath.string();
            }

            packages.push_back(info);
            LOG_DEBUG("Found built-in package: {}", info.name);

        } catch (const YAML::Exception& e) {
            LOG_WARN("Failed to parse {}: {}", robotYaml.string(), e.what());
        }
    }

    return packages;
}

std::vector<std::string> RobotPackageLoader::getBuiltInPackageNames() {
    std::vector<std::string> names;
    auto packages = getBuiltInPackages();
    for (const auto& pkg : packages) {
        names.push_back(pkg.id);
    }
    return names;
}

std::optional<RobotPackage> RobotPackageLoader::loadBuiltIn(const std::string& name) {
    if (s_libraryPath.empty()) {
        LOG_ERROR("Robot library path not set");
        return std::nullopt;
    }

    auto packagePath = s_libraryPath / name;
    if (!std::filesystem::exists(packagePath)) {
        LOG_ERROR("Built-in package not found: {}", name);
        return std::nullopt;
    }

    return loadFromDirectory(packagePath);
}

bool RobotPackageLoader::parseRobotYaml(const std::filesystem::path& yamlPath,
                                         RobotPackage& package) {
    try {
        YAML::Node config = YAML::LoadFile(yamlPath.string());

        // Metadata
        package.name = config["name"].as<std::string>("");
        package.id = yamlPath.parent_path().filename().string();
        package.manufacturer = config["manufacturer"].as<std::string>("");
        package.model_type = config["type"].as<std::string>("6-axis-industrial");
        package.payload_kg = config["payload_kg"].as<double>(0);
        package.reach_mm = config["reach_mm"].as<double>(0);

        // Kinematics
        if (config["kinematics"]) {
            auto kin = config["kinematics"];
            package.dh_convention = kin["convention"].as<std::string>("modified_dh");

            // Parse joints
            if (kin["joints"]) {
                for (const auto& jointNode : kin["joints"]) {
                    JointDefinition joint;

                    joint.name = jointNode["name"].as<std::string>("");
                    joint.type = jointNode["type"].as<std::string>("revolute");

                    // DH parameters
                    if (jointNode["dh"]) {
                        auto dh = jointNode["dh"];
                        joint.dh_a = dh["a"].as<double>(0);
                        joint.dh_alpha = dh["alpha"].as<double>(0);
                        joint.dh_d = dh["d"].as<double>(0);
                        joint.dh_theta_offset = dh["theta_offset"].as<double>(0);
                    }

                    // URDF Joint Origin (optional - for visualization)
                    if (jointNode["origin"]) {
                        auto origin = jointNode["origin"];

                        if (origin["xyz"]) {
                            auto xyz = origin["xyz"];
                            if (xyz.IsSequence() && xyz.size() >= 3) {
                                joint.origin_xyz = std::array<double, 3>{
                                    xyz[0].as<double>(0),
                                    xyz[1].as<double>(0),
                                    xyz[2].as<double>(0)
                                };
                                LOG_DEBUG("Joint {} URDF origin_xyz: [{}, {}, {}]",
                                    joint.name, xyz[0].as<double>(0), xyz[1].as<double>(0), xyz[2].as<double>(0));
                            }
                        }

                        if (origin["rpy"]) {
                            auto rpy = origin["rpy"];
                            if (rpy.IsSequence() && rpy.size() >= 3) {
                                joint.origin_rpy = std::array<double, 3>{
                                    rpy[0].as<double>(0),
                                    rpy[1].as<double>(0),
                                    rpy[2].as<double>(0)
                                };
                                LOG_DEBUG("Joint {} URDF origin_rpy: [{}, {}, {}]",
                                    joint.name, rpy[0].as<double>(0), rpy[1].as<double>(0), rpy[2].as<double>(0));
                            }
                        }
                    } else {
                        LOG_WARN("Joint {} has NO origin field in YAML", joint.name);
                    }

                    // URDF Axis (optional - for visualization)
                    if (jointNode["axis"]) {
                        auto axis = jointNode["axis"];
                        if (axis.IsSequence() && axis.size() >= 3) {
                            joint.axis = std::array<double, 3>{
                                axis[0].as<double>(0),
                                axis[1].as<double>(0),
                                axis[2].as<double>(1)  // Default Z-axis
                            };
                        }
                    }

                    // Limits
                    if (jointNode["limits"]) {
                        auto limits = jointNode["limits"];
                        joint.limit_min = limits["min"].as<double>(-180);
                        joint.limit_max = limits["max"].as<double>(180);
                        joint.velocity_max = limits["vel_max"].as<double>(180);
                        joint.acceleration_max = limits["accel_max"].as<double>(360);
                    }

                    // Mesh
                    if (jointNode["mesh"]) {
                        auto mesh = jointNode["mesh"];
                        joint.mesh.visual_mesh = mesh["visual"].as<std::string>("");
                        joint.mesh.collision_mesh = mesh["collision"].as<std::string>("");

                        if (mesh["origin"]) {
                            auto origin = mesh["origin"];
                            if (origin.IsSequence() && origin.size() >= 3) {
                                joint.mesh.origin = {
                                    origin[0].as<double>(0),
                                    origin[1].as<double>(0),
                                    origin[2].as<double>(0)
                                };
                            }
                        }
                    }

                    package.joints.push_back(joint);
                }
            }
        }

        // Home position
        if (config["home_position"]) {
            for (const auto& val : config["home_position"]) {
                package.home_position.push_back(val.as<double>(0));
            }
        }

        // Base
        if (config["base"]) {
            auto base = config["base"];
            package.base_mesh = base["mesh"].as<std::string>("");

            if (base["origin"]) {
                auto origin = base["origin"];
                package.base_origin = {
                    origin["x"].as<double>(0),
                    origin["y"].as<double>(0),
                    origin["z"].as<double>(0),
                    origin["rx"].as<double>(0),
                    origin["ry"].as<double>(0),
                    origin["rz"].as<double>(0)
                };
            }
        }

        // Flange
        if (config["flange"] && config["flange"]["offset"]) {
            auto offset = config["flange"]["offset"];
            package.flange_offset = {
                offset["x"].as<double>(0),
                offset["y"].as<double>(0),
                offset["z"].as<double>(0)
            };
        }

        return true;

    } catch (const YAML::Exception& e) {
        LOG_ERROR("YAML parsing error: {}", e.what());
        return false;
    } catch (const std::exception& e) {
        LOG_ERROR("Error parsing robot.yaml: {}", e.what());
        return false;
    }
}

} // namespace config
} // namespace robot_controller
