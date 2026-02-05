/**
 * @file UrdfParser.hpp
 * @brief URDF/Xacro parser for automatic robot package import
 *
 * Part of Phase 8: Virtual Simulation - Auto Import Feature
 *
 * Parses URDF or simplified xacro files to extract:
 * - Joint origins (xyz, rpy)
 * - Joint axes
 * - Joint limits
 * - Mesh file references
 *
 * Note: This is a simplified parser that handles basic xacro syntax.
 * For full xacro support, use ROS tools to convert xacro to URDF first.
 */

#pragma once

#include <string>
#include <vector>
#include <array>
#include <optional>
#include <filesystem>
#include <map>

namespace robot_controller {
namespace config {

/**
 * Parsed joint data from URDF
 */
struct UrdfJoint {
    std::string name;
    std::string type = "revolute";  // revolute, prismatic, fixed
    std::string parent_link;
    std::string child_link;

    // Origin (in meters - needs conversion to mm)
    std::array<double, 3> origin_xyz = {0, 0, 0};
    std::array<double, 3> origin_rpy = {0, 0, 0};

    // Axis
    std::array<double, 3> axis = {0, 0, 1};

    // Limits (in radians - needs conversion to degrees)
    double limit_lower = -3.14159;
    double limit_upper = 3.14159;
    double limit_velocity = 1.0;
    double limit_effort = 0;
};

/**
 * Parsed link data from URDF
 */
struct UrdfLink {
    std::string name;
    std::string visual_mesh;      // Mesh filename from URDF
    std::string collision_mesh;

    // Visual origin (usually 0,0,0 for ROS-Industrial meshes)
    std::array<double, 3> visual_origin_xyz = {0, 0, 0};
    std::array<double, 3> visual_origin_rpy = {0, 0, 0};
};

/**
 * Complete parsed URDF model
 */
struct UrdfModel {
    std::string name;
    std::vector<UrdfLink> links;
    std::vector<UrdfJoint> joints;

    // Derived data
    std::string base_link_name;
    std::vector<std::string> joint_order;  // Ordered from base to tip
};

/**
 * Result of URDF parsing
 */
struct UrdfParseResult {
    bool success = false;
    std::string error;
    UrdfModel model;
};

/**
 * URDF Parser class
 *
 * Usage:
 *   UrdfParser parser;
 *   auto result = parser.parseFile("/path/to/robot.urdf");
 *   if (result.success) {
 *       // Use result.model
 *   }
 */
class UrdfParser {
public:
    UrdfParser() = default;

    /**
     * Parse URDF file
     * @param filepath Path to .urdf or .xacro file
     * @return Parse result with model or error
     */
    UrdfParseResult parseFile(const std::filesystem::path& filepath);

    /**
     * Parse URDF from string
     * @param xml_content URDF XML content
     * @return Parse result with model or error
     */
    UrdfParseResult parseString(const std::string& xml_content);

    /**
     * Generate robot.yaml content from parsed URDF
     * @param model Parsed URDF model
     * @param robot_name Display name for robot
     * @param manufacturer Manufacturer name
     * @return YAML content string
     */
    std::string generateYaml(
        const UrdfModel& model,
        const std::string& robot_name,
        const std::string& manufacturer = "Unknown");

    /**
     * Convert meters to millimeters
     */
    static double metersToMm(double meters) { return meters * 1000.0; }

    /**
     * Convert radians to degrees
     */
    static double radiansToDegrees(double radians) { return radians * 180.0 / 3.14159265358979; }

private:
    /**
     * Parse origin element: <origin xyz="x y z" rpy="r p y"/>
     */
    void parseOrigin(const std::string& xyz_str, const std::string& rpy_str,
                     std::array<double, 3>& xyz, std::array<double, 3>& rpy);

    /**
     * Parse axis element: <axis xyz="x y z"/>
     */
    void parseAxis(const std::string& xyz_str, std::array<double, 3>& axis);

    /**
     * Parse limit element
     */
    void parseLimit(const std::string& lower, const std::string& upper,
                    const std::string& velocity, const std::string& effort,
                    UrdfJoint& joint);

    /**
     * Expand simple xacro expressions like ${radians(180)}
     */
    double expandXacroExpression(const std::string& expr);

    /**
     * Extract mesh filename from package:// or file:// path
     */
    std::string extractMeshFilename(const std::string& mesh_path);

    /**
     * Order joints from base to tip
     */
    void orderJoints(UrdfModel& model);

    /**
     * Find base link (link with no parent joint)
     */
    std::string findBaseLink(const UrdfModel& model);
};

} // namespace config
} // namespace robot_controller
