/**
 * @file RobotPackageSchema.hpp
 * @brief Robot Package Schema definitions for Virtual Simulation
 *
 * Defines structures for robot model packages with 3D mesh support.
 * Part of Phase 8: Virtual Simulation (IMPL_P8_01)
 */

#pragma once

#include <string>
#include <vector>
#include <array>
#include <optional>

namespace robot_controller {
namespace config {

/**
 * Joint mesh information for 3D visualization
 */
struct JointMeshInfo {
    std::string visual_mesh;      // Path to visual STL (relative to package)
    std::string collision_mesh;   // Path to collision STL (optional)
    std::array<double, 3> origin = {0, 0, 0}; // Mesh origin offset [x, y, z] in mm
};

/**
 * Joint definition including DH parameters, URDF origins, limits, and mesh
 */
struct JointDefinition {
    std::string name;             // A1, A2, etc.
    std::string type = "revolute"; // "revolute" or "prismatic"

    // DH Parameters (Modified DH convention) - for kinematics
    double dh_a = 0;              // Link length (mm)
    double dh_alpha = 0;          // Link twist (degrees)
    double dh_d = 0;              // Link offset (mm)
    double dh_theta_offset = 0;   // Joint angle offset (degrees)

    // URDF Joint Origin (for visualization) - optional, overrides DH for display
    std::optional<std::array<double, 3>> origin_xyz;  // Translation [x, y, z] in mm
    std::optional<std::array<double, 3>> origin_rpy;  // Rotation [roll, pitch, yaw] in radians
    std::optional<std::array<double, 3>> axis;        // Rotation axis [x, y, z]

    // Limits
    double limit_min = -180;      // degrees or mm
    double limit_max = 180;
    double velocity_max = 180;    // deg/s or mm/s
    double acceleration_max = 360;// deg/s^2 or mm/s^2

    // Mesh for visualization
    JointMeshInfo mesh;
};

/**
 * Complete robot package definition
 *
 * A robot package contains all information needed to simulate
 * and visualize a robot without hardware:
 * - Kinematic parameters (DH)
 * - Joint limits
 * - 3D mesh files (STL)
 * - Default positions
 */
struct RobotPackage {
    // Metadata
    std::string name;             // Display name: "KUKA KR 6 R900"
    std::string id;               // Unique ID: "kuka_kr6_r900"
    std::string manufacturer;     // "KUKA", "ABB", etc.
    std::string model_type;       // "6-axis-industrial", etc.
    double payload_kg = 0;
    double reach_mm = 0;

    // Kinematics
    std::string dh_convention = "modified_dh"; // "modified_dh" or "standard_dh"
    std::vector<JointDefinition> joints;

    // Base
    std::string base_mesh;        // Path to base STL
    std::array<double, 6> base_origin = {0, 0, 0, 0, 0, 0}; // x, y, z, rx, ry, rz

    // Flange (tool mounting point)
    std::array<double, 3> flange_offset = {0, 0, 0}; // x, y, z from last joint

    // Default positions
    std::vector<double> home_position;  // Default home position for all joints

    // Package path (for loading meshes)
    std::string package_path;

    /**
     * Get number of degrees of freedom
     */
    size_t getDOF() const { return joints.size(); }

    /**
     * Check if package has meshes
     */
    bool hasMeshes() const {
        if (!base_mesh.empty()) return true;
        for (const auto& joint : joints) {
            if (!joint.mesh.visual_mesh.empty()) return true;
        }
        return false;
    }
};

/**
 * Lightweight package info for listing
 */
struct RobotPackageInfo {
    std::string id;
    std::string name;
    std::string manufacturer;
    std::string model_type;
    double payload_kg = 0;
    double reach_mm = 0;
    size_t dof = 6;
    bool has_meshes = false;
    std::string thumbnail_path;   // Path to preview image
};

} // namespace config
} // namespace robot_controller
