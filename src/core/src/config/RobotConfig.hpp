/**
 * @file RobotConfig.hpp
 * @brief Robot configuration data structures
 */

#pragma once

#include <string>
#include <vector>
#include <array>

namespace robot_controller {
namespace config {

/**
 * DH Parameter for a single joint
 */
struct DHParameter {
    int joint = 0;
    double a = 0.0;             // Link length (mm)
    double alpha = 0.0;         // Link twist (degrees)
    double d = 0.0;             // Link offset (mm)
    double theta_offset = 0.0;  // Joint angle offset (degrees)

    // Convert alpha to radians
    double alphaRad() const { return alpha * 3.14159265358979 / 180.0; }
    // Convert theta_offset to radians
    double thetaOffsetRad() const { return theta_offset * 3.14159265358979 / 180.0; }
};

/**
 * Joint limits for a single joint
 */
struct JointLimit {
    int joint = 0;
    double min = -180.0;            // Min angle (degrees)
    double max = 180.0;             // Max angle (degrees)
    double max_velocity = 100.0;    // Max velocity (deg/s)
    double max_acceleration = 500.0; // Max acceleration (deg/s²)

    // Convert to radians
    double minRad() const { return min * 3.14159265358979 / 180.0; }
    double maxRad() const { return max * 3.14159265358979 / 180.0; }
    double maxVelocityRad() const { return max_velocity * 3.14159265358979 / 180.0; }
    double maxAccelerationRad() const { return max_acceleration * 3.14159265358979 / 180.0; }
};

/**
 * TCP (Tool Center Point) offset
 */
struct TcpOffset {
    double x = 0.0;     // mm
    double y = 0.0;     // mm
    double z = 0.0;     // mm
    double rx = 0.0;    // degrees
    double ry = 0.0;    // degrees
    double rz = 0.0;    // degrees
};

/**
 * Complete robot configuration
 */
struct RobotConfig {
    // Identification
    std::string name = "Robot6DOF";
    std::string type = "6-axis-articulated";
    std::string manufacturer = "Custom";
    std::string model = "RC-6DOF-01";

    // Kinematics
    std::vector<DHParameter> dh_parameters;
    std::vector<JointLimit> joint_limits;
    TcpOffset tcp_offset;

    // Home position (degrees)
    std::array<double, 6> home_position = {0, 0, 0, 0, 0, 0};

    // Specifications
    double max_payload_kg = 6.0;
    double reach_mm = 1200.0;

    /**
     * Get number of joints (should always be 6)
     */
    size_t numJoints() const {
        return dh_parameters.size();
    }

    /**
     * Check if configuration is valid
     */
    bool isValid() const {
        return dh_parameters.size() == 6 && joint_limits.size() == 6;
    }
};

} // namespace config
} // namespace robot_controller
