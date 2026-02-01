/**
 * @file MathTypes.hpp
 * @brief Math types and utilities for robot kinematics
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <cmath>

namespace robot_controller {
namespace kinematics {

// ============================================================================
// Type Definitions
// ============================================================================

// Basic types
using Vector3d = Eigen::Vector3d;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Quaterniond = Eigen::Quaterniond;
using AngleAxisd = Eigen::AngleAxisd;

// Joint space
constexpr int NUM_JOINTS = 6;
using JointAngles = std::array<double, NUM_JOINTS>;
using JointVelocities = std::array<double, NUM_JOINTS>;
using JointTorques = std::array<double, NUM_JOINTS>;

// Jacobian matrix (6x6 for 6-DOF robot)
using Jacobian = Eigen::Matrix<double, 6, NUM_JOINTS>;

// ============================================================================
// Constants
// ============================================================================

constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;
constexpr double EPSILON = 1e-10;

// ============================================================================
// Utility Functions
// ============================================================================

inline double degToRad(double degrees) {
    return degrees * DEG_TO_RAD;
}

inline double radToDeg(double radians) {
    return radians * RAD_TO_DEG;
}

inline bool isNearZero(double value, double tolerance = EPSILON) {
    return std::abs(value) < tolerance;
}

inline double normalizeAngle(double angle) {
    while (angle > PI) angle -= 2.0 * PI;
    while (angle < -PI) angle += 2.0 * PI;
    return angle;
}

/**
 * Convert rotation matrix to RPY (Roll-Pitch-Yaw) angles
 * Convention: ZYX (Yaw-Pitch-Roll applied in that order)
 */
inline Vector3d rotationToRPY(const Matrix3d& R) {
    Vector3d rpy;

    // Check for gimbal lock
    if (std::abs(R(2, 0)) >= 1.0 - EPSILON) {
        // Gimbal lock: pitch = +/- 90 degrees
        rpy(2) = 0.0;  // Set yaw to 0
        if (R(2, 0) < 0) {
            rpy(1) = PI / 2.0;
            rpy(0) = std::atan2(R(0, 1), R(0, 2));
        } else {
            rpy(1) = -PI / 2.0;
            rpy(0) = std::atan2(-R(0, 1), -R(0, 2));
        }
    } else {
        rpy(1) = std::asin(-R(2, 0));  // Pitch
        rpy(0) = std::atan2(R(2, 1), R(2, 2));  // Roll
        rpy(2) = std::atan2(R(1, 0), R(0, 0));  // Yaw
    }

    return rpy;
}

/**
 * Convert RPY angles to rotation matrix
 * Convention: ZYX (Rz * Ry * Rx)
 */
inline Matrix3d rpyToRotation(const Vector3d& rpy) {
    double roll = rpy(0);
    double pitch = rpy(1);
    double yaw = rpy(2);

    Matrix3d Rx, Ry, Rz;

    Rx << 1, 0, 0,
          0, std::cos(roll), -std::sin(roll),
          0, std::sin(roll), std::cos(roll);

    Ry << std::cos(pitch), 0, std::sin(pitch),
          0, 1, 0,
          -std::sin(pitch), 0, std::cos(pitch);

    Rz << std::cos(yaw), -std::sin(yaw), 0,
          std::sin(yaw), std::cos(yaw), 0,
          0, 0, 1;

    return Rz * Ry * Rx;  // ZYX convention
}

} // namespace kinematics
} // namespace robot_controller
