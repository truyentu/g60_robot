/**
 * @file DHParameters.hpp
 * @brief Denavit-Hartenberg parameters for robot kinematics
 */

#pragma once

#include "MathTypes.hpp"
#include <vector>
#include <string>

namespace robot_controller {
namespace kinematics {

// ============================================================================
// DH Parameter Structure
// ============================================================================

/**
 * Denavit-Hartenberg parameters for a single joint
 * Using Modified DH convention (Craig's convention)
 *
 * Parameters:
 *   a (alpha_{i-1}): Twist angle - rotation about X_{i-1} axis
 *   alpha: Link length - distance along X_{i-1} axis
 *   d: Link offset - distance along Z_i axis
 *   theta: Joint angle - rotation about Z_i axis (variable for revolute)
 */
struct DHParameter {
    double a;           // Link length (mm)
    double alpha;       // Twist angle (rad)
    double d;           // Link offset (mm)
    double theta;       // Joint angle offset (rad) - added to actual joint angle

    // Joint limits
    double minAngle;    // Minimum joint angle (rad)
    double maxAngle;    // Maximum joint angle (rad)

    // Joint properties
    std::string name;
    bool isRevolute;    // true = revolute, false = prismatic

    DHParameter()
        : a(0), alpha(0), d(0), theta(0),
          minAngle(-PI), maxAngle(PI),
          name(""), isRevolute(true) {}

    DHParameter(double a_, double alpha_, double d_, double theta_,
                double minAngle_ = -PI, double maxAngle_ = PI,
                const std::string& name_ = "", bool isRevolute_ = true)
        : a(a_), alpha(alpha_), d(d_), theta(theta_),
          minAngle(minAngle_), maxAngle(maxAngle_),
          name(name_), isRevolute(isRevolute_) {}
};

// ============================================================================
// Robot Configuration
// ============================================================================

/**
 * Complete robot kinematic configuration
 */
struct RobotKinematicConfig {
    std::vector<DHParameter> dhParams;

    // Tool offset (from flange to TCP)
    Vector3d toolOffset;
    Matrix3d toolRotation;

    // Base offset (from world to robot base)
    Vector3d baseOffset;
    Matrix3d baseRotation;

    RobotKinematicConfig()
        : toolOffset(Vector3d::Zero()),
          toolRotation(Matrix3d::Identity()),
          baseOffset(Vector3d::Zero()),
          baseRotation(Matrix3d::Identity()) {}
};

// ============================================================================
// Default Robot Configurations
// ============================================================================

/**
 * Create DH parameters for a typical 6-DOF industrial robot
 * Similar to KUKA KR6 / ABB IRB1200 / Universal Robots UR5
 *
 * Dimensions in mm, angles in radians
 */
inline RobotKinematicConfig createDefault6DOFConfig() {
    RobotKinematicConfig config;

    // DH Parameters for 6-DOF robot (Modified DH convention)
    // These are typical values - adjust for specific robot
    //
    // Joint layout:
    //   J1: Base rotation (vertical axis)
    //   J2: Shoulder (horizontal axis)
    //   J3: Elbow (horizontal axis)
    //   J4: Wrist 1 rotation
    //   J5: Wrist 2 bend
    //   J6: Wrist 3 rotation (tool rotation)

    config.dhParams = {
        // Joint 1: Base
        DHParameter(
            0.0,                    // a: link length
            -PI / 2.0,              // alpha: -90 deg
            400.0,                  // d: base height
            0.0,                    // theta offset
            degToRad(-170.0),       // min angle
            degToRad(170.0),        // max angle
            "J1_Base",
            true
        ),
        // Joint 2: Shoulder
        DHParameter(
            350.0,                  // a: upper arm length
            0.0,                    // alpha: 0
            0.0,                    // d: 0
            -PI / 2.0,              // theta offset: -90 deg
            degToRad(-135.0),
            degToRad(35.0),
            "J2_Shoulder",
            true
        ),
        // Joint 3: Elbow
        DHParameter(
            42.0,                   // a: elbow offset
            -PI / 2.0,              // alpha: -90 deg
            0.0,                    // d: 0
            0.0,                    // theta offset
            degToRad(-120.0),
            degToRad(158.0),
            "J3_Elbow",
            true
        ),
        // Joint 4: Wrist 1
        DHParameter(
            0.0,                    // a: 0
            PI / 2.0,               // alpha: 90 deg
            350.0,                  // d: forearm length
            0.0,                    // theta offset
            degToRad(-350.0),
            degToRad(350.0),
            "J4_Wrist1",
            true
        ),
        // Joint 5: Wrist 2
        DHParameter(
            0.0,                    // a: 0
            -PI / 2.0,              // alpha: -90 deg
            0.0,                    // d: 0
            0.0,                    // theta offset
            degToRad(-125.0),
            degToRad(125.0),
            "J5_Wrist2",
            true
        ),
        // Joint 6: Wrist 3 (Tool)
        DHParameter(
            0.0,                    // a: 0
            0.0,                    // alpha: 0
            80.0,                   // d: tool flange offset
            0.0,                    // theta offset
            degToRad(-350.0),
            degToRad(350.0),
            "J6_Wrist3",
            true
        )
    };

    // Default tool offset (can be updated for specific tool)
    config.toolOffset = Vector3d(0, 0, 100.0);  // 100mm tool length
    config.toolRotation = Matrix3d::Identity();

    // Base at origin
    config.baseOffset = Vector3d::Zero();
    config.baseRotation = Matrix3d::Identity();

    return config;
}

/**
 * Create DH parameters for welding robot configuration
 * Includes typical welding torch offset
 */
inline RobotKinematicConfig createWeldingRobotConfig() {
    auto config = createDefault6DOFConfig();

    // Welding torch offset
    // Typical MIG/MAG torch: ~150mm length, 45 deg angle
    config.toolOffset = Vector3d(0, 0, 150.0);

    // Torch angled 45 degrees
    config.toolRotation = AngleAxisd(degToRad(45.0), Vector3d::UnitY()).toRotationMatrix();

    return config;
}

} // namespace kinematics
} // namespace robot_controller
