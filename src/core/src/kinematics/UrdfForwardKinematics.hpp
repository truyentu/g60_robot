/**
 * @file UrdfForwardKinematics.hpp
 * @brief URDF-based Forward Kinematics solver for 6-DOF robot
 *
 * Replaces Modified DH FK with URDF convention.
 * URDF FK formula per joint:
 *   T_joint = Translation(xyz) * RPY(rpy) * Rotation(axis, q)
 *
 * Part of Phase 10: Kinematics Overhaul (IMPL_P10_01)
 */

#pragma once

#include "MathTypes.hpp"
#include "ForwardKinematics.hpp"  // For TCPPose
#include <vector>
#include <string>

// Forward declaration
namespace robot_controller { namespace config { struct RobotPackage; } }

namespace robot_controller {
namespace kinematics {

/**
 * URDF Joint Definition
 * Stores origin, axis, and limits from URDF/YAML data
 */
struct UrdfJointDef {
    std::string name;
    Vector3d originXyz = Vector3d::Zero();   // mm
    Vector3d originRpy = Vector3d::Zero();   // rad (roll, pitch, yaw)
    Vector3d axis = Vector3d::UnitZ();       // Unit vector
    double minAngle = -PI;                   // rad
    double maxAngle = PI;                    // rad
    bool isRevolute = true;
};

/**
 * URDF-based Forward Kinematics solver
 *
 * Uses URDF joint convention instead of DH parameters:
 * - Each joint transform = Translation(xyz) * RPY(rpy) * Rotation(axis, q)
 * - Guaranteed to match viewport visualization
 * - Axes can be arbitrary (not restricted to Z-axis like DH)
 */
class UrdfForwardKinematics {
public:
    explicit UrdfForwardKinematics(const std::vector<UrdfJointDef>& joints,
                                    const Vector3d& toolOffset = Vector3d::Zero());

    /**
     * Compute TCP pose from joint angles
     * @param jointAngles Joint angles in radians
     * @return TCP pose (position in mm, orientation as rotation matrix/RPY/quaternion)
     */
    TCPPose compute(const JointAngles& jointAngles) const;

    /**
     * Compute all intermediate transforms (base→J1, base→J2, ..., base→TCP)
     * @param jointAngles Joint angles in radians
     * @return Vector of 4x4 transforms for each joint + tool
     */
    std::vector<Matrix4d> computeAllTransforms(const JointAngles& jointAngles) const;

    /**
     * Compute joint positions in world frame
     * @param jointAngles Joint angles in radians
     * @return Vector of 3D positions for each joint origin
     */
    std::vector<Vector3d> computeJointPositions(const JointAngles& jointAngles) const;

    /**
     * Compute numerical Jacobian using central differences
     * @param jointAngles Joint angles in radians
     * @return 6xN Jacobian matrix (top 3 rows = linear, bottom 3 = angular)
     */
    Jacobian computeJacobian(const JointAngles& jointAngles) const;

    /**
     * Set tool offset (fixed translation from last joint to TCP)
     */
    void setToolOffset(const Vector3d& offset);

    /**
     * Get number of joints
     */
    size_t numJoints() const { return joints_.size(); }

private:
    /**
     * Compute single joint transform: Trans(xyz) * RPY(rpy) * Rot(axis, angle)
     */
    Matrix4d computeJointTransform(size_t index, double angle) const;

    std::vector<UrdfJointDef> joints_;
    Vector3d toolOffset_;
};

/**
 * Build UrdfJointDef vector from RobotPackage YAML data
 * Converts URDF origin/axis data from RobotPackage to UrdfJointDef format
 */
std::vector<UrdfJointDef> buildUrdfJointsFromPackage(const config::RobotPackage& pkg);

} // namespace kinematics
} // namespace robot_controller
