/**
 * @file ForwardKinematics.hpp
 * @brief Forward kinematics solver for 6-DOF robot
 *
 * DEPRECATED: Modified DH FK produces incorrect results for MA2010.
 * Use UrdfForwardKinematics instead (IMPL_P10_01).
 * Kept for IK backward compatibility until KDL IK is integrated (IMPL_P10_02).
 */

#pragma once

#include "DHParameters.hpp"
#include <vector>

namespace robot_controller {
namespace kinematics {

// ============================================================================
// TCP Pose Result
// ============================================================================

/**
 * Complete pose of Tool Center Point (TCP)
 */
struct TCPPose {
    Vector3d position;      // XYZ position in mm
    Matrix3d rotation;      // Rotation matrix
    Vector3d rpy;           // Roll-Pitch-Yaw angles in rad
    Quaterniond quaternion; // Orientation as quaternion

    TCPPose()
        : position(Vector3d::Zero()),
          rotation(Matrix3d::Identity()),
          rpy(Vector3d::Zero()),
          quaternion(Quaterniond::Identity()) {}

    // Create from homogeneous transformation matrix
    static TCPPose fromTransform(const Matrix4d& T) {
        TCPPose pose;
        pose.position = T.block<3, 1>(0, 3);
        pose.rotation = T.block<3, 3>(0, 0);
        pose.rpy = rotationToRPY(pose.rotation);
        pose.quaternion = Quaterniond(pose.rotation);
        return pose;
    }

    // Convert to homogeneous transformation matrix
    Matrix4d toTransform() const {
        Matrix4d T = Matrix4d::Identity();
        T.block<3, 3>(0, 0) = rotation;
        T.block<3, 1>(0, 3) = position;
        return T;
    }

    // Get as 6-element vector [x, y, z, roll, pitch, yaw]
    Vector6d toVector() const {
        Vector6d v;
        v << position, rpy;
        return v;
    }
};

// ============================================================================
// Forward Kinematics Class
// ============================================================================

/**
 * Forward Kinematics solver using DH convention
 * Computes TCP pose from joint angles
 */
class ForwardKinematics {
public:
    explicit ForwardKinematics(const RobotKinematicConfig& config);
    ~ForwardKinematics() = default;

    // ========================================================================
    // Main FK Methods
    // ========================================================================

    /**
     * Compute TCP pose from joint angles
     * @param jointAngles Current joint angles in radians
     * @return TCP pose (position and orientation)
     */
    TCPPose compute(const JointAngles& jointAngles) const;

    /**
     * Compute transformation matrix for single joint
     * @param jointIndex Index of joint (0-5)
     * @param jointAngle Current angle of joint in radians
     * @return 4x4 homogeneous transformation matrix
     */
    Matrix4d computeJointTransform(int jointIndex, double jointAngle) const;

    /**
     * Compute transformation matrices for all frames
     * Returns transforms from base to each joint frame
     * @param jointAngles Current joint angles
     * @return Vector of transforms [T_0^1, T_0^2, ..., T_0^6, T_0^TCP]
     */
    std::vector<Matrix4d> computeAllTransforms(const JointAngles& jointAngles) const;

    /**
     * Get position of each joint in world frame
     * Useful for visualization and collision detection
     */
    std::vector<Vector3d> computeJointPositions(const JointAngles& jointAngles) const;

    // ========================================================================
    // Configuration
    // ========================================================================

    void setConfig(const RobotKinematicConfig& config);
    const RobotKinematicConfig& getConfig() const { return config_; }

    void setToolOffset(const Vector3d& offset, const Matrix3d& rotation);
    void setBaseOffset(const Vector3d& offset, const Matrix3d& rotation);

private:
    RobotKinematicConfig config_;

    // Precomputed base and tool transforms
    Matrix4d baseTransform_;
    Matrix4d toolTransform_;

    void updateTransforms();
};

// ============================================================================
// Implementation
// ============================================================================

inline ForwardKinematics::ForwardKinematics(const RobotKinematicConfig& config)
    : config_(config) {
    updateTransforms();
}

inline void ForwardKinematics::updateTransforms() {
    // Base transform
    baseTransform_ = Matrix4d::Identity();
    baseTransform_.block<3, 3>(0, 0) = config_.baseRotation;
    baseTransform_.block<3, 1>(0, 3) = config_.baseOffset;

    // Tool transform
    toolTransform_ = Matrix4d::Identity();
    toolTransform_.block<3, 3>(0, 0) = config_.toolRotation;
    toolTransform_.block<3, 1>(0, 3) = config_.toolOffset;
}

inline void ForwardKinematics::setConfig(const RobotKinematicConfig& config) {
    config_ = config;
    updateTransforms();
}

inline void ForwardKinematics::setToolOffset(const Vector3d& offset, const Matrix3d& rotation) {
    config_.toolOffset = offset;
    config_.toolRotation = rotation;
    updateTransforms();
}

inline void ForwardKinematics::setBaseOffset(const Vector3d& offset, const Matrix3d& rotation) {
    config_.baseOffset = offset;
    config_.baseRotation = rotation;
    updateTransforms();
}

inline Matrix4d ForwardKinematics::computeJointTransform(int jointIndex, double jointAngle) const {
    if (jointIndex < 0 || jointIndex >= static_cast<int>(config_.dhParams.size())) {
        return Matrix4d::Identity();
    }

    const auto& dh = config_.dhParams[jointIndex];

    // Modified DH transformation matrix
    // T = Rot_x(alpha) * Trans_x(a) * Rot_z(theta) * Trans_z(d)

    double theta = jointAngle + dh.theta;  // Add offset
    double ct = std::cos(theta);
    double st = std::sin(theta);
    double ca = std::cos(dh.alpha);
    double sa = std::sin(dh.alpha);

    Matrix4d T;
    T << ct,       -st,       0,      dh.a,
         st * ca,  ct * ca,  -sa,    -sa * dh.d,
         st * sa,  ct * sa,   ca,     ca * dh.d,
         0,        0,         0,      1;

    return T;
}

inline TCPPose ForwardKinematics::compute(const JointAngles& jointAngles) const {
    // Start with base transform
    Matrix4d T = baseTransform_;

    // Multiply through all joint transforms
    for (size_t i = 0; i < config_.dhParams.size(); ++i) {
        T = T * computeJointTransform(static_cast<int>(i), jointAngles[i]);
    }

    // Apply tool transform
    T = T * toolTransform_;

    return TCPPose::fromTransform(T);
}

inline std::vector<Matrix4d> ForwardKinematics::computeAllTransforms(const JointAngles& jointAngles) const {
    std::vector<Matrix4d> transforms;
    transforms.reserve(config_.dhParams.size() + 1);

    Matrix4d T = baseTransform_;

    for (size_t i = 0; i < config_.dhParams.size(); ++i) {
        T = T * computeJointTransform(static_cast<int>(i), jointAngles[i]);
        transforms.push_back(T);
    }

    // Add TCP transform
    transforms.push_back(T * toolTransform_);

    return transforms;
}

inline std::vector<Vector3d> ForwardKinematics::computeJointPositions(const JointAngles& jointAngles) const {
    auto transforms = computeAllTransforms(jointAngles);

    std::vector<Vector3d> positions;
    positions.reserve(transforms.size() + 1);

    // Base position
    positions.push_back(config_.baseOffset);

    // Joint positions
    for (const auto& T : transforms) {
        positions.push_back(T.block<3, 1>(0, 3));
    }

    return positions;
}

} // namespace kinematics
} // namespace robot_controller
