/**
 * @file UrdfForwardKinematics.cpp
 * @brief URDF-based Forward Kinematics implementation
 *
 * Part of Phase 10: Kinematics Overhaul (IMPL_P10_01)
 */

#include "UrdfForwardKinematics.hpp"
#include "../config/RobotPackageSchema.hpp"
#include "../logging/Logger.hpp"
#include <cassert>

namespace robot_controller {
namespace kinematics {

// ============================================================================
// Constructor
// ============================================================================

UrdfForwardKinematics::UrdfForwardKinematics(
    const std::vector<UrdfJointDef>& joints,
    const Vector3d& toolOffset)
    : joints_(joints), toolOffset_(toolOffset)
{
    LOG_INFO("UrdfForwardKinematics created with {} joints, tool offset: ({}, {}, {})",
        joints_.size(), toolOffset_.x(), toolOffset_.y(), toolOffset_.z());

    for (size_t i = 0; i < joints_.size(); ++i) {
        const auto& j = joints_[i];
        LOG_DEBUG("  Joint[{}] '{}': xyz=({},{},{}), rpy=({},{},{}), axis=({},{},{})",
            i, j.name,
            j.originXyz.x(), j.originXyz.y(), j.originXyz.z(),
            j.originRpy.x(), j.originRpy.y(), j.originRpy.z(),
            j.axis.x(), j.axis.y(), j.axis.z());
    }
}

// ============================================================================
// Single Joint Transform
// ============================================================================

Matrix4d UrdfForwardKinematics::computeJointTransform(size_t i, double angle) const {
    assert(i < joints_.size());
    const auto& j = joints_[i];

    // 1. Translation
    Matrix4d T = Matrix4d::Identity();
    T.block<3,1>(0,3) = j.originXyz;

    // 2. RPY rotation (URDF convention: R = Rz(yaw) * Ry(pitch) * Rx(roll))
    // For MA2010, all RPY = [0,0,0], so this is Identity - skip for performance
    if (j.originRpy.squaredNorm() > 1e-10) {
        Matrix3d R;
        R = AngleAxisd(j.originRpy[2], Vector3d::UnitZ())    // yaw
          * AngleAxisd(j.originRpy[1], Vector3d::UnitY())    // pitch
          * AngleAxisd(j.originRpy[0], Vector3d::UnitX());   // roll
        T.block<3,3>(0,0) = R;
    }

    // 3. Joint rotation about axis
    if (j.isRevolute && std::abs(angle) > 1e-15) {
        Matrix4d Rj = Matrix4d::Identity();
        Rj.block<3,3>(0,0) = AngleAxisd(angle, j.axis).toRotationMatrix();
        T = T * Rj;
    }

    return T;
}

// ============================================================================
// Full FK Chain
// ============================================================================

TCPPose UrdfForwardKinematics::compute(const JointAngles& jointAngles) const {
    Matrix4d T = Matrix4d::Identity(); // Base frame

    const size_t n = std::min(joints_.size(), static_cast<size_t>(NUM_JOINTS));
    for (size_t i = 0; i < n; ++i) {
        T = T * computeJointTransform(i, jointAngles[i]);
    }

    // Apply tool offset (fixed translation in tool frame)
    Matrix4d Ttool = Matrix4d::Identity();
    Ttool.block<3,1>(0,3) = toolOffset_;
    T = T * Ttool;

    return TCPPose::fromTransform(T);
}

// ============================================================================
// All Intermediate Transforms
// ============================================================================

std::vector<Matrix4d> UrdfForwardKinematics::computeAllTransforms(
    const JointAngles& jointAngles) const
{
    std::vector<Matrix4d> transforms;
    transforms.reserve(joints_.size() + 1); // +1 for tool

    Matrix4d T = Matrix4d::Identity();

    const size_t n = std::min(joints_.size(), static_cast<size_t>(NUM_JOINTS));
    for (size_t i = 0; i < n; ++i) {
        T = T * computeJointTransform(i, jointAngles[i]);
        transforms.push_back(T);
    }

    // Tool transform
    Matrix4d Ttool = Matrix4d::Identity();
    Ttool.block<3,1>(0,3) = toolOffset_;
    T = T * Ttool;
    transforms.push_back(T);

    return transforms;
}

// ============================================================================
// Joint Positions
// ============================================================================

std::vector<Vector3d> UrdfForwardKinematics::computeJointPositions(
    const JointAngles& jointAngles) const
{
    auto transforms = computeAllTransforms(jointAngles);
    std::vector<Vector3d> positions;
    positions.reserve(transforms.size());

    for (const auto& T : transforms) {
        positions.push_back(T.block<3,1>(0,3));
    }

    return positions;
}

// ============================================================================
// Numerical Jacobian (Central Differences)
// ============================================================================

Jacobian UrdfForwardKinematics::computeJacobian(const JointAngles& q) const {
    Jacobian J = Jacobian::Zero();
    constexpr double delta = 1e-6; // rad

    auto pose0 = compute(q);

    const size_t n = std::min(joints_.size(), static_cast<size_t>(NUM_JOINTS));
    for (size_t i = 0; i < n; ++i) {
        JointAngles q_plus = q;
        JointAngles q_minus = q;
        q_plus[i] += delta;
        q_minus[i] -= delta;

        auto pose_plus = compute(q_plus);
        auto pose_minus = compute(q_minus);

        // Linear velocity (position derivative)
        J.block<3,1>(0, i) = (pose_plus.position - pose_minus.position) / (2.0 * delta);

        // Angular velocity (orientation derivative via log map)
        Matrix3d R_diff = pose_minus.rotation.transpose() * pose_plus.rotation;
        AngleAxisd aa(R_diff);
        double angle = aa.angle();
        Vector3d axis_vec = aa.axis();

        // Handle near-zero rotation
        Vector3d omega;
        if (std::abs(angle) < 1e-12) {
            omega = Vector3d::Zero();
        } else {
            omega = angle * axis_vec / (2.0 * delta);
        }

        // Transform to world frame
        J.block<3,1>(3, i) = pose0.rotation * omega;
    }

    return J;
}

// ============================================================================
// Configuration
// ============================================================================

void UrdfForwardKinematics::setToolOffset(const Vector3d& offset) {
    toolOffset_ = offset;
    LOG_INFO("Tool offset updated: ({}, {}, {})",
        offset.x(), offset.y(), offset.z());
}

// ============================================================================
// Helper: Build UrdfJointDefs from RobotPackage
// ============================================================================

std::vector<UrdfJointDef> buildUrdfJointsFromPackage(const config::RobotPackage& pkg) {
    std::vector<UrdfJointDef> joints;
    joints.reserve(pkg.joints.size());

    for (const auto& jd : pkg.joints) {
        UrdfJointDef uj;
        uj.name = jd.name;
        uj.isRevolute = (jd.type == "revolute");

        // Origin XYZ (already in mm from YAML)
        if (jd.origin_xyz) {
            uj.originXyz = Vector3d(
                (*jd.origin_xyz)[0],
                (*jd.origin_xyz)[1],
                (*jd.origin_xyz)[2]
            );
        }

        // Origin RPY (already in radians from YAML)
        if (jd.origin_rpy) {
            uj.originRpy = Vector3d(
                (*jd.origin_rpy)[0],
                (*jd.origin_rpy)[1],
                (*jd.origin_rpy)[2]
            );
        }

        // Axis
        if (jd.axis) {
            uj.axis = Vector3d(
                (*jd.axis)[0],
                (*jd.axis)[1],
                (*jd.axis)[2]
            );
        } else {
            uj.axis = Vector3d::UnitZ(); // Default Z-axis
        }

        // Limits (degrees → radians)
        uj.minAngle = jd.limit_min * DEG_TO_RAD;
        uj.maxAngle = jd.limit_max * DEG_TO_RAD;

        joints.push_back(uj);

        LOG_DEBUG("buildUrdfJointsFromPackage: Joint '{}' xyz=({},{},{}) rpy=({},{},{}) axis=({},{},{})",
            uj.name,
            uj.originXyz.x(), uj.originXyz.y(), uj.originXyz.z(),
            uj.originRpy.x(), uj.originRpy.y(), uj.originRpy.z(),
            uj.axis.x(), uj.axis.y(), uj.axis.z());
    }

    LOG_INFO("buildUrdfJointsFromPackage: Built {} joints from package '{}'",
        joints.size(), pkg.name);

    return joints;
}

} // namespace kinematics
} // namespace robot_controller
