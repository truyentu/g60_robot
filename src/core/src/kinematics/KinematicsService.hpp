/**
 * @file KinematicsService.hpp
 * @brief Thread-safe kinematics service interface
 */

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "InverseKinematics.hpp"
#include "UrdfForwardKinematics.hpp"
#include "KDLKinematics.hpp"
#include <Eigen/Geometry>
#include <memory>
#include <mutex>
#include <array>

namespace robot_controller {
namespace kinematics {

/**
 * Thread-safe Kinematics Service Interface
 * Provides FK/IK computations for robot controller
 */
class IKinematicsService {
public:
    virtual ~IKinematicsService() = default;

    // ========================================================================
    // Forward Kinematics
    // ========================================================================

    virtual TCPPose computeFK(const JointAngles& jointAngles) = 0;
    virtual Matrix4d calculateFK(const JointAngles& jointAngles) = 0;
    virtual std::vector<Vector3d> computeJointPositions(const JointAngles& jointAngles) = 0;

    // ========================================================================
    // Inverse Kinematics
    // ========================================================================

    virtual std::optional<IKSolution> computeIK(
        const TCPPose& targetPose,
        const JointAngles& currentAngles) = 0;

    virtual IKSolutions computeAllIK(const TCPPose& targetPose) = 0;

    // ========================================================================
    // Jacobian & Singularity
    // ========================================================================

    virtual Jacobian computeJacobian(const JointAngles& jointAngles) = 0;
    virtual bool isNearSingularity(const JointAngles& jointAngles) = 0;
    virtual double computeManipulability(const JointAngles& jointAngles) = 0;

    // ========================================================================
    // Configuration
    // ========================================================================

    virtual void setRobotConfig(const RobotKinematicConfig& config) = 0;
    virtual void setToolOffset(const Vector3d& offset, const Matrix3d& rotation) = 0;
    virtual void setToolOffset(const std::array<double, 6>& tcpOffset) = 0;
    virtual void setIKConfig(const IKConfig& config) = 0;

    virtual const RobotKinematicConfig& getRobotConfig() const = 0;

    // ========================================================================
    // Validation
    // ========================================================================

    virtual bool isReachable(const TCPPose& pose) = 0;
    virtual bool areJointAnglesValid(const JointAngles& angles) = 0;
};

/**
 * Default implementation of KinematicsService
 */
class KinematicsService : public IKinematicsService {
public:
    explicit KinematicsService(const RobotKinematicConfig& config);
    ~KinematicsService() override = default;

    // FK
    TCPPose computeFK(const JointAngles& jointAngles) override;
    Matrix4d calculateFK(const JointAngles& jointAngles) override;
    std::vector<Vector3d> computeJointPositions(const JointAngles& jointAngles) override;

    // IK
    std::optional<IKSolution> computeIK(
        const TCPPose& targetPose,
        const JointAngles& currentAngles) override;
    IKSolutions computeAllIK(const TCPPose& targetPose) override;

    // Jacobian
    Jacobian computeJacobian(const JointAngles& jointAngles) override;
    bool isNearSingularity(const JointAngles& jointAngles) override;
    double computeManipulability(const JointAngles& jointAngles) override;

    // Configuration
    void setRobotConfig(const RobotKinematicConfig& config) override;
    void setToolOffset(const Vector3d& offset, const Matrix3d& rotation) override;
    void setToolOffset(const std::array<double, 6>& tcpOffset) override;
    void setIKConfig(const IKConfig& config) override;
    const RobotKinematicConfig& getRobotConfig() const override;

    // Validation
    bool isReachable(const TCPPose& pose) override;
    bool areJointAnglesValid(const JointAngles& angles) override;

    // URDF kinematics initialization (Phase 10)
    void initializeUrdfKinematics(const std::vector<UrdfJointDef>& joints, const Vector3d& toolOffset);
    bool hasUrdfFK() const { return urdfFk_ != nullptr; }
    bool hasKDLIK() const { return kdlKin_ != nullptr && kdlKin_->isInitialized(); }

    // True TCP support (Phase 11)
    TCPPose computeFlangePose(const JointAngles& jointAngles);
    TCPPose computeTcpPose(const JointAngles& jointAngles);
    std::optional<IKSolution> computeTcpIK(const TCPPose& tcpTarget, const JointAngles& currentAngles);
    Eigen::Matrix4d getToolTransform() const;
    Eigen::Matrix4d getToolInverseTransform() const;
    bool hasToolOffset() const;

private:
    mutable std::mutex mutex_;
    RobotKinematicConfig config_;
    ForwardKinematics fk_;               // DH FK (deprecated, kept as fallback)
    std::unique_ptr<UrdfForwardKinematics> urdfFk_;  // URDF FK (primary)
    std::unique_ptr<KDLKinematics> kdlKin_;          // KDL IK (primary)
    InverseKinematics ik_;               // DH IK (deprecated fallback)

    // Tool transform (Phase 11)
    Eigen::Matrix4d toolTransform_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d toolInvTransform_ = Eigen::Matrix4d::Identity();
    bool hasToolOffset_ = false;
};

// ============================================================================
// KinematicsService Implementation
// ============================================================================

inline KinematicsService::KinematicsService(const RobotKinematicConfig& config)
    : config_(config), fk_(config), ik_(config) {
}

inline TCPPose KinematicsService::computeFK(const JointAngles& jointAngles) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (urdfFk_) {
        return urdfFk_->compute(jointAngles);
    }
    return fk_.compute(jointAngles);  // DH fallback
}

inline Matrix4d KinematicsService::calculateFK(const JointAngles& jointAngles) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (urdfFk_) {
        return urdfFk_->compute(jointAngles).toTransform();
    }
    return fk_.compute(jointAngles).toTransform();  // DH fallback
}

inline std::vector<Vector3d> KinematicsService::computeJointPositions(const JointAngles& jointAngles) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (urdfFk_) {
        return urdfFk_->computeJointPositions(jointAngles);
    }
    return fk_.computeJointPositions(jointAngles);  // DH fallback
}

inline std::optional<IKSolution> KinematicsService::computeIK(
    const TCPPose& targetPose,
    const JointAngles& currentAngles) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (kdlKin_ && kdlKin_->isInitialized()) {
        auto result = kdlKin_->computeIK(targetPose, currentAngles);
        if (result.has_value()) return result;
    }
    return ik_.compute(targetPose, currentAngles);  // DH fallback
}

inline IKSolutions KinematicsService::computeAllIK(const TCPPose& targetPose) {
    std::lock_guard<std::mutex> lock(mutex_);
    return ik_.computeAll(targetPose);
}

inline Jacobian KinematicsService::computeJacobian(const JointAngles& jointAngles) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (kdlKin_ && kdlKin_->isInitialized()) {
        return kdlKin_->computeJacobian(jointAngles);
    }
    if (urdfFk_) {
        return urdfFk_->computeJacobian(jointAngles);
    }
    return ik_.computeJacobian(jointAngles);  // DH fallback
}

inline bool KinematicsService::isNearSingularity(const JointAngles& jointAngles) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (kdlKin_ && kdlKin_->isInitialized()) {
        return kdlKin_->isNearSingularity(jointAngles);
    }
    return ik_.isNearSingularity(jointAngles);  // DH fallback
}

inline double KinematicsService::computeManipulability(const JointAngles& jointAngles) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (kdlKin_ && kdlKin_->isInitialized()) {
        return kdlKin_->computeManipulability(jointAngles);
    }
    return ik_.computeManipulability(jointAngles);  // DH fallback
}

inline void KinematicsService::setRobotConfig(const RobotKinematicConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;
    fk_.setConfig(config);
    ik_.setConfig(config);
}

inline void KinematicsService::setToolOffset(const Vector3d& offset, const Matrix3d& rotation) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_.toolOffset = offset;
    config_.toolRotation = rotation;
    fk_.setToolOffset(offset, rotation);
    ik_.setConfig(config_);

    // Cache T_tool matrices for True TCP (Phase 11)
    toolTransform_ = Eigen::Matrix4d::Identity();
    toolTransform_.block<3,3>(0,0) = rotation;
    toolTransform_.block<3,1>(0,3) = Eigen::Vector3d(offset.x(), offset.y(), offset.z());

    // Efficient inverse: T⁻¹ = [R' | -R'*t]
    toolInvTransform_ = Eigen::Matrix4d::Identity();
    toolInvTransform_.block<3,3>(0,0) = rotation.transpose();
    toolInvTransform_.block<3,1>(0,3) = -rotation.transpose() * Eigen::Vector3d(offset.x(), offset.y(), offset.z());

    hasToolOffset_ = (offset.norm() > 1e-9 || !rotation.isIdentity(1e-9));
}

inline void KinematicsService::setToolOffset(const std::array<double, 6>& tcpOffset) {
    Vector3d offset = {tcpOffset[0], tcpOffset[1], tcpOffset[2]};

    // Convert Euler angles (degrees) to rotation matrix
    double rx = tcpOffset[3] * M_PI / 180.0;
    double ry = tcpOffset[4] * M_PI / 180.0;
    double rz = tcpOffset[5] * M_PI / 180.0;

    Matrix3d rotation;
    rotation = Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ())
             * Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX());

    setToolOffset(offset, rotation);
}

inline void KinematicsService::setIKConfig(const IKConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    ik_.setIKConfig(config);
}

inline const RobotKinematicConfig& KinematicsService::getRobotConfig() const {
    return config_;
}

inline bool KinematicsService::isReachable(const TCPPose& pose) {
    JointAngles zeroAngles = {0, 0, 0, 0, 0, 0};
    auto solution = computeIK(pose, zeroAngles);
    return solution.has_value();
}

inline bool KinematicsService::areJointAnglesValid(const JointAngles& angles) {
    std::lock_guard<std::mutex> lock(mutex_);
    // Use URDF joints limits if available
    if (urdfFk_) {
        // Validation still uses DH params for now (same limits stored in both)
    }
    for (size_t i = 0; i < config_.dhParams.size(); ++i) {
        const auto& dh = config_.dhParams[i];
        if (angles[i] < dh.minAngle || angles[i] > dh.maxAngle) {
            return false;
        }
    }
    return true;
}

inline void KinematicsService::initializeUrdfKinematics(
    const std::vector<UrdfJointDef>& joints, const Vector3d& toolOffset) {
    std::lock_guard<std::mutex> lock(mutex_);
    urdfFk_ = std::make_unique<UrdfForwardKinematics>(joints, toolOffset);
    kdlKin_ = std::make_unique<KDLKinematics>();
    kdlKin_->buildFromUrdfJoints(joints, toolOffset);
}

// ============================================================================
// True TCP Support (Phase 11)
// ============================================================================

inline TCPPose KinematicsService::computeFlangePose(const JointAngles& jointAngles) {
    // Returns FLANGE pose (FK chain output, no ToolData TCP applied)
    // KDL/URDF FK chain already returns flange pose (flangeOffset is robot geometry, not tool)
    std::lock_guard<std::mutex> lock(mutex_);
    if (urdfFk_) {
        return urdfFk_->compute(jointAngles);
    }
    return fk_.compute(jointAngles);
}

inline TCPPose KinematicsService::computeTcpPose(const JointAngles& jointAngles) {
    // Returns TRUE TCP pose = T_flange × T_tool
    std::lock_guard<std::mutex> lock(mutex_);

    TCPPose flangePose;
    if (urdfFk_) {
        flangePose = urdfFk_->compute(jointAngles);
    } else {
        flangePose = fk_.compute(jointAngles);
    }

    if (!hasToolOffset_) {
        return flangePose;
    }

    // T_tcp = T_flange × T_tool
    Eigen::Matrix4d T_flange = flangePose.toTransform();
    Eigen::Matrix4d T_tcp = T_flange * toolTransform_;

    return TCPPose::fromTransform(T_tcp);
}

inline std::optional<IKSolution> KinematicsService::computeTcpIK(
    const TCPPose& tcpTarget, const JointAngles& currentAngles) {
    // Back-calculation: T_flange = T_tcp_desired × T_tool⁻¹
    std::lock_guard<std::mutex> lock(mutex_);

    TCPPose flangeTarget = tcpTarget;
    if (hasToolOffset_) {
        Eigen::Matrix4d T_tcp_desired = tcpTarget.toTransform();
        Eigen::Matrix4d T_flange_desired = T_tcp_desired * toolInvTransform_;
        flangeTarget = TCPPose::fromTransform(T_flange_desired);
    }

    // Solve IK for flange pose
    if (kdlKin_ && kdlKin_->isInitialized()) {
        auto result = kdlKin_->computeIK(flangeTarget, currentAngles);
        if (result.has_value()) return result;
    }
    return ik_.compute(flangeTarget, currentAngles);
}

inline Eigen::Matrix4d KinematicsService::getToolTransform() const {
    return toolTransform_;
}

inline Eigen::Matrix4d KinematicsService::getToolInverseTransform() const {
    return toolInvTransform_;
}

inline bool KinematicsService::hasToolOffset() const {
    return hasToolOffset_;
}

} // namespace kinematics
} // namespace robot_controller
