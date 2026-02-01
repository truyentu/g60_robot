/**
 * @file KinematicsService.hpp
 * @brief Thread-safe kinematics service interface
 */

#pragma once

#include "InverseKinematics.hpp"
#include <memory>
#include <mutex>

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
    void setIKConfig(const IKConfig& config) override;
    const RobotKinematicConfig& getRobotConfig() const override;

    // Validation
    bool isReachable(const TCPPose& pose) override;
    bool areJointAnglesValid(const JointAngles& angles) override;

private:
    mutable std::mutex mutex_;
    RobotKinematicConfig config_;
    ForwardKinematics fk_;
    InverseKinematics ik_;
};

// ============================================================================
// KinematicsService Implementation
// ============================================================================

inline KinematicsService::KinematicsService(const RobotKinematicConfig& config)
    : config_(config), fk_(config), ik_(config) {
}

inline TCPPose KinematicsService::computeFK(const JointAngles& jointAngles) {
    std::lock_guard<std::mutex> lock(mutex_);
    return fk_.compute(jointAngles);
}

inline std::vector<Vector3d> KinematicsService::computeJointPositions(const JointAngles& jointAngles) {
    std::lock_guard<std::mutex> lock(mutex_);
    return fk_.computeJointPositions(jointAngles);
}

inline std::optional<IKSolution> KinematicsService::computeIK(
    const TCPPose& targetPose,
    const JointAngles& currentAngles) {
    std::lock_guard<std::mutex> lock(mutex_);
    return ik_.compute(targetPose, currentAngles);
}

inline IKSolutions KinematicsService::computeAllIK(const TCPPose& targetPose) {
    std::lock_guard<std::mutex> lock(mutex_);
    return ik_.computeAll(targetPose);
}

inline Jacobian KinematicsService::computeJacobian(const JointAngles& jointAngles) {
    std::lock_guard<std::mutex> lock(mutex_);
    return ik_.computeJacobian(jointAngles);
}

inline bool KinematicsService::isNearSingularity(const JointAngles& jointAngles) {
    std::lock_guard<std::mutex> lock(mutex_);
    return ik_.isNearSingularity(jointAngles);
}

inline double KinematicsService::computeManipulability(const JointAngles& jointAngles) {
    std::lock_guard<std::mutex> lock(mutex_);
    return ik_.computeManipulability(jointAngles);
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
    for (size_t i = 0; i < config_.dhParams.size(); ++i) {
        const auto& dh = config_.dhParams[i];
        if (angles[i] < dh.minAngle || angles[i] > dh.maxAngle) {
            return false;
        }
    }
    return true;
}

} // namespace kinematics
} // namespace robot_controller
