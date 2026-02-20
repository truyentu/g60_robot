/**
 * @file KDLKinematics.hpp
 * @brief KDL-based Inverse Kinematics wrapper
 *
 * Uses Orocos KDL library for numerical IK (LMA solver).
 * KDL chain is built from URDF joint data (same source as UrdfForwardKinematics).
 *
 * Part of Phase 10: Kinematics Overhaul (IMPL_P10_02)
 */

#pragma once

#include "MathTypes.hpp"
#include "ForwardKinematics.hpp"  // TCPPose, IKSolution types
#include "InverseKinematics.hpp"  // IKSolution, IKSolutions, IKConfig
#include "UrdfForwardKinematics.hpp"  // UrdfJointDef
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <memory>

namespace robot_controller {
namespace kinematics {

/**
 * KDL-based Kinematics wrapper
 *
 * Provides IK using KDL LMA solver, built from URDF joint definitions.
 * FK is available for cross-validation with UrdfForwardKinematics.
 */
class KDLKinematics {
public:
    KDLKinematics() = default;
    ~KDLKinematics() = default;

    /**
     * Build KDL chain from URDF joint definitions
     * Uses same data source as UrdfForwardKinematics
     */
    void buildFromUrdfJoints(const std::vector<UrdfJointDef>& joints,
                             const Vector3d& toolOffset);

    /**
     * FK (for cross-validation with UrdfForwardKinematics)
     */
    TCPPose computeFK(const JointAngles& q) const;

    /**
     * IK - single solution closest to seed
     */
    std::optional<IKSolution> computeIK(const TCPPose& target,
                                         const JointAngles& seed,
                                         bool skipConfigFlipCheck = false) const;

    /**
     * Jacobian (6xN from KDL)
     */
    Jacobian computeJacobian(const JointAngles& q) const;

    /**
     * Singularity detection
     */
    bool isNearSingularity(const JointAngles& q, double threshold = 0.01) const;
    double computeManipulability(const JointAngles& q) const;

    /**
     * Singular values of Jacobian (for TWA PS index computation)
     * Returns sorted descending: σ₁ ≥ σ₂ ≥ ... ≥ σ₆
     */
    Eigen::VectorXd computeSingularValues(const JointAngles& q) const;

    /**
     * Set joint limits for IK solution clamping
     */
    void setJointLimits(const std::vector<std::pair<double,double>>& limits);

    bool isInitialized() const { return initialized_; }

private:
    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fkSolver_;
    mutable std::unique_ptr<KDL::ChainIkSolverPos_LMA> ikSolver_;
    std::unique_ptr<KDL::ChainJntToJacSolver> jacSolver_;

    std::vector<std::pair<double,double>> jointLimits_;
    bool initialized_ = false;

    // Eigen <-> KDL conversion helpers
    static KDL::Frame poseToKdlFrame(const TCPPose& pose);
    static TCPPose kdlFrameToTcpPose(const KDL::Frame& frame);
};

} // namespace kinematics
} // namespace robot_controller
