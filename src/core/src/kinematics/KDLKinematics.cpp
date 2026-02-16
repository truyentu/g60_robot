/**
 * @file KDLKinematics.cpp
 * @brief KDL-based Kinematics implementation
 *
 * Part of Phase 10: Kinematics Overhaul (IMPL_P10_02)
 */

#define _USE_MATH_DEFINES
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "KDLKinematics.hpp"
#include "../logging/Logger.hpp"
#include <kdl/frames.hpp>
#include <cassert>
#include <algorithm>

namespace robot_controller {
namespace kinematics {

// ============================================================================
// Build KDL Chain from URDF Joint Definitions
// ============================================================================

void KDLKinematics::buildFromUrdfJoints(
    const std::vector<UrdfJointDef>& joints,
    const Vector3d& toolOffset)
{
    chain_ = KDL::Chain();

    // Build chain matching URDF convention:
    // URDF: T_joint = Translation(xyz) * RPY(rpy) * Rotation(axis, q)
    //
    // KDL Segment: pose(q) = joint.pose(q) * f_tip_stored
    //   where f_tip_stored = joint.pose(0).Inverse() * f_tip_arg
    //
    // For Joint::RotAxis with origin=Vector::Zero():
    //   joint.pose(q) = Frame(Rot2(axis, q), Vector::Zero()) = pure rotation
    //   joint.pose(0) = Identity
    //   f_tip_stored = f_tip_arg
    //   segment.pose(q) = Rot(axis, q) * f_tip_arg
    //
    // URDF wants: f_tip * Rot(axis, q) = Translation(xyz) * RPY(rpy) * Rot(axis, q)
    // KDL gives:  Rot(axis, q) * f_tip
    //
    // These are DIFFERENT orderings. To make KDL match URDF, we use the
    // "shifted origin" approach:
    //   - Segment i gets f_tip = URDF origin of joint i+1
    //   - The first URDF origin goes into a leading fixed segment
    //
    // Chain: FixedSeg(F_origin_1) * Seg1(Rot1, F_origin_2) * Seg2(Rot2, F_origin_3) * ...
    // FK = F_origin_1 * Rot1 * F_origin_2 * Rot2 * ... * RotN * F_tool
    // = URDF convention!

    for (size_t i = 0; i < joints.size(); ++i) {
        const auto& j = joints[i];

        // URDF origin frame for this joint
        KDL::Frame f_origin(
            KDL::Rotation::RPY(j.originRpy.x(), j.originRpy.y(), j.originRpy.z()),
            KDL::Vector(j.originXyz.x(), j.originXyz.y(), j.originXyz.z())
        );

        if (i == 0) {
            // First joint: add fixed segment for the origin transform
            chain_.addSegment(KDL::Segment(
                j.name + "_origin",
                KDL::Joint(j.name + "_origin_fixed", KDL::Joint::Fixed),
                f_origin
            ));
        }

        // Determine f_tip for this segment: URDF origin of NEXT joint (or tool)
        KDL::Frame f_next;
        if (i + 1 < joints.size()) {
            const auto& jnext = joints[i + 1];
            f_next = KDL::Frame(
                KDL::Rotation::RPY(jnext.originRpy.x(), jnext.originRpy.y(), jnext.originRpy.z()),
                KDL::Vector(jnext.originXyz.x(), jnext.originXyz.y(), jnext.originXyz.z())
            );
        } else {
            // Last joint: f_tip = tool offset
            f_next = KDL::Frame(KDL::Rotation::Identity(),
                                KDL::Vector(toolOffset.x(), toolOffset.y(), toolOffset.z()));
        }

        // Create KDL Joint: RotAxis with origin=Zero, axis from URDF
        KDL::Joint kdl_joint;
        if (j.isRevolute) {
            kdl_joint = KDL::Joint(
                j.name,
                KDL::Vector::Zero(),
                KDL::Vector(j.axis.x(), j.axis.y(), j.axis.z()),
                KDL::Joint::RotAxis
            );
        } else {
            kdl_joint = KDL::Joint(j.name, KDL::Joint::Fixed);
        }

        // Segment: joint rotation THEN f_next (shift to next origin)
        chain_.addSegment(KDL::Segment(
            j.name + "_seg",
            kdl_joint,
            f_next
        ));
    }

    // Create solvers
    fkSolver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);

    // LMA solver with weights adjusted for mm-scale
    // L weights control the relative importance of position vs orientation.
    // Position: 1mm error → 1.0 weighted error
    // Rotation: 1rad (~57°) error → weight * 1.0 weighted error
    //
    // Old weights [1,1,1,0.01,0.01,0.01] caused orientation drift during
    // Cartesian jogging because rotation was 100x less important than position.
    // Fix: equal weights so solver preserves both position AND orientation.
    Eigen::Matrix<double, 6, 1> L;
    L << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    ikSolver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(
        chain_, L,
        1e-5,   // eps (tight tolerance for position+orientation)
        500,    // max iterations
        1e-15   // eps_joints
    );

    jacSolver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);

    initialized_ = true;

    LOG_INFO("KDLKinematics: Chain built with {} segments ({} joints), tool offset: ({}, {}, {})",
        chain_.getNrOfSegments(), chain_.getNrOfJoints(),
        toolOffset.x(), toolOffset.y(), toolOffset.z());
}

// ============================================================================
// FK (for cross-validation)
// ============================================================================

TCPPose KDLKinematics::computeFK(const JointAngles& q) const {
    if (!initialized_ || !fkSolver_) {
        return TCPPose{};
    }

    unsigned int nj = chain_.getNrOfJoints();
    KDL::JntArray q_kdl(nj);
    for (unsigned int i = 0; i < nj && i < NUM_JOINTS; ++i) {
        q_kdl(i) = q[i];
    }

    KDL::Frame result;
    int ret = fkSolver_->JntToCart(q_kdl, result);
    if (ret < 0) {
        LOG_WARN("KDLKinematics: FK failed with error {}", ret);
        return TCPPose{};
    }

    return kdlFrameToTcpPose(result);
}

// ============================================================================
// IK
// ============================================================================

std::optional<IKSolution> KDLKinematics::computeIK(
    const TCPPose& target, const JointAngles& seed) const
{
    if (!initialized_ || !ikSolver_) return std::nullopt;

    unsigned int nj = chain_.getNrOfJoints();
    KDL::JntArray q_seed(nj), q_result(nj);

    for (unsigned int i = 0; i < nj && i < NUM_JOINTS; ++i) {
        q_seed(i) = seed[i];
    }

    KDL::Frame target_frame = poseToKdlFrame(target);

    int ret = ikSolver_->CartToJnt(q_seed, target_frame, q_result);

    if (ret >= 0 || ret == KDL::ChainIkSolverPos_LMA::E_GRADIENT_JOINTS_TOO_SMALL
                 || ret == KDL::ChainIkSolverPos_LMA::E_INCREMENT_JOINTS_TOO_SMALL) {
        // Even gradient/increment too small can be valid if close enough
        IKSolution sol;
        for (int i = 0; i < NUM_JOINTS; ++i) {
            sol.angles[i] = (i < static_cast<int>(nj)) ? q_result(i) : 0.0;
        }
        sol.isValid = true;
        sol.iterations = ikSolver_->lastNrOfIter;

        // Normalize result angles to be closest to seed (avoid 2π wrapping)
        for (int i = 0; i < NUM_JOINTS; ++i) {
            double diff = sol.angles[i] - seed[i];
            while (diff > M_PI) { sol.angles[i] -= 2 * M_PI; diff -= 2 * M_PI; }
            while (diff < -M_PI) { sol.angles[i] += 2 * M_PI; diff += 2 * M_PI; }
        }

        // Clamp to joint limits if set
        if (!jointLimits_.empty()) {
            for (size_t i = 0; i < jointLimits_.size() && i < NUM_JOINTS; ++i) {
                if (sol.angles[i] < jointLimits_[i].first ||
                    sol.angles[i] > jointLimits_[i].second) {
                    // Out of limits - try to normalize angle
                    double a = sol.angles[i];
                    while (a > M_PI) a -= 2 * M_PI;
                    while (a < -M_PI) a += 2 * M_PI;
                    if (a >= jointLimits_[i].first && a <= jointLimits_[i].second) {
                        sol.angles[i] = a;
                    } else {
                        sol.isValid = false;
                        break;
                    }
                }
            }
        }

        if (sol.isValid) {
            // Verify result with KDL FK — check BOTH position AND orientation
            auto verifyPose = computeFK(sol.angles);
            sol.residualError = (verifyPose.position - target.position).norm();

            // Check position residual
            if (sol.residualError > 1.0) {  // > 1mm position error
                LOG_WARN("KDLKinematics: IK position residual too large: {:.3f}mm", sol.residualError);
                sol.isValid = false;
            }

            // Check orientation residual
            if (sol.isValid) {
                Matrix3d R_err = verifyPose.rotation.transpose() * target.rotation;
                double oriErr = std::acos(std::clamp((R_err.trace() - 1.0) / 2.0, -1.0, 1.0));
                if (oriErr > 0.01) {  // > 0.01 rad (~0.57°)
                    LOG_WARN("KDLKinematics: IK orientation residual too large: {:.4f}rad ({:.2f}deg)",
                             oriErr, oriErr * 180.0 / M_PI);
                    sol.isValid = false;
                }
            }
        }

        if (sol.isValid) {
            return sol;
        }
    }

    LOG_DEBUG("KDLKinematics: IK failed with ret={}, lastDiff={:.6f}, lastTransDiff={:.6f}mm, lastRotDiff={:.6f}rad",
        ret, ikSolver_->lastDifference, ikSolver_->lastTransDiff, ikSolver_->lastRotDiff);

    return std::nullopt;
}

// ============================================================================
// Jacobian
// ============================================================================

Jacobian KDLKinematics::computeJacobian(const JointAngles& q) const {
    Jacobian result = Jacobian::Zero();
    if (!initialized_ || !jacSolver_) return result;

    unsigned int nj = chain_.getNrOfJoints();
    KDL::JntArray q_kdl(nj);
    for (unsigned int i = 0; i < nj && i < NUM_JOINTS; ++i) {
        q_kdl(i) = q[i];
    }

    KDL::Jacobian jac_kdl(nj);
    int ret = jacSolver_->JntToJac(q_kdl, jac_kdl);
    if (ret < 0) {
        LOG_WARN("KDLKinematics: Jacobian computation failed with error {}", ret);
        return result;
    }

    // Copy KDL Jacobian to Eigen matrix
    for (unsigned int i = 0; i < 6; ++i) {
        for (unsigned int j = 0; j < nj && j < NUM_JOINTS; ++j) {
            result(i, j) = jac_kdl(i, j);
        }
    }

    return result;
}

// ============================================================================
// Singularity
// ============================================================================

bool KDLKinematics::isNearSingularity(const JointAngles& q, double threshold) const {
    Jacobian J = computeJacobian(q);

    Eigen::JacobiSVD<Jacobian> svd(J);
    auto sv = svd.singularValues();

    double minSV = sv(sv.size() - 1);
    double maxSV = sv(0);

    if (maxSV < 1e-10) return true;
    return (minSV / maxSV) < threshold;
}

double KDLKinematics::computeManipulability(const JointAngles& q) const {
    Jacobian J = computeJacobian(q);
    Matrix6d JJT = J * J.transpose();
    return std::sqrt(std::abs(JJT.determinant()));
}

// ============================================================================
// Configuration
// ============================================================================

void KDLKinematics::setJointLimits(const std::vector<std::pair<double,double>>& limits) {
    jointLimits_ = limits;
}

// ============================================================================
// Conversion Helpers
// ============================================================================

KDL::Frame KDLKinematics::poseToKdlFrame(const TCPPose& pose) {
    KDL::Rotation R(
        pose.rotation(0,0), pose.rotation(0,1), pose.rotation(0,2),
        pose.rotation(1,0), pose.rotation(1,1), pose.rotation(1,2),
        pose.rotation(2,0), pose.rotation(2,1), pose.rotation(2,2)
    );
    KDL::Vector p(pose.position.x(), pose.position.y(), pose.position.z());
    return KDL::Frame(R, p);
}

TCPPose KDLKinematics::kdlFrameToTcpPose(const KDL::Frame& frame) {
    TCPPose pose;

    // Position
    pose.position = Vector3d(frame.p.x(), frame.p.y(), frame.p.z());

    // Rotation matrix
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            pose.rotation(r, c) = frame.M(r, c);
        }
    }

    // RPY from KDL rotation
    double roll, pitch, yaw;
    frame.M.GetRPY(roll, pitch, yaw);
    pose.rpy = Vector3d(roll, pitch, yaw);

    // Quaternion
    double x, y, z, w;
    frame.M.GetQuaternion(x, y, z, w);
    pose.quaternion = Eigen::Quaterniond(w, x, y, z);

    return pose;
}

} // namespace kinematics
} // namespace robot_controller
