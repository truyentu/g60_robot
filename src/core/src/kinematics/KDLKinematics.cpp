/**
 * @file KDLKinematics.cpp
 * @brief KDL-based Kinematics implementation
 *
 * Part of Phase 10: Kinematics Overhaul (IMPL_P10_02)
 */

#include <cmath>
#include <fstream>

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
    const TCPPose& target, const JointAngles& seed, bool skipConfigFlipCheck) const
{
    // Debug file logging
    static std::ofstream dbg;
    static bool dbgOpened = false;
    if (!dbgOpened) {
        dbg.open("ik_debug.txt", std::ios::trunc);
        dbgOpened = true;
    }

    if (!initialized_ || !ikSolver_) {
        dbg << "[FAIL] not initialized\n"; dbg.flush();
        return std::nullopt;
    }

    unsigned int nj = chain_.getNrOfJoints();
    KDL::JntArray q_seed(nj), q_result(nj);

    for (unsigned int i = 0; i < nj && i < NUM_JOINTS; ++i) {
        q_seed(i) = seed[i];
    }

    KDL::Frame target_frame = poseToKdlFrame(target);

    dbg << "=== computeIK call === skipFlip=" << skipConfigFlipCheck << "\n";
    dbg << "  target pos=[" << target.position[0] << "," << target.position[1] << "," << target.position[2] << "]\n";
    dbg << "  seed deg=[";
    for (int i=0;i<6;i++) dbg << seed[i]*180/M_PI << (i<5?",":"");
    dbg << "]\n";

    int ret = ikSolver_->CartToJnt(q_seed, target_frame, q_result);

    dbg << "  KDL ret=" << ret << " lastTransDiff=" << ikSolver_->lastTransDiff
        << " lastRotDiff=" << ikSolver_->lastRotDiff
        << " iterations=" << ikSolver_->lastNrOfIter << "\n";
    dbg << "  q_result deg=[";
    for (unsigned int i=0;i<nj;i++) dbg << q_result(i)*180/M_PI << (i<nj-1?",":"");
    dbg << "]\n";

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

        dbg << "  normalized deg=[";
        for (int i=0;i<6;i++) dbg << sol.angles[i]*180/M_PI << (i<5?",":"");
        dbg << "]\n";

        // Clamp to joint limits if set
        if (!jointLimits_.empty()) {
            dbg << "  jointLimits count=" << jointLimits_.size() << "\n";
            for (size_t i = 0; i < jointLimits_.size() && i < NUM_JOINTS; ++i) {
                dbg << "    J" << i+1 << ": val=" << sol.angles[i]*180/M_PI
                    << " limits=[" << jointLimits_[i].first*180/M_PI
                    << "," << jointLimits_[i].second*180/M_PI << "]";
                if (sol.angles[i] < jointLimits_[i].first ||
                    sol.angles[i] > jointLimits_[i].second) {
                    double a = sol.angles[i];
                    while (a > M_PI) a -= 2 * M_PI;
                    while (a < -M_PI) a += 2 * M_PI;
                    if (a >= jointLimits_[i].first && a <= jointLimits_[i].second) {
                        sol.angles[i] = a;
                        dbg << " -> normalized to " << a*180/M_PI;
                    } else {
                        dbg << " -> OUT OF LIMITS (normalized=" << a*180/M_PI << ")";
                        LOG_WARN("KDL IK: Joint {} out of limits: {:.1f}deg (limits [{:.1f}, {:.1f}]deg)",
                            i+1, a * 180.0 / M_PI,
                            jointLimits_[i].first * 180.0 / M_PI,
                            jointLimits_[i].second * 180.0 / M_PI);
                        sol.isValid = false;
                        dbg << "\n"; dbg.flush();
                        break;
                    }
                } else {
                    dbg << " OK";
                }
                dbg << "\n";
            }
        } else {
            dbg << "  no joint limits set\n";
        }

        if (sol.isValid) {
            // Verify result with KDL FK — check BOTH position AND orientation
            auto verifyPose = computeFK(sol.angles);
            sol.residualError = (verifyPose.position - target.position).norm();

            dbg << "  FK verify pos=[" << verifyPose.position[0] << ","
                << verifyPose.position[1] << "," << verifyPose.position[2]
                << "] target pos=[" << target.position[0] << ","
                << target.position[1] << "," << target.position[2]
                << "] posErr=" << sol.residualError << "mm\n";

            // Check position residual
            if (sol.residualError > 1.0) {  // > 1mm position error
                dbg << "  -> REJECTED: position residual " << sol.residualError << "mm > 1.0mm\n";
                LOG_WARN("KDLKinematics: IK position residual too large: {:.3f}mm", sol.residualError);
                sol.isValid = false;
            }

            // Check orientation residual
            if (sol.isValid) {
                Matrix3d R_err = verifyPose.rotation.transpose() * target.rotation;
                double oriErr = std::acos(std::clamp((R_err.trace() - 1.0) / 2.0, -1.0, 1.0));
                dbg << "  oriErr=" << oriErr*180/M_PI << "deg\n";
                if (oriErr > 0.01) {  // > 0.01 rad (~0.57°)
                    dbg << "  -> REJECTED: orientation residual " << oriErr*180/M_PI << "deg > 0.57deg\n";
                    LOG_WARN("KDLKinematics: IK orientation residual too large: {:.4f}rad ({:.2f}deg) skipFlip={}",
                             oriErr, oriErr * 180.0 / M_PI, skipConfigFlipCheck);
                    sol.isValid = false;
                }
            }
        } else {
            dbg << "  sol.isValid=false after joint limits check\n";
        }

        if (sol.isValid && !skipConfigFlipCheck) {
            // Reject configuration flips on structural joints (J1-J3 only).
            // J4-J6 (wrist joints) can legitimately change significantly during
            // CIRC arcs and Cartesian motions — only reject extreme flips (>90°).
            static constexpr double MAX_STRUCT_JUMP_RAD = 0.52;  // ~30 degrees for J1-J3
            static constexpr double MAX_WRIST_JUMP_RAD = 1.57;   // ~90 degrees for J4-J6

            double maxStructJump = 0;
            int structJumpIdx = -1;
            for (int i = 0; i < 3; ++i) {  // Only J1-J3 (structural)
                double jump = std::abs(sol.angles[i] - seed[i]);
                if (jump > maxStructJump) { maxStructJump = jump; structJumpIdx = i; }
            }

            double maxWristJump = 0;
            int wristJumpIdx = -1;
            for (int i = 3; i < NUM_JOINTS; ++i) {  // J4-J6 (wrist)
                double jump = std::abs(sol.angles[i] - seed[i]);
                if (jump > maxWristJump) { maxWristJump = jump; wristJumpIdx = i; }
            }

            if (maxStructJump > MAX_STRUCT_JUMP_RAD) {
                LOG_WARN("KDL IK: Rejected config flip — J{} jump {:.1f}deg (max {:.0f}deg). "
                    "seed=[{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f}] "
                    "result=[{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f}]",
                    structJumpIdx+1, maxStructJump * 180.0 / M_PI, MAX_STRUCT_JUMP_RAD * 180.0 / M_PI,
                    seed[0]*180/M_PI, seed[1]*180/M_PI, seed[2]*180/M_PI,
                    seed[3]*180/M_PI, seed[4]*180/M_PI, seed[5]*180/M_PI,
                    sol.angles[0]*180/M_PI, sol.angles[1]*180/M_PI, sol.angles[2]*180/M_PI,
                    sol.angles[3]*180/M_PI, sol.angles[4]*180/M_PI, sol.angles[5]*180/M_PI);
                return std::nullopt;
            }

            if (maxWristJump > MAX_WRIST_JUMP_RAD) {
                LOG_WARN("KDL IK: Rejected wrist flip — J{} jump {:.1f}deg (max {:.0f}deg). "
                    "seed=[{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f}] "
                    "result=[{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f}]",
                    wristJumpIdx+1, maxWristJump * 180.0 / M_PI, MAX_WRIST_JUMP_RAD * 180.0 / M_PI,
                    seed[0]*180/M_PI, seed[1]*180/M_PI, seed[2]*180/M_PI,
                    seed[3]*180/M_PI, seed[4]*180/M_PI, seed[5]*180/M_PI,
                    sol.angles[0]*180/M_PI, sol.angles[1]*180/M_PI, sol.angles[2]*180/M_PI,
                    sol.angles[3]*180/M_PI, sol.angles[4]*180/M_PI, sol.angles[5]*180/M_PI);
                return std::nullopt;
            }

            // Wrist singularity guard: when J5 is near 0, J4 and J6 axes align
            // and the IK solver can distribute rotation arbitrarily between them.
            // Only clamp the "spin" component (J4-J6 coupling) — preserve the
            // "useful" component (J4+J6 sum) that affects TCP pose.
            static constexpr double WRIST_SPIN_MAX_RAD = 0.10;   // ~5.7 deg max spin drift per step
            bool nearWristSing = (std::abs(sol.angles[4]) < 0.18);  // ~10 degrees
            if (nearWristSing) {
                double dJ4 = sol.angles[3] - seed[3];
                double dJ6 = sol.angles[5] - seed[5];

                // Decompose into sum (affects TCP) and diff (wrist spin, redundant near singularity)
                // When J5≈0: Rz(J4)·Ry(0)·Rz(J6) ≈ Rz(J4+J6) → only sum matters
                double dSum  = dJ4 + dJ6;  // "useful" rotation
                double dDiff = dJ4 - dJ6;  // "spin" redundancy

                // Only clamp the spin component if it's large
                if (std::abs(dDiff) > 2.0 * WRIST_SPIN_MAX_RAD) {
                    // Redistribute: keep sum the same, reduce diff
                    double clampedDiff = (dDiff > 0 ? 1.0 : -1.0) * WRIST_SPIN_MAX_RAD;
                    double newJ4 = seed[3] + (dSum + clampedDiff) / 2.0;
                    double newJ6 = seed[5] + (dSum - clampedDiff) / 2.0;
                    sol.angles[3] = newJ4;
                    sol.angles[5] = newJ6;
                    LOG_DEBUG("KDL IK: Wrist spin clamp (J5={:.1f}deg) — "
                        "dDiff {:.1f}->{:.1f}deg, sum preserved {:.1f}deg",
                        sol.angles[4] * 180.0 / M_PI,
                        dDiff * 180.0 / M_PI, clampedDiff * 180.0 / M_PI,
                        dSum * 180.0 / M_PI);
                }
            }

            // Diagnostic: warn if any joint jumped significantly (after clamping)
            double maxJumpPost = 0;
            int jumpIdxPost = -1;
            for (int i = 0; i < NUM_JOINTS; ++i) {
                double jump = std::abs(sol.angles[i] - seed[i]);
                if (jump > maxJumpPost) { maxJumpPost = jump; jumpIdxPost = i; }
            }
            if (maxJumpPost > 0.35) {  // > ~20 degrees — warn but accept
                LOG_WARN("KDL IK: J{} jump {:.1f}deg seed=[{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f}] result=[{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f}]",
                    jumpIdxPost+1, maxJumpPost * 180.0 / M_PI,
                    seed[0]*180/M_PI, seed[1]*180/M_PI, seed[2]*180/M_PI,
                    seed[3]*180/M_PI, seed[4]*180/M_PI, seed[5]*180/M_PI,
                    sol.angles[0]*180/M_PI, sol.angles[1]*180/M_PI, sol.angles[2]*180/M_PI,
                    sol.angles[3]*180/M_PI, sol.angles[4]*180/M_PI, sol.angles[5]*180/M_PI);
            }
            dbg << "  -> SUCCESS (config flip check)\n"; dbg.flush();
        }

        // Return valid solution (either after config flip check or skipped)
        if (sol.isValid) {
            dbg << "  -> SUCCESS\n"; dbg.flush();
            return sol;
        }
    }

    dbg << "  -> FINAL FAIL: ret=" << ret << " (did not enter ret>=0 branch)\n";
    dbg.flush();

    LOG_WARN("KDLKinematics: IK failed — ret={}, lastTransDiff={:.2f}mm, lastRotDiff={:.3f}rad, "
        "skipFlip={}, seed=[{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f}]",
        ret, ikSolver_->lastTransDiff, ikSolver_->lastRotDiff, skipConfigFlipCheck,
        seed[0]*180/M_PI, seed[1]*180/M_PI, seed[2]*180/M_PI,
        seed[3]*180/M_PI, seed[4]*180/M_PI, seed[5]*180/M_PI);

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

Eigen::VectorXd KDLKinematics::computeSingularValues(const JointAngles& q) const {
    Jacobian J = computeJacobian(q);
    Eigen::JacobiSVD<Jacobian> svd(J);
    return svd.singularValues();
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
