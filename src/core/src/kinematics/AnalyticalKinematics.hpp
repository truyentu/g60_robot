/**
 * @file AnalyticalKinematics.hpp
 * @brief Analytical (closed-form) FK/IK for 6-DOF robots with spherical wrist
 *
 * Designed for Yaskawa MA2010 welding robot.
 * All functions are stateless and mutex-free (safe for 1kHz real-time loop).
 *
 * DH Convention: Standard DH
 *   T_i = Rz(theta_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)
 *
 * References:
 *   - "Determination of the Inverse Kinematic of the Industrial Robot
 *      Yaskawa-Motoman-GP7" (da Silva et al., 2022)
 *   - "FORWARD & INVERSE KINEMATICS SOLUTION OF 6-DOF ROBOTS THOSE HAVE
 *      OFFSET & SPHERICAL WRISTS" (Dikmenli, 2022)
 *   - "Kinematics Modeling and Analysis of MOTOMAN-HP20 Robot"
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <vector>
#include <optional>
#include <cmath>

namespace robot_controller {
namespace kinematics {
namespace analytical {

// ============================================================================
// Constants
// ============================================================================

constexpr int NUM_JOINTS = 6;
constexpr double PI = 3.14159265358979323846;
constexpr double DEG2RAD = PI / 180.0;
constexpr double RAD2DEG = 180.0 / PI;

// ============================================================================
// Data Structures
// ============================================================================

struct JointAngles {
    std::array<double, NUM_JOINTS> q = {0};

    double& operator[](int i) { return q[i]; }
    double  operator[](int i) const { return q[i]; }
};

struct Pose {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

    Eigen::Matrix3d rotationMatrix() const { return orientation.toRotationMatrix(); }

    Eigen::Matrix4d toTransform() const {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0,0) = rotationMatrix();
        T.block<3,1>(0,3) = position;
        return T;
    }

    static Pose fromTransform(const Eigen::Matrix4d& T) {
        Pose p;
        p.position = T.block<3,1>(0,3);
        p.orientation = Eigen::Quaterniond(T.block<3,3>(0,0));
        p.orientation.normalize();
        return p;
    }
};

struct IKSolution {
    std::vector<JointAngles> solutions;  // Up to 8 solutions
    bool hasSolution() const { return !solutions.empty(); }
    size_t count() const { return solutions.size(); }
};

struct WeldingIKResult {
    JointAngles joints;
    double psi;             // Tool Z rotation angle (rad) used
    double jointLimitCost;  // H(q) = Σ((qi - qi_mid) / qi_range)²
    double manipulability;  // Yoshikawa manipulability measure
    bool valid = false;
};

// Robot configuration flags
enum class ShoulderConfig { LEFT, RIGHT };
enum class ElbowConfig { UP, DOWN };
enum class WristConfig { FLIP, NO_FLIP };

// ============================================================================
// DH Parameters (Hard-coded for MA2010)
// ============================================================================

struct DHParam {
    double a;       // Link length (mm)
    double alpha;   // Link twist (rad)
    double d;       // Link offset (mm)
    double theta_offset;  // Joint angle offset (rad)
    double sign;    // Joint angle sign (+1 or -1)
    double q_min;   // Joint limit min (rad)
    double q_max;   // Joint limit max (rad)
};

// ============================================================================
// AnalyticalKinematics Class
// ============================================================================

/**
 * Analytical kinematics for 6-DOF robot with spherical wrist.
 *
 * ALL methods are const, stateless, and thread-safe.
 * No mutex, no dynamic allocation in hot path.
 */
class AnalyticalKinematics {
public:
    AnalyticalKinematics();

    // ========================================================================
    // Forward Kinematics
    // ========================================================================

    /**
     * Compute TCP pose from joint angles.
     * @param q Joint angles in URDF convention (radians)
     * @return TCP pose (position in mm, orientation as quaternion)
     */
    Pose computeFK(const JointAngles& q) const;

    /**
     * Compute all intermediate transforms (base→frame_i for i=1..6)
     * @param q Joint angles in URDF convention (radians)
     * @return Array of 7 transforms: T01, T02, T03, T04, T05, T06, T_tcp
     */
    std::array<Eigen::Matrix4d, 7> computeAllTransforms(const JointAngles& q) const;

    // ========================================================================
    // Inverse Kinematics (Analytical / Closed-Form)
    // ========================================================================

    /**
     * Compute ALL valid IK solutions (up to 8).
     * Solutions are filtered against joint limits.
     * NO clamping — out-of-range solutions are REJECTED.
     *
     * @param target Target TCP pose
     * @return IKSolution containing 0-8 valid solutions
     */
    IKSolution computeIK(const Pose& target) const;

    /**
     * Compute IK and return the solution closest to a reference configuration.
     * @param target Target TCP pose
     * @param q_ref Reference joint angles (typically current position)
     * @return Best solution or nullopt if unreachable
     */
    std::optional<JointAngles> computeIKNearest(
        const Pose& target, const JointAngles& q_ref) const;

    // ========================================================================
    // Welding IK — Redundancy Resolution via Tool Z Rotation
    // ========================================================================

    /**
     * Compute IK for welding application exploiting tool Z rotation redundancy.
     *
     * In arc welding, the wire is along tool Z axis, so rotating around tool Z
     * does not affect weld quality. This DOF is used to find joint configurations
     * that stay furthest from joint limits.
     *
     * Algorithm: Coarse sampling of ψ (rotation around tool Z), solve analytical
     * IK for each rotated target, select solution with lowest joint-limit cost.
     *
     * @param target Target TCP pose (position + orientation)
     * @param currentJoints Current joint angles (for nearest-solution selection)
     * @return WeldingIKResult with best solution, or invalid if unreachable
     */
    WeldingIKResult computeWeldingIK(
        const Pose& target, const JointAngles& currentJoints) const;

    /**
     * Compute joint-limit cost function.
     * H(q) = Σ ((qi - qi_mid) / qi_range)²
     * Lower is better (joints closer to center of range).
     */
    double jointLimitCost(const JointAngles& q) const;

    // ========================================================================
    // Jacobian & Singularity
    // ========================================================================

    /**
     * Compute the 6x6 analytical Jacobian matrix.
     * @param q Joint angles (URDF convention, radians)
     * @return 6x6 Jacobian [linear_velocity; angular_velocity]
     */
    Eigen::Matrix<double, 6, 6> computeJacobian(const JointAngles& q) const;

    /**
     * Check singularity condition.
     * @param q Joint angles
     * @return true if near singularity (det(J) < threshold)
     */
    bool isSingularity(const JointAngles& q) const;

    /**
     * Get singularity details.
     * @param q Joint angles
     * @return Determinant of Jacobian (0 = singular)
     */
    double manipulabilityMeasure(const JointAngles& q) const;

    // ========================================================================
    // Accessors
    // ========================================================================

    const std::array<DHParam, NUM_JOINTS>& dhParams() const { return dh_; }

    bool isWithinJointLimits(const JointAngles& q) const;

    /**
     * Normalize angle to [-pi, pi]
     */
    static double normalizeAngle(double a);

private:
    std::array<DHParam, NUM_JOINTS> dh_;

    // Tool frame correction: DH frame 6 → URDF tool frame
    Eigen::Matrix4d T_tool_correction_;

    // ========================================================================
    // Internal Helpers
    // ========================================================================

    /**
     * Single DH transformation: Rz(theta)*Tz(d)*Tx(a)*Rx(alpha)
     */
    static Eigen::Matrix4d dhTransform(double theta, double d, double a, double alpha);

    /**
     * Convert URDF joint angles to DH theta values
     */
    std::array<double, NUM_JOINTS> urdfToDH(const JointAngles& q) const;

    /**
     * Convert DH theta values to URDF joint angles
     */
    JointAngles dhToUrdf(const std::array<double, NUM_JOINTS>& dh_theta) const;

    /**
     * Solve position sub-problem (theta1, theta2, theta3)
     * from wrist center position.
     *
     * Returns 0-4 solutions (shoulder L/R × elbow U/D)
     */
    std::vector<std::array<double, 3>> solvePosition(
        const Eigen::Vector3d& wristCenter) const;

    /**
     * Solve orientation sub-problem (theta4, theta5, theta6)
     * from R_36 rotation matrix.
     *
     * Returns 0-2 solutions (wrist flip/no-flip)
     */
    std::vector<std::array<double, 3>> solveOrientation(
        const Eigen::Matrix3d& R_36) const;
};

} // namespace analytical
} // namespace kinematics
} // namespace robot_controller
