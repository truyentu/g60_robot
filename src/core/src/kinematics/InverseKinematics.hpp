/**
 * @file InverseKinematics.hpp
 * @brief Inverse kinematics solver for 6-DOF robot
 */

#pragma once

#include "ForwardKinematics.hpp"
#include <optional>
#include <functional>
#include <limits>

namespace robot_controller {
namespace kinematics {

// ============================================================================
// IK Solution
// ============================================================================

/**
 * Result of inverse kinematics computation
 */
struct IKSolution {
    JointAngles angles;
    bool isValid;
    int configuration;      // Robot configuration index (elbow up/down, etc.)
    double residualError;   // Position error if iterative method used
    int iterations;         // Number of iterations (for iterative methods)

    IKSolution()
        : angles{0}, isValid(false), configuration(0),
          residualError(0), iterations(0) {}

    explicit IKSolution(const JointAngles& a, bool valid = true, int config = 0)
        : angles(a), isValid(valid), configuration(config),
          residualError(0), iterations(0) {}
};

/**
 * All possible IK solutions
 * For 6-DOF robot, can have up to 8 solutions
 */
struct IKSolutions {
    std::vector<IKSolution> solutions;
    bool hasAnySolution() const { return !solutions.empty(); }
    size_t count() const { return solutions.size(); }

    // Get solution closest to reference configuration
    std::optional<IKSolution> getClosestTo(const JointAngles& reference) const;

    // Get solution with minimum joint travel from reference
    std::optional<IKSolution> getMinimumTravel(const JointAngles& reference) const;
};

// ============================================================================
// IK Configuration
// ============================================================================

struct IKConfig {
    // Position tolerance (mm)
    double positionTolerance = 0.01;

    // Orientation tolerance (rad)
    double orientationTolerance = 0.001;

    // Maximum iterations for iterative solver
    int maxIterations = 100;

    // Damping factor for damped least squares
    double dampingFactor = 0.1;

    // Step size limit for iterative solver
    double maxStepSize = 0.1;  // rad

    // Use analytical solution when available
    bool preferAnalytical = true;

    // Check joint limits
    bool enforceJointLimits = true;
};

// ============================================================================
// Inverse Kinematics Class
// ============================================================================

/**
 * Inverse Kinematics solver
 * Supports both analytical (for standard 6-DOF) and numerical methods
 */
class InverseKinematics {
public:
    explicit InverseKinematics(const RobotKinematicConfig& config);
    ~InverseKinematics() = default;

    // ========================================================================
    // Main IK Methods
    // ========================================================================

    /**
     * Compute all IK solutions for target pose
     * @param targetPose Desired TCP pose
     * @return All valid solutions
     */
    IKSolutions computeAll(const TCPPose& targetPose) const;

    /**
     * Compute single IK solution closest to reference
     * @param targetPose Desired TCP pose
     * @param referenceAngles Current/reference joint angles
     * @return Best solution or nullopt if no solution
     */
    std::optional<IKSolution> compute(
        const TCPPose& targetPose,
        const JointAngles& referenceAngles) const;

    /**
     * Compute IK using iterative (numerical) method
     * Uses Damped Least Squares (DLS) / Levenberg-Marquardt
     */
    IKSolution computeIterative(
        const TCPPose& targetPose,
        const JointAngles& initialGuess) const;

    // ========================================================================
    // Configuration
    // ========================================================================

    void setConfig(const RobotKinematicConfig& config);
    void setIKConfig(const IKConfig& ikConfig) { ikConfig_ = ikConfig; }
    const IKConfig& getIKConfig() const { return ikConfig_; }

    // ========================================================================
    // Jacobian
    // ========================================================================

    /**
     * Compute Jacobian matrix at given configuration
     * @param jointAngles Current joint angles
     * @return 6x6 Jacobian matrix
     */
    Jacobian computeJacobian(const JointAngles& jointAngles) const;

    /**
     * Check if configuration is near singularity
     * @param jointAngles Current joint angles
     * @param threshold Condition number threshold
     * @return true if near singularity
     */
    bool isNearSingularity(const JointAngles& jointAngles, double threshold = 0.01) const;

    /**
     * Compute manipulability measure (Yoshikawa)
     * Higher value = further from singularity
     */
    double computeManipulability(const JointAngles& jointAngles) const;

private:
    RobotKinematicConfig config_;
    IKConfig ikConfig_;
    ForwardKinematics fk_;

    // Check if angles are within joint limits
    bool checkJointLimits(const JointAngles& angles) const;

    // Clamp angles to joint limits
    JointAngles clampToLimits(const JointAngles& angles) const;

    // Compute pose error
    Vector6d computePoseError(const TCPPose& current, const TCPPose& target) const;
};

// ============================================================================
// IKSolutions Implementation
// ============================================================================

inline std::optional<IKSolution> IKSolutions::getClosestTo(const JointAngles& reference) const {
    if (solutions.empty()) return std::nullopt;

    const IKSolution* best = nullptr;
    double minDistance = std::numeric_limits<double>::max();

    for (const auto& sol : solutions) {
        if (!sol.isValid) continue;

        double distance = 0;
        for (int i = 0; i < NUM_JOINTS; ++i) {
            double diff = normalizeAngle(sol.angles[i] - reference[i]);
            distance += diff * diff;
        }

        if (distance < minDistance) {
            minDistance = distance;
            best = &sol;
        }
    }

    return best ? std::optional<IKSolution>(*best) : std::nullopt;
}

inline std::optional<IKSolution> IKSolutions::getMinimumTravel(const JointAngles& reference) const {
    // Same as getClosestTo for revolute joints
    return getClosestTo(reference);
}

// ============================================================================
// InverseKinematics Implementation
// ============================================================================

inline InverseKinematics::InverseKinematics(const RobotKinematicConfig& config)
    : config_(config), fk_(config) {
}

inline void InverseKinematics::setConfig(const RobotKinematicConfig& config) {
    config_ = config;
    fk_.setConfig(config);
}

inline bool InverseKinematics::checkJointLimits(const JointAngles& angles) const {
    for (size_t i = 0; i < config_.dhParams.size(); ++i) {
        const auto& dh = config_.dhParams[i];
        if (angles[i] < dh.minAngle || angles[i] > dh.maxAngle) {
            return false;
        }
    }
    return true;
}

inline JointAngles InverseKinematics::clampToLimits(const JointAngles& angles) const {
    JointAngles clamped = angles;
    for (size_t i = 0; i < config_.dhParams.size(); ++i) {
        const auto& dh = config_.dhParams[i];
        clamped[i] = std::clamp(clamped[i], dh.minAngle, dh.maxAngle);
    }
    return clamped;
}

inline Vector6d InverseKinematics::computePoseError(
    const TCPPose& current, const TCPPose& target) const {

    Vector6d error;

    // Position error
    error.head<3>() = target.position - current.position;

    // Orientation error (using angle-axis representation)
    Matrix3d Re = target.rotation * current.rotation.transpose();
    AngleAxisd aa(Re);
    error.tail<3>() = aa.angle() * aa.axis();

    return error;
}

inline Jacobian InverseKinematics::computeJacobian(const JointAngles& jointAngles) const {
    Jacobian J = Jacobian::Zero();

    // Compute all transforms
    auto transforms = fk_.computeAllTransforms(jointAngles);

    // TCP position
    Vector3d p_tcp = transforms.back().block<3, 1>(0, 3);

    // Build Jacobian column by column
    Matrix4d T = Matrix4d::Identity();
    T.block<3, 3>(0, 0) = config_.baseRotation;
    T.block<3, 1>(0, 3) = config_.baseOffset;

    for (size_t i = 0; i < config_.dhParams.size(); ++i) {
        Vector3d z_i = T.block<3, 3>(0, 0).col(2);  // z-axis of frame i
        Vector3d p_i = T.block<3, 1>(0, 3);         // origin of frame i

        if (config_.dhParams[i].isRevolute) {
            // Revolute joint: J = [z x (p_tcp - p_i); z]
            J.block<3, 1>(0, i) = z_i.cross(p_tcp - p_i);
            J.block<3, 1>(3, i) = z_i;
        } else {
            // Prismatic joint: J = [z; 0]
            J.block<3, 1>(0, i) = z_i;
            J.block<3, 1>(3, i) = Vector3d::Zero();
        }

        // Update transform to next frame
        T = T * fk_.computeJointTransform(static_cast<int>(i), jointAngles[i]);
    }

    return J;
}

inline bool InverseKinematics::isNearSingularity(
    const JointAngles& jointAngles, double threshold) const {

    Jacobian J = computeJacobian(jointAngles);

    // Compute condition number using SVD
    Eigen::JacobiSVD<Jacobian> svd(J);
    auto singularValues = svd.singularValues();

    double minSV = singularValues(singularValues.size() - 1);
    double maxSV = singularValues(0);

    if (maxSV < EPSILON) return true;

    return (minSV / maxSV) < threshold;
}

inline double InverseKinematics::computeManipulability(const JointAngles& jointAngles) const {
    Jacobian J = computeJacobian(jointAngles);

    // Yoshikawa manipulability: sqrt(det(J * J^T))
    Matrix6d JJT = J * J.transpose();
    return std::sqrt(std::abs(JJT.determinant()));
}

inline IKSolution InverseKinematics::computeIterative(
    const TCPPose& targetPose,
    const JointAngles& initialGuess) const {

    IKSolution solution;
    solution.angles = initialGuess;
    solution.isValid = false;

    JointAngles q = initialGuess;

    for (int iter = 0; iter < ikConfig_.maxIterations; ++iter) {
        // Compute current pose
        TCPPose currentPose = fk_.compute(q);

        // Compute error
        Vector6d error = computePoseError(currentPose, targetPose);

        // Check convergence
        double posError = error.head<3>().norm();
        double oriError = error.tail<3>().norm();

        if (posError < ikConfig_.positionTolerance &&
            oriError < ikConfig_.orientationTolerance) {
            solution.angles = q;
            solution.isValid = true;
            solution.residualError = posError;
            solution.iterations = iter + 1;
            break;
        }

        // Compute Jacobian
        Jacobian J = computeJacobian(q);

        // Sugihara 2011 Adaptive Damping (Levenberg-Marquardt):
        // λ² = wn * ||error||² where wn = dampingFactor
        // Near singularity → error large → damping increases → stable
        // Near target → error small → damping decreases → fast convergence
        double errorNormSq = error.squaredNorm();
        double lambdaSq = ikConfig_.dampingFactor * errorNormSq;

        // Ensure minimum damping to avoid pure pseudoinverse instability
        constexpr double MIN_LAMBDA_SQ = 1e-6;
        lambdaSq = std::max(lambdaSq, MIN_LAMBDA_SQ);

        Matrix6d JJT = J * J.transpose();
        Matrix6d damped = JJT + lambdaSq * Matrix6d::Identity();

        Eigen::Matrix<double, NUM_JOINTS, 1> dq = J.transpose() * damped.ldlt().solve(error);

        // Limit step size
        double stepNorm = dq.norm();
        if (stepNorm > ikConfig_.maxStepSize) {
            dq *= ikConfig_.maxStepSize / stepNorm;
        }

        // Update joint angles
        for (int i = 0; i < NUM_JOINTS; ++i) {
            q[i] += dq(i);
            q[i] = normalizeAngle(q[i]);
        }

        // Clamp to joint limits if required
        if (ikConfig_.enforceJointLimits) {
            q = clampToLimits(q);
        }

        solution.iterations = iter + 1;
    }

    // Final check
    if (!solution.isValid) {
        TCPPose finalPose = fk_.compute(q);
        Vector6d finalError = computePoseError(finalPose, targetPose);
        solution.residualError = finalError.head<3>().norm();
        solution.angles = q;

        // Accept if close enough
        if (solution.residualError < ikConfig_.positionTolerance * 10) {
            solution.isValid = true;
        }
    }

    // Verify joint limits
    if (solution.isValid && ikConfig_.enforceJointLimits) {
        solution.isValid = checkJointLimits(solution.angles);
    }

    return solution;
}

inline IKSolutions InverseKinematics::computeAll(const TCPPose& targetPose) const {
    IKSolutions solutions;

    // For general 6-DOF robot, use iterative method with different initial guesses
    // This gives approximate "all solutions" behavior

    std::vector<JointAngles> initialGuesses;

    // Zero configuration
    initialGuesses.push_back({0, 0, 0, 0, 0, 0});

    // Various configurations
    initialGuesses.push_back({0, -PI/4, PI/2, 0, PI/4, 0});
    initialGuesses.push_back({0, -PI/4, PI/2, PI, PI/4, 0});
    initialGuesses.push_back({PI, -PI/4, PI/2, 0, PI/4, 0});
    initialGuesses.push_back({0, PI/4, -PI/2, 0, -PI/4, 0});
    initialGuesses.push_back({0, -PI/2, PI/2, 0, 0, 0});
    initialGuesses.push_back({PI/2, 0, 0, PI/2, 0, PI/2});
    initialGuesses.push_back({-PI/2, 0, 0, -PI/2, 0, -PI/2});

    for (const auto& guess : initialGuesses) {
        IKSolution sol = computeIterative(targetPose, guess);

        if (sol.isValid) {
            // Check if this solution is unique
            bool isDuplicate = false;
            for (const auto& existing : solutions.solutions) {
                double diff = 0;
                for (int i = 0; i < NUM_JOINTS; ++i) {
                    diff += std::abs(normalizeAngle(sol.angles[i] - existing.angles[i]));
                }
                if (diff < 0.1) {  // Solutions are essentially the same
                    isDuplicate = true;
                    break;
                }
            }

            if (!isDuplicate) {
                solutions.solutions.push_back(sol);
            }
        }
    }

    return solutions;
}

inline std::optional<IKSolution> InverseKinematics::compute(
    const TCPPose& targetPose,
    const JointAngles& referenceAngles) const {

    // First try iterative from reference (fastest convergence)
    IKSolution sol = computeIterative(targetPose, referenceAngles);

    if (sol.isValid) {
        return sol;
    }

    // If that fails, try all solutions and pick closest
    IKSolutions allSolutions = computeAll(targetPose);
    return allSolutions.getClosestTo(referenceAngles);
}

} // namespace kinematics
} // namespace robot_controller
