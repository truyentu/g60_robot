/**
 * @file TwistDecomposition.cpp
 * @brief TWA implementation for singularity avoidance in welding robots
 *
 * References:
 *   Huo & Baron - "The Joint-limits and Singularity Avoidance in Robotic Welding"
 *   Huo & Baron - "The self-adaptation of weights for joint-limits and singularity avoidances"
 */

#include "TwistDecomposition.hpp"
#include <cmath>
#include <algorithm>

namespace robot_controller {
namespace kinematics {

// ============================================================================
// computeTwistProjector
// ============================================================================
Matrix6d TwistDecomposition::computeTwistProjector(const Vector3d& torchAxis) {
    // Normalize torch axis (safety)
    Vector3d e = torchAxis.normalized();

    Matrix6d T = Matrix6d::Zero();

    // Angular part: M_ω = I - e·eᵀ
    // Projects out the spin component (rotation around torch axis)
    // Note: KDL geometric Jacobian has [angular; linear] ordering (ω first, v second)
    T.block<3, 3>(0, 0) = Matrix3d::Identity() - e * e.transpose();

    // Linear part: M_v = I (preserve all translation)
    T.block<3, 3>(3, 3) = Matrix3d::Identity();

    return T;
}

// ============================================================================
// computePS (Parameter of Singularity)
// ============================================================================
double TwistDecomposition::computePS(const Jacobian& J) {
    // SVD of Jacobian
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J.cast<double>(), Eigen::ComputeThinU | Eigen::ComputeThinV);
    auto sv = svd.singularValues();

    int n = static_cast<int>(sv.size());
    if (n < 2) return 1e6;

    // σ₁ = largest, σ_n = smallest
    double sigma1 = sv(0);
    double sigma_n = sv(n - 1);

    // Guard against degenerate Jacobian
    if (sigma1 < 1e-12) return 1e6;

    // ωps = √(σ₁ / (σ₂·σ₃·...·σₙ²))
    // = √(σ₁ / (σ₂·σ₃·...·σₙ·σₙ))
    double denominator = 1.0;
    for (int i = 1; i < n; ++i) {
        denominator *= sv(i);
    }
    denominator *= sigma_n;  // σₙ appears squared

    if (std::abs(denominator) < 1e-15) return 1e6;

    return std::sqrt(sigma1 / denominator);
}

// ============================================================================
// computeGradient
// ============================================================================
Vector6d TwistDecomposition::computeGradient(
    const JointAngles& current,
    const JointAngles& mid,
    const JointAngles& singEntry,
    const Vector6d& W,
    const Vector6d& K,
    double ps)
{
    Vector6d h = Vector6d::Zero();

    for (int i = 0; i < NUM_JOINTS; ++i) {
        // Joint-limit avoidance: W·(θ̄ - θ)
        double jointLimitTerm = W(i) * (mid[i] - current[i]);

        // Singularity avoidance: K·ωps·(θTs - θ)
        double singTerm = K(i) * ps * (singEntry[i] - current[i]);

        h(i) = jointLimitTerm + singTerm;
    }

    return h;
}

// ============================================================================
// resolve (Full TWA resolution)
// ============================================================================
TWAResult TwistDecomposition::resolve(
    const Jacobian& J,
    const Matrix6d& T,
    const Vector6d& taskTwist,
    const Vector6d& h,
    const TWAConfig& config,
    double currentPS)
{
    TWAResult result;
    result.psIndex = currentPS;
    result.singularityActive = (currentPS > config.psThreshold);
    result.criticalJoint = -1;

    // Smooth quadratic velocity scaling
    // Formula: scale = 1 - (1 - min) * (1 - (threshold/PS)²)
    // At PS = threshold: ratio=1 → scale = 1.0 (seamless entry, no step)
    // At PS → ∞: ratio→0 → scale → velocityScaleMin
    if (currentPS > config.psThreshold && config.psThreshold > 0.0) {
        double ratio = config.psThreshold / currentPS;
        double ratioSq = ratio * ratio;
        result.velocityScale = std::max(config.velocityScaleMin,
                                         1.0 - (1.0 - config.velocityScaleMin) * (1.0 - ratioSq));
    } else {
        result.velocityScale = 1.0;
    }

    // Scale the task twist by velocity scale
    Vector6d scaledTwist = taskTwist * result.velocityScale;

    // Compute J†  (right pseudo-inverse: J† = Jᵀ(JJᵀ)⁻¹)
    Matrix6d JJT = J * J.transpose();

    // Check if JJᵀ is invertible — if not, use DLS
    double det = JJT.determinant();
    Eigen::Matrix<double, NUM_JOINTS, 6> J_pinv;

    if (std::abs(det) < 1e-10) {
        // Near-singular: use DLS fallback
        double lambda = config.dlsLambdaMax;
        Matrix6d damped = JJT + lambda * lambda * Matrix6d::Identity();
        Matrix6d damped_inv = damped.ldlt().solve(Matrix6d::Identity());
        J_pinv = J.transpose() * damped_inv;
    } else {
        Matrix6d JJT_inv = JJT.ldlt().solve(Matrix6d::Identity());
        J_pinv = J.transpose() * JJT_inv;
    }

    // Task component: J†·T·Δt
    // Ensures Cartesian path accuracy for position + torch direction
    Eigen::Matrix<double, NUM_JOINTS, 1> taskComponent = J_pinv * T * scaledTwist;

    // Redundancy component: J†·(I-T)·J·h
    // Exploits spin angle freedom to avoid joint-limits and singularity
    Matrix6d I_minus_T = Matrix6d::Identity() - T;
    Eigen::Matrix<double, NUM_JOINTS, 1> redundancyComponent;

    if (result.singularityActive) {
        redundancyComponent = J_pinv * I_minus_T * J * h;
    } else {
        redundancyComponent = Eigen::Matrix<double, NUM_JOINTS, 1>::Zero();
    }

    // Combine
    Eigen::Matrix<double, NUM_JOINTS, 1> totalDisplacement = taskComponent + redundancyComponent;

    // Convert to JointVelocities
    for (int i = 0; i < NUM_JOINTS; ++i) {
        result.jointDisplacement[i] = totalDisplacement(i);
    }

    return result;
}

// ============================================================================
// computeSafeVelocity (Full pipeline)
// ============================================================================
TWAResult TwistDecomposition::computeSafeVelocity(
    const Jacobian& J,
    const Vector3d& torchAxis,
    const Vector6d& desiredTwist,
    const JointAngles& currentJoints,
    const JointAngles& singEntryJoints,
    const TWAConfig& config)
{
    // Step 1: Twist Projector
    Matrix6d T = computeTwistProjector(torchAxis);

    // Step 2: PS index (capped to prevent gradient explosion)
    double ps = computePS(J);
    static constexpr double PS_MAX_CAP = 500.0;  // Prevent gradient explosion
    if (ps > PS_MAX_CAP) ps = PS_MAX_CAP;

    // Step 3: Secondary task gradient (only compute if near singularity)
    Vector6d h = Vector6d::Zero();
    if (ps > config.psThreshold) {
        h = computeGradient(
            currentJoints,
            config.jointMidPositions,
            singEntryJoints,
            config.jointLimitWeights,
            config.singularityWeights,
            ps);
    }

    // Step 4: Resolve
    TWAResult result = resolve(J, T, desiredTwist, h, config, ps);

    // Step 5: Find critical joint
    result.criticalJoint = findCriticalJoint(
        currentJoints, config.jointMinLimits, config.jointMaxLimits);

    return result;
}

// ============================================================================
// dampedLeastSquares (DLS fallback)
// ============================================================================
JointVelocities TwistDecomposition::dampedLeastSquares(
    const Jacobian& J,
    const Vector6d& twist,
    double lambda)
{
    // q̇ = Jᵀ(JJᵀ + λ²I)⁻¹ẋ
    Matrix6d JJT = J * J.transpose();
    Matrix6d damped = JJT + lambda * lambda * Matrix6d::Identity();
    Vector6d intermediate = damped.ldlt().solve(twist);
    Eigen::Matrix<double, NUM_JOINTS, 1> qdot = J.transpose() * intermediate;

    JointVelocities result;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        result[i] = qdot(i);
    }
    return result;
}

// ============================================================================
// findCriticalJoint
// ============================================================================
int TwistDecomposition::findCriticalJoint(
    const JointAngles& current,
    const JointAngles& minLimits,
    const JointAngles& maxLimits,
    double warningThreshold)
{
    int criticalJoint = -1;
    double worstRatio = 0.0;

    for (int i = 0; i < NUM_JOINTS; ++i) {
        double range = maxLimits[i] - minLimits[i];
        if (range < 1e-6) continue;

        double mid = (maxLimits[i] + minLimits[i]) / 2.0;
        double halfRange = range / 2.0;
        double ratio = std::abs(current[i] - mid) / halfRange;

        if (ratio > warningThreshold && ratio > worstRatio) {
            worstRatio = ratio;
            criticalJoint = i;
        }
    }

    return criticalJoint;
}

} // namespace kinematics
} // namespace robot_controller
