/**
 * @file TwistDecomposition.hpp
 * @brief Twist Decomposition Algorithm (TWA) for singularity avoidance
 *
 * Implements the TWA from Huo & Baron papers for welding robots:
 * - "The Joint-limits and Singularity Avoidance in Robotic Welding"
 * - "The self-adaptation of weights for joint-limits and singularity avoidances"
 *
 * Key concept: Welding tasks need only 5 DOF (XYZ + torch direction).
 * The spin angle around the torch axis is redundant and can be exploited
 * to avoid wrist singularity without deviating from the weld path.
 *
 * Core equations:
 *   T = [I-eeᵀ, 0; 0, I]          Twist Projector (e = torch axis)
 *   ωps = √(σ₁/(σ₂·σ₃·...·σ₆²))  Parameter of Singularity
 *   h = W(θ̄-θ) + K·ωps·(θTs-θ)    Secondary task gradient
 *   Δθ = J†·T·Δt + J†·(I-T)·J·h   TWA resolution
 */

#pragma once

#include "MathTypes.hpp"
#include <Eigen/SVD>

namespace robot_controller {
namespace kinematics {

/**
 * Configuration for TWA singularity avoidance
 */
struct TWAConfig {
    double psThreshold = 0.3;        // PS threshold to ENTER singularity zone (~J5≈10°)
    double psThresholdExit = 0.2;    // PS threshold to EXIT singularity zone (hysteresis)
    double velocityScaleMin = 1.0;   // No velocity reduction near singularity (100%)
    double dlsLambdaMax = 0.1;       // Maximum DLS damping factor
    Vector6d jointLimitWeights;      // W diagonal (joint-limit avoidance)
    Vector6d singularityWeights;     // K diagonal (singularity avoidance)
    JointAngles jointMidPositions;   // θ̄ (mid-range of each joint)
    JointAngles jointMinLimits;      // Lower joint limits (rad)
    JointAngles jointMaxLimits;      // Upper joint limits (rad)

    TWAConfig() {
        jointLimitWeights = Vector6d::Ones();
        singularityWeights = Vector6d::Ones();
        jointMidPositions = {};
        jointMinLimits = {};
        jointMaxLimits = {};
    }
};

/**
 * Result of TWA computation
 */
struct TWAResult {
    JointVelocities jointDisplacement;  // Safe Δθ (rad)
    double psIndex;                      // Current PS value
    double velocityScale;                // Applied velocity scaling [0.1, 1.0]
    bool singularityActive;              // Whether avoidance is active
    int criticalJoint;                   // Joint closest to limit/singularity (-1 if none)
};

/**
 * Twist Decomposition Algorithm for singularity avoidance
 *
 * Designed for 6-DOF welding robots performing 5-DOF tasks.
 * All methods are static — no internal state.
 */
class TwistDecomposition {
public:
    // ================================================================
    // Core TWA Methods
    // ================================================================

    /**
     * Compute Twist Projector for 5-DOF welding task
     *
     * T = [M_ω, 0; 0, M_v] where:
     *   M_ω = I - e·eᵀ  (project out spin rotation around torch axis)
     *   M_v = I           (preserve all translation)
     *
     * @param torchAxis Unit vector along torch Z-axis (in base frame)
     * @return 6×6 projector matrix T
     */
    static Matrix6d computeTwistProjector(const Vector3d& torchAxis);

    /**
     * Parameter of Singularity (PS) index
     *
     * ωps = √(σ₁ / (σ₂·σ₃·...·σ₆²))
     *
     * Combines condition number and manipulability into a single index.
     * Higher value = closer to singularity.
     *
     * @param J 6×6 Jacobian matrix (geometric, from KDL)
     * @return PS index value (≥1, increases near singularity)
     */
    static double computePS(const Jacobian& J);

    /**
     * Compute secondary task gradient for joint-limits + singularity avoidance
     *
     * h = W·(θ̄ - θ) + K·ωps·(θTs - θ)
     *
     * First term pushes joints toward mid-range (joint-limit avoidance).
     * Second term pushes toward safe configuration recorded at singularity entry.
     *
     * @param current   Current joint angles θ (rad)
     * @param mid       Mid-range joint angles θ̄ (rad)
     * @param singEntry Joint angles at singularity entry θTs (rad)
     * @param W         Joint-limit weight vector (diagonal of W matrix)
     * @param K         Singularity weight vector (diagonal of K matrix)
     * @param ps        Current PS index value
     * @return 6×1 gradient vector h
     */
    static Vector6d computeGradient(
        const JointAngles& current,
        const JointAngles& mid,
        const JointAngles& singEntry,
        const Vector6d& W,
        const Vector6d& K,
        double ps);

    /**
     * Full TWA resolution: compute safe joint displacement
     *
     * Δθ = J†·T·Δt + J†·(I-T)·J·h
     *
     * Task component: J†·T·Δt ensures Cartesian path accuracy (XYZ + direction).
     * Redundancy component: J†·(I-T)·J·h exploits spin angle for avoidance.
     *
     * Falls back to DLS when Jacobian is ill-conditioned.
     *
     * @param J          6×6 Jacobian (from KDL)
     * @param T          6×6 Twist Projector
     * @param taskTwist  Desired end-effector twist Δt (6×1)
     * @param h          Secondary task gradient (6×1)
     * @param config     TWA configuration
     * @param currentPS  Current PS index
     * @return TWAResult with safe joint displacement and diagnostics
     */
    static TWAResult resolve(
        const Jacobian& J,
        const Matrix6d& T,
        const Vector6d& taskTwist,
        const Vector6d& h,
        const TWAConfig& config,
        double currentPS);

    // ================================================================
    // Convenience: Full Pipeline
    // ================================================================

    /**
     * One-call interface: compute safe velocity from all inputs
     *
     * Combines computeTwistProjector, computePS, computeGradient, and resolve
     * into a single call. Use this from JogController/RobotController.
     *
     * @param J              6×6 Jacobian (from KDL)
     * @param torchAxis      Torch Z-axis unit vector (in base frame)
     * @param desiredTwist   Desired end-effector twist (6×1)
     * @param currentJoints  Current joint angles (rad)
     * @param singEntryJoints Joint angles recorded at singularity entry (rad)
     * @param config         TWA configuration
     * @return TWAResult with safe joint displacement and diagnostics
     */
    static TWAResult computeSafeVelocity(
        const Jacobian& J,
        const Vector3d& torchAxis,
        const Vector6d& desiredTwist,
        const JointAngles& currentJoints,
        const JointAngles& singEntryJoints,
        const TWAConfig& config);

    // ================================================================
    // DLS Fallback
    // ================================================================

    /**
     * Damped Least Squares IK (fallback for extreme singularity)
     *
     * q̇ = Jᵀ(JJᵀ + λ²I)⁻¹ẋ
     *
     * Guarantees bounded joint velocities even at exact singularity.
     *
     * @param J      6×6 Jacobian
     * @param twist  Desired twist (6×1)
     * @param lambda Damping factor (0 = pure pseudo-inverse)
     * @return Joint velocities (rad/s or rad)
     */
    static JointVelocities dampedLeastSquares(
        const Jacobian& J,
        const Vector6d& twist,
        double lambda);

    // ================================================================
    // Utilities
    // ================================================================

    /**
     * Find the joint closest to its limit
     * @return Joint index (0-5) or -1 if all joints are well within limits
     */
    static int findCriticalJoint(
        const JointAngles& current,
        const JointAngles& minLimits,
        const JointAngles& maxLimits,
        double warningThreshold = 0.9);
};

} // namespace kinematics
} // namespace robot_controller
