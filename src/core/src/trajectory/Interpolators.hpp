/**
 * @file Interpolators.hpp
 * @brief Interpolation methods for trajectory generation
 */

#pragma once

#include "TrajectoryTypes.hpp"

namespace robot_controller {
namespace trajectory {

// ============================================================================
// Linear Interpolator (for LIN motion)
// ============================================================================

/**
 * Linear interpolation in Cartesian space
 */
class LinearInterpolator {
public:
    /**
     * Interpolate between two poses
     * @param start Start pose
     * @param end End pose
     * @param t Normalized time (0-1)
     * @return Interpolated pose
     */
    static TCPPose interpolate(const TCPPose& start, const TCPPose& end, double t);

    /**
     * Calculate distance between two poses
     */
    static double calculateDistance(const TCPPose& start, const TCPPose& end);
};

inline TCPPose LinearInterpolator::interpolate(
    const TCPPose& start, const TCPPose& end, double t) {

    t = std::clamp(t, 0.0, 1.0);

    TCPPose result;

    // Linear position interpolation
    result.position = start.position + t * (end.position - start.position);

    // Spherical linear interpolation (SLERP) for orientation
    result.quaternion = start.quaternion.slerp(t, end.quaternion);
    result.rotation = result.quaternion.toRotationMatrix();
    result.rpy = rotationToRPY(result.rotation);

    return result;
}

inline double LinearInterpolator::calculateDistance(
    const TCPPose& start, const TCPPose& end) {

    // Position distance
    double posDistance = (end.position - start.position).norm();

    return posDistance;
}

// ============================================================================
// Joint Interpolator (for PTP motion)
// ============================================================================

/**
 * Interpolation in joint space
 */
class JointInterpolator {
public:
    /**
     * Linear interpolation of joint angles
     */
    static JointAngles linearInterpolate(
        const JointAngles& start,
        const JointAngles& end,
        double t);

    /**
     * Cubic interpolation (smooth start/end)
     */
    static JointAngles cubicInterpolate(
        const JointAngles& start,
        const JointAngles& end,
        const JointVelocities& startVel,
        const JointVelocities& endVel,
        double t,
        double duration);

    /**
     * Quintic interpolation (smooth velocity and acceleration)
     */
    static JointAngles quinticInterpolate(
        const JointAngles& start,
        const JointAngles& end,
        const JointVelocities& startVel,
        const JointVelocities& endVel,
        const JointAngles& startAccel,
        const JointAngles& endAccel,
        double t,
        double duration);

    /**
     * Calculate maximum joint travel
     */
    static double calculateMaxJointTravel(
        const JointAngles& start,
        const JointAngles& end);

    /**
     * Calculate joint velocities at given interpolation point
     */
    static JointVelocities calculateVelocities(
        const JointAngles& start,
        const JointAngles& end,
        double t,
        double duration);
};

inline JointAngles JointInterpolator::linearInterpolate(
    const JointAngles& start,
    const JointAngles& end,
    double t) {

    t = std::clamp(t, 0.0, 1.0);

    JointAngles result;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        result[i] = start[i] + t * (end[i] - start[i]);
    }
    return result;
}

inline JointAngles JointInterpolator::cubicInterpolate(
    const JointAngles& start,
    const JointAngles& end,
    const JointVelocities& startVel,
    const JointVelocities& endVel,
    double t,
    double duration) {

    // Cubic polynomial: p(t) = a0 + a1*t + a2*t² + a3*t³
    // Boundary conditions:
    //   p(0) = start, p(T) = end
    //   p'(0) = startVel, p'(T) = endVel

    if (duration <= 0) {
        return (t >= 0) ? end : start;
    }

    double T = duration;
    double T2 = T * T;
    double T3 = T2 * T;

    JointAngles result;

    for (int i = 0; i < NUM_JOINTS; ++i) {
        double p0 = start[i];
        double p1 = end[i];
        double v0 = startVel[i];
        double v1 = endVel[i];

        // Solve for coefficients
        double a0 = p0;
        double a1 = v0;
        double a2 = (3.0 * (p1 - p0) - (2.0 * v0 + v1) * T) / T2;
        double a3 = (2.0 * (p0 - p1) + (v0 + v1) * T) / T3;

        double t2 = t * t;
        double t3 = t2 * t;

        result[i] = a0 + a1 * t + a2 * t2 + a3 * t3;
    }

    return result;
}

inline JointAngles JointInterpolator::quinticInterpolate(
    const JointAngles& start,
    const JointAngles& end,
    const JointVelocities& startVel,
    const JointVelocities& endVel,
    const JointAngles& startAccel,
    const JointAngles& endAccel,
    double t,
    double duration) {

    // Quintic polynomial: p(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
    // Boundary conditions:
    //   p(0) = start, p(T) = end
    //   p'(0) = startVel, p'(T) = endVel
    //   p''(0) = startAccel, p''(T) = endAccel

    if (duration <= 0) {
        return (t >= 0) ? end : start;
    }

    double T = duration;
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;

    JointAngles result;

    for (int i = 0; i < NUM_JOINTS; ++i) {
        double p0 = start[i];
        double p1 = end[i];
        double v0 = startVel[i];
        double v1 = endVel[i];
        double a0_val = startAccel[i];
        double a1_val = endAccel[i];

        // Coefficients
        double a0 = p0;
        double a1 = v0;
        double a2 = a0_val / 2.0;
        double a3 = (20.0 * (p1 - p0) - (8.0 * v1 + 12.0 * v0) * T
                     - (3.0 * a0_val - a1_val) * T2) / (2.0 * T3);
        double a4 = (30.0 * (p0 - p1) + (14.0 * v1 + 16.0 * v0) * T
                     + (3.0 * a0_val - 2.0 * a1_val) * T2) / (2.0 * T4);
        double a5 = (12.0 * (p1 - p0) - 6.0 * (v1 + v0) * T
                     - (a0_val - a1_val) * T2) / (2.0 * T5);

        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        double t5 = t4 * t;

        result[i] = a0 + a1 * t + a2 * t2 + a3 * t3 + a4 * t4 + a5 * t5;
    }

    return result;
}

inline double JointInterpolator::calculateMaxJointTravel(
    const JointAngles& start,
    const JointAngles& end) {

    double maxTravel = 0;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        double travel = std::abs(end[i] - start[i]);
        maxTravel = std::max(maxTravel, travel);
    }
    return maxTravel;
}

inline JointVelocities JointInterpolator::calculateVelocities(
    const JointAngles& start,
    const JointAngles& end,
    double t,
    double duration) {

    JointVelocities velocities{0};

    if (duration <= 0) {
        return velocities;
    }

    // For linear interpolation, velocity is constant
    for (int i = 0; i < NUM_JOINTS; ++i) {
        velocities[i] = (end[i] - start[i]) / duration;
    }

    return velocities;
}

// ============================================================================
// Circular Interpolator (for CIRC motion)
// ============================================================================

/**
 * Circular arc interpolation
 */
class CircularInterpolator {
public:
    /**
     * Define circular arc from 3 points
     * @param start Start point
     * @param via Via point (on arc)
     * @param end End point
     * @param center Output: computed center
     * @param radius Output: computed radius
     * @param normal Output: arc plane normal
     * @return true if valid arc computed
     */
    static bool defineArc(
        const Vector3d& start,
        const Vector3d& via,
        const Vector3d& end,
        Vector3d& center,
        double& radius,
        Vector3d& normal);

    /**
     * Interpolate along circular arc
     * @param center Arc center
     * @param radius Arc radius
     * @param normal Arc plane normal
     * @param startDir Start direction from center
     * @param sweepAngle Total angle to sweep
     * @param t Normalized time (0-1)
     * @return Point on arc
     */
    static Vector3d interpolateArc(
        const Vector3d& center,
        double radius,
        const Vector3d& normal,
        const Vector3d& startDir,
        double sweepAngle,
        double t);

    /**
     * Calculate arc length
     */
    static double calculateArcLength(double radius, double sweepAngle) {
        return std::abs(radius * sweepAngle);
    }
};

inline bool CircularInterpolator::defineArc(
    const Vector3d& start,
    const Vector3d& via,
    const Vector3d& end,
    Vector3d& center,
    double& radius,
    Vector3d& normal) {

    // Vectors from start
    Vector3d v1 = via - start;
    Vector3d v2 = end - start;

    // Normal to arc plane
    normal = v1.cross(v2);
    double normalLen = normal.norm();

    if (normalLen < EPSILON) {
        // Points are collinear - no arc possible
        return false;
    }
    normal.normalize();

    // Find circle center using circumcenter formula
    double a = v1.norm();
    double b = (end - via).norm();
    double c = v2.norm();

    // Area of triangle
    double area = 0.5 * normalLen;

    // Circumradius: R = abc / (4 * area)
    radius = (a * b * c) / (4.0 * area);

    // Find center using barycentric coordinates
    double a2 = a * a;
    double b2 = b * b;
    double c2 = c * c;

    double alpha = b2 * (c2 + a2 - b2);
    double beta = c2 * (a2 + b2 - c2);
    double gamma = a2 * (b2 + c2 - a2);
    double sum = alpha + beta + gamma;

    if (std::abs(sum) < EPSILON) {
        return false;
    }

    center = (alpha * end + beta * start + gamma * via) / sum;

    return true;
}

inline Vector3d CircularInterpolator::interpolateArc(
    const Vector3d& center,
    double radius,
    const Vector3d& normal,
    const Vector3d& startDir,
    double sweepAngle,
    double t) {

    t = std::clamp(t, 0.0, 1.0);
    double angle = t * sweepAngle;

    // Create rotation around normal
    AngleAxisd rotation(angle, normal);

    // Rotate start direction
    Vector3d dir = rotation * startDir;

    // Point on arc
    return center + radius * dir;
}

} // namespace trajectory
} // namespace robot_controller
