/**
 * @file VelocityProfile.hpp
 * @brief Velocity profile generators for trajectory planning
 */

#pragma once

#include "TrajectoryTypes.hpp"
#include <cmath>
#include <memory>

namespace robot_controller {
namespace trajectory {

// ============================================================================
// Velocity Profile Result
// ============================================================================

/**
 * Result of velocity profile computation
 */
struct ProfileResult {
    double position;      // Normalized position (0-1)
    double velocity;      // Normalized velocity
    double acceleration;  // Normalized acceleration
    double jerk;          // Normalized jerk

    ProfileResult() : position(0), velocity(0), acceleration(0), jerk(0) {}
};

// ============================================================================
// Base Profile Interface
// ============================================================================

/**
 * Abstract base class for velocity profiles
 */
class IVelocityProfile {
public:
    virtual ~IVelocityProfile() = default;

    /**
     * Compute profile at given time
     * @param t Current time (0 to duration)
     * @param duration Total duration
     * @return Normalized position/velocity/acceleration
     */
    virtual ProfileResult compute(double t, double duration) const = 0;

    /**
     * Calculate minimum duration for given distance and limits
     * @param distance Total distance to travel
     * @param maxVelocity Maximum velocity
     * @param maxAcceleration Maximum acceleration
     * @param maxJerk Maximum jerk (for S-curve)
     * @return Minimum duration
     */
    virtual double calculateDuration(
        double distance,
        double maxVelocity,
        double maxAcceleration,
        double maxJerk = 0) const = 0;
};

// ============================================================================
// Trapezoidal Profile
// ============================================================================

/**
 * Trapezoidal velocity profile
 *
 * Phases:
 * 1. Acceleration (constant accel)
 * 2. Cruise (constant velocity)
 * 3. Deceleration (constant decel)
 */
class TrapezoidalProfile : public IVelocityProfile {
public:
    ProfileResult compute(double t, double duration) const override;

    double calculateDuration(
        double distance,
        double maxVelocity,
        double maxAcceleration,
        double maxJerk = 0) const override;

    /**
     * Compute profile with specific parameters
     */
    ProfileResult computeWithParams(
        double t,
        double accelTime,
        double cruiseTime,
        double decelTime,
        double peakVelocity) const;

    /**
     * Calculate phase times for given distance and limits
     */
    void calculatePhases(
        double distance,
        double maxVelocity,
        double maxAcceleration,
        double& accelTime,
        double& cruiseTime,
        double& decelTime,
        double& peakVelocity) const;
};

// TrapezoidalProfile Implementation
inline ProfileResult TrapezoidalProfile::compute(double t, double duration) const {
    if (duration <= 0) {
        ProfileResult result;
        result.position = (t >= 0) ? 1.0 : 0.0;
        return result;
    }

    // Assume symmetric accel/decel with 1/3 time for each phase
    double accelTime = duration / 3.0;
    double cruiseTime = duration / 3.0;
    double decelTime = duration / 3.0;
    double peakVelocity = 1.5 / duration;  // Normalized for total distance = 1

    return computeWithParams(t, accelTime, cruiseTime, decelTime, peakVelocity);
}

inline ProfileResult TrapezoidalProfile::computeWithParams(
    double t,
    double accelTime,
    double cruiseTime,
    double decelTime,
    double peakVelocity) const {

    ProfileResult result;
    double duration = accelTime + cruiseTime + decelTime;

    if (t < 0) {
        result.position = 0;
        result.velocity = 0;
        result.acceleration = 0;
    }
    else if (t < accelTime && accelTime > 0) {
        // Acceleration phase
        double accel = peakVelocity / accelTime;
        result.acceleration = accel;
        result.velocity = accel * t;
        result.position = 0.5 * accel * t * t;
    }
    else if (t < accelTime + cruiseTime) {
        // Cruise phase
        double t1 = t - accelTime;
        double posAtEndAccel = 0.5 * peakVelocity * accelTime;

        result.acceleration = 0;
        result.velocity = peakVelocity;
        result.position = posAtEndAccel + peakVelocity * t1;
    }
    else if (t < duration && decelTime > 0) {
        // Deceleration phase
        double t2 = t - accelTime - cruiseTime;
        double decel = peakVelocity / decelTime;
        double posAtEndCruise = 0.5 * peakVelocity * accelTime + peakVelocity * cruiseTime;

        result.acceleration = -decel;
        result.velocity = peakVelocity - decel * t2;
        result.position = posAtEndCruise + peakVelocity * t2 - 0.5 * decel * t2 * t2;
    }
    else {
        // After motion
        result.position = 1.0;
        result.velocity = 0;
        result.acceleration = 0;
    }

    return result;
}

inline void TrapezoidalProfile::calculatePhases(
    double distance,
    double maxVelocity,
    double maxAcceleration,
    double& accelTime,
    double& cruiseTime,
    double& decelTime,
    double& peakVelocity) const {

    if (distance <= 0 || maxVelocity <= 0 || maxAcceleration <= 0) {
        accelTime = cruiseTime = decelTime = 0;
        peakVelocity = 0;
        return;
    }

    // Time to accelerate to max velocity
    double t_accel = maxVelocity / maxAcceleration;

    // Distance covered during accel and decel (assuming symmetric)
    double d_accel = 0.5 * maxAcceleration * t_accel * t_accel;
    double d_total_accel_decel = 2.0 * d_accel;

    if (d_total_accel_decel >= distance) {
        // Triangular profile (no cruise phase)
        accelTime = std::sqrt(distance / maxAcceleration);
        decelTime = accelTime;
        cruiseTime = 0;
        peakVelocity = maxAcceleration * accelTime;
    }
    else {
        // Full trapezoidal profile
        accelTime = t_accel;
        decelTime = t_accel;
        peakVelocity = maxVelocity;

        double d_cruise = distance - d_total_accel_decel;
        cruiseTime = d_cruise / maxVelocity;
    }
}

inline double TrapezoidalProfile::calculateDuration(
    double distance,
    double maxVelocity,
    double maxAcceleration,
    double maxJerk) const {

    double accelTime, cruiseTime, decelTime, peakVelocity;
    calculatePhases(distance, maxVelocity, maxAcceleration,
                    accelTime, cruiseTime, decelTime, peakVelocity);

    return accelTime + cruiseTime + decelTime;
}

// ============================================================================
// S-Curve Profile (Jerk-Limited)
// ============================================================================

/**
 * S-Curve (jerk-limited) velocity profile
 *
 * Uses cosine-based interpolation for smooth acceleration
 * This provides continuous jerk without explicit 7-phase calculation
 */
class SCurveProfile : public IVelocityProfile {
public:
    ProfileResult compute(double t, double duration) const override;

    double calculateDuration(
        double distance,
        double maxVelocity,
        double maxAcceleration,
        double maxJerk) const override;
};

inline ProfileResult SCurveProfile::compute(double t, double duration) const {
    ProfileResult result;

    if (t <= 0) {
        return result;
    }

    if (duration <= 0 || t >= duration) {
        result.position = 1.0;
        return result;
    }

    double s = t / duration;  // Normalized time (0-1)

    // Use cosine-based for S-curve-like behavior
    // position = (1 - cos(π*s)) / 2
    // velocity = (π * sin(π*s)) / 2
    // acceleration = (π² * cos(π*s)) / 2

    result.position = (1.0 - std::cos(PI * s)) / 2.0;
    result.velocity = (PI * std::sin(PI * s)) / (2.0 * duration);
    result.acceleration = (PI * PI * std::cos(PI * s)) / (2.0 * duration * duration);

    // Normalize velocity to match the total distance = 1 convention
    result.velocity *= duration;
    result.acceleration *= duration * duration;

    return result;
}

inline double SCurveProfile::calculateDuration(
    double distance,
    double maxVelocity,
    double maxAcceleration,
    double maxJerk) const {

    if (distance <= 0 || maxVelocity <= 0 || maxAcceleration <= 0) {
        return 0;
    }

    // For S-curve with cosine profile, peak velocity is π/2 * distance/duration
    // So duration = π * distance / (2 * maxVelocity)
    double durationFromVel = PI * distance / (2.0 * maxVelocity);

    // Peak acceleration is π²/2 * distance/duration²
    // So duration = sqrt(π² * distance / (2 * maxAcceleration))
    double durationFromAccel = std::sqrt(PI * PI * distance / (2.0 * maxAcceleration));

    // Take the larger (more conservative) duration
    return std::max(durationFromVel, durationFromAccel);
}

// ============================================================================
// Constant Velocity Profile
// ============================================================================

/**
 * Constant velocity profile (no acceleration phases)
 */
class ConstantProfile : public IVelocityProfile {
public:
    ProfileResult compute(double t, double duration) const override {
        ProfileResult result;

        if (duration <= 0) {
            result.position = (t >= 0) ? 1.0 : 0.0;
            return result;
        }

        double s = std::clamp(t / duration, 0.0, 1.0);
        result.position = s;
        result.velocity = 1.0 / duration;
        result.acceleration = 0;

        return result;
    }

    double calculateDuration(
        double distance,
        double maxVelocity,
        double maxAcceleration,
        double maxJerk = 0) const override {

        if (distance <= 0 || maxVelocity <= 0) {
            return 0;
        }
        return distance / maxVelocity;
    }
};

// ============================================================================
// Profile Factory
// ============================================================================

inline std::unique_ptr<IVelocityProfile> createVelocityProfile(VelocityProfile type) {
    switch (type) {
        case VelocityProfile::TRAPEZOIDAL:
            return std::make_unique<TrapezoidalProfile>();
        case VelocityProfile::SCURVE:
            return std::make_unique<SCurveProfile>();
        case VelocityProfile::CONSTANT:
            return std::make_unique<ConstantProfile>();
        default:
            return std::make_unique<TrapezoidalProfile>();
    }
}

} // namespace trajectory
} // namespace robot_controller
