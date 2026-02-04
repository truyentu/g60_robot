# IMPL_P2_03: Trajectory Generator

| Metadata | Value |
|----------|-------|
| Plan ID | IMPL_P2_03 |
| Phase | 2 - Motion Core |
| Priority | P0 (Critical) |
| Depends On | IMPL_P2_02 (Kinematics Engine) |
| Status | Ready for Implementation |

---

## Required Reading (ĐỌC TRƯỚC KHI CODE)

| Priority | Document | Lý do |
|----------|----------|-------|
| P0 | `ressearch_doc_md/Tích hợp Ruckig cho Robot Hàn 6-DOF.md` | Ruckig OTG, jerk-limited profiles, online replanning |

---

## Overview

Implementation plan cho Trajectory Generator của 6-DOF Robot Controller:
- **Path Types:** Linear (PTP, LIN), Circular (CIRC), Spline
- **Interpolation:** Joint-space và Cartesian-space interpolation
- **Velocity Profiles:** Trapezoidal và S-curve acceleration
- **Blending:** Smooth transition giữa các segments
- **Real-time Execution:** Trajectory sampling cho motion control loop

---

## Prerequisites

- [ ] IMPL_P2_02 (Kinematics Engine) đã hoàn thành
- [ ] Eigen library available
- [ ] C++ project builds successfully
- [ ] Hiểu velocity profiles và motion planning concepts

---

## Step 1: Create Trajectory Types

### 1.1 Create TrajectoryTypes.hpp

**File:** `src/cpp/include/trajectory/TrajectoryTypes.hpp`

```cpp
#pragma once

#include "kinematics/MathTypes.hpp"
#include "kinematics/ForwardKinematics.hpp"
#include <vector>
#include <memory>
#include <optional>

namespace robotics {
namespace trajectory {

using namespace math;
using namespace kinematics;

// ============================================================================
// Enumerations
// ============================================================================

/**
 * Motion type
 */
enum class MotionType {
    PTP,    // Point-to-Point (joint interpolation)
    LIN,    // Linear (Cartesian interpolation)
    CIRC,   // Circular arc
    SPLINE  // Spline path through waypoints
};

/**
 * Velocity profile type
 */
enum class VelocityProfile {
    TRAPEZOIDAL,  // Constant acceleration/deceleration
    SCURVE,       // Jerk-limited (smooth acceleration)
    CONSTANT      // Constant velocity (no accel/decel phases)
};

/**
 * Blend type for segment transitions
 */
enum class BlendType {
    NONE,       // Full stop between segments
    PARABOLIC,  // Parabolic blend
    CIRCULAR,   // Circular arc blend
    SMOOTH      // Smooth spline blend
};

/**
 * Trajectory state
 */
enum class TrajectoryState {
    IDLE,
    RUNNING,
    PAUSED,
    COMPLETED,
    ERROR
};

// ============================================================================
// Motion Parameters
// ============================================================================

/**
 * Velocity and acceleration limits
 */
struct MotionLimits {
    // Joint space limits (per joint)
    std::array<double, NUM_JOINTS> maxJointVelocity;      // rad/s
    std::array<double, NUM_JOINTS> maxJointAcceleration;  // rad/s²
    std::array<double, NUM_JOINTS> maxJointJerk;          // rad/s³

    // Cartesian space limits
    double maxLinearVelocity;       // mm/s
    double maxLinearAcceleration;   // mm/s²
    double maxLinearJerk;           // mm/s³

    double maxAngularVelocity;      // rad/s
    double maxAngularAcceleration;  // rad/s²

    MotionLimits() {
        // Default limits (conservative)
        for (int i = 0; i < NUM_JOINTS; ++i) {
            maxJointVelocity[i] = degToRad(180.0);     // 180 deg/s
            maxJointAcceleration[i] = degToRad(360.0); // 360 deg/s²
            maxJointJerk[i] = degToRad(720.0);         // 720 deg/s³
        }

        maxLinearVelocity = 500.0;       // 500 mm/s
        maxLinearAcceleration = 2000.0;  // 2000 mm/s²
        maxLinearJerk = 10000.0;         // 10000 mm/s³

        maxAngularVelocity = degToRad(180.0);
        maxAngularAcceleration = degToRad(360.0);
    }
};

/**
 * Motion command parameters
 */
struct MotionParams {
    double velocityScale;      // 0.0 - 1.0 (percentage of max)
    double accelerationScale;  // 0.0 - 1.0 (percentage of max)
    double blendRadius;        // mm (for blending, 0 = no blend)
    VelocityProfile profile;
    BlendType blendType;

    MotionParams()
        : velocityScale(0.5),
          accelerationScale(0.5),
          blendRadius(0.0),
          profile(VelocityProfile::TRAPEZOIDAL),
          blendType(BlendType::NONE) {}
};

// ============================================================================
// Waypoint
// ============================================================================

/**
 * A point in the trajectory
 */
struct Waypoint {
    // Position (either joint or Cartesian, depending on motion type)
    JointAngles jointAngles;
    TCPPose cartesianPose;

    // Timing
    double timestamp;           // Time at this waypoint (s)

    // Velocities at this point
    JointVelocities jointVelocities;
    Vector6d cartesianVelocity;  // [vx, vy, vz, wx, wy, wz]

    // Flags
    bool isJointSpace;          // true = joint angles valid, false = Cartesian valid
    bool isViaPoint;            // true = pass through, false = stop at

    Waypoint()
        : jointAngles{0}, timestamp(0),
          jointVelocities{0}, cartesianVelocity(Vector6d::Zero()),
          isJointSpace(true), isViaPoint(false) {}
};

// ============================================================================
// Trajectory Segment
// ============================================================================

/**
 * A single segment of a trajectory (between two waypoints)
 */
struct TrajectorySegment {
    Waypoint start;
    Waypoint end;

    MotionType motionType;
    MotionParams params;

    // For circular motion
    Vector3d circleCenter;
    Vector3d circleNormal;
    double circleRadius;

    // Timing
    double duration;            // Segment duration (s)
    double startTime;           // Start time in trajectory (s)

    // Precomputed coefficients for interpolation
    // For polynomial interpolation: pos = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
    std::array<Vector6d, 6> cartesianCoeffs;  // Quintic coefficients for Cartesian
    std::array<std::array<double, 6>, NUM_JOINTS> jointCoeffs;  // Per joint

    TrajectorySegment()
        : motionType(MotionType::PTP),
          circleCenter(Vector3d::Zero()),
          circleNormal(Vector3d::UnitZ()),
          circleRadius(0),
          duration(0),
          startTime(0) {
        for (auto& c : cartesianCoeffs) c = Vector6d::Zero();
        for (auto& jc : jointCoeffs) {
            for (auto& c : jc) c = 0;
        }
    }
};

// ============================================================================
// Trajectory Point (Sampled)
// ============================================================================

/**
 * A sampled point from trajectory for execution
 */
struct TrajectoryPoint {
    double time;                    // Time from trajectory start (s)

    JointAngles jointPositions;     // Target joint positions (rad)
    JointVelocities jointVelocities;// Target joint velocities (rad/s)
    JointAngles jointAccelerations; // Target joint accelerations (rad/s²)

    TCPPose cartesianPose;          // TCP pose
    Vector6d cartesianVelocity;     // TCP velocity
    Vector6d cartesianAcceleration; // TCP acceleration

    int segmentIndex;               // Current segment
    double segmentProgress;         // 0.0 - 1.0 within segment

    bool isValid;

    TrajectoryPoint()
        : time(0),
          jointPositions{0},
          jointVelocities{0},
          jointAccelerations{0},
          cartesianVelocity(Vector6d::Zero()),
          cartesianAcceleration(Vector6d::Zero()),
          segmentIndex(0),
          segmentProgress(0),
          isValid(false) {}
};

// ============================================================================
// Complete Trajectory
// ============================================================================

/**
 * A complete trajectory consisting of multiple segments
 */
class Trajectory {
public:
    std::vector<TrajectorySegment> segments;
    MotionLimits limits;

    double totalDuration;
    bool isValid;
    std::string errorMessage;

    Trajectory() : totalDuration(0), isValid(false) {}

    /**
     * Sample trajectory at given time
     */
    TrajectoryPoint sample(double time) const;

    /**
     * Get segment at given time
     */
    int getSegmentAt(double time) const;

    /**
     * Check if time is within trajectory
     */
    bool isTimeValid(double time) const {
        return time >= 0 && time <= totalDuration;
    }
};

} // namespace trajectory
} // namespace robotics
```

---

## Step 2: Create Velocity Profile Generator

### 2.1 Create VelocityProfile.hpp

**File:** `src/cpp/include/trajectory/VelocityProfile.hpp`

```cpp
#pragma once

#include "TrajectoryTypes.hpp"
#include <cmath>

namespace robotics {
namespace trajectory {

// ============================================================================
// Velocity Profile Result
// ============================================================================

/**
 * Result of velocity profile computation
 */
struct ProfileResult {
    double position;      // Normalized position (0-1)
    double velocity;      // Normalized velocity (0-1)
    double acceleration;  // Normalized acceleration
    double jerk;          // Normalized jerk
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

// Implementation
inline ProfileResult TrapezoidalProfile::compute(double t, double duration) const {
    // Assume symmetric accel/decel with 1/3 time for each phase
    double accelTime = duration / 3.0;
    double cruiseTime = duration / 3.0;
    double decelTime = duration / 3.0;
    double peakVelocity = 1.5;  // Normalized peak for total distance = 1

    return computeWithParams(t, accelTime, cruiseTime, decelTime, peakVelocity);
}

inline ProfileResult TrapezoidalProfile::computeWithParams(
    double t,
    double accelTime,
    double cruiseTime,
    double decelTime,
    double peakVelocity) const {

    ProfileResult result{0, 0, 0, 0};
    double duration = accelTime + cruiseTime + decelTime;

    if (t < 0) {
        result.position = 0;
        result.velocity = 0;
        result.acceleration = 0;
    }
    else if (t < accelTime) {
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
    else if (t < duration) {
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
        // distance = 2 * (0.5 * a * t²) = a * t²
        // t = sqrt(distance / a)
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
// S-Curve Profile
// ============================================================================

/**
 * S-Curve (jerk-limited) velocity profile
 *
 * 7 phases:
 * 1. Jerk+ (increasing acceleration)
 * 2. Constant acceleration
 * 3. Jerk- (decreasing acceleration to 0)
 * 4. Cruise (constant velocity)
 * 5. Jerk- (increasing deceleration)
 * 6. Constant deceleration
 * 7. Jerk+ (decreasing deceleration to 0)
 */
class SCurveProfile : public IVelocityProfile {
public:
    ProfileResult compute(double t, double duration) const override;

    double calculateDuration(
        double distance,
        double maxVelocity,
        double maxAcceleration,
        double maxJerk) const override;

private:
    struct PhaseParams {
        double jerkTime;        // Duration of each jerk phase
        double accelTime;       // Duration of constant acceleration
        double cruiseTime;      // Duration of cruise
        double peakVelocity;
        double peakAcceleration;
        double jerk;
    };

    PhaseParams calculatePhaseParams(
        double distance,
        double maxVelocity,
        double maxAcceleration,
        double maxJerk) const;
};

inline SCurveProfile::PhaseParams SCurveProfile::calculatePhaseParams(
    double distance,
    double maxVelocity,
    double maxAcceleration,
    double maxJerk) const {

    PhaseParams params{0, 0, 0, 0, 0, 0};

    if (distance <= 0 || maxVelocity <= 0 || maxAcceleration <= 0 || maxJerk <= 0) {
        return params;
    }

    params.jerk = maxJerk;

    // Time to reach max acceleration
    double t_jerk = maxAcceleration / maxJerk;

    // Velocity gained during one jerk phase
    double v_jerk = 0.5 * maxJerk * t_jerk * t_jerk;

    // Check if we can reach max velocity
    double v_during_accel = 2 * v_jerk;  // Velocity during both jerk phases

    if (v_during_accel >= maxVelocity) {
        // Can't reach max acceleration, pure jerk phases
        params.jerkTime = std::sqrt(maxVelocity / maxJerk);
        params.accelTime = 0;
        params.peakAcceleration = maxJerk * params.jerkTime;
        params.peakVelocity = maxVelocity;
    }
    else {
        // Full S-curve with constant acceleration phase
        params.jerkTime = t_jerk;
        params.peakAcceleration = maxAcceleration;

        // Time at constant acceleration to reach max velocity
        double v_remaining = maxVelocity - v_during_accel;
        params.accelTime = v_remaining / maxAcceleration;
        params.peakVelocity = maxVelocity;
    }

    // Calculate distance covered during acceleration phases
    // (This is simplified - full calculation is complex)
    double t_accel_total = 2 * params.jerkTime + params.accelTime;
    double d_accel = params.peakVelocity * t_accel_total / 2;  // Approximate

    double d_cruise = distance - 2 * d_accel;
    if (d_cruise < 0) {
        // Need to reduce velocity - simplify to trapezoidal-like
        params.cruiseTime = 0;
        // Recalculate with reduced velocity (simplified)
        params.peakVelocity = std::sqrt(distance * maxAcceleration);
        if (params.peakVelocity > maxVelocity) params.peakVelocity = maxVelocity;
    }
    else {
        params.cruiseTime = d_cruise / params.peakVelocity;
    }

    return params;
}

inline ProfileResult SCurveProfile::compute(double t, double duration) const {
    // Simplified S-curve using sine-based acceleration
    // This gives smooth acceleration without explicit 7-phase calculation

    ProfileResult result{0, 0, 0, 0};

    if (t <= 0) {
        return result;
    }

    if (t >= duration) {
        result.position = 1.0;
        return result;
    }

    double s = t / duration;  // Normalized time (0-1)

    // Use smoothstep for S-curve-like behavior
    // position = 3s² - 2s³
    // velocity = 6s - 6s²
    // acceleration = 6 - 12s

    // Or use cosine-based for smoother profile
    // position = (1 - cos(π*s)) / 2
    // velocity = (π * sin(π*s)) / 2
    // acceleration = (π² * cos(π*s)) / 2

    result.position = (1.0 - std::cos(PI * s)) / 2.0;
    result.velocity = (PI * std::sin(PI * s)) / (2.0 * duration);
    result.acceleration = (PI * PI * std::cos(PI * s)) / (2.0 * duration * duration);

    // Normalize velocity to peak at 1.0
    result.velocity *= duration;
    result.acceleration *= duration * duration;

    return result;
}

inline double SCurveProfile::calculateDuration(
    double distance,
    double maxVelocity,
    double maxAcceleration,
    double maxJerk) const {

    auto params = calculatePhaseParams(distance, maxVelocity, maxAcceleration, maxJerk);

    // Total duration = 4 * jerkTime + 2 * accelTime + cruiseTime
    return 4 * params.jerkTime + 2 * params.accelTime + params.cruiseTime;
}

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
        default:
            return std::make_unique<TrapezoidalProfile>();
    }
}

} // namespace trajectory
} // namespace robotics
```

---

## Step 3: Create Interpolators

### 3.1 Create Interpolators.hpp

**File:** `src/cpp/include/trajectory/Interpolators.hpp`

```cpp
#pragma once

#include "TrajectoryTypes.hpp"
#include "VelocityProfile.hpp"

namespace robotics {
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

    // Angular distance (approximate arc length)
    double angle = start.quaternion.angularDistance(end.quaternion);

    // Combine (weighted - position usually more important for timing)
    return posDistance;  // Could add: + angularWeight * angle
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
     * @param startAngle Start angle on arc
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
    // The center lies at intersection of perpendicular bisectors

    Vector3d mid1 = (start + via) / 2.0;
    Vector3d mid2 = (start + end) / 2.0;

    Vector3d dir1 = v1.cross(normal);
    Vector3d dir2 = v2.cross(normal);

    // Solve for intersection parameter
    // mid1 + t1*dir1 = mid2 + t2*dir2 (approximately)

    Vector3d midDiff = mid2 - mid1;
    double denom = dir1.x() * dir2.y() - dir1.y() * dir2.x();

    if (std::abs(denom) < EPSILON) {
        // Fallback: use different components
        denom = dir1.x() * dir2.z() - dir1.z() * dir2.x();
        if (std::abs(denom) < EPSILON) {
            denom = dir1.y() * dir2.z() - dir1.z() * dir2.y();
        }
    }

    if (std::abs(denom) < EPSILON) {
        return false;  // Can't solve
    }

    // Simple approximation: center is equidistant from all three points
    // Using least squares or iterative method for robustness

    // For simplicity, use geometric approach:
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
} // namespace robotics
```

---

## Step 4: Create Trajectory Planner

### 4.1 Create TrajectoryPlanner.hpp

**File:** `src/cpp/include/trajectory/TrajectoryPlanner.hpp`

```cpp
#pragma once

#include "Interpolators.hpp"
#include "kinematics/IKinematicsService.hpp"
#include <memory>

namespace robotics {
namespace trajectory {

// ============================================================================
// Motion Command
// ============================================================================

/**
 * A motion command to execute
 */
struct MotionCommand {
    MotionType type;
    MotionParams params;

    // Target (one of these is used based on motion type)
    JointAngles targetJoints;
    TCPPose targetPose;

    // For CIRC motion
    TCPPose viaPoint;
    bool useViaPoint;

    // Flags
    bool isRelative;  // Relative to current position

    MotionCommand()
        : type(MotionType::PTP),
          targetJoints{0},
          useViaPoint(false),
          isRelative(false) {}
};

// ============================================================================
// Trajectory Planner
// ============================================================================

/**
 * Plans trajectories from motion commands
 */
class TrajectoryPlanner {
public:
    explicit TrajectoryPlanner(
        std::shared_ptr<kinematics::IKinematicsService> kinematics);

    ~TrajectoryPlanner() = default;

    // ========================================================================
    // Planning Methods
    // ========================================================================

    /**
     * Plan trajectory for single motion command
     * @param command Motion command
     * @param currentJoints Current joint positions
     * @return Planned trajectory
     */
    Trajectory planMotion(
        const MotionCommand& command,
        const JointAngles& currentJoints);

    /**
     * Plan trajectory through multiple waypoints
     * @param waypoints List of waypoints
     * @param currentJoints Current joint positions
     * @param params Motion parameters
     * @return Planned trajectory
     */
    Trajectory planPath(
        const std::vector<Waypoint>& waypoints,
        const JointAngles& currentJoints,
        const MotionParams& params);

    /**
     * Plan blended trajectory (continuous motion through waypoints)
     * @param commands List of motion commands
     * @param currentJoints Current joint positions
     * @return Planned trajectory with blending
     */
    Trajectory planBlendedPath(
        const std::vector<MotionCommand>& commands,
        const JointAngles& currentJoints);

    // ========================================================================
    // Configuration
    // ========================================================================

    void setMotionLimits(const MotionLimits& limits) { limits_ = limits; }
    const MotionLimits& getMotionLimits() const { return limits_; }

private:
    std::shared_ptr<kinematics::IKinematicsService> kinematics_;
    MotionLimits limits_;

    // ========================================================================
    // Internal Planning Methods
    // ========================================================================

    /**
     * Plan PTP (joint space) motion
     */
    TrajectorySegment planPTP(
        const JointAngles& start,
        const JointAngles& end,
        const MotionParams& params);

    /**
     * Plan LIN (linear Cartesian) motion
     */
    TrajectorySegment planLIN(
        const JointAngles& startJoints,
        const TCPPose& startPose,
        const TCPPose& endPose,
        const MotionParams& params);

    /**
     * Plan CIRC (circular arc) motion
     */
    TrajectorySegment planCIRC(
        const JointAngles& startJoints,
        const TCPPose& startPose,
        const TCPPose& viaPose,
        const TCPPose& endPose,
        const MotionParams& params);

    /**
     * Calculate segment duration based on limits
     */
    double calculateDuration(
        const TrajectorySegment& segment,
        const MotionParams& params);

    /**
     * Validate trajectory (check IK, limits, etc.)
     */
    bool validateTrajectory(Trajectory& trajectory);
};

// ============================================================================
// Implementation
// ============================================================================

inline TrajectoryPlanner::TrajectoryPlanner(
    std::shared_ptr<kinematics::IKinematicsService> kinematics)
    : kinematics_(kinematics) {
}

inline TrajectorySegment TrajectoryPlanner::planPTP(
    const JointAngles& start,
    const JointAngles& end,
    const MotionParams& params) {

    TrajectorySegment segment;
    segment.motionType = MotionType::PTP;
    segment.params = params;

    // Set waypoints
    segment.start.jointAngles = start;
    segment.start.isJointSpace = true;
    segment.start.cartesianPose = kinematics_->computeFK(start);

    segment.end.jointAngles = end;
    segment.end.isJointSpace = true;
    segment.end.cartesianPose = kinematics_->computeFK(end);

    // Calculate duration based on max joint travel
    double maxTravel = JointInterpolator::calculateMaxJointTravel(start, end);
    double maxVel = limits_.maxJointVelocity[0] * params.velocityScale;
    double maxAccel = limits_.maxJointAcceleration[0] * params.accelerationScale;

    auto profile = createVelocityProfile(params.profile);
    segment.duration = profile->calculateDuration(maxTravel, maxVel, maxAccel);

    // Ensure minimum duration
    if (segment.duration < 0.01) segment.duration = 0.01;

    return segment;
}

inline TrajectorySegment TrajectoryPlanner::planLIN(
    const JointAngles& startJoints,
    const TCPPose& startPose,
    const TCPPose& endPose,
    const MotionParams& params) {

    TrajectorySegment segment;
    segment.motionType = MotionType::LIN;
    segment.params = params;

    // Set waypoints
    segment.start.jointAngles = startJoints;
    segment.start.cartesianPose = startPose;
    segment.start.isJointSpace = false;

    segment.end.cartesianPose = endPose;
    segment.end.isJointSpace = false;

    // Compute IK for end pose
    auto ikResult = kinematics_->computeIK(endPose, startJoints);
    if (ikResult.has_value()) {
        segment.end.jointAngles = ikResult->angles;
    } else {
        // IK failed - trajectory will be invalid
        segment.end.jointAngles = startJoints;
    }

    // Calculate duration based on linear distance
    double distance = LinearInterpolator::calculateDistance(startPose, endPose);
    double maxVel = limits_.maxLinearVelocity * params.velocityScale;
    double maxAccel = limits_.maxLinearAcceleration * params.accelerationScale;

    auto profile = createVelocityProfile(params.profile);
    segment.duration = profile->calculateDuration(distance, maxVel, maxAccel);

    if (segment.duration < 0.01) segment.duration = 0.01;

    return segment;
}

inline TrajectorySegment TrajectoryPlanner::planCIRC(
    const JointAngles& startJoints,
    const TCPPose& startPose,
    const TCPPose& viaPose,
    const TCPPose& endPose,
    const MotionParams& params) {

    TrajectorySegment segment;
    segment.motionType = MotionType::CIRC;
    segment.params = params;

    // Set waypoints
    segment.start.jointAngles = startJoints;
    segment.start.cartesianPose = startPose;
    segment.start.isJointSpace = false;

    segment.end.cartesianPose = endPose;
    segment.end.isJointSpace = false;

    // Compute arc parameters
    bool arcValid = CircularInterpolator::defineArc(
        startPose.position,
        viaPose.position,
        endPose.position,
        segment.circleCenter,
        segment.circleRadius,
        segment.circleNormal);

    if (!arcValid) {
        // Fallback to linear motion
        return planLIN(startJoints, startPose, endPose, params);
    }

    // Compute IK for end pose
    auto ikResult = kinematics_->computeIK(endPose, startJoints);
    if (ikResult.has_value()) {
        segment.end.jointAngles = ikResult->angles;
    }

    // Calculate arc length for duration
    Vector3d startDir = (startPose.position - segment.circleCenter).normalized();
    Vector3d endDir = (endPose.position - segment.circleCenter).normalized();
    double sweepAngle = std::acos(std::clamp(startDir.dot(endDir), -1.0, 1.0));
    double arcLength = segment.circleRadius * sweepAngle;

    double maxVel = limits_.maxLinearVelocity * params.velocityScale;
    double maxAccel = limits_.maxLinearAcceleration * params.accelerationScale;

    auto profile = createVelocityProfile(params.profile);
    segment.duration = profile->calculateDuration(arcLength, maxVel, maxAccel);

    if (segment.duration < 0.01) segment.duration = 0.01;

    return segment;
}

inline Trajectory TrajectoryPlanner::planMotion(
    const MotionCommand& command,
    const JointAngles& currentJoints) {

    Trajectory trajectory;
    trajectory.limits = limits_;

    // Get current pose
    TCPPose currentPose = kinematics_->computeFK(currentJoints);

    TrajectorySegment segment;

    switch (command.type) {
        case MotionType::PTP: {
            JointAngles targetJoints = command.targetJoints;

            // If target is Cartesian, compute IK
            if (!command.isRelative) {
                auto ikResult = kinematics_->computeIK(command.targetPose, currentJoints);
                if (ikResult.has_value()) {
                    targetJoints = ikResult->angles;
                } else {
                    trajectory.isValid = false;
                    trajectory.errorMessage = "IK failed for target pose";
                    return trajectory;
                }
            }

            segment = planPTP(currentJoints, targetJoints, command.params);
            break;
        }

        case MotionType::LIN:
            segment = planLIN(currentJoints, currentPose, command.targetPose, command.params);
            break;

        case MotionType::CIRC:
            if (command.useViaPoint) {
                segment = planCIRC(currentJoints, currentPose,
                                   command.viaPoint, command.targetPose, command.params);
            } else {
                segment = planLIN(currentJoints, currentPose, command.targetPose, command.params);
            }
            break;

        default:
            segment = planPTP(currentJoints, command.targetJoints, command.params);
            break;
    }

    segment.startTime = 0;
    trajectory.segments.push_back(segment);
    trajectory.totalDuration = segment.duration;

    // Validate
    trajectory.isValid = validateTrajectory(trajectory);

    return trajectory;
}

inline Trajectory TrajectoryPlanner::planPath(
    const std::vector<Waypoint>& waypoints,
    const JointAngles& currentJoints,
    const MotionParams& params) {

    Trajectory trajectory;
    trajectory.limits = limits_;

    if (waypoints.empty()) {
        trajectory.isValid = false;
        trajectory.errorMessage = "No waypoints provided";
        return trajectory;
    }

    JointAngles prevJoints = currentJoints;
    double currentTime = 0;

    for (size_t i = 0; i < waypoints.size(); ++i) {
        const auto& wp = waypoints[i];

        TrajectorySegment segment;

        if (wp.isJointSpace) {
            segment = planPTP(prevJoints, wp.jointAngles, params);
        } else {
            TCPPose prevPose = kinematics_->computeFK(prevJoints);
            segment = planLIN(prevJoints, prevPose, wp.cartesianPose, params);
        }

        segment.startTime = currentTime;
        currentTime += segment.duration;

        trajectory.segments.push_back(segment);
        prevJoints = segment.end.jointAngles;
    }

    trajectory.totalDuration = currentTime;
    trajectory.isValid = validateTrajectory(trajectory);

    return trajectory;
}

inline Trajectory TrajectoryPlanner::planBlendedPath(
    const std::vector<MotionCommand>& commands,
    const JointAngles& currentJoints) {

    Trajectory trajectory;
    trajectory.limits = limits_;

    if (commands.empty()) {
        trajectory.isValid = false;
        return trajectory;
    }

    // First, plan each segment independently
    std::vector<TrajectorySegment> segments;
    JointAngles prevJoints = currentJoints;

    for (const auto& cmd : commands) {
        Trajectory singleTraj = planMotion(cmd, prevJoints);
        if (!singleTraj.segments.empty()) {
            segments.push_back(singleTraj.segments[0]);
            prevJoints = singleTraj.segments[0].end.jointAngles;
        }
    }

    // Apply blending between segments
    double currentTime = 0;
    for (size_t i = 0; i < segments.size(); ++i) {
        auto& seg = segments[i];
        seg.startTime = currentTime;

        // Adjust for blending (reduce duration at corners)
        if (i > 0 && seg.params.blendRadius > 0) {
            // Simplified blending - just reduce segment time
            double blendTime = 0.1;  // 100ms blend
            if (seg.duration > 2 * blendTime) {
                // Could adjust timing here for smooth blending
            }
        }

        currentTime += seg.duration;
        trajectory.segments.push_back(seg);
    }

    trajectory.totalDuration = currentTime;
    trajectory.isValid = validateTrajectory(trajectory);

    return trajectory;
}

inline bool TrajectoryPlanner::validateTrajectory(Trajectory& trajectory) {
    if (trajectory.segments.empty()) {
        trajectory.errorMessage = "No segments in trajectory";
        return false;
    }

    // Check each segment
    for (size_t i = 0; i < trajectory.segments.size(); ++i) {
        const auto& seg = trajectory.segments[i];

        // Check joint limits
        if (!kinematics_->areJointAnglesValid(seg.start.jointAngles)) {
            trajectory.errorMessage = "Start joints out of limits at segment " + std::to_string(i);
            return false;
        }

        if (!kinematics_->areJointAnglesValid(seg.end.jointAngles)) {
            trajectory.errorMessage = "End joints out of limits at segment " + std::to_string(i);
            return false;
        }

        // Check duration
        if (seg.duration <= 0) {
            trajectory.errorMessage = "Invalid duration at segment " + std::to_string(i);
            return false;
        }

        // For LIN/CIRC, sample along path to check IK
        if (seg.motionType == MotionType::LIN || seg.motionType == MotionType::CIRC) {
            const int numSamples = 10;
            for (int s = 1; s < numSamples; ++s) {
                double t = static_cast<double>(s) / numSamples;
                TCPPose pose = LinearInterpolator::interpolate(
                    seg.start.cartesianPose, seg.end.cartesianPose, t);

                JointAngles prevAngles = (s == 1) ? seg.start.jointAngles :
                    JointInterpolator::linearInterpolate(
                        seg.start.jointAngles, seg.end.jointAngles,
                        static_cast<double>(s-1) / numSamples);

                auto ikResult = kinematics_->computeIK(pose, prevAngles);
                if (!ikResult.has_value()) {
                    trajectory.errorMessage = "IK failed along path at segment " +
                        std::to_string(i) + ", t=" + std::to_string(t);
                    return false;
                }
            }
        }
    }

    return true;
}

} // namespace trajectory
} // namespace robotics
```

---

## Step 5: Create Trajectory Executor

### 5.1 Create TrajectoryExecutor.hpp

**File:** `src/cpp/include/trajectory/TrajectoryExecutor.hpp`

```cpp
#pragma once

#include "TrajectoryPlanner.hpp"
#include <functional>
#include <atomic>
#include <thread>
#include <chrono>

namespace robotics {
namespace trajectory {

// ============================================================================
// Executor Callback Types
// ============================================================================

using TrajectoryPointCallback = std::function<void(const TrajectoryPoint&)>;
using TrajectoryCompleteCallback = std::function<void(bool success, const std::string& message)>;
using TrajectoryErrorCallback = std::function<void(const std::string& error)>;

// ============================================================================
// Trajectory Executor
// ============================================================================

/**
 * Executes trajectories by sampling at control rate
 */
class TrajectoryExecutor {
public:
    explicit TrajectoryExecutor(
        std::shared_ptr<kinematics::IKinematicsService> kinematics);

    ~TrajectoryExecutor();

    // ========================================================================
    // Execution Control
    // ========================================================================

    /**
     * Start executing trajectory
     * @param trajectory Trajectory to execute
     * @return true if started successfully
     */
    bool start(const Trajectory& trajectory);

    /**
     * Pause execution
     */
    void pause();

    /**
     * Resume execution
     */
    void resume();

    /**
     * Stop execution immediately
     */
    void stop();

    /**
     * Get current state
     */
    TrajectoryState getState() const { return state_.load(); }

    /**
     * Get current trajectory time
     */
    double getCurrentTime() const { return currentTime_.load(); }

    /**
     * Get progress (0.0 - 1.0)
     */
    double getProgress() const;

    // ========================================================================
    // Sampling
    // ========================================================================

    /**
     * Sample trajectory at given time
     * Used for real-time control loop
     */
    TrajectoryPoint sample(double time) const;

    /**
     * Sample next point (advances internal time)
     * Call this at control loop rate
     */
    TrajectoryPoint sampleNext();

    // ========================================================================
    // Configuration
    // ========================================================================

    void setControlRate(double rateHz) { controlRate_ = rateHz; }
    double getControlRate() const { return controlRate_; }

    void setPointCallback(TrajectoryPointCallback callback) {
        pointCallback_ = callback;
    }

    void setCompleteCallback(TrajectoryCompleteCallback callback) {
        completeCallback_ = callback;
    }

    void setErrorCallback(TrajectoryErrorCallback callback) {
        errorCallback_ = callback;
    }

private:
    std::shared_ptr<kinematics::IKinematicsService> kinematics_;

    Trajectory trajectory_;
    double controlRate_;              // Hz

    std::atomic<TrajectoryState> state_;
    std::atomic<double> currentTime_;
    std::atomic<bool> stopRequested_;

    TrajectoryPointCallback pointCallback_;
    TrajectoryCompleteCallback completeCallback_;
    TrajectoryErrorCallback errorCallback_;

    std::thread executionThread_;

    /**
     * Internal execution loop
     */
    void executionLoop();

    /**
     * Sample segment at local time
     */
    TrajectoryPoint sampleSegment(
        const TrajectorySegment& segment,
        double localTime) const;
};

// ============================================================================
// Implementation
// ============================================================================

inline TrajectoryExecutor::TrajectoryExecutor(
    std::shared_ptr<kinematics::IKinematicsService> kinematics)
    : kinematics_(kinematics),
      controlRate_(1000.0),  // 1kHz default
      state_(TrajectoryState::IDLE),
      currentTime_(0),
      stopRequested_(false) {
}

inline TrajectoryExecutor::~TrajectoryExecutor() {
    stop();
    if (executionThread_.joinable()) {
        executionThread_.join();
    }
}

inline bool TrajectoryExecutor::start(const Trajectory& trajectory) {
    if (state_.load() == TrajectoryState::RUNNING) {
        return false;  // Already running
    }

    if (!trajectory.isValid) {
        if (errorCallback_) {
            errorCallback_(trajectory.errorMessage);
        }
        return false;
    }

    // Stop any existing execution
    stop();
    if (executionThread_.joinable()) {
        executionThread_.join();
    }

    trajectory_ = trajectory;
    currentTime_.store(0);
    stopRequested_.store(false);
    state_.store(TrajectoryState::RUNNING);

    // Start execution thread
    executionThread_ = std::thread(&TrajectoryExecutor::executionLoop, this);

    return true;
}

inline void TrajectoryExecutor::pause() {
    if (state_.load() == TrajectoryState::RUNNING) {
        state_.store(TrajectoryState::PAUSED);
    }
}

inline void TrajectoryExecutor::resume() {
    if (state_.load() == TrajectoryState::PAUSED) {
        state_.store(TrajectoryState::RUNNING);
    }
}

inline void TrajectoryExecutor::stop() {
    stopRequested_.store(true);
    state_.store(TrajectoryState::IDLE);
}

inline double TrajectoryExecutor::getProgress() const {
    if (trajectory_.totalDuration <= 0) return 0;
    return std::clamp(currentTime_.load() / trajectory_.totalDuration, 0.0, 1.0);
}

inline TrajectoryPoint TrajectoryExecutor::sample(double time) const {
    TrajectoryPoint point;
    point.time = time;

    if (trajectory_.segments.empty() || !trajectory_.isValid) {
        point.isValid = false;
        return point;
    }

    // Find active segment
    int segmentIndex = -1;
    for (size_t i = 0; i < trajectory_.segments.size(); ++i) {
        const auto& seg = trajectory_.segments[i];
        if (time >= seg.startTime && time < seg.startTime + seg.duration) {
            segmentIndex = static_cast<int>(i);
            break;
        }
    }

    // Check if past end
    if (segmentIndex < 0) {
        if (time >= trajectory_.totalDuration && !trajectory_.segments.empty()) {
            // At or past end - return final position
            const auto& lastSeg = trajectory_.segments.back();
            point.jointPositions = lastSeg.end.jointAngles;
            point.cartesianPose = lastSeg.end.cartesianPose;
            point.segmentIndex = trajectory_.segments.size() - 1;
            point.segmentProgress = 1.0;
            point.isValid = true;

            // Zero velocities at end
            for (auto& v : point.jointVelocities) v = 0;
            point.cartesianVelocity = Vector6d::Zero();
        }
        return point;
    }

    // Sample the segment
    const auto& segment = trajectory_.segments[segmentIndex];
    double localTime = time - segment.startTime;

    point = sampleSegment(segment, localTime);
    point.time = time;
    point.segmentIndex = segmentIndex;
    point.segmentProgress = localTime / segment.duration;

    return point;
}

inline TrajectoryPoint TrajectoryExecutor::sampleSegment(
    const TrajectorySegment& segment,
    double localTime) const {

    TrajectoryPoint point;

    double t_norm = std::clamp(localTime / segment.duration, 0.0, 1.0);

    // Get velocity profile value
    auto profile = createVelocityProfile(segment.params.profile);
    auto profileValue = profile->compute(localTime, segment.duration);

    // Use profile position for interpolation
    double s = profileValue.position;

    switch (segment.motionType) {
        case MotionType::PTP: {
            // Joint space interpolation
            point.jointPositions = JointInterpolator::linearInterpolate(
                segment.start.jointAngles,
                segment.end.jointAngles,
                s);

            // Compute velocities
            double vel_scale = profileValue.velocity / segment.duration;
            for (int i = 0; i < NUM_JOINTS; ++i) {
                double delta = segment.end.jointAngles[i] - segment.start.jointAngles[i];
                point.jointVelocities[i] = delta * vel_scale;
            }

            // Compute Cartesian pose from joints
            point.cartesianPose = kinematics_->computeFK(point.jointPositions);
            break;
        }

        case MotionType::LIN: {
            // Cartesian space interpolation
            point.cartesianPose = LinearInterpolator::interpolate(
                segment.start.cartesianPose,
                segment.end.cartesianPose,
                s);

            // Compute IK for joint positions
            auto ikResult = kinematics_->computeIK(
                point.cartesianPose,
                segment.start.jointAngles);

            if (ikResult.has_value()) {
                point.jointPositions = ikResult->angles;
            } else {
                // Fallback to linear joint interpolation
                point.jointPositions = JointInterpolator::linearInterpolate(
                    segment.start.jointAngles,
                    segment.end.jointAngles,
                    s);
            }

            // Compute Cartesian velocity
            double vel_scale = profileValue.velocity / segment.duration;
            Vector3d posDelta = segment.end.cartesianPose.position -
                               segment.start.cartesianPose.position;
            point.cartesianVelocity.head<3>() = posDelta * vel_scale;

            // Angular velocity (approximate)
            Vector3d rpyDelta = segment.end.cartesianPose.rpy -
                               segment.start.cartesianPose.rpy;
            point.cartesianVelocity.tail<3>() = rpyDelta * vel_scale;
            break;
        }

        case MotionType::CIRC: {
            // Circular interpolation
            Vector3d startDir = (segment.start.cartesianPose.position -
                                segment.circleCenter).normalized();

            Vector3d endDir = (segment.end.cartesianPose.position -
                              segment.circleCenter).normalized();
            double sweepAngle = std::acos(std::clamp(startDir.dot(endDir), -1.0, 1.0));

            Vector3d position = CircularInterpolator::interpolateArc(
                segment.circleCenter,
                segment.circleRadius,
                segment.circleNormal,
                startDir,
                sweepAngle,
                s);

            point.cartesianPose.position = position;

            // Interpolate orientation
            point.cartesianPose.quaternion = segment.start.cartesianPose.quaternion.slerp(
                s, segment.end.cartesianPose.quaternion);
            point.cartesianPose.rotation = point.cartesianPose.quaternion.toRotationMatrix();
            point.cartesianPose.rpy = rotationToRPY(point.cartesianPose.rotation);

            // Compute IK
            auto ikResult = kinematics_->computeIK(
                point.cartesianPose,
                segment.start.jointAngles);

            if (ikResult.has_value()) {
                point.jointPositions = ikResult->angles;
            }
            break;
        }

        default:
            point.jointPositions = segment.start.jointAngles;
            point.cartesianPose = segment.start.cartesianPose;
            break;
    }

    point.isValid = true;
    return point;
}

inline TrajectoryPoint TrajectoryExecutor::sampleNext() {
    double dt = 1.0 / controlRate_;
    double time = currentTime_.load();

    if (state_.load() == TrajectoryState::RUNNING) {
        time += dt;
        currentTime_.store(time);
    }

    return sample(time);
}

inline void TrajectoryExecutor::executionLoop() {
    auto startTime = std::chrono::high_resolution_clock::now();
    double dt = 1.0 / controlRate_;

    while (!stopRequested_.load()) {
        auto state = state_.load();

        if (state == TrajectoryState::RUNNING) {
            double time = currentTime_.load();

            // Sample trajectory
            TrajectoryPoint point = sample(time);

            // Call callback
            if (pointCallback_ && point.isValid) {
                pointCallback_(point);
            }

            // Check if complete
            if (time >= trajectory_.totalDuration) {
                state_.store(TrajectoryState::COMPLETED);
                if (completeCallback_) {
                    completeCallback_(true, "Trajectory completed");
                }
                break;
            }

            // Advance time
            time += dt;
            currentTime_.store(time);
        }
        else if (state == TrajectoryState::PAUSED) {
            // Don't advance time, just wait
        }
        else {
            // IDLE, COMPLETED, or ERROR - exit loop
            break;
        }

        // Sleep to maintain control rate
        std::this_thread::sleep_for(
            std::chrono::microseconds(static_cast<int>(dt * 1000000)));
    }
}

} // namespace trajectory
} // namespace robotics
```

---

## Step 6: Create Trajectory Implementation (Sampling)

### 6.1 Implement Trajectory::sample

**Add to TrajectoryTypes.hpp (after class declaration):**

```cpp
// ============================================================================
// Trajectory::sample Implementation
// ============================================================================

inline TrajectoryPoint Trajectory::sample(double time) const {
    TrajectoryPoint point;
    point.time = time;

    if (segments.empty() || !isValid) {
        point.isValid = false;
        return point;
    }

    // Clamp time
    time = std::clamp(time, 0.0, totalDuration);

    // Find segment
    int idx = getSegmentAt(time);
    if (idx < 0) {
        point.isValid = false;
        return point;
    }

    const auto& seg = segments[idx];
    double localTime = time - seg.startTime;
    double t = localTime / seg.duration;

    // Linear interpolation for now
    point.jointPositions = JointInterpolator::linearInterpolate(
        seg.start.jointAngles, seg.end.jointAngles, t);

    point.segmentIndex = idx;
    point.segmentProgress = t;
    point.isValid = true;

    return point;
}

inline int Trajectory::getSegmentAt(double time) const {
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg = segments[i];
        if (time >= seg.startTime && time < seg.startTime + seg.duration) {
            return static_cast<int>(i);
        }
    }

    // Check if at very end
    if (!segments.empty() && time >= totalDuration - EPSILON) {
        return static_cast<int>(segments.size() - 1);
    }

    return -1;
}
```

---

## Step 7: Create Unit Tests

### 7.1 Create test_trajectory.cpp

**File:** `src/cpp/tests/test_trajectory.cpp`

```cpp
#include <gtest/gtest.h>
#include "trajectory/TrajectoryTypes.hpp"
#include "trajectory/VelocityProfile.hpp"
#include "trajectory/Interpolators.hpp"
#include "trajectory/TrajectoryPlanner.hpp"
#include "trajectory/TrajectoryExecutor.hpp"
#include "kinematics/IKinematicsService.hpp"

using namespace robotics::trajectory;
using namespace robotics::kinematics;
using namespace robotics::math;

class TrajectoryTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto config = createDefault6DOFConfig();
        kinematics_ = std::make_shared<KinematicsService>(config);
        planner_ = std::make_unique<TrajectoryPlanner>(kinematics_);
    }

    std::shared_ptr<KinematicsService> kinematics_;
    std::unique_ptr<TrajectoryPlanner> planner_;
};

// ============================================================================
// Velocity Profile Tests
// ============================================================================

TEST(VelocityProfileTest, TrapezoidalBasic) {
    TrapezoidalProfile profile;

    // At start
    auto result = profile.compute(0, 1.0);
    EXPECT_NEAR(result.position, 0, 0.01);
    EXPECT_NEAR(result.velocity, 0, 0.01);

    // At end
    result = profile.compute(1.0, 1.0);
    EXPECT_NEAR(result.position, 1.0, 0.01);
    EXPECT_NEAR(result.velocity, 0, 0.01);

    // In middle (should have velocity)
    result = profile.compute(0.5, 1.0);
    EXPECT_GT(result.velocity, 0);
}

TEST(VelocityProfileTest, TrapezoidalDuration) {
    TrapezoidalProfile profile;

    double duration = profile.calculateDuration(100.0, 50.0, 100.0);
    EXPECT_GT(duration, 0);

    // Should take at least time to accelerate and decelerate
    double minDuration = 2.0 * 50.0 / 100.0;  // 2 * v_max / a_max
    EXPECT_GE(duration, minDuration * 0.9);  // Allow some tolerance
}

TEST(VelocityProfileTest, SCurveSmooth) {
    SCurveProfile profile;

    // At start
    auto result = profile.compute(0, 1.0);
    EXPECT_NEAR(result.position, 0, 0.01);

    // At end
    result = profile.compute(1.0, 1.0);
    EXPECT_NEAR(result.position, 1.0, 0.01);

    // Check smoothness - velocity should be continuous
    double prev_vel = 0;
    for (int i = 1; i < 100; ++i) {
        double t = i * 0.01;
        result = profile.compute(t, 1.0);
        // Velocity change should be limited (smooth)
        EXPECT_LT(std::abs(result.velocity - prev_vel), 0.2);
        prev_vel = result.velocity;
    }
}

// ============================================================================
// Interpolator Tests
// ============================================================================

TEST(InterpolatorTest, LinearPose) {
    TCPPose start, end;
    start.position = Vector3d(0, 0, 0);
    start.rotation = Matrix3d::Identity();
    start.quaternion = Quaterniond::Identity();

    end.position = Vector3d(100, 0, 0);
    end.rotation = Matrix3d::Identity();
    end.quaternion = Quaterniond::Identity();

    // At start
    auto result = LinearInterpolator::interpolate(start, end, 0);
    EXPECT_NEAR(result.position.x(), 0, 0.01);

    // At end
    result = LinearInterpolator::interpolate(start, end, 1.0);
    EXPECT_NEAR(result.position.x(), 100, 0.01);

    // In middle
    result = LinearInterpolator::interpolate(start, end, 0.5);
    EXPECT_NEAR(result.position.x(), 50, 0.01);
}

TEST(InterpolatorTest, JointLinear) {
    JointAngles start = {0, 0, 0, 0, 0, 0};
    JointAngles end = {1, 2, 3, 4, 5, 6};

    auto result = JointInterpolator::linearInterpolate(start, end, 0.5);

    EXPECT_NEAR(result[0], 0.5, 0.01);
    EXPECT_NEAR(result[1], 1.0, 0.01);
    EXPECT_NEAR(result[5], 3.0, 0.01);
}

TEST(InterpolatorTest, JointCubic) {
    JointAngles start = {0, 0, 0, 0, 0, 0};
    JointAngles end = {1, 1, 1, 1, 1, 1};
    JointVelocities startVel = {0, 0, 0, 0, 0, 0};
    JointVelocities endVel = {0, 0, 0, 0, 0, 0};

    // With zero boundary velocities, should still reach endpoints
    auto result = JointInterpolator::cubicInterpolate(
        start, end, startVel, endVel, 0, 1.0);
    EXPECT_NEAR(result[0], 0, 0.01);

    result = JointInterpolator::cubicInterpolate(
        start, end, startVel, endVel, 1.0, 1.0);
    EXPECT_NEAR(result[0], 1.0, 0.01);
}

TEST(InterpolatorTest, CircularArc) {
    Vector3d start(100, 0, 0);
    Vector3d via(0, 100, 0);
    Vector3d end(-100, 0, 0);

    Vector3d center;
    double radius;
    Vector3d normal;

    bool success = CircularInterpolator::defineArc(start, via, end, center, radius, normal);

    EXPECT_TRUE(success);
    EXPECT_NEAR(radius, 100, 1.0);
    EXPECT_NEAR(center.norm(), 0, 1.0);  // Center should be at origin
}

// ============================================================================
// Trajectory Planner Tests
// ============================================================================

TEST_F(TrajectoryTest, PlanPTP) {
    MotionCommand cmd;
    cmd.type = MotionType::PTP;
    cmd.targetJoints = {0.5, -0.3, 0.8, 0.2, -0.4, 0.1};
    cmd.params.velocityScale = 0.5;

    JointAngles current = {0, 0, 0, 0, 0, 0};

    Trajectory traj = planner_->planMotion(cmd, current);

    EXPECT_TRUE(traj.isValid);
    EXPECT_EQ(traj.segments.size(), 1);
    EXPECT_GT(traj.totalDuration, 0);
}

TEST_F(TrajectoryTest, PlanLIN) {
    MotionCommand cmd;
    cmd.type = MotionType::LIN;
    cmd.params.velocityScale = 0.3;

    JointAngles current = {0, -PI/4, PI/2, 0, PI/4, 0};

    // Get current pose
    TCPPose currentPose = kinematics_->computeFK(current);

    // Target 100mm forward
    cmd.targetPose = currentPose;
    cmd.targetPose.position.x() += 100;

    Trajectory traj = planner_->planMotion(cmd, current);

    EXPECT_TRUE(traj.isValid);
    EXPECT_GT(traj.totalDuration, 0);
}

TEST_F(TrajectoryTest, PlanPath) {
    JointAngles current = {0, 0, 0, 0, 0, 0};

    std::vector<Waypoint> waypoints;

    Waypoint wp1;
    wp1.isJointSpace = true;
    wp1.jointAngles = {0.2, -0.1, 0.3, 0, 0, 0};
    waypoints.push_back(wp1);

    Waypoint wp2;
    wp2.isJointSpace = true;
    wp2.jointAngles = {0.4, -0.2, 0.6, 0.1, 0, 0};
    waypoints.push_back(wp2);

    Waypoint wp3;
    wp3.isJointSpace = true;
    wp3.jointAngles = {0, 0, 0, 0, 0, 0};
    waypoints.push_back(wp3);

    MotionParams params;
    params.velocityScale = 0.5;

    Trajectory traj = planner_->planPath(waypoints, current, params);

    EXPECT_TRUE(traj.isValid);
    EXPECT_EQ(traj.segments.size(), 3);
}

// ============================================================================
// Trajectory Sampling Tests
// ============================================================================

TEST_F(TrajectoryTest, SampleTrajectory) {
    MotionCommand cmd;
    cmd.type = MotionType::PTP;
    cmd.targetJoints = {0.5, 0, 0, 0, 0, 0};

    JointAngles current = {0, 0, 0, 0, 0, 0};

    Trajectory traj = planner_->planMotion(cmd, current);
    ASSERT_TRUE(traj.isValid);

    // Sample at start
    auto point = traj.sample(0);
    EXPECT_TRUE(point.isValid);
    EXPECT_NEAR(point.jointPositions[0], 0, 0.01);

    // Sample at end
    point = traj.sample(traj.totalDuration);
    EXPECT_TRUE(point.isValid);
    EXPECT_NEAR(point.jointPositions[0], 0.5, 0.01);

    // Sample in middle
    point = traj.sample(traj.totalDuration / 2);
    EXPECT_TRUE(point.isValid);
    EXPECT_GT(point.jointPositions[0], 0);
    EXPECT_LT(point.jointPositions[0], 0.5);
}

// ============================================================================
// Trajectory Executor Tests
// ============================================================================

TEST_F(TrajectoryTest, ExecutorBasic) {
    TrajectoryExecutor executor(kinematics_);

    EXPECT_EQ(executor.getState(), TrajectoryState::IDLE);

    MotionCommand cmd;
    cmd.type = MotionType::PTP;
    cmd.targetJoints = {0.1, 0, 0, 0, 0, 0};

    JointAngles current = {0, 0, 0, 0, 0, 0};
    Trajectory traj = planner_->planMotion(cmd, current);

    // Start execution
    bool started = executor.start(traj);
    EXPECT_TRUE(started);
    EXPECT_EQ(executor.getState(), TrajectoryState::RUNNING);

    // Stop
    executor.stop();
    EXPECT_EQ(executor.getState(), TrajectoryState::IDLE);
}

TEST_F(TrajectoryTest, ExecutorSampling) {
    TrajectoryExecutor executor(kinematics_);
    executor.setControlRate(100);  // 100 Hz for testing

    MotionCommand cmd;
    cmd.type = MotionType::PTP;
    cmd.targetJoints = {0.5, 0, 0, 0, 0, 0};

    JointAngles current = {0, 0, 0, 0, 0, 0};
    Trajectory traj = planner_->planMotion(cmd, current);

    executor.start(traj);

    // Sample multiple points
    std::vector<TrajectoryPoint> points;
    for (int i = 0; i < 10; ++i) {
        auto point = executor.sampleNext();
        points.push_back(point);
    }

    // Should be progressing
    EXPECT_GT(executor.getCurrentTime(), 0);

    executor.stop();
}

// ============================================================================
// Motion Limits Tests
// ============================================================================

TEST(MotionLimitsTest, DefaultValues) {
    MotionLimits limits;

    EXPECT_GT(limits.maxLinearVelocity, 0);
    EXPECT_GT(limits.maxLinearAcceleration, 0);

    for (int i = 0; i < NUM_JOINTS; ++i) {
        EXPECT_GT(limits.maxJointVelocity[i], 0);
        EXPECT_GT(limits.maxJointAcceleration[i], 0);
    }
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
```

---

## Step 8: Update CMakeLists.txt

### 8.1 Add Trajectory to Build

**File:** `src/cpp/CMakeLists.txt`

**Add to include directories:**
```cmake
target_include_directories(RobotCore
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/kinematics
    ${CMAKE_CURRENT_SOURCE_DIR}/include/trajectory
)
```

**Add test executable:**
```cmake
# Trajectory tests
add_executable(test_trajectory tests/test_trajectory.cpp)
target_link_libraries(test_trajectory
    PRIVATE
    GTest::gtest
    GTest::gtest_main
    Eigen3::Eigen
)
target_include_directories(test_trajectory PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME TrajectoryTests COMMAND test_trajectory)
```

**Validation:**
```powershell
cd E:\DEV_CONTEXT_PROJECTs\Robot_controller
cmake --build build --config Debug
.\build\Debug\test_trajectory.exe
```

---

## Step 9: Create IPC Message Types for Trajectory

### 9.1 Create TrajectoryPayloads.hpp

**File:** `src/cpp/include/ipc/TrajectoryPayloads.hpp`

```cpp
#pragma once

#include <nlohmann/json.hpp>
#include <array>
#include <vector>
#include <string>

namespace robotics {
namespace ipc {

// ============================================================================
// Motion Command Request
// ============================================================================

struct MotionCommandRequest {
    std::string motionType;  // "PTP", "LIN", "CIRC"
    std::array<double, 6> targetJoints;
    std::array<double, 3> targetPosition;
    std::array<double, 3> targetRPY;
    std::array<double, 3> viaPosition;  // For CIRC
    double velocityScale;
    double accelerationScale;
    std::string velocityProfile;  // "TRAPEZOIDAL", "SCURVE"

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(MotionCommandRequest,
        motionType, targetJoints, targetPosition, targetRPY,
        viaPosition, velocityScale, accelerationScale, velocityProfile)
};

struct MotionCommandResponse {
    bool success;
    double duration;
    std::string error;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(MotionCommandResponse, success, duration, error)
};

// ============================================================================
// Trajectory Status
// ============================================================================

struct TrajectoryStatusRequest {
    bool dummy;  // Just to have a field

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(TrajectoryStatusRequest, dummy)
};

struct TrajectoryStatusResponse {
    std::string state;  // "IDLE", "RUNNING", "PAUSED", "COMPLETED", "ERROR"
    double currentTime;
    double totalDuration;
    double progress;
    int currentSegment;
    std::array<double, 6> currentJoints;
    std::array<double, 3> currentPosition;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(TrajectoryStatusResponse,
        state, currentTime, totalDuration, progress,
        currentSegment, currentJoints, currentPosition)
};

// ============================================================================
// Trajectory Control
// ============================================================================

struct TrajectoryControlRequest {
    std::string action;  // "PAUSE", "RESUME", "STOP"

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(TrajectoryControlRequest, action)
};

struct TrajectoryControlResponse {
    bool success;
    std::string newState;
    std::string error;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(TrajectoryControlResponse, success, newState, error)
};

// ============================================================================
// Waypoint Path
// ============================================================================

struct WaypointData {
    bool isJointSpace;
    std::array<double, 6> jointAngles;
    std::array<double, 3> position;
    std::array<double, 3> rpy;
    bool isViaPoint;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(WaypointData,
        isJointSpace, jointAngles, position, rpy, isViaPoint)
};

struct PathPlanRequest {
    std::vector<WaypointData> waypoints;
    double velocityScale;
    double accelerationScale;
    double blendRadius;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(PathPlanRequest,
        waypoints, velocityScale, accelerationScale, blendRadius)
};

struct PathPlanResponse {
    bool success;
    double totalDuration;
    int segmentCount;
    std::string error;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(PathPlanResponse,
        success, totalDuration, segmentCount, error)
};

} // namespace ipc
} // namespace robotics
```

### 9.2 Update MessageTypes.hpp

**Add new message types:**
```cpp
// Trajectory messages
MOTION_COMMAND_REQUEST = 40,
MOTION_COMMAND_RESPONSE = 41,
TRAJECTORY_STATUS_REQUEST = 42,
TRAJECTORY_STATUS_RESPONSE = 43,
TRAJECTORY_CONTROL_REQUEST = 44,
TRAJECTORY_CONTROL_RESPONSE = 45,
PATH_PLAN_REQUEST = 46,
PATH_PLAN_RESPONSE = 47,
```

---

## Step 10: Create C# Trajectory Client

### 10.1 Create TrajectoryPayloads.cs

**File:** `src/csharp/RobotController.Core/IPC/TrajectoryPayloads.cs`

```csharp
using System.Text.Json.Serialization;

namespace RobotController.Core.IPC;

// ============================================================================
// Motion Command
// ============================================================================

public record MotionCommandRequest
{
    [JsonPropertyName("motionType")]
    public string MotionType { get; init; } = "PTP";

    [JsonPropertyName("targetJoints")]
    public double[] TargetJoints { get; init; } = new double[6];

    [JsonPropertyName("targetPosition")]
    public double[] TargetPosition { get; init; } = new double[3];

    [JsonPropertyName("targetRPY")]
    public double[] TargetRPY { get; init; } = new double[3];

    [JsonPropertyName("viaPosition")]
    public double[] ViaPosition { get; init; } = new double[3];

    [JsonPropertyName("velocityScale")]
    public double VelocityScale { get; init; } = 0.5;

    [JsonPropertyName("accelerationScale")]
    public double AccelerationScale { get; init; } = 0.5;

    [JsonPropertyName("velocityProfile")]
    public string VelocityProfile { get; init; } = "TRAPEZOIDAL";
}

public record MotionCommandResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; init; }

    [JsonPropertyName("duration")]
    public double Duration { get; init; }

    [JsonPropertyName("error")]
    public string Error { get; init; } = "";
}

// ============================================================================
// Trajectory Status
// ============================================================================

public record TrajectoryStatusResponse
{
    [JsonPropertyName("state")]
    public string State { get; init; } = "IDLE";

    [JsonPropertyName("currentTime")]
    public double CurrentTime { get; init; }

    [JsonPropertyName("totalDuration")]
    public double TotalDuration { get; init; }

    [JsonPropertyName("progress")]
    public double Progress { get; init; }

    [JsonPropertyName("currentSegment")]
    public int CurrentSegment { get; init; }

    [JsonPropertyName("currentJoints")]
    public double[] CurrentJoints { get; init; } = new double[6];

    [JsonPropertyName("currentPosition")]
    public double[] CurrentPosition { get; init; } = new double[3];
}

// ============================================================================
// Trajectory Control
// ============================================================================

public record TrajectoryControlRequest
{
    [JsonPropertyName("action")]
    public string Action { get; init; } = "";
}

public record TrajectoryControlResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; init; }

    [JsonPropertyName("newState")]
    public string NewState { get; init; } = "";

    [JsonPropertyName("error")]
    public string Error { get; init; } = "";
}

// ============================================================================
// Path Planning
// ============================================================================

public record WaypointData
{
    [JsonPropertyName("isJointSpace")]
    public bool IsJointSpace { get; init; }

    [JsonPropertyName("jointAngles")]
    public double[] JointAngles { get; init; } = new double[6];

    [JsonPropertyName("position")]
    public double[] Position { get; init; } = new double[3];

    [JsonPropertyName("rpy")]
    public double[] RPY { get; init; } = new double[3];

    [JsonPropertyName("isViaPoint")]
    public bool IsViaPoint { get; init; }
}

public record PathPlanRequest
{
    [JsonPropertyName("waypoints")]
    public WaypointData[] Waypoints { get; init; } = Array.Empty<WaypointData>();

    [JsonPropertyName("velocityScale")]
    public double VelocityScale { get; init; } = 0.5;

    [JsonPropertyName("accelerationScale")]
    public double AccelerationScale { get; init; } = 0.5;

    [JsonPropertyName("blendRadius")]
    public double BlendRadius { get; init; }
}

public record PathPlanResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; init; }

    [JsonPropertyName("totalDuration")]
    public double TotalDuration { get; init; }

    [JsonPropertyName("segmentCount")]
    public int SegmentCount { get; init; }

    [JsonPropertyName("error")]
    public string Error { get; init; } = "";
}
```

### 10.2 Create ITrajectoryClientService.cs

**File:** `src/csharp/RobotController.Core/Services/ITrajectoryClientService.cs`

```csharp
using RobotController.Core.IPC;

namespace RobotController.Core.Services;

/// <summary>
/// Client service for trajectory planning and execution
/// </summary>
public interface ITrajectoryClientService
{
    /// <summary>
    /// Send motion command (PTP, LIN, CIRC)
    /// </summary>
    Task<MotionCommandResponse> SendMotionCommandAsync(
        MotionCommandRequest request,
        CancellationToken ct = default);

    /// <summary>
    /// Get current trajectory status
    /// </summary>
    Task<TrajectoryStatusResponse> GetStatusAsync(CancellationToken ct = default);

    /// <summary>
    /// Control trajectory (pause, resume, stop)
    /// </summary>
    Task<TrajectoryControlResponse> ControlAsync(
        string action,
        CancellationToken ct = default);

    /// <summary>
    /// Plan path through waypoints
    /// </summary>
    Task<PathPlanResponse> PlanPathAsync(
        PathPlanRequest request,
        CancellationToken ct = default);

    // Events
    event EventHandler<TrajectoryStatusResponse>? StatusChanged;
}

public class TrajectoryClientService : ITrajectoryClientService
{
    private readonly IIpcClientService _ipc;
    private readonly ILogger<TrajectoryClientService> _logger;

    public event EventHandler<TrajectoryStatusResponse>? StatusChanged;

    public TrajectoryClientService(
        IIpcClientService ipc,
        ILogger<TrajectoryClientService> logger)
    {
        _ipc = ipc;
        _logger = logger;
    }

    public async Task<MotionCommandResponse> SendMotionCommandAsync(
        MotionCommandRequest request,
        CancellationToken ct = default)
    {
        try
        {
            var response = await _ipc.SendRequestAsync<MotionCommandRequest, MotionCommandResponse>(
                MessageType.MOTION_COMMAND_REQUEST,
                request,
                ct);

            return response ?? new MotionCommandResponse { Success = false, Error = "No response" };
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Motion command failed");
            return new MotionCommandResponse { Success = false, Error = ex.Message };
        }
    }

    public async Task<TrajectoryStatusResponse> GetStatusAsync(CancellationToken ct = default)
    {
        try
        {
            var response = await _ipc.SendRequestAsync<object, TrajectoryStatusResponse>(
                MessageType.TRAJECTORY_STATUS_REQUEST,
                new { dummy = true },
                ct);

            return response ?? new TrajectoryStatusResponse();
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Get trajectory status failed");
            return new TrajectoryStatusResponse { State = "ERROR" };
        }
    }

    public async Task<TrajectoryControlResponse> ControlAsync(
        string action,
        CancellationToken ct = default)
    {
        try
        {
            var request = new TrajectoryControlRequest { Action = action };

            var response = await _ipc.SendRequestAsync<TrajectoryControlRequest, TrajectoryControlResponse>(
                MessageType.TRAJECTORY_CONTROL_REQUEST,
                request,
                ct);

            return response ?? new TrajectoryControlResponse { Success = false, Error = "No response" };
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Trajectory control failed");
            return new TrajectoryControlResponse { Success = false, Error = ex.Message };
        }
    }

    public async Task<PathPlanResponse> PlanPathAsync(
        PathPlanRequest request,
        CancellationToken ct = default)
    {
        try
        {
            var response = await _ipc.SendRequestAsync<PathPlanRequest, PathPlanResponse>(
                MessageType.PATH_PLAN_REQUEST,
                request,
                ct);

            return response ?? new PathPlanResponse { Success = false, Error = "No response" };
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Path plan failed");
            return new PathPlanResponse { Success = false, Error = ex.Message };
        }
    }
}
```

---

## Step 11: Validation

### 11.1 Build and Test

**Commands:**
```powershell
cd E:\DEV_CONTEXT_PROJECTs\Robot_controller

# Build
cmake --build build --config Debug

# Run trajectory tests
.\build\Debug\test_trajectory.exe
```

**Expected Output:**
```
[==========] Running 15 tests from 5 test suites.
...
[  PASSED  ] 15 tests.
```

---

## Completion Checklist

- [ ] TrajectoryTypes.hpp created (MotionType, VelocityProfile, Waypoint, etc.)
- [ ] VelocityProfile.hpp created (Trapezoidal, S-curve)
- [ ] Interpolators.hpp created (Linear, Joint, Circular)
- [ ] TrajectoryPlanner.hpp created (PTP, LIN, CIRC planning)
- [ ] TrajectoryExecutor.hpp created (real-time sampling)
- [ ] test_trajectory.cpp created with 15+ tests
- [ ] CMakeLists.txt updated for trajectory
- [ ] IPC message types added for trajectory
- [ ] C# TrajectoryClientService created
- [ ] All trajectory tests pass

---

## Troubleshooting

### IK Failures in Linear Motion
- Check if path stays within workspace
- Reduce path length or adjust target
- Use PTP motion for large movements

### Velocity Profile Issues
- Verify limits are reasonable
- Check velocityScale is 0-1
- Ensure distance > 0

### Circular Arc Not Computing
- Points may be collinear (use LIN instead)
- Check via point is not on line between start and end

---

## Notes

### Velocity Profile Selection
- **Trapezoidal:** Simple, fast computation, suitable for most motions
- **S-curve:** Smoother acceleration, reduces mechanical stress, better for precision

### Motion Type Guidelines
- **PTP:** Fastest path, use when path doesn't matter
- **LIN:** Straight line in Cartesian space, use for approach/retract
- **CIRC:** Circular arc, use for welding curves

### Control Rate
Default 1000 Hz matches typical industrial robot controllers. Adjust based on:
- Hardware capability
- Motion smoothness requirements
- CPU load

---

## Git Commit

```powershell
git add -A
git commit -m "IMPL_P2_03: Add trajectory generator

- Create TrajectoryTypes with motion enums and data structures
- Implement Trapezoidal and S-curve velocity profiles
- Add Linear, Joint, and Circular interpolators
- Create TrajectoryPlanner for PTP, LIN, CIRC motions
- Implement TrajectoryExecutor for real-time sampling
- Add motion limits and blending support
- Create 15 unit tests for trajectory
- Add IPC message types for trajectory control
- Create C# TrajectoryClientService

Co-Authored-By: Claude <noreply@anthropic.com>"
```
