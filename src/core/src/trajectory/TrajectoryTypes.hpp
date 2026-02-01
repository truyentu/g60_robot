/**
 * @file TrajectoryTypes.hpp
 * @brief Trajectory types and structures for motion planning
 */

#pragma once

#include "../kinematics/MathTypes.hpp"
#include "../kinematics/ForwardKinematics.hpp"
#include <vector>
#include <string>

namespace robot_controller {
namespace trajectory {

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

    TrajectorySegment()
        : motionType(MotionType::PTP),
          circleCenter(Vector3d::Zero()),
          circleNormal(Vector3d::UnitZ()),
          circleRadius(0),
          duration(0),
          startTime(0) {}
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

// Forward declarations for Trajectory methods
class JointInterpolator;

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

// ============================================================================
// Trajectory Implementation (forward declared, implemented after Interpolators)
// ============================================================================

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
    double t = (seg.duration > 0) ? (localTime / seg.duration) : 0.0;
    t = std::clamp(t, 0.0, 1.0);

    // Linear interpolation
    for (int i = 0; i < NUM_JOINTS; ++i) {
        point.jointPositions[i] = seg.start.jointAngles[i] +
            t * (seg.end.jointAngles[i] - seg.start.jointAngles[i]);
    }

    point.segmentIndex = idx;
    point.segmentProgress = t;
    point.isValid = true;

    return point;
}

} // namespace trajectory
} // namespace robot_controller
