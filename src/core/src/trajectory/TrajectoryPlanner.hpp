/**
 * @file TrajectoryPlanner.hpp
 * @brief Trajectory planning for robot motion
 */

#pragma once

#include "Interpolators.hpp"
#include "VelocityProfile.hpp"
#include "../kinematics/KinematicsService.hpp"
#include <memory>

namespace robot_controller {
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
    bool useJointTarget;  // Use targetJoints instead of targetPose

    MotionCommand()
        : type(MotionType::PTP),
          targetJoints{0},
          useViaPoint(false),
          isRelative(false),
          useJointTarget(true) {}
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
        std::shared_ptr<IKinematicsService> kinematics);

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
    std::shared_ptr<IKinematicsService> kinematics_;
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
     * Validate trajectory (check IK, limits, etc.)
     */
    bool validateTrajectory(Trajectory& trajectory);
};

// ============================================================================
// Implementation
// ============================================================================

inline TrajectoryPlanner::TrajectoryPlanner(
    std::shared_ptr<IKinematicsService> kinematics)
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
        // IK failed - use start joints as fallback
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

            // If target is Cartesian pose, compute IK
            if (!command.useJointTarget) {
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

    // Plan each segment independently
    std::vector<TrajectorySegment> segments;
    JointAngles prevJoints = currentJoints;

    for (const auto& cmd : commands) {
        Trajectory singleTraj = planMotion(cmd, prevJoints);
        if (!singleTraj.segments.empty()) {
            segments.push_back(singleTraj.segments[0]);
            prevJoints = singleTraj.segments[0].end.jointAngles;
        }
    }

    // Apply timing
    double currentTime = 0;
    for (size_t i = 0; i < segments.size(); ++i) {
        auto& seg = segments[i];
        seg.startTime = currentTime;
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
    }

    return true;
}

} // namespace trajectory
} // namespace robot_controller
