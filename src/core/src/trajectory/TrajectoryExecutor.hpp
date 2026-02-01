/**
 * @file TrajectoryExecutor.hpp
 * @brief Trajectory execution and real-time sampling
 */

#pragma once

#include "TrajectoryPlanner.hpp"
#include <functional>
#include <atomic>
#include <thread>
#include <chrono>

namespace robot_controller {
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
        std::shared_ptr<IKinematicsService> kinematics);

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

    /**
     * Check if trajectory is loaded
     */
    bool hasTrajectory() const { return trajectory_.isValid; }

    /**
     * Get total duration
     */
    double getTotalDuration() const { return trajectory_.totalDuration; }

private:
    std::shared_ptr<IKinematicsService> kinematics_;

    Trajectory trajectory_;
    double controlRate_;              // Hz

    std::atomic<TrajectoryState> state_;
    std::atomic<double> currentTime_;
    std::atomic<bool> stopRequested_;

    TrajectoryPointCallback pointCallback_;
    TrajectoryCompleteCallback completeCallback_;
    TrajectoryErrorCallback errorCallback_;

    std::thread executionThread_;
    mutable std::mutex trajectoryMutex_;

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
    std::shared_ptr<IKinematicsService> kinematics)
    : kinematics_(kinematics),
      controlRate_(250.0),  // 250Hz default (4ms cycle)
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

    {
        std::lock_guard<std::mutex> lock(trajectoryMutex_);
        trajectory_ = trajectory;
    }

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
    std::lock_guard<std::mutex> lock(trajectoryMutex_);
    if (trajectory_.totalDuration <= 0) return 0;
    return std::clamp(currentTime_.load() / trajectory_.totalDuration, 0.0, 1.0);
}

inline TrajectoryPoint TrajectoryExecutor::sample(double time) const {
    std::lock_guard<std::mutex> lock(trajectoryMutex_);

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
            point.segmentIndex = static_cast<int>(trajectory_.segments.size() - 1);
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
    point.segmentProgress = (segment.duration > 0) ? (localTime / segment.duration) : 1.0;

    return point;
}

inline TrajectoryPoint TrajectoryExecutor::sampleSegment(
    const TrajectorySegment& segment,
    double localTime) const {

    TrajectoryPoint point;

    double t_norm = (segment.duration > 0) ?
        std::clamp(localTime / segment.duration, 0.0, 1.0) : 1.0;

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
            if (segment.duration > 0) {
                double vel_scale = profileValue.velocity / segment.duration;
                for (int i = 0; i < NUM_JOINTS; ++i) {
                    double delta = segment.end.jointAngles[i] - segment.start.jointAngles[i];
                    point.jointVelocities[i] = delta * vel_scale;
                }
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
            if (segment.duration > 0) {
                double vel_scale = profileValue.velocity / segment.duration;
                Vector3d posDelta = segment.end.cartesianPose.position -
                                   segment.start.cartesianPose.position;
                point.cartesianVelocity.head<3>() = posDelta * vel_scale;

                // Angular velocity (approximate)
                Vector3d rpyDelta = segment.end.cartesianPose.rpy -
                                   segment.start.cartesianPose.rpy;
                point.cartesianVelocity.tail<3>() = rpyDelta * vel_scale;
            }
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
    double dt = 1.0 / controlRate_;
    auto sleepDuration = std::chrono::microseconds(static_cast<int>(dt * 1000000));

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
            double totalDuration;
            {
                std::lock_guard<std::mutex> lock(trajectoryMutex_);
                totalDuration = trajectory_.totalDuration;
            }

            if (time >= totalDuration) {
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
        std::this_thread::sleep_for(sleepDuration);
    }
}

} // namespace trajectory
} // namespace robot_controller
