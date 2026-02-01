/**
 * @file MotionStreamer.hpp
 * @brief Streams trajectory points to firmware for real-time execution
 */

#pragma once

#include "FirmwareInterface.hpp"
#include "../trajectory/TrajectoryExecutor.hpp"
#include <queue>

namespace robot_controller {
namespace firmware {

using namespace trajectory;

/**
 * Streams trajectory points to firmware
 * Maintains command buffer for smooth motion
 */
class MotionStreamer {
public:
    explicit MotionStreamer(
        std::shared_ptr<FirmwareInterface> firmware,
        std::shared_ptr<kinematics::IKinematicsService> kinematics);

    ~MotionStreamer();

    // ========================================================================
    // Trajectory Execution
    // ========================================================================

    /**
     * Start streaming trajectory to firmware
     */
    bool startTrajectory(const Trajectory& trajectory);

    /**
     * Stop streaming
     */
    void stop();

    /**
     * Pause streaming
     */
    void pause();

    /**
     * Resume streaming
     */
    void resume();

    /**
     * Get current state
     */
    TrajectoryState getState() const { return state_.load(); }

    /**
     * Get progress (0.0 - 1.0)
     */
    double getProgress() const;

    // ========================================================================
    // Configuration
    // ========================================================================

    void setStreamRate(int rateHz) { streamRate_ = rateHz; }
    void setBufferSize(int size) { bufferSize_ = size; }

    // ========================================================================
    // Callbacks
    // ========================================================================

    void setCompleteCallback(std::function<void(bool)> callback) {
        completeCallback_ = callback;
    }

private:
    std::shared_ptr<FirmwareInterface> firmware_;
    std::shared_ptr<kinematics::IKinematicsService> kinematics_;

    TrajectoryExecutor executor_;

    std::atomic<TrajectoryState> state_;
    std::atomic<bool> stopRequested_;

    int streamRate_;
    int bufferSize_;

    std::thread streamThread_;
    std::function<void(bool)> completeCallback_;

    // Command queue
    std::mutex queueMutex_;
    std::queue<TrajectoryPoint> pointQueue_;

    void streamLoop();
    void sendPoint(const TrajectoryPoint& point);
};

// ============================================================================
// Implementation
// ============================================================================

inline MotionStreamer::MotionStreamer(
    std::shared_ptr<FirmwareInterface> firmware,
    std::shared_ptr<kinematics::IKinematicsService> kinematics)
    : firmware_(firmware),
      kinematics_(kinematics),
      executor_(kinematics),
      state_(TrajectoryState::IDLE),
      stopRequested_(false),
      streamRate_(50),
      bufferSize_(16) {
}

inline MotionStreamer::~MotionStreamer() {
    stop();
}

inline bool MotionStreamer::startTrajectory(const Trajectory& trajectory) {
    if (!firmware_->isConnected()) return false;
    if (!trajectory.isValid) return false;

    stop();

    state_.store(TrajectoryState::RUNNING);
    stopRequested_.store(false);

    executor_.setControlRate(streamRate_);

    executor_.setPointCallback([this](const TrajectoryPoint& point) {
        std::lock_guard<std::mutex> lock(queueMutex_);
        pointQueue_.push(point);
    });

    executor_.setCompleteCallback([this](bool success, const std::string& msg) {
        state_.store(TrajectoryState::COMPLETED);
        if (completeCallback_) {
            completeCallback_(success);
        }
    });

    executor_.start(trajectory);

    streamThread_ = std::thread(&MotionStreamer::streamLoop, this);

    return true;
}

inline void MotionStreamer::stop() {
    stopRequested_.store(true);
    executor_.stop();
    firmware_->stopMotion();

    if (streamThread_.joinable()) {
        streamThread_.join();
    }

    state_.store(TrajectoryState::IDLE);

    std::lock_guard<std::mutex> lock(queueMutex_);
    while (!pointQueue_.empty()) pointQueue_.pop();
}

inline void MotionStreamer::pause() {
    executor_.pause();
    firmware_->feedHold();
    state_.store(TrajectoryState::PAUSED);
}

inline void MotionStreamer::resume() {
    firmware_->cycleStart();
    executor_.resume();
    state_.store(TrajectoryState::RUNNING);
}

inline double MotionStreamer::getProgress() const {
    return executor_.getProgress();
}

inline void MotionStreamer::streamLoop() {
    auto lastSendTime = std::chrono::steady_clock::now();
    double sendInterval = 1.0 / streamRate_;

    while (!stopRequested_.load()) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - lastSendTime).count();

        if (elapsed >= sendInterval) {
            // Check firmware buffer status
            auto status = firmware_->getCachedStatus();

            if (status.motionBuffer < static_cast<uint8_t>(bufferSize_)) {
                // Get next point from queue
                TrajectoryPoint point;
                bool hasPoint = false;

                {
                    std::lock_guard<std::mutex> lock(queueMutex_);
                    if (!pointQueue_.empty()) {
                        point = pointQueue_.front();
                        pointQueue_.pop();
                        hasPoint = true;
                    }
                }

                if (hasPoint && point.isValid) {
                    sendPoint(point);
                }
            }

            lastSendTime = now;
        }

        // Check for completion
        if (executor_.getState() == TrajectoryState::COMPLETED) {
            // Wait for firmware buffer to empty
            auto status = firmware_->getCachedStatus();
            if (status.motionBuffer == 0 && status.state == MachineState::IDLE) {
                break;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

inline void MotionStreamer::sendPoint(const TrajectoryPoint& point) {
    // Convert joint positions to degrees for firmware
    std::array<double, NUM_AXES> positions;
    for (int i = 0; i < NUM_AXES; ++i) {
        positions[i] = radToDeg(point.jointPositions[i]);
    }

    // Calculate feed rate from velocity
    double maxVel = 0;
    for (int i = 0; i < NUM_AXES; ++i) {
        maxVel = std::max(maxVel, std::abs(point.jointVelocities[i]));
    }
    double feedRate = radToDeg(maxVel);

    if (feedRate < 1.0) feedRate = 1.0;

    firmware_->moveJoints(positions, feedRate, false);
}

} // namespace firmware
} // namespace robot_controller
