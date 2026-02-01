#pragma once

#include "WeldingStateMachine.hpp"
#include <memory>
#include <thread>
#include <atomic>

namespace robotics {
namespace welding {

/**
 * High-level welding controller
 * Coordinates welding state machine with robot motion
 */
class WeldingController {
public:
    WeldingController();
    ~WeldingController();

    // ========================================================================
    // Control
    // ========================================================================

    /**
     * Initialize controller
     */
    bool initialize();

    /**
     * Start welding
     * @param job Welding job parameters
     * @return true if started successfully
     */
    bool startWeld(const WeldingJob& job);

    /**
     * Stop welding (normal end with crater fill)
     */
    void stopWeld();

    /**
     * Abort welding immediately
     */
    void abortWeld();

    /**
     * Emergency stop
     */
    void emergencyStop();

    /**
     * Reset from fault
     */
    void reset();

    // ========================================================================
    // Status
    // ========================================================================

    WeldingStatus getStatus() const;
    WeldingState getState() const { return stateMachine_.getState(); }
    bool isWelding() const { return stateMachine_.isWelding(); }
    bool hasFault() const { return stateMachine_.hasFault(); }

    // ========================================================================
    // Job Management
    // ========================================================================

    void setCurrentJob(const WeldingJob& job);
    const WeldingJob& getCurrentJob() const { return currentJob_; }

    // ========================================================================
    // Real-time Parameters
    // ========================================================================

    /**
     * Adjust parameters during welding
     */
    void adjustCurrent(float current);
    void adjustVoltage(float voltage);
    void adjustWireSpeed(float speed);
    void adjustTravelSpeed(float speed);

    // ========================================================================
    // I/O Interface
    // ========================================================================

    void setOutputCallback(std::function<void(const WeldingOutputs&)> callback);
    void updateInputs(const WeldingInputs& inputs);
    void updateFeedback(const WeldingFeedback& feedback);

    // ========================================================================
    // Motion Coordination
    // ========================================================================

    void setMotionStartCallback(std::function<void()> callback) {
        motionStartCallback_ = callback;
    }

    void setMotionStopCallback(std::function<void()> callback) {
        motionStopCallback_ = callback;
    }

    /**
     * Called by motion system when robot is at weld start position
     */
    void onMotionAtStart();

    /**
     * Called by motion system when robot reaches weld end position
     */
    void onMotionAtEnd();

private:
    WeldingStateMachine stateMachine_;
    WeldingJob currentJob_;

    // Control loop
    std::thread controlThread_;
    std::atomic<bool> running_;
    uint32_t updateRateMs_;

    // Callbacks
    std::function<void()> motionStartCallback_;
    std::function<void()> motionStopCallback_;

    // State tracking
    std::atomic<bool> motionAtStart_;
    std::atomic<bool> motionAtEnd_;

    void controlLoop();
    void onStateChange(WeldingState oldState, WeldingState newState);
    void onFault(WeldingFault fault);
};

// ============================================================================
// Implementation
// ============================================================================

inline WeldingController::WeldingController()
    : running_(false),
      updateRateMs_(10),
      motionAtStart_(false),
      motionAtEnd_(false) {

    stateMachine_.setStateChangeCallback(
        [this](WeldingState old, WeldingState newState) {
            onStateChange(old, newState);
        });

    stateMachine_.setFaultCallback(
        [this](WeldingFault fault) {
            onFault(fault);
        });
}

inline WeldingController::~WeldingController() {
    running_ = false;
    if (controlThread_.joinable()) {
        controlThread_.join();
    }
}

inline bool WeldingController::initialize() {
    if (running_) return true;

    running_ = true;
    controlThread_ = std::thread(&WeldingController::controlLoop, this);

    return true;
}

inline bool WeldingController::startWeld(const WeldingJob& job) {
    if (stateMachine_.hasFault()) {
        return false;
    }

    currentJob_ = job;
    stateMachine_.setJob(job);
    motionAtStart_ = false;
    motionAtEnd_ = false;

    return stateMachine_.processEvent(WeldingEvent::START_WELD);
}

inline void WeldingController::stopWeld() {
    stateMachine_.processEvent(WeldingEvent::STOP_WELD);
}

inline void WeldingController::abortWeld() {
    stateMachine_.processEvent(WeldingEvent::ABORT);
}

inline void WeldingController::emergencyStop() {
    stateMachine_.processEvent(WeldingEvent::EMERGENCY_STOP);
}

inline void WeldingController::reset() {
    stateMachine_.processEvent(WeldingEvent::RESET);
    motionAtStart_ = false;
    motionAtEnd_ = false;
}

inline WeldingStatus WeldingController::getStatus() const {
    return stateMachine_.getStatus();
}

inline void WeldingController::setCurrentJob(const WeldingJob& job) {
    currentJob_ = job;
    stateMachine_.setJob(job);
}

inline void WeldingController::adjustCurrent(float current) {
    currentJob_.params.current = current;
    stateMachine_.setParams(currentJob_.params);
}

inline void WeldingController::adjustVoltage(float voltage) {
    currentJob_.params.voltage = voltage;
    stateMachine_.setParams(currentJob_.params);
}

inline void WeldingController::adjustWireSpeed(float speed) {
    currentJob_.params.wireSpeed = speed;
    stateMachine_.setParams(currentJob_.params);
}

inline void WeldingController::adjustTravelSpeed(float speed) {
    currentJob_.params.travelSpeed = speed;
    // Notify motion system
}

inline void WeldingController::setOutputCallback(
    std::function<void(const WeldingOutputs&)> callback) {
    stateMachine_.setOutputChangeCallback(callback);
}

inline void WeldingController::updateInputs(const WeldingInputs& inputs) {
    stateMachine_.updateInputs(inputs);
}

inline void WeldingController::updateFeedback(const WeldingFeedback& feedback) {
    stateMachine_.updateFeedback(feedback);
}

inline void WeldingController::onMotionAtStart() {
    motionAtStart_ = true;
    // Could start welding automatically here
}

inline void WeldingController::onMotionAtEnd() {
    motionAtEnd_ = true;
    // Trigger weld stop if not already stopping
    if (stateMachine_.isWelding()) {
        stopWeld();
    }
}

inline void WeldingController::controlLoop() {
    auto lastUpdate = std::chrono::steady_clock::now();

    while (running_) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - lastUpdate).count();

        if (elapsed >= updateRateMs_) {
            stateMachine_.update(static_cast<uint32_t>(elapsed));
            lastUpdate = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

inline void WeldingController::onStateChange(
    WeldingState oldState, WeldingState newState) {

    (void)oldState;

    // Coordinate with motion system
    if (newState == WeldingState::WELDING && motionStartCallback_) {
        motionStartCallback_();  // Start robot motion
    }

    if ((newState == WeldingState::CRATER_FILL ||
         newState == WeldingState::FAULT ||
         newState == WeldingState::EMERGENCY_STOP) && motionStopCallback_) {
        motionStopCallback_();  // Stop robot motion
    }
}

inline void WeldingController::onFault(WeldingFault fault) {
    (void)fault;
    // Stop motion on fault
    if (motionStopCallback_) {
        motionStopCallback_();
    }
}

} // namespace welding
} // namespace robotics
