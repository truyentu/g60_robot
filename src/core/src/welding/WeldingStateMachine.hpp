#pragma once

#include "WeldingTypes.hpp"
#include <functional>
#include <chrono>
#include <mutex>
#include <map>

namespace robotics {
namespace welding {

// ============================================================================
// Callbacks
// ============================================================================

using StateChangeCallback = std::function<void(WeldingState oldState, WeldingState newState)>;
using FaultCallback = std::function<void(WeldingFault fault)>;
using OutputChangeCallback = std::function<void(const WeldingOutputs&)>;

// ============================================================================
// Welding State Machine
// ============================================================================

/**
 * State machine for welding sequence control
 */
class WeldingStateMachine {
public:
    WeldingStateMachine();
    ~WeldingStateMachine() = default;

    // ========================================================================
    // State Control
    // ========================================================================

    /**
     * Process an event
     */
    bool processEvent(WeldingEvent event);

    /**
     * Update state machine (call periodically)
     * @param deltaMs Time since last update in milliseconds
     */
    void update(uint32_t deltaMs);

    /**
     * Get current state
     */
    WeldingState getState() const { return state_; }

    /**
     * Get current status
     */
    WeldingStatus getStatus() const;

    /**
     * Get current outputs
     */
    WeldingOutputs getOutputs() const { return outputs_; }

    // ========================================================================
    // Configuration
    // ========================================================================

    void setJob(const WeldingJob& job);
    const WeldingJob& getJob() const { return job_; }

    void setTimings(const WeldingTimings& timings);
    void setParams(const WeldingParams& params);

    // ========================================================================
    // Input Updates
    // ========================================================================

    void updateInputs(const WeldingInputs& inputs);
    void updateFeedback(const WeldingFeedback& feedback);

    // ========================================================================
    // Callbacks
    // ========================================================================

    void setStateChangeCallback(StateChangeCallback callback) {
        stateChangeCallback_ = callback;
    }

    void setFaultCallback(FaultCallback callback) {
        faultCallback_ = callback;
    }

    void setOutputChangeCallback(OutputChangeCallback callback) {
        outputChangeCallback_ = callback;
    }

    // ========================================================================
    // Status Queries
    // ========================================================================

    bool isReady() const;
    bool isWelding() const;
    bool hasFault() const { return fault_ != WeldingFault::NONE; }
    WeldingFault getFault() const { return fault_; }

    void clearFault();
    void reset();

private:
    // Current state
    WeldingState state_;
    WeldingFault fault_;

    // Timing
    uint32_t stateTimer_;       // Time in current state (ms)
    uint32_t weldTimer_;        // Total weld time (ms)
    uint8_t arcRetryCount_;

    // Configuration
    WeldingJob job_;

    // I/O
    WeldingInputs inputs_;
    WeldingOutputs outputs_;
    WeldingFeedback feedback_;

    // Thread safety
    mutable std::mutex mutex_;

    // Callbacks
    StateChangeCallback stateChangeCallback_;
    FaultCallback faultCallback_;
    OutputChangeCallback outputChangeCallback_;

    // ========================================================================
    // Internal State Handlers
    // ========================================================================

    void enterState(WeldingState newState);
    void exitState(WeldingState oldState);

    void handleIdle(WeldingEvent event);
    void handlePreFlow(WeldingEvent event);
    void handleArcStart(WeldingEvent event);
    void handleWelding(WeldingEvent event);
    void handleCraterFill(WeldingEvent event);
    void handleArcEnd(WeldingEvent event);
    void handleBurnBack(WeldingEvent event);
    void handlePostFlow(WeldingEvent event);
    void handleFaultState(WeldingEvent event);
    void handleEmergencyStop(WeldingEvent event);

    void updateOutputs();
    void setFaultInternal(WeldingFault fault);
    void transitionTo(WeldingState newState);

    // Check conditions
    bool checkSafetyInterlocks() const;
    bool checkArcEstablished() const;
};

// ============================================================================
// Implementation
// ============================================================================

inline WeldingStateMachine::WeldingStateMachine()
    : state_(WeldingState::IDLE),
      fault_(WeldingFault::NONE),
      stateTimer_(0),
      weldTimer_(0),
      arcRetryCount_(0) {
}

inline void WeldingStateMachine::setJob(const WeldingJob& job) {
    std::lock_guard<std::mutex> lock(mutex_);
    job_ = job;
}

inline void WeldingStateMachine::setTimings(const WeldingTimings& timings) {
    std::lock_guard<std::mutex> lock(mutex_);
    job_.timings = timings;
}

inline void WeldingStateMachine::setParams(const WeldingParams& params) {
    std::lock_guard<std::mutex> lock(mutex_);
    job_.params = params;
}

inline void WeldingStateMachine::updateInputs(const WeldingInputs& inputs) {
    std::lock_guard<std::mutex> lock(mutex_);
    inputs_ = inputs;

    // Check for faults
    if (inputs.powerSourceFault && state_ != WeldingState::IDLE) {
        setFaultInternal(WeldingFault::POWER_SUPPLY_FAULT);
    }

    if (inputs.wireStick && state_ == WeldingState::WELDING) {
        setFaultInternal(WeldingFault::STICK_WIRE);
    }
}

inline void WeldingStateMachine::updateFeedback(const WeldingFeedback& feedback) {
    std::lock_guard<std::mutex> lock(mutex_);
    feedback_ = feedback;
}

inline WeldingStatus WeldingStateMachine::getStatus() const {
    std::lock_guard<std::mutex> lock(mutex_);

    WeldingStatus status;
    status.state = state_;
    status.fault = fault_;
    status.feedback = feedback_;
    status.isReady = isReady();
    status.isWelding = isWelding();
    status.hasFault = fault_ != WeldingFault::NONE;
    status.stateTime = stateTimer_;
    status.arcRetries = arcRetryCount_;

    return status;
}

inline bool WeldingStateMachine::isReady() const {
    return state_ == WeldingState::IDLE &&
           fault_ == WeldingFault::NONE &&
           inputs_.powerSourceReady;
}

inline bool WeldingStateMachine::isWelding() const {
    return state_ == WeldingState::WELDING ||
           state_ == WeldingState::CRATER_FILL;
}

inline void WeldingStateMachine::clearFault() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == WeldingState::FAULT) {
        fault_ = WeldingFault::NONE;
        transitionTo(WeldingState::IDLE);
    }
}

inline void WeldingStateMachine::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    fault_ = WeldingFault::NONE;
    stateTimer_ = 0;
    weldTimer_ = 0;
    arcRetryCount_ = 0;
    outputs_ = WeldingOutputs();
    transitionTo(WeldingState::IDLE);
    updateOutputs();
}

inline bool WeldingStateMachine::processEvent(WeldingEvent event) {
    std::lock_guard<std::mutex> lock(mutex_);

    // Handle emergency stop from any state
    if (event == WeldingEvent::EMERGENCY_STOP) {
        transitionTo(WeldingState::EMERGENCY_STOP);
        return true;
    }

    // Handle abort from any state
    if (event == WeldingEvent::ABORT) {
        outputs_ = WeldingOutputs();  // All off
        updateOutputs();
        transitionTo(WeldingState::IDLE);
        return true;
    }

    // State-specific handling
    switch (state_) {
        case WeldingState::IDLE:
            handleIdle(event);
            break;
        case WeldingState::PRE_FLOW:
            handlePreFlow(event);
            break;
        case WeldingState::ARC_START:
            handleArcStart(event);
            break;
        case WeldingState::WELDING:
            handleWelding(event);
            break;
        case WeldingState::CRATER_FILL:
            handleCraterFill(event);
            break;
        case WeldingState::ARC_END:
            handleArcEnd(event);
            break;
        case WeldingState::BURN_BACK:
            handleBurnBack(event);
            break;
        case WeldingState::POST_FLOW:
            handlePostFlow(event);
            break;
        case WeldingState::FAULT:
            handleFaultState(event);
            break;
        case WeldingState::EMERGENCY_STOP:
            handleEmergencyStop(event);
            break;
    }

    return true;
}

inline void WeldingStateMachine::update(uint32_t deltaMs) {
    std::lock_guard<std::mutex> lock(mutex_);

    stateTimer_ += deltaMs;

    if (isWelding()) {
        weldTimer_ += deltaMs;
    }

    // Check state timers
    switch (state_) {
        case WeldingState::PRE_FLOW:
            if (stateTimer_ >= job_.timings.preFlowTime) {
                transitionTo(WeldingState::ARC_START);
            }
            break;

        case WeldingState::ARC_START:
            if (checkArcEstablished()) {
                transitionTo(WeldingState::WELDING);
            } else if (stateTimer_ >= job_.timings.arcStartTime) {
                // Arc failed to start
                if (arcRetryCount_ < job_.timings.arcRetryCount) {
                    arcRetryCount_++;
                    stateTimer_ = 0;
                    // Retry arc start
                } else {
                    setFaultInternal(WeldingFault::ARC_FAIL);
                }
            }
            break;

        case WeldingState::CRATER_FILL:
            if (stateTimer_ >= job_.timings.craterFillTime) {
                transitionTo(WeldingState::ARC_END);
            }
            break;

        case WeldingState::ARC_END:
            // Arc should extinguish quickly
            if (!feedback_.arcPresent || stateTimer_ >= 200) {
                transitionTo(WeldingState::BURN_BACK);
            }
            break;

        case WeldingState::BURN_BACK:
            if (stateTimer_ >= job_.timings.burnBackTime) {
                transitionTo(WeldingState::POST_FLOW);
            }
            break;

        case WeldingState::POST_FLOW:
            if (stateTimer_ >= job_.timings.postFlowTime) {
                transitionTo(WeldingState::IDLE);
            }
            break;

        case WeldingState::WELDING:
            // Check for arc loss
            if (!feedback_.arcPresent && stateTimer_ > 100) {
                setFaultInternal(WeldingFault::ARC_LOST);
            }
            // Check for gas loss
            if (!inputs_.gasFlowOk && stateTimer_ > 500) {
                setFaultInternal(WeldingFault::NO_GAS);
            }
            break;

        default:
            break;
    }
}

inline void WeldingStateMachine::handleIdle(WeldingEvent event) {
    if (event == WeldingEvent::START_WELD) {
        if (!checkSafetyInterlocks()) {
            setFaultInternal(WeldingFault::SAFETY_FAULT);
            return;
        }

        arcRetryCount_ = 0;
        weldTimer_ = 0;
        transitionTo(WeldingState::PRE_FLOW);
    }
}

inline void WeldingStateMachine::handlePreFlow(WeldingEvent event) {
    if (event == WeldingEvent::STOP_WELD) {
        transitionTo(WeldingState::POST_FLOW);
    }
    // Timer handled in update()
}

inline void WeldingStateMachine::handleArcStart(WeldingEvent event) {
    if (event == WeldingEvent::STOP_WELD) {
        transitionTo(WeldingState::POST_FLOW);
    }
    if (event == WeldingEvent::ARC_DETECTED) {
        transitionTo(WeldingState::WELDING);
    }
}

inline void WeldingStateMachine::handleWelding(WeldingEvent event) {
    if (event == WeldingEvent::STOP_WELD) {
        transitionTo(WeldingState::CRATER_FILL);
    }
    if (event == WeldingEvent::ARC_LOST) {
        setFaultInternal(WeldingFault::ARC_LOST);
    }
}

inline void WeldingStateMachine::handleCraterFill(WeldingEvent event) {
    // Timer handled in update()
    (void)event;
}

inline void WeldingStateMachine::handleArcEnd(WeldingEvent event) {
    // Timer handled in update()
    (void)event;
}

inline void WeldingStateMachine::handleBurnBack(WeldingEvent event) {
    // Timer handled in update()
    (void)event;
}

inline void WeldingStateMachine::handlePostFlow(WeldingEvent event) {
    // Timer handled in update()
    (void)event;
}

inline void WeldingStateMachine::handleFaultState(WeldingEvent event) {
    if (event == WeldingEvent::RESET) {
        fault_ = WeldingFault::NONE;
        transitionTo(WeldingState::IDLE);
    }
}

inline void WeldingStateMachine::handleEmergencyStop(WeldingEvent event) {
    if (event == WeldingEvent::RESET) {
        fault_ = WeldingFault::NONE;
        transitionTo(WeldingState::IDLE);
    }
}

inline void WeldingStateMachine::transitionTo(WeldingState newState) {
    if (state_ == newState) return;

    WeldingState oldState = state_;
    exitState(oldState);
    state_ = newState;
    stateTimer_ = 0;
    enterState(newState);
    updateOutputs();

    if (stateChangeCallback_) {
        stateChangeCallback_(oldState, newState);
    }
}

inline void WeldingStateMachine::enterState(WeldingState newState) {
    switch (newState) {
        case WeldingState::PRE_FLOW:
            outputs_.gasValve = true;
            outputs_.robotReady = true;
            break;

        case WeldingState::ARC_START:
            outputs_.gasValve = true;
            outputs_.wireFeed = true;
            outputs_.arcEnable = true;
            outputs_.torchTrigger = true;
            break;

        case WeldingState::WELDING:
            outputs_.gasValve = true;
            outputs_.wireFeed = true;
            outputs_.arcEnable = true;
            outputs_.torchTrigger = true;
            break;

        case WeldingState::CRATER_FILL:
            // Same as welding but with reduced parameters
            outputs_.gasValve = true;
            outputs_.wireFeed = true;
            outputs_.arcEnable = true;
            outputs_.torchTrigger = true;
            break;

        case WeldingState::ARC_END:
            outputs_.gasValve = true;
            outputs_.wireFeed = false;
            outputs_.arcEnable = false;
            outputs_.torchTrigger = false;
            break;

        case WeldingState::BURN_BACK:
            outputs_.gasValve = true;
            outputs_.wireFeed = false;
            outputs_.arcEnable = false;
            break;

        case WeldingState::POST_FLOW:
            outputs_.gasValve = true;
            outputs_.wireFeed = false;
            outputs_.arcEnable = false;
            outputs_.torchTrigger = false;
            break;

        case WeldingState::IDLE:
        case WeldingState::FAULT:
        case WeldingState::EMERGENCY_STOP:
            outputs_ = WeldingOutputs();  // All off
            break;
    }
}

inline void WeldingStateMachine::exitState(WeldingState oldState) {
    // Clean up state if needed
    (void)oldState;
}

inline void WeldingStateMachine::updateOutputs() {
    if (outputChangeCallback_) {
        outputChangeCallback_(outputs_);
    }
}

inline void WeldingStateMachine::setFaultInternal(WeldingFault fault) {
    fault_ = fault;
    transitionTo(WeldingState::FAULT);

    if (faultCallback_) {
        faultCallback_(fault);
    }
}

inline bool WeldingStateMachine::checkSafetyInterlocks() const {
    return inputs_.powerSourceReady &&
           !inputs_.powerSourceFault &&
           inputs_.gasFlowOk;
}

inline bool WeldingStateMachine::checkArcEstablished() const {
    return inputs_.arcDetect || feedback_.arcPresent ||
           (feedback_.actualCurrent > 10.0f && feedback_.actualVoltage > 10.0f);
}

} // namespace welding
} // namespace robotics
