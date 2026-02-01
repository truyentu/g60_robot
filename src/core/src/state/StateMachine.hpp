/**
 * @file StateMachine.hpp
 * @brief Robot state machine implementation
 */

#pragma once

#include "IStateMachine.hpp"
#include <mutex>
#include <vector>
#include <chrono>

namespace robot_controller {
namespace state {

/**
 * State transition rule
 */
struct TransitionRule {
    RobotState fromState;
    StateEvent event;
    RobotState toState;
    std::function<bool()> guard;  // Optional guard condition
};

/**
 * State machine implementation
 */
class StateMachine : public IStateMachine {
public:
    StateMachine();
    ~StateMachine() override = default;

    // IStateMachine interface
    RobotState currentState() const override;
    RobotMode currentMode() const override;
    SafetyState safetyState() const override;
    bool isEnabled() const override;
    bool isHomed() const override;
    bool isMoving() const override;
    bool hasError() const override;
    ErrorCode lastError() const override;
    std::string lastErrorMessage() const override;

    bool processEvent(StateEvent event) override;
    bool enable() override;
    bool disable() override;
    bool reset() override;

    bool setMode(RobotMode mode) override;

    void setError(ErrorCode code, const std::string& message = "") override;
    void clearError() override;

    void setStateChangeCallback(StateChangeCallback callback) override;
    void setErrorCallback(ErrorCallback callback) override;

    bool canTransitionTo(RobotState targetState) const override;
    bool canExecuteMotion() const override;
    bool canJog() const override;
    bool canRunProgram() const override;

    // Additional methods
    void update();  // Called from control loop
    std::chrono::milliseconds timeInCurrentState() const;

private:
    void initTransitionTable();
    bool executeTransition(RobotState newState);
    void onEnterState(RobotState state);
    void onExitState(RobotState state);

    // State data
    RobotState m_currentState;
    RobotState m_previousState;
    RobotMode m_currentMode;
    SafetyState m_safetyState;

    bool m_enabled;
    bool m_homed;

    ErrorCode m_lastError;
    std::string m_lastErrorMessage;

    // Timing
    std::chrono::steady_clock::time_point m_stateEntryTime;

    // Transition table
    std::vector<TransitionRule> m_transitions;

    // Callbacks
    StateChangeCallback m_stateChangeCallback;
    ErrorCallback m_errorCallback;

    // Thread safety
    mutable std::mutex m_mutex;
};

} // namespace state
} // namespace robot_controller
