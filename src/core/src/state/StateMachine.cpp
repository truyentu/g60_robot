/**
 * @file StateMachine.cpp
 * @brief State machine implementation
 */

#include "StateMachine.hpp"
#include "../logging/Logger.hpp"

namespace robot_controller {
namespace state {

StateMachine::StateMachine()
    : m_currentState(RobotState::INIT)
    , m_previousState(RobotState::INIT)
    , m_currentMode(RobotMode::MANUAL)
    , m_safetyState(SafetyState::NORMAL)
    , m_enabled(false)
    , m_homed(false)
    , m_lastError(ErrorCode::NONE)
    , m_stateEntryTime(std::chrono::steady_clock::now())
{
    initTransitionTable();
    LOG_INFO("StateMachine initialized in state: {}", toString(m_currentState));
}

void StateMachine::initTransitionTable() {
    // Define all valid state transitions

    // From INIT
    m_transitions.push_back({RobotState::INIT, StateEvent::INIT_COMPLETE, RobotState::IDLE, nullptr});

    // From IDLE
    m_transitions.push_back({RobotState::IDLE, StateEvent::ENABLE_REQUEST, RobotState::IDLE,
        [this]() { return m_safetyState == SafetyState::NORMAL; }});
    m_transitions.push_back({RobotState::IDLE, StateEvent::HOME_REQUEST, RobotState::HOMING,
        [this]() { return m_enabled && m_safetyState == SafetyState::NORMAL; }});

    // From HOMING
    m_transitions.push_back({RobotState::HOMING, StateEvent::HOME_COMPLETE, RobotState::READY, nullptr});
    m_transitions.push_back({RobotState::HOMING, StateEvent::HOME_ERROR, RobotState::ERROR, nullptr});
    m_transitions.push_back({RobotState::HOMING, StateEvent::DISABLE_REQUEST, RobotState::IDLE, nullptr});

    // From READY
    m_transitions.push_back({RobotState::READY, StateEvent::MOTION_START, RobotState::MOVING,
        [this]() { return m_enabled && m_homed; }});
    m_transitions.push_back({RobotState::READY, StateEvent::PROGRAM_START, RobotState::EXECUTING,
        [this]() { return m_enabled && m_homed && m_currentMode == RobotMode::AUTO; }});
    m_transitions.push_back({RobotState::READY, StateEvent::DISABLE_REQUEST, RobotState::IDLE, nullptr});

    // From MOVING
    m_transitions.push_back({RobotState::MOVING, StateEvent::MOTION_COMPLETE, RobotState::READY, nullptr});
    m_transitions.push_back({RobotState::MOVING, StateEvent::MOTION_ERROR, RobotState::ERROR, nullptr});
    m_transitions.push_back({RobotState::MOVING, StateEvent::DISABLE_REQUEST, RobotState::IDLE, nullptr});

    // From EXECUTING
    m_transitions.push_back({RobotState::EXECUTING, StateEvent::PROGRAM_PAUSE, RobotState::PAUSED, nullptr});
    m_transitions.push_back({RobotState::EXECUTING, StateEvent::PROGRAM_COMPLETE, RobotState::READY, nullptr});
    m_transitions.push_back({RobotState::EXECUTING, StateEvent::PROGRAM_STOP, RobotState::READY, nullptr});
    m_transitions.push_back({RobotState::EXECUTING, StateEvent::PROGRAM_ERROR, RobotState::ERROR, nullptr});
    m_transitions.push_back({RobotState::EXECUTING, StateEvent::MOTION_START, RobotState::EXECUTING, nullptr});
    m_transitions.push_back({RobotState::EXECUTING, StateEvent::MOTION_COMPLETE, RobotState::EXECUTING, nullptr});

    // From PAUSED
    m_transitions.push_back({RobotState::PAUSED, StateEvent::PROGRAM_RESUME, RobotState::EXECUTING, nullptr});
    m_transitions.push_back({RobotState::PAUSED, StateEvent::PROGRAM_STOP, RobotState::READY, nullptr});
    m_transitions.push_back({RobotState::PAUSED, StateEvent::DISABLE_REQUEST, RobotState::IDLE, nullptr});

    // From ERROR
    m_transitions.push_back({RobotState::ERROR, StateEvent::RESET_REQUEST, RobotState::IDLE,
        [this]() { return m_safetyState != SafetyState::EMERGENCY_STOP; }});
    m_transitions.push_back({RobotState::ERROR, StateEvent::ERROR_CLEARED, RobotState::IDLE,
        [this]() { return m_safetyState != SafetyState::EMERGENCY_STOP; }});

    // From ESTOP (any state can go to ESTOP)
    m_transitions.push_back({RobotState::ESTOP, StateEvent::ESTOP_RELEASED, RobotState::ERROR, nullptr});
    m_transitions.push_back({RobotState::ESTOP, StateEvent::RESET_REQUEST, RobotState::IDLE,
        [this]() { return m_safetyState == SafetyState::NORMAL; }});

    LOG_DEBUG("Transition table initialized with {} rules", m_transitions.size());
}

RobotState StateMachine::currentState() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_currentState;
}

RobotMode StateMachine::currentMode() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_currentMode;
}

SafetyState StateMachine::safetyState() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_safetyState;
}

bool StateMachine::isEnabled() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_enabled;
}

bool StateMachine::isHomed() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_homed;
}

bool StateMachine::isMoving() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_currentState == RobotState::MOVING ||
           m_currentState == RobotState::EXECUTING ||
           m_currentState == RobotState::HOMING;
}

bool StateMachine::hasError() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_currentState == RobotState::ERROR ||
           m_currentState == RobotState::ESTOP ||
           m_lastError != ErrorCode::NONE;
}

ErrorCode StateMachine::lastError() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_lastError;
}

std::string StateMachine::lastErrorMessage() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_lastErrorMessage;
}

bool StateMachine::processEvent(StateEvent event) {
    std::lock_guard<std::mutex> lock(m_mutex);

    LOG_DEBUG("Processing event: {} in state: {}", toString(event), toString(m_currentState));

    // E-Stop always takes priority
    if (event == StateEvent::ESTOP_PRESSED) {
        m_safetyState = SafetyState::EMERGENCY_STOP;
        m_lastError = ErrorCode::SAFETY_ESTOP;
        m_lastErrorMessage = "Emergency stop activated";
        if (m_errorCallback) {
            m_errorCallback(m_lastError, m_lastErrorMessage);
        }
        return executeTransition(RobotState::ESTOP);
    }

    // E-Stop release
    if (event == StateEvent::ESTOP_RELEASED) {
        m_safetyState = SafetyState::NORMAL;
    }

    // Safety door events
    if (event == StateEvent::SAFETY_DOOR_OPEN) {
        bool wasMoving = m_currentState == RobotState::MOVING ||
                         m_currentState == RobotState::EXECUTING ||
                         m_currentState == RobotState::HOMING;
        if (wasMoving) {
            m_safetyState = SafetyState::PROTECTIVE_STOP;
            m_lastError = ErrorCode::SAFETY_DOOR_OPEN;
            m_lastErrorMessage = "Safety door opened during motion";
            if (m_errorCallback) {
                m_errorCallback(m_lastError, m_lastErrorMessage);
            }
            return executeTransition(RobotState::ERROR);
        }
    }

    // Find matching transition
    for (const auto& rule : m_transitions) {
        if (rule.fromState == m_currentState && rule.event == event) {
            // Check guard condition
            if (rule.guard && !rule.guard()) {
                LOG_WARN("Transition guard failed for event: {}", toString(event));
                return false;
            }

            return executeTransition(rule.toState);
        }
    }

    LOG_WARN("No transition found for event: {} in state: {}",
             toString(event), toString(m_currentState));
    return false;
}

bool StateMachine::enable() {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_safetyState != SafetyState::NORMAL) {
        LOG_WARN("Cannot enable: safety state is {}", toString(m_safetyState));
        return false;
    }

    if (m_currentState != RobotState::IDLE && m_currentState != RobotState::READY) {
        LOG_WARN("Cannot enable: current state is {}", toString(m_currentState));
        return false;
    }

    m_enabled = true;
    LOG_INFO("Robot enabled");
    return true;
}

bool StateMachine::disable() {
    std::lock_guard<std::mutex> lock(m_mutex);

    // Check if moving
    bool wasMoving = m_currentState == RobotState::MOVING ||
                     m_currentState == RobotState::EXECUTING ||
                     m_currentState == RobotState::HOMING;
    if (wasMoving) {
        LOG_WARN("Disabling while moving - stopping motion");
    }

    m_enabled = false;
    LOG_INFO("Robot disabled");

    // Transition to IDLE if in a state that requires enabled
    if (m_currentState == RobotState::READY ||
        m_currentState == RobotState::MOVING ||
        m_currentState == RobotState::EXECUTING ||
        m_currentState == RobotState::PAUSED) {
        return executeTransition(RobotState::IDLE);
    }

    return true;
}

bool StateMachine::reset() {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_safetyState == SafetyState::EMERGENCY_STOP) {
        LOG_WARN("Cannot reset: E-Stop still active");
        return false;
    }

    if (m_currentState == RobotState::ERROR || m_currentState == RobotState::ESTOP) {
        m_lastError = ErrorCode::NONE;
        m_lastErrorMessage.clear();
        m_safetyState = SafetyState::NORMAL;
        m_enabled = false;
        m_homed = false;  // Require re-homing after error
        LOG_INFO("Error cleared, resetting to IDLE");
        return executeTransition(RobotState::IDLE);
    }

    return false;
}

bool StateMachine::setMode(RobotMode mode) {
    std::lock_guard<std::mutex> lock(m_mutex);

    // Can only change mode when not moving
    bool isCurrentlyMoving = m_currentState == RobotState::MOVING ||
                             m_currentState == RobotState::EXECUTING ||
                             m_currentState == RobotState::HOMING;
    if (isCurrentlyMoving) {
        LOG_WARN("Cannot change mode while moving");
        return false;
    }

    m_currentMode = mode;
    LOG_INFO("Mode changed to: {}", toString(mode));
    return true;
}

void StateMachine::setError(ErrorCode code, const std::string& message) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_lastError = code;
    m_lastErrorMessage = message.empty() ? toString(code) : message;

    LOG_ERROR("Error set: {} - {}", toString(code), m_lastErrorMessage);

    if (m_errorCallback) {
        m_errorCallback(code, m_lastErrorMessage);
    }
}

void StateMachine::clearError() {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_safetyState == SafetyState::EMERGENCY_STOP) {
        LOG_WARN("Cannot clear error: E-Stop still active");
        return;
    }

    m_lastError = ErrorCode::NONE;
    m_lastErrorMessage.clear();
    LOG_INFO("Error cleared");
}

void StateMachine::setStateChangeCallback(StateChangeCallback callback) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_stateChangeCallback = std::move(callback);
}

void StateMachine::setErrorCallback(ErrorCallback callback) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_errorCallback = std::move(callback);
}

bool StateMachine::canTransitionTo(RobotState targetState) const {
    std::lock_guard<std::mutex> lock(m_mutex);

    for (const auto& rule : m_transitions) {
        if (rule.fromState == m_currentState && rule.toState == targetState) {
            if (rule.guard && !rule.guard()) {
                return false;
            }
            return true;
        }
    }
    return false;
}

bool StateMachine::canExecuteMotion() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_enabled && m_homed &&
           m_safetyState == SafetyState::NORMAL &&
           (m_currentState == RobotState::READY ||
            m_currentState == RobotState::EXECUTING);
}

bool StateMachine::canJog() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_enabled && m_homed &&
           m_safetyState == SafetyState::NORMAL &&
           m_currentState == RobotState::READY &&
           m_currentMode == RobotMode::MANUAL;
}

bool StateMachine::canRunProgram() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_enabled && m_homed &&
           m_safetyState == SafetyState::NORMAL &&
           m_currentState == RobotState::READY &&
           m_currentMode == RobotMode::AUTO;
}

void StateMachine::update() {
    // Called from control loop - check for timeouts, watchdogs, etc.
    std::lock_guard<std::mutex> lock(m_mutex);

    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_stateEntryTime);

    // State-specific timeout checks
    switch (m_currentState) {
        case RobotState::HOMING:
            // Homing timeout (e.g., 60 seconds)
            if (duration.count() > 60000) {
                LOG_ERROR("Homing timeout");
                m_lastError = ErrorCode::MOTION_FOLLOWING_ERROR;
                m_lastErrorMessage = "Homing timeout";
                if (m_errorCallback) {
                    m_errorCallback(m_lastError, m_lastErrorMessage);
                }
                executeTransition(RobotState::ERROR);
            }
            break;

        default:
            break;
    }
}

std::chrono::milliseconds StateMachine::timeInCurrentState() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - m_stateEntryTime);
}

bool StateMachine::executeTransition(RobotState newState) {
    // Note: mutex should be held by caller

    if (newState == m_currentState) {
        return true;  // No transition needed
    }

    RobotState oldState = m_currentState;

    // Execute exit actions
    onExitState(oldState);

    // Update state
    m_previousState = m_currentState;
    m_currentState = newState;
    m_stateEntryTime = std::chrono::steady_clock::now();

    // Execute entry actions
    onEnterState(newState);

    LOG_INFO("State transition: {} -> {}", toString(oldState), toString(newState));

    // Notify callback
    if (m_stateChangeCallback) {
        m_stateChangeCallback(oldState, newState);
    }

    return true;
}

void StateMachine::onEnterState(RobotState state) {
    switch (state) {
        case RobotState::IDLE:
            m_enabled = false;
            break;

        case RobotState::READY:
            m_homed = true;
            break;

        case RobotState::ERROR:
            m_enabled = false;
            break;

        case RobotState::ESTOP:
            m_enabled = false;
            break;

        default:
            break;
    }
}

void StateMachine::onExitState(RobotState state) {
    switch (state) {
        case RobotState::HOMING:
            // Homing completion handled by HOME_COMPLETE event
            break;

        default:
            break;
    }
}

} // namespace state
} // namespace robot_controller
