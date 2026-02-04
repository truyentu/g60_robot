# IMPL_P2_01: State Machine

| Metadata      | Value                           |
|---------------|---------------------------------|
| Plan ID       | IMPL_P2_01                      |
| Phase         | 2 - Motion Core                 |
| Status        | DRAFT                           |
| Version       | 1.0                             |
| Created       | 2026-02-01                      |
| Prerequisites | Phase 1 completed               |

---

## Required Reading (ĐỌC TRƯỚC KHI CODE)

| Priority | Document | Lý do |
|----------|----------|-------|
| P0 | `ressearch_doc_md/Thiết Kế FSM Robot Công Nghiệp An Toàn.md` | FSM design, states, transitions, safety interlocks |

---

## Prerequisites

Trước khi bắt đầu, đảm bảo:

| Requirement | Check |
|-------------|-------|
| Phase 1 complete | "Hello Robot" milestone achieved |
| C++ Core builds | robot_core.exe runs |
| IPC working | UI connects to Core |
| Config loads | robot_config.yaml, system_config.yaml |

---

## Overview

### State Machine Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         ROBOT STATE MACHINE                              │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│   States:                          Modes:                                │
│   ┌──────────────────────┐        ┌──────────────────────┐              │
│   │  INIT                │        │  MANUAL              │              │
│   │  IDLE                │        │  AUTO                │              │
│   │  HOMING              │        │  T1 (Reduced Speed)  │              │
│   │  READY               │        │  T2 (Full Speed)     │              │
│   │  MOVING              │        └──────────────────────┘              │
│   │  EXECUTING           │                                               │
│   │  PAUSED              │        Safety States:                         │
│   │  ERROR               │        ┌──────────────────────┐              │
│   │  ESTOP               │        │  NORMAL              │              │
│   └──────────────────────┘        │  PROTECTIVE_STOP     │              │
│                                    │  EMERGENCY_STOP      │              │
│                                    └──────────────────────┘              │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### State Transition Diagram

```
                              ┌─────────┐
                              │  INIT   │
                              └────┬────┘
                                   │ Initialize OK
                                   ▼
        ┌──────────────────►┌─────────┐◄──────────────────┐
        │                   │  IDLE   │                   │
        │                   └────┬────┘                   │
        │                        │                        │
        │         ┌──────────────┼──────────────┐         │
        │         │              │              │         │
        │         ▼              ▼              ▼         │
        │   ┌─────────┐    ┌─────────┐    ┌─────────┐    │
        │   │ HOMING  │    │  READY  │◄───│ MOVING  │    │
        │   └────┬────┘    └────┬────┘    └────┬────┘    │
        │        │              │              │         │
        │        │ Done         │ Execute      │ Done    │
        │        ▼              ▼              │         │
        │   ┌─────────┐    ┌─────────┐         │         │
        │   │  READY  │    │EXECUTING│─────────┘         │
        │   └─────────┘    └────┬────┘                   │
        │                       │                        │
        │                       ▼                        │
        │                  ┌─────────┐                   │
        │                  │ PAUSED  │───────────────────┘
        │                  └─────────┘
        │
        │   Any State + E-Stop
        │         │
        │         ▼
        │   ┌─────────┐         ┌─────────┐
        └───│  ESTOP  │◄────────│  ERROR  │
            └─────────┘         └─────────┘
                                     ▲
                                     │ Error condition
                                     │ from any state
```

---

## PART A: State Definitions

### Step 1: Create State Enums

**File:** `src/core/src/state/States.hpp`

```powershell
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller"

# Create state directory
New-Item -ItemType Directory -Force -Path "src\core\src\state"

$statesHpp = @"
/**
 * @file States.hpp
 * @brief Robot state and mode definitions
 */

#pragma once

#include <string>
#include <cstdint>

namespace robot_controller {
namespace state {

/**
 * Robot operational states
 */
enum class RobotState : uint8_t {
    INIT = 0,           // Initializing
    IDLE,               // Idle, not enabled
    HOMING,             // Homing in progress
    READY,              // Ready to move (enabled, homed)
    MOVING,             // Executing motion (jog, move)
    EXECUTING,          // Executing program
    PAUSED,             // Program paused
    ERROR,              // Error state (recoverable)
    ESTOP               // Emergency stop (requires reset)
};

/**
 * Robot operation modes
 */
enum class RobotMode : uint8_t {
    MANUAL = 0,         // Manual jog mode
    AUTO,               // Automatic program execution
    T1,                 // Teach mode 1 (reduced speed, 250mm/s max)
    T2                  // Teach mode 2 (full speed, requires enabling device)
};

/**
 * Safety states
 */
enum class SafetyState : uint8_t {
    NORMAL = 0,         // Normal operation
    PROTECTIVE_STOP,    // Protective stop (safety door, etc.)
    EMERGENCY_STOP      // Emergency stop pressed
};

/**
 * Motion types
 */
enum class MotionType : uint8_t {
    NONE = 0,
    JOG_JOINT,          // Joint space jog
    JOG_CARTESIAN,      // Cartesian space jog
    MOVE_JOINT,         // Joint interpolated move
    MOVE_LINEAR,        // Linear interpolated move
    MOVE_CIRCULAR       // Circular interpolated move
};

/**
 * State transition events
 */
enum class StateEvent : uint8_t {
    // System events
    INIT_COMPLETE = 0,
    ENABLE_REQUEST,
    DISABLE_REQUEST,
    RESET_REQUEST,

    // Homing events
    HOME_REQUEST,
    HOME_COMPLETE,
    HOME_ERROR,

    // Motion events
    MOTION_START,
    MOTION_COMPLETE,
    MOTION_ERROR,

    // Program events
    PROGRAM_START,
    PROGRAM_PAUSE,
    PROGRAM_RESUME,
    PROGRAM_STOP,
    PROGRAM_COMPLETE,
    PROGRAM_ERROR,

    // Error events
    ERROR_OCCURRED,
    ERROR_CLEARED,

    // Safety events
    ESTOP_PRESSED,
    ESTOP_RELEASED,
    SAFETY_DOOR_OPEN,
    SAFETY_DOOR_CLOSED
};

/**
 * Error codes
 */
enum class ErrorCode : uint16_t {
    NONE = 0,

    // Communication errors (100-199)
    COMM_TIMEOUT = 100,
    COMM_CRC_ERROR,
    COMM_DISCONNECTED,

    // Motion errors (200-299)
    MOTION_LIMIT_EXCEEDED = 200,
    MOTION_FOLLOWING_ERROR,
    MOTION_COLLISION,
    MOTION_SINGULARITY,
    MOTION_IK_FAILED,

    // Safety errors (300-399)
    SAFETY_ESTOP = 300,
    SAFETY_DOOR_OPEN,
    SAFETY_OVERLOAD,
    SAFETY_OVERHEAT,

    // Hardware errors (400-499)
    HW_DRIVER_FAULT = 400,
    HW_ENCODER_ERROR,
    HW_BRAKE_FAULT,
    HW_POWER_FAULT,

    // Configuration errors (500-599)
    CONFIG_INVALID = 500,
    CONFIG_MISSING,

    // General errors (900-999)
    UNKNOWN_ERROR = 999
};

// String conversion functions
inline std::string toString(RobotState state) {
    switch (state) {
        case RobotState::INIT:      return "INIT";
        case RobotState::IDLE:      return "IDLE";
        case RobotState::HOMING:    return "HOMING";
        case RobotState::READY:     return "READY";
        case RobotState::MOVING:    return "MOVING";
        case RobotState::EXECUTING: return "EXECUTING";
        case RobotState::PAUSED:    return "PAUSED";
        case RobotState::ERROR:     return "ERROR";
        case RobotState::ESTOP:     return "ESTOP";
        default:                    return "UNKNOWN";
    }
}

inline std::string toString(RobotMode mode) {
    switch (mode) {
        case RobotMode::MANUAL: return "MANUAL";
        case RobotMode::AUTO:   return "AUTO";
        case RobotMode::T1:     return "T1";
        case RobotMode::T2:     return "T2";
        default:                return "UNKNOWN";
    }
}

inline std::string toString(SafetyState state) {
    switch (state) {
        case SafetyState::NORMAL:          return "NORMAL";
        case SafetyState::PROTECTIVE_STOP: return "PROTECTIVE_STOP";
        case SafetyState::EMERGENCY_STOP:  return "EMERGENCY_STOP";
        default:                           return "UNKNOWN";
    }
}

inline std::string toString(StateEvent event) {
    switch (event) {
        case StateEvent::INIT_COMPLETE:      return "INIT_COMPLETE";
        case StateEvent::ENABLE_REQUEST:     return "ENABLE_REQUEST";
        case StateEvent::DISABLE_REQUEST:    return "DISABLE_REQUEST";
        case StateEvent::RESET_REQUEST:      return "RESET_REQUEST";
        case StateEvent::HOME_REQUEST:       return "HOME_REQUEST";
        case StateEvent::HOME_COMPLETE:      return "HOME_COMPLETE";
        case StateEvent::HOME_ERROR:         return "HOME_ERROR";
        case StateEvent::MOTION_START:       return "MOTION_START";
        case StateEvent::MOTION_COMPLETE:    return "MOTION_COMPLETE";
        case StateEvent::MOTION_ERROR:       return "MOTION_ERROR";
        case StateEvent::PROGRAM_START:      return "PROGRAM_START";
        case StateEvent::PROGRAM_PAUSE:      return "PROGRAM_PAUSE";
        case StateEvent::PROGRAM_RESUME:     return "PROGRAM_RESUME";
        case StateEvent::PROGRAM_STOP:       return "PROGRAM_STOP";
        case StateEvent::PROGRAM_COMPLETE:   return "PROGRAM_COMPLETE";
        case StateEvent::PROGRAM_ERROR:      return "PROGRAM_ERROR";
        case StateEvent::ERROR_OCCURRED:     return "ERROR_OCCURRED";
        case StateEvent::ERROR_CLEARED:      return "ERROR_CLEARED";
        case StateEvent::ESTOP_PRESSED:      return "ESTOP_PRESSED";
        case StateEvent::ESTOP_RELEASED:     return "ESTOP_RELEASED";
        case StateEvent::SAFETY_DOOR_OPEN:   return "SAFETY_DOOR_OPEN";
        case StateEvent::SAFETY_DOOR_CLOSED: return "SAFETY_DOOR_CLOSED";
        default:                             return "UNKNOWN";
    }
}

inline std::string toString(ErrorCode code) {
    switch (code) {
        case ErrorCode::NONE:                  return "NONE";
        case ErrorCode::COMM_TIMEOUT:          return "COMM_TIMEOUT";
        case ErrorCode::COMM_CRC_ERROR:        return "COMM_CRC_ERROR";
        case ErrorCode::COMM_DISCONNECTED:     return "COMM_DISCONNECTED";
        case ErrorCode::MOTION_LIMIT_EXCEEDED: return "MOTION_LIMIT_EXCEEDED";
        case ErrorCode::MOTION_FOLLOWING_ERROR:return "MOTION_FOLLOWING_ERROR";
        case ErrorCode::MOTION_COLLISION:      return "MOTION_COLLISION";
        case ErrorCode::MOTION_SINGULARITY:    return "MOTION_SINGULARITY";
        case ErrorCode::MOTION_IK_FAILED:      return "MOTION_IK_FAILED";
        case ErrorCode::SAFETY_ESTOP:          return "SAFETY_ESTOP";
        case ErrorCode::SAFETY_DOOR_OPEN:      return "SAFETY_DOOR_OPEN";
        case ErrorCode::SAFETY_OVERLOAD:       return "SAFETY_OVERLOAD";
        case ErrorCode::SAFETY_OVERHEAT:       return "SAFETY_OVERHEAT";
        case ErrorCode::HW_DRIVER_FAULT:       return "HW_DRIVER_FAULT";
        case ErrorCode::HW_ENCODER_ERROR:      return "HW_ENCODER_ERROR";
        case ErrorCode::HW_BRAKE_FAULT:        return "HW_BRAKE_FAULT";
        case ErrorCode::HW_POWER_FAULT:        return "HW_POWER_FAULT";
        case ErrorCode::CONFIG_INVALID:        return "CONFIG_INVALID";
        case ErrorCode::CONFIG_MISSING:        return "CONFIG_MISSING";
        default:                               return "UNKNOWN_ERROR";
    }
}

} // namespace state
} // namespace robot_controller
"@

Set-Content -Path "src\core\src\state\States.hpp" -Value $statesHpp -Encoding UTF8
Write-Host "[OK] States.hpp created" -ForegroundColor Green
```

---

### Step 2: Create State Machine Interface

**File:** `src/core/src/state/IStateMachine.hpp`

```powershell
$iStateMachineHpp = @"
/**
 * @file IStateMachine.hpp
 * @brief State machine interface
 */

#pragma once

#include "States.hpp"
#include <functional>
#include <string>

namespace robot_controller {
namespace state {

/**
 * State change callback signature
 */
using StateChangeCallback = std::function<void(RobotState oldState, RobotState newState)>;

/**
 * Error callback signature
 */
using ErrorCallback = std::function<void(ErrorCode code, const std::string& message)>;

/**
 * State machine interface
 */
class IStateMachine {
public:
    virtual ~IStateMachine() = default;

    // State queries
    virtual RobotState currentState() const = 0;
    virtual RobotMode currentMode() const = 0;
    virtual SafetyState safetyState() const = 0;
    virtual bool isEnabled() const = 0;
    virtual bool isHomed() const = 0;
    virtual bool isMoving() const = 0;
    virtual bool hasError() const = 0;
    virtual ErrorCode lastError() const = 0;
    virtual std::string lastErrorMessage() const = 0;

    // State control
    virtual bool processEvent(StateEvent event) = 0;
    virtual bool enable() = 0;
    virtual bool disable() = 0;
    virtual bool reset() = 0;

    // Mode control
    virtual bool setMode(RobotMode mode) = 0;

    // Error handling
    virtual void setError(ErrorCode code, const std::string& message = "") = 0;
    virtual void clearError() = 0;

    // Callbacks
    virtual void setStateChangeCallback(StateChangeCallback callback) = 0;
    virtual void setErrorCallback(ErrorCallback callback) = 0;

    // State validation
    virtual bool canTransitionTo(RobotState targetState) const = 0;
    virtual bool canExecuteMotion() const = 0;
    virtual bool canJog() const = 0;
    virtual bool canRunProgram() const = 0;
};

} // namespace state
} // namespace robot_controller
"@

Set-Content -Path "src\core\src\state\IStateMachine.hpp" -Value $iStateMachineHpp -Encoding UTF8
Write-Host "[OK] IStateMachine.hpp created" -ForegroundColor Green
```

---

## PART B: State Machine Implementation

### Step 3: Create State Machine Header

**File:** `src/core/src/state/StateMachine.hpp`

```powershell
$stateMachineHpp = @"
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
"@

Set-Content -Path "src\core\src\state\StateMachine.hpp" -Value $stateMachineHpp -Encoding UTF8
Write-Host "[OK] StateMachine.hpp created" -ForegroundColor Green
```

---

### Step 4: Create State Machine Implementation

**File:** `src/core/src/state/StateMachine.cpp`

```powershell
$stateMachineCpp = @"
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
        setError(ErrorCode::SAFETY_ESTOP, "Emergency stop activated");
        return executeTransition(RobotState::ESTOP);
    }

    // Safety door events
    if (event == StateEvent::SAFETY_DOOR_OPEN && isMoving()) {
        m_safetyState = SafetyState::PROTECTIVE_STOP;
        setError(ErrorCode::SAFETY_DOOR_OPEN, "Safety door opened during motion");
        return executeTransition(RobotState::ERROR);
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

    // Stop any motion first
    if (isMoving()) {
        LOG_WARN("Disabling while moving - stopping motion");
        // Motion controller should be notified separately
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
    if (isMoving()) {
        LOG_WARN("Cannot change mode while moving");
        return false;
    }

    m_currentMode = mode;
    LOG_INFO("Mode changed to: {}", toString(mode));
    return true;
}

void StateMachine::setError(ErrorCode code, const std::string& message) {
    // Note: mutex should be held by caller
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
                setError(ErrorCode::MOTION_FOLLOWING_ERROR, "Homing timeout");
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
"@

Set-Content -Path "src\core\src\state\StateMachine.cpp" -Value $stateMachineCpp -Encoding UTF8
Write-Host "[OK] StateMachine.cpp created" -ForegroundColor Green
```

---

## PART C: Robot Controller Class

### Step 5: Create Robot Controller

**File:** `src/core/src/controller/RobotController.hpp`

```powershell
New-Item -ItemType Directory -Force -Path "src\core\src\controller"

$robotControllerHpp = @"
/**
 * @file RobotController.hpp
 * @brief Main robot controller class
 */

#pragma once

#include "../state/StateMachine.hpp"
#include "../config/ConfigManager.hpp"
#include "../ipc/IpcServer.hpp"
#include <memory>
#include <array>
#include <atomic>

namespace robot_controller {

/**
 * Robot status data
 */
struct RobotStatus {
    state::RobotState state = state::RobotState::INIT;
    state::RobotMode mode = state::RobotMode::MANUAL;
    state::SafetyState safety = state::SafetyState::NORMAL;

    bool enabled = false;
    bool homed = false;

    std::array<double, 6> jointPositions = {0};    // degrees
    std::array<double, 6> jointVelocities = {0};   // deg/s
    std::array<double, 6> tcpPose = {0};           // X,Y,Z,Rx,Ry,Rz

    state::ErrorCode errorCode = state::ErrorCode::NONE;
    std::string errorMessage;

    double speedOverride = 100.0;  // 0-100%
};

/**
 * Main Robot Controller
 *
 * Coordinates state machine, motion, and IPC
 */
class RobotController {
public:
    RobotController();
    ~RobotController();

    // Non-copyable
    RobotController(const RobotController&) = delete;
    RobotController& operator=(const RobotController&) = delete;

    /**
     * Initialize the controller
     */
    bool initialize(const std::string& configDir = "config");

    /**
     * Start the control loop
     */
    bool start();

    /**
     * Stop the controller
     */
    void stop();

    /**
     * Check if running
     */
    bool isRunning() const { return m_running; }

    /**
     * Get current status
     */
    RobotStatus getStatus() const;

    /**
     * Get state machine
     */
    state::IStateMachine& stateMachine() { return *m_stateMachine; }

    // Commands
    bool enable();
    bool disable();
    bool reset();
    bool home();
    bool setMode(state::RobotMode mode);
    bool setSpeedOverride(double percent);

    // Jog commands (Phase 2)
    bool jogJoint(int joint, double speed);
    bool jogCartesian(int axis, double speed);
    bool stopJog();

private:
    void controlLoop();
    void updateStatus();
    void publishStatus();
    void registerIpcHandlers();

    // Components
    std::unique_ptr<state::StateMachine> m_stateMachine;
    std::unique_ptr<ipc::IpcServer> m_ipcServer;

    // Status
    RobotStatus m_status;
    mutable std::mutex m_statusMutex;

    // Control loop
    std::thread m_controlThread;
    std::atomic<bool> m_running{false};
    int m_cycleTimeMs = 4;  // 250 Hz
    int m_statusPublishHz = 10;
};

} // namespace robot_controller
"@

Set-Content -Path "src\core\src\controller\RobotController.hpp" -Value $robotControllerHpp -Encoding UTF8
Write-Host "[OK] RobotController.hpp created" -ForegroundColor Green
```

---

### Step 6: Create Robot Controller Implementation

**File:** `src/core/src/controller/RobotController.cpp`

```powershell
$robotControllerCpp = @"
/**
 * @file RobotController.cpp
 * @brief Robot controller implementation
 */

#include "RobotController.hpp"
#include "../logging/Logger.hpp"
#include <chrono>

namespace robot_controller {

using namespace state;
using namespace config;
using namespace ipc;

RobotController::RobotController()
    : m_stateMachine(std::make_unique<StateMachine>())
{
    LOG_DEBUG("RobotController created");
}

RobotController::~RobotController() {
    stop();
}

bool RobotController::initialize(const std::string& configDir) {
    LOG_INFO("Initializing RobotController...");

    // Load configuration
    auto& config = ConfigManager::instance();
    if (!config.loadAll(configDir)) {
        LOG_ERROR("Failed to load configuration");
        return false;
    }

    const auto& sysConfig = config.systemConfig();

    // Store cycle time
    m_cycleTimeMs = sysConfig.control.cycle_time_ms;
    m_statusPublishHz = sysConfig.control.status_publish_hz;

    // Create IPC server
    std::string repAddr = "tcp://" + sysConfig.ipc.bind_address + ":" +
                          std::to_string(sysConfig.ipc.rep_port);
    std::string pubAddr = "tcp://" + sysConfig.ipc.bind_address + ":" +
                          std::to_string(sysConfig.ipc.pub_port);

    m_ipcServer = std::make_unique<IpcServer>(repAddr, pubAddr);
    registerIpcHandlers();

    // Setup state machine callbacks
    m_stateMachine->setStateChangeCallback(
        [this](RobotState oldState, RobotState newState) {
            LOG_INFO("State changed: {} -> {}", toString(oldState), toString(newState));
            updateStatus();
        });

    m_stateMachine->setErrorCallback(
        [this](ErrorCode code, const std::string& msg) {
            LOG_ERROR("Error: {} - {}", toString(code), msg);
            updateStatus();
        });

    // Initialize complete
    m_stateMachine->processEvent(StateEvent::INIT_COMPLETE);

    LOG_INFO("RobotController initialized");
    return true;
}

bool RobotController::start() {
    if (m_running) {
        LOG_WARN("RobotController already running");
        return true;
    }

    // Start IPC server
    if (!m_ipcServer->start()) {
        LOG_ERROR("Failed to start IPC server");
        return false;
    }

    // Start control loop
    m_running = true;
    m_controlThread = std::thread(&RobotController::controlLoop, this);

    LOG_INFO("RobotController started");
    return true;
}

void RobotController::stop() {
    if (!m_running) {
        return;
    }

    LOG_INFO("Stopping RobotController...");

    m_running = false;

    if (m_controlThread.joinable()) {
        m_controlThread.join();
    }

    if (m_ipcServer) {
        m_ipcServer->stop();
    }

    LOG_INFO("RobotController stopped");
}

RobotStatus RobotController::getStatus() const {
    std::lock_guard<std::mutex> lock(m_statusMutex);
    return m_status;
}

bool RobotController::enable() {
    return m_stateMachine->enable();
}

bool RobotController::disable() {
    return m_stateMachine->disable();
}

bool RobotController::reset() {
    return m_stateMachine->reset();
}

bool RobotController::home() {
    if (!m_stateMachine->isEnabled()) {
        LOG_WARN("Cannot home: robot not enabled");
        return false;
    }

    if (m_stateMachine->processEvent(StateEvent::HOME_REQUEST)) {
        LOG_INFO("Homing started");

        // TODO: Actual homing implementation in Phase 2
        // For now, simulate homing complete
        std::thread([this]() {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            m_stateMachine->processEvent(StateEvent::HOME_COMPLETE);
            LOG_INFO("Homing complete (simulated)");
        }).detach();

        return true;
    }

    return false;
}

bool RobotController::setMode(RobotMode mode) {
    return m_stateMachine->setMode(mode);
}

bool RobotController::setSpeedOverride(double percent) {
    std::lock_guard<std::mutex> lock(m_statusMutex);
    m_status.speedOverride = std::clamp(percent, 0.0, 100.0);
    LOG_DEBUG("Speed override set to {}%", m_status.speedOverride);
    return true;
}

bool RobotController::jogJoint(int joint, double speed) {
    if (!m_stateMachine->canJog()) {
        LOG_WARN("Cannot jog: not in jog-able state");
        return false;
    }

    // TODO: Implement in Phase 2
    LOG_DEBUG("Jog joint {} at speed {}", joint, speed);
    return true;
}

bool RobotController::jogCartesian(int axis, double speed) {
    if (!m_stateMachine->canJog()) {
        LOG_WARN("Cannot jog: not in jog-able state");
        return false;
    }

    // TODO: Implement in Phase 2
    LOG_DEBUG("Jog Cartesian axis {} at speed {}", axis, speed);
    return true;
}

bool RobotController::stopJog() {
    // TODO: Implement in Phase 2
    LOG_DEBUG("Stop jog");
    return true;
}

void RobotController::controlLoop() {
    LOG_DEBUG("Control loop started, cycle time: {}ms", m_cycleTimeMs);

    auto lastStatusPublish = std::chrono::steady_clock::now();
    int statusIntervalMs = 1000 / m_statusPublishHz;

    while (m_running) {
        auto cycleStart = std::chrono::steady_clock::now();

        // Update state machine (check timeouts, etc.)
        m_stateMachine->update();

        // Update status
        updateStatus();

        // Publish status at configured rate
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastStatusPublish);
        if (elapsed.count() >= statusIntervalMs) {
            publishStatus();
            lastStatusPublish = now;
        }

        // TODO: Motion control updates (Phase 2)

        // Calculate sleep time to maintain cycle rate
        auto cycleEnd = std::chrono::steady_clock::now();
        auto cycleDuration = std::chrono::duration_cast<std::chrono::milliseconds>(cycleEnd - cycleStart);
        int sleepMs = m_cycleTimeMs - static_cast<int>(cycleDuration.count());
        if (sleepMs > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleepMs));
        }
    }

    LOG_DEBUG("Control loop stopped");
}

void RobotController::updateStatus() {
    std::lock_guard<std::mutex> lock(m_statusMutex);

    m_status.state = m_stateMachine->currentState();
    m_status.mode = m_stateMachine->currentMode();
    m_status.safety = m_stateMachine->safetyState();
    m_status.enabled = m_stateMachine->isEnabled();
    m_status.homed = m_stateMachine->isHomed();
    m_status.errorCode = m_stateMachine->lastError();
    m_status.errorMessage = m_stateMachine->lastErrorMessage();

    // TODO: Get actual joint positions from motion controller (Phase 2)
    // For now, use home position from config
    const auto& robotConfig = ConfigManager::instance().robotConfig();
    for (size_t i = 0; i < 6; i++) {
        m_status.jointPositions[i] = robotConfig.home_position[i];
    }
}

void RobotController::publishStatus() {
    if (!m_ipcServer) return;

    RobotStatus status = getStatus();

    nlohmann::json payload = {
        {"state", toString(status.state)},
        {"mode", toString(status.mode)},
        {"safety", toString(status.safety)},
        {"enabled", status.enabled},
        {"homed", status.homed},
        {"joints", status.jointPositions},
        {"tcp_position", status.tcpPose},
        {"speed_override", status.speedOverride},
        {"errors", nlohmann::json::array()}
    };

    if (status.errorCode != ErrorCode::NONE) {
        payload["errors"].push_back({
            {"code", static_cast<int>(status.errorCode)},
            {"message", status.errorMessage}
        });
    }

    m_ipcServer->publishStatus(payload);
}

void RobotController::registerIpcHandlers() {
    // GET_STATUS handler
    m_ipcServer->registerHandler(MessageType::GET_STATUS,
        [this](const Message& request) -> nlohmann::json {
            RobotStatus status = getStatus();
            return {
                {"state", toString(status.state)},
                {"mode", toString(status.mode)},
                {"safety", toString(status.safety)},
                {"enabled", status.enabled},
                {"homed", status.homed},
                {"joints", status.jointPositions},
                {"tcp_position", status.tcpPose},
                {"speed_override", status.speedOverride},
                {"error_code", static_cast<int>(status.errorCode)},
                {"error_message", status.errorMessage}
            };
        });

    // GET_JOINT_POSITIONS handler
    m_ipcServer->registerHandler(MessageType::GET_JOINT_POSITIONS,
        [this](const Message& request) -> nlohmann::json {
            RobotStatus status = getStatus();
            return {
                {"joints", status.jointPositions},
                {"unit", "degrees"}
            };
        });

    // GET_CONFIG handler
    m_ipcServer->registerHandler(MessageType::GET_CONFIG,
        [](const Message& request) -> nlohmann::json {
            return nlohmann::json::parse(ConfigManager::instance().robotConfigToJson());
        });

    // COMMAND handler
    m_ipcServer->registerHandler(MessageType::COMMAND,
        [this](const Message& request) -> nlohmann::json {
            std::string cmd = request.payload.value("command", "");
            bool success = false;
            std::string message;

            if (cmd == "enable") {
                success = enable();
                message = success ? "Robot enabled" : "Failed to enable";
            }
            else if (cmd == "disable") {
                success = disable();
                message = success ? "Robot disabled" : "Failed to disable";
            }
            else if (cmd == "reset") {
                success = reset();
                message = success ? "Reset complete" : "Failed to reset";
            }
            else if (cmd == "home") {
                success = home();
                message = success ? "Homing started" : "Failed to start homing";
            }
            else if (cmd == "set_mode") {
                std::string modeStr = request.payload.value("mode", "MANUAL");
                RobotMode mode = RobotMode::MANUAL;
                if (modeStr == "AUTO") mode = RobotMode::AUTO;
                else if (modeStr == "T1") mode = RobotMode::T1;
                else if (modeStr == "T2") mode = RobotMode::T2;
                success = setMode(mode);
                message = success ? "Mode changed" : "Failed to change mode";
            }
            else if (cmd == "set_speed") {
                double speed = request.payload.value("speed", 100.0);
                success = setSpeedOverride(speed);
                message = success ? "Speed set" : "Failed to set speed";
            }
            else {
                message = "Unknown command: " + cmd;
            }

            return {
                {"success", success},
                {"message", message}
            };
        });

    LOG_DEBUG("IPC handlers registered");
}

} // namespace robot_controller
"@

Set-Content -Path "src\core\src\controller\RobotController.cpp" -Value $robotControllerCpp -Encoding UTF8
Write-Host "[OK] RobotController.cpp created" -ForegroundColor Green
```

---

### Step 7: Update main.cpp

**File:** `src/core/src/main.cpp`

```powershell
$mainCpp = @"
/**
 * @file main.cpp
 * @brief Robot Controller Core - Entry Point
 */

#include <iostream>
#include <csignal>
#include <atomic>

#include "logging/Logger.hpp"
#include "controller/RobotController.hpp"

// Global controller pointer for signal handler
robot_controller::RobotController* g_controller = nullptr;

void signalHandler(int signal) {
    LOG_INFO("Received signal {}, shutting down...", signal);
    if (g_controller) {
        g_controller->stop();
    }
}

int main(int argc, char* argv[]) {
    // Setup signal handlers
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // Determine config directory
    std::string configDir = "../../config";
    if (argc > 1) {
        configDir = argv[1];
    }

    // Initialize logging
    robot_controller::Logger::init("../../logs/core.log", "debug");

    LOG_INFO("========================================");
    LOG_INFO("Robot Controller Core v1.0.0");
    LOG_INFO("========================================");

    // Create and initialize controller
    robot_controller::RobotController controller;
    g_controller = &controller;

    if (!controller.initialize(configDir)) {
        LOG_ERROR("Failed to initialize controller");
        return 1;
    }

    // Start controller
    if (!controller.start()) {
        LOG_ERROR("Failed to start controller");
        return 1;
    }

    LOG_INFO("Robot Controller running. Press Ctrl+C to exit.");

    // Wait for shutdown
    while (controller.isRunning()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    LOG_INFO("Robot Controller Core shutdown complete");
    return 0;
}
"@

Set-Content -Path "src\core\src\main.cpp" -Value $mainCpp -Encoding UTF8
Write-Host "[OK] main.cpp updated" -ForegroundColor Green
```

---

## PART D: Update CMakeLists.txt

### Step 8: Update Build Configuration

```powershell
$cmakeContent = @"
cmake_minimum_required(VERSION 3.20)
project(RobotControllerCore VERSION 1.0.0 LANGUAGES CXX)

# ============================================================================
# Build Settings
# ============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY `${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY `${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY `${CMAKE_BINARY_DIR}/lib)

# ============================================================================
# vcpkg Integration
# ============================================================================
if(DEFINED ENV{VCPKG_ROOT})
    set(CMAKE_TOOLCHAIN_FILE "`$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "Vcpkg toolchain file")
elseif(EXISTS "C:/vcpkg/scripts/buildsystems/vcpkg.cmake")
    set(CMAKE_TOOLCHAIN_FILE "C:/vcpkg/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "Vcpkg toolchain file")
endif()

# ============================================================================
# Find Dependencies
# ============================================================================
find_package(spdlog CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(cppzmq CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)
find_package(GTest CONFIG REQUIRED)

# ============================================================================
# Source Files
# ============================================================================
set(CORE_SOURCES
    src/main.cpp
    src/logging/Logger.cpp
    src/config/ConfigManager.cpp
    src/ipc/IpcServer.cpp
    src/state/StateMachine.cpp
    src/controller/RobotController.cpp
    src/robot/RobotModel.cpp
)

set(CORE_HEADERS
    include/robot_controller/core.hpp
    src/logging/Logger.hpp
    src/config/ConfigManager.hpp
    src/config/RobotConfig.hpp
    src/config/SystemConfig.hpp
    src/ipc/IpcServer.hpp
    src/ipc/Message.hpp
    src/ipc/MessageTypes.hpp
    src/state/States.hpp
    src/state/IStateMachine.hpp
    src/state/StateMachine.hpp
    src/controller/RobotController.hpp
    src/robot/RobotModel.hpp
    src/robot/DHParameters.hpp
)

# ============================================================================
# Main Executable
# ============================================================================
add_executable(robot_core `${CORE_SOURCES} `${CORE_HEADERS})

target_include_directories(robot_core
    PRIVATE
        `${CMAKE_CURRENT_SOURCE_DIR}/src
        `${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(robot_core
    PRIVATE
        spdlog::spdlog
        nlohmann_json::nlohmann_json
        cppzmq
        yaml-cpp::yaml-cpp
)

if(WIN32)
    target_compile_definitions(robot_core PRIVATE
        _WIN32_WINNT=0x0A00
        NOMINMAX
    )
endif()

# ============================================================================
# Tests
# ============================================================================
enable_testing()

add_executable(robot_core_tests
    tests/test_main.cpp
    tests/test_config.cpp
    tests/test_ipc.cpp
    tests/test_state_machine.cpp
    src/logging/Logger.cpp
    src/config/ConfigManager.cpp
    src/state/StateMachine.cpp
)

target_include_directories(robot_core_tests
    PRIVATE
        `${CMAKE_CURRENT_SOURCE_DIR}/src
        `${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(robot_core_tests
    PRIVATE
        GTest::gtest
        GTest::gtest_main
        spdlog::spdlog
        nlohmann_json::nlohmann_json
        cppzmq
        yaml-cpp::yaml-cpp
)

include(GoogleTest)
gtest_discover_tests(robot_core_tests)

# ============================================================================
# Summary
# ============================================================================
message(STATUS "")
message(STATUS "=== Robot Controller Core ===")
message(STATUS "Version:      `${PROJECT_VERSION}")
message(STATUS "C++ Standard: `${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type:   `${CMAKE_BUILD_TYPE}")
message(STATUS "")
"@

Set-Content -Path "src\core\CMakeLists.txt" -Value $cmakeContent -Encoding UTF8
Write-Host "[OK] CMakeLists.txt updated" -ForegroundColor Green
```

---

## PART E: State Machine Tests

### Step 9: Create State Machine Tests

**File:** `src/core/tests/test_state_machine.cpp`

```powershell
$testStateMachineCpp = @"
/**
 * @file test_state_machine.cpp
 * @brief State machine tests
 */

#include <gtest/gtest.h>
#include "state/StateMachine.hpp"
#include "logging/Logger.hpp"

using namespace robot_controller::state;

class StateMachineTest : public ::testing::Test {
protected:
    void SetUp() override {
        robot_controller::Logger::init("test_state_machine.log", "debug");
        sm = std::make_unique<StateMachine>();
    }

    std::unique_ptr<StateMachine> sm;
};

// Initial state tests
TEST_F(StateMachineTest, InitialState) {
    EXPECT_EQ(sm->currentState(), RobotState::INIT);
    EXPECT_EQ(sm->currentMode(), RobotMode::MANUAL);
    EXPECT_FALSE(sm->isEnabled());
    EXPECT_FALSE(sm->isHomed());
}

TEST_F(StateMachineTest, InitToIdle) {
    EXPECT_TRUE(sm->processEvent(StateEvent::INIT_COMPLETE));
    EXPECT_EQ(sm->currentState(), RobotState::IDLE);
}

// Enable/Disable tests
TEST_F(StateMachineTest, EnableFromIdle) {
    sm->processEvent(StateEvent::INIT_COMPLETE);
    EXPECT_TRUE(sm->enable());
    EXPECT_TRUE(sm->isEnabled());
}

TEST_F(StateMachineTest, DisableFromEnabled) {
    sm->processEvent(StateEvent::INIT_COMPLETE);
    sm->enable();
    EXPECT_TRUE(sm->disable());
    EXPECT_FALSE(sm->isEnabled());
}

// Homing tests
TEST_F(StateMachineTest, HomingSequence) {
    sm->processEvent(StateEvent::INIT_COMPLETE);
    sm->enable();

    // Start homing
    EXPECT_TRUE(sm->processEvent(StateEvent::HOME_REQUEST));
    EXPECT_EQ(sm->currentState(), RobotState::HOMING);

    // Complete homing
    EXPECT_TRUE(sm->processEvent(StateEvent::HOME_COMPLETE));
    EXPECT_EQ(sm->currentState(), RobotState::READY);
    EXPECT_TRUE(sm->isHomed());
}

TEST_F(StateMachineTest, CannotHomeWithoutEnable) {
    sm->processEvent(StateEvent::INIT_COMPLETE);
    // Not enabled
    EXPECT_FALSE(sm->processEvent(StateEvent::HOME_REQUEST));
    EXPECT_EQ(sm->currentState(), RobotState::IDLE);
}

// Motion tests
TEST_F(StateMachineTest, MotionFromReady) {
    sm->processEvent(StateEvent::INIT_COMPLETE);
    sm->enable();
    sm->processEvent(StateEvent::HOME_REQUEST);
    sm->processEvent(StateEvent::HOME_COMPLETE);

    EXPECT_TRUE(sm->processEvent(StateEvent::MOTION_START));
    EXPECT_EQ(sm->currentState(), RobotState::MOVING);
    EXPECT_TRUE(sm->isMoving());

    EXPECT_TRUE(sm->processEvent(StateEvent::MOTION_COMPLETE));
    EXPECT_EQ(sm->currentState(), RobotState::READY);
}

TEST_F(StateMachineTest, CanJogInManualMode) {
    sm->processEvent(StateEvent::INIT_COMPLETE);
    sm->enable();
    sm->processEvent(StateEvent::HOME_REQUEST);
    sm->processEvent(StateEvent::HOME_COMPLETE);

    EXPECT_TRUE(sm->canJog());
}

// Mode tests
TEST_F(StateMachineTest, ChangeModeWhenReady) {
    sm->processEvent(StateEvent::INIT_COMPLETE);
    sm->enable();
    sm->processEvent(StateEvent::HOME_REQUEST);
    sm->processEvent(StateEvent::HOME_COMPLETE);

    EXPECT_TRUE(sm->setMode(RobotMode::AUTO));
    EXPECT_EQ(sm->currentMode(), RobotMode::AUTO);
}

// E-Stop tests
TEST_F(StateMachineTest, EStopFromAnyState) {
    sm->processEvent(StateEvent::INIT_COMPLETE);
    sm->enable();
    sm->processEvent(StateEvent::HOME_REQUEST);
    sm->processEvent(StateEvent::HOME_COMPLETE);
    sm->processEvent(StateEvent::MOTION_START);

    // E-Stop while moving
    EXPECT_TRUE(sm->processEvent(StateEvent::ESTOP_PRESSED));
    EXPECT_EQ(sm->currentState(), RobotState::ESTOP);
    EXPECT_FALSE(sm->isEnabled());
}

TEST_F(StateMachineTest, ResetFromEStop) {
    sm->processEvent(StateEvent::INIT_COMPLETE);
    sm->processEvent(StateEvent::ESTOP_PRESSED);
    EXPECT_EQ(sm->currentState(), RobotState::ESTOP);

    // Cannot reset while E-Stop active
    EXPECT_FALSE(sm->reset());

    // Release E-Stop first
    sm->processEvent(StateEvent::ESTOP_RELEASED);
    EXPECT_EQ(sm->currentState(), RobotState::ERROR);

    // Now can reset
    EXPECT_TRUE(sm->reset());
    EXPECT_EQ(sm->currentState(), RobotState::IDLE);
}

// Error tests
TEST_F(StateMachineTest, ErrorSetsState) {
    sm->processEvent(StateEvent::INIT_COMPLETE);
    sm->setError(ErrorCode::MOTION_LIMIT_EXCEEDED, "Joint 1 limit exceeded");

    EXPECT_TRUE(sm->hasError());
    EXPECT_EQ(sm->lastError(), ErrorCode::MOTION_LIMIT_EXCEEDED);
}

TEST_F(StateMachineTest, ClearError) {
    sm->processEvent(StateEvent::INIT_COMPLETE);
    sm->setError(ErrorCode::MOTION_LIMIT_EXCEEDED);

    sm->clearError();
    EXPECT_EQ(sm->lastError(), ErrorCode::NONE);
}

// Callback tests
TEST_F(StateMachineTest, StateChangeCallback) {
    bool callbackCalled = false;
    RobotState oldState, newState;

    sm->setStateChangeCallback([&](RobotState o, RobotState n) {
        callbackCalled = true;
        oldState = o;
        newState = n;
    });

    sm->processEvent(StateEvent::INIT_COMPLETE);

    EXPECT_TRUE(callbackCalled);
    EXPECT_EQ(oldState, RobotState::INIT);
    EXPECT_EQ(newState, RobotState::IDLE);
}
"@

Set-Content -Path "src\core\tests\test_state_machine.cpp" -Value $testStateMachineCpp -Encoding UTF8
Write-Host "[OK] test_state_machine.cpp created" -ForegroundColor Green
```

---

## Step 10: Build and Test

### 10.1 Build C++ Core

```powershell
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller\src\core"

# Clean and rebuild
Remove-Item -Recurse -Force build -ErrorAction SilentlyContinue

cmake -B build -G "Visual Studio 17 2022" -A x64 `
    -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake"

cmake --build build --config Release

if ($LASTEXITCODE -ne 0) {
    Write-Host "[ERROR] Build failed" -ForegroundColor Red
    exit 1
}

Write-Host "[OK] C++ Core built successfully" -ForegroundColor Green
```

### 10.2 Run Tests

```powershell
cmake --build build --config Release --target robot_core_tests
ctest --test-dir build -C Release --output-on-failure -R StateMachine

Write-Host "[OK] State machine tests completed" -ForegroundColor Green
```

**Expected Output:**
```
[==========] Running 12 tests from 1 test suite.
...
[  PASSED  ] 12 tests.
```

---

## Step 11: Git Commit

```powershell
Set-Location "E:\DEV_CONTEXT_PROJECTs\Robot_controller"

git add .
git commit -m "IMPL_P2_01: State Machine implementation

State Definitions:
- RobotState enum (INIT, IDLE, HOMING, READY, MOVING, etc.)
- RobotMode enum (MANUAL, AUTO, T1, T2)
- SafetyState enum (NORMAL, PROTECTIVE_STOP, EMERGENCY_STOP)
- StateEvent enum for transitions
- ErrorCode enum for error handling

State Machine:
- Transition table with guard conditions
- State change callbacks
- Error handling and recovery
- E-Stop priority handling
- Thread-safe implementation

Robot Controller:
- Integrates StateMachine with IPC
- Command handling (enable, disable, reset, home)
- Status publishing
- Control loop framework

Tests:
- 12 state machine unit tests
- Transition validation
- E-Stop behavior tests

Tasks completed: State Machine core for Phase 2

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Completion Checklist

| Item | Status |
|------|--------|
| States.hpp created | [ ] |
| IStateMachine.hpp created | [ ] |
| StateMachine.hpp created | [ ] |
| StateMachine.cpp created | [ ] |
| RobotController.hpp created | [ ] |
| RobotController.cpp created | [ ] |
| main.cpp updated | [ ] |
| CMakeLists.txt updated | [ ] |
| test_state_machine.cpp created | [ ] |
| C++ builds successfully | [ ] |
| State machine tests pass | [ ] |
| Integration test passes | [ ] |
| Git commit created | [ ] |

---

## Troubleshooting

### Problem: Transition not happening

**Solution:**
- Check guard condition returns true
- Verify current state matches fromState
- Check logs for "No transition found"

### Problem: E-Stop not working

**Solution:**
- E-Stop is checked before transition table
- Verify SafetyState is updated

### Problem: Callback not called

**Solution:**
- Ensure callback is set before transition
- Check callback is not null

---

## Next Steps

After completing IMPL_P2_01:
1. Update IMPLEMENTATION_PLAN_TRACKER.md
2. Proceed to **IMPL_P2_02: Kinematics Engine**

---

*Document Version: 1.0 | Created: 2026-02-01*
