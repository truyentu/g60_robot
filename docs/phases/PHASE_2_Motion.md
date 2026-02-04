# PHASE 2: MOTION CORE

## Document Info
| Item | Value |
|------|-------|
| **Phase** | 2 - Motion Core |
| **Status** | Planning |
| **Version** | 1.0 |
| **Last Updated** | 2026-02-01 |
| **Prerequisites** | Phase 1 Complete |
| **Next Phase** | Phase 3 - Welding |

---

## 1. PHASE OVERVIEW

### 1.1. Objective
Xây dựng Motion Core - "trái tim" của Robot Controller, bao gồm State Machine, Kinematics, Trajectory Generation, và kết nối với Firmware. Sau Phase này, robot có thể di chuyển được bằng lệnh Jog.

### 1.2. Scope

**In Scope:**
- SystemStateManager (FSM) theo ISO 10218-1
- Safety Interlocks (E-Stop, Deadman, Limits)
- Kinematics Module (IK/FK)
- Trajectory Generator (Ruckig OTG)
- grblHAL Firmware integration
- Jog Mode (Joint & Cartesian)
- Simulation Mode (VirtualController)
- Motion visualization

**Out of Scope:**
- Welding sequences
- Vision/Sensor integration
- Program execution
- Teach pendant

### 1.3. Milestone: "Robot Moves"

Phase 2 hoàn thành khi đạt được:
- [ ] State Machine hoạt động đúng các transitions
- [ ] E-Stop kích hoạt → Robot dừng ngay lập tức
- [ ] Jog Joint mode: Điều khiển từng trục
- [ ] Jog Cartesian mode: Điều khiển TCP (X, Y, Z, Rx, Ry, Rz)
- [ ] Simulation Mode: Robot ảo di chuyển giống robot thật
- [ ] Real Mode: Firmware nhận lệnh và điều khiển motor

---

## 2. ARCHITECTURE

### 2.1. Phase 2 Components

```
┌─────────────────────────────────────────────────────────────────┐
│                      PHASE 2 SCOPE                               │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  UI LAYER (C# WPF) - Extensions from Phase 1               │ │
│  │                                                             │ │
│  │  ┌───────────┐  ┌───────────┐  ┌───────────┐              │ │
│  │  │  Jog      │  │  Status   │  │  Ghost    │              │ │
│  │  │  Panel    │  │  Display  │  │  Robot    │              │ │
│  │  └───────────┘  └───────────┘  └───────────┘              │ │
│  │                                                             │ │
│  └────────────────────────────────────────────────────────────┘ │
│                              │                                   │
│                         ZeroMQ                                   │
│                              │                                   │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  CORE LAYER (C++) - Main Phase 2 Implementation            │ │
│  │                                                             │ │
│  │  ┌─────────────────────────────────────────────────────┐   │ │
│  │  │              SystemStateManager (FSM)               │   │ │
│  │  │  ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐   │   │ │
│  │  │  │BOOT │→│IDLE │→│ARM- │→│OPER-│→│STOP-│→│ESTOP│   │   │ │
│  │  │  │     │ │     │ │ING  │ │ATION│ │PING │ │     │   │   │ │
│  │  │  └─────┘ └─────┘ └─────┘ └─────┘ └─────┘ └─────┘   │   │ │
│  │  └─────────────────────────────────────────────────────┘   │ │
│  │         │                                                   │ │
│  │         ▼                                                   │ │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐         │ │
│  │  │  Safety     │  │  Kinematics │  │  Trajectory │         │ │
│  │  │  Interlock  │  │  (IK/FK)    │  │  (Ruckig)   │         │ │
│  │  └─────────────┘  └──────┬──────┘  └──────┬──────┘         │ │
│  │                          │                │                 │ │
│  │                          └────────┬───────┘                 │ │
│  │                                   │                         │ │
│  │                          ┌────────┴────────┐                │ │
│  │                          │  Motion Queue   │                │ │
│  │                          └────────┬────────┘                │ │
│  │                                   │                         │ │
│  │         ┌─────────────────────────┼─────────────────────┐   │ │
│  │         │                         │                     │   │ │
│  │         ▼                         ▼                     │   │ │
│  │  ┌─────────────┐           ┌─────────────┐              │   │ │
│  │  │  Virtual    │           │  grblHAL    │              │   │ │
│  │  │ Controller  │           │  Driver     │              │   │ │
│  │  │ (Simulation)│           │  (Real HW)  │              │   │ │
│  │  └─────────────┘           └──────┬──────┘              │   │ │
│  │                                   │                     │   │ │
│  └───────────────────────────────────┼─────────────────────┘   │ │
│                                      │                          │
│                                Serial/USB                       │
│                                      │                          │
│  ┌───────────────────────────────────┼─────────────────────────┐│
│  │  FIRMWARE (Teensy 4.1 + grblHAL)  │                         ││
│  │                            ┌──────┴──────┐                  ││
│  │                            │  grblHAL    │                  ││
│  │                            │  (6-Axis)   │                  ││
│  │                            └──────┬──────┘                  ││
│  │                                   │                         ││
│  │                    ┌──────────────┼──────────────┐          ││
│  │                    │              │              │          ││
│  │               ┌────┴────┐   ┌─────┴─────┐  ┌────┴────┐     ││
│  │               │ Step/Dir│   │  Encoder  │  │ Safety  │     ││
│  │               │ Output  │   │  Input    │  │  I/O    │     ││
│  │               └─────────┘   └───────────┘  └─────────┘     ││
│  └─────────────────────────────────────────────────────────────┘│
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2. Data Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                       JOG COMMAND FLOW                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  User Input (UI)                                                 │
│       │                                                          │
│       ▼                                                          │
│  ┌─────────────┐                                                │
│  │ Jog Request │  { axis: 1, direction: +1, speed: 50% }        │
│  └──────┬──────┘                                                │
│         │ IPC                                                    │
│         ▼                                                        │
│  ┌─────────────┐                                                │
│  │ StateManager│  Check: State == OPERATIONAL?                  │
│  │             │  Check: Mode == MANUAL?                        │
│  │             │  Check: Deadman == ON? (if T1/T2)              │
│  └──────┬──────┘                                                │
│         │ Validated                                              │
│         ▼                                                        │
│  ┌─────────────┐                                                │
│  │ Kinematics  │  If Cartesian: IK → Joint angles               │
│  │   (IK/FK)   │  Check: Joint limits OK?                       │
│  └──────┬──────┘                                                │
│         │ Target joints                                          │
│         ▼                                                        │
│  ┌─────────────┐                                                │
│  │   Ruckig    │  Generate trajectory with:                     │
│  │    OTG      │  - Max velocity                                │
│  │             │  - Max acceleration                            │
│  │             │  - Max jerk (smooth motion)                    │
│  └──────┬──────┘                                                │
│         │ Position/Velocity commands @ 1kHz                     │
│         ▼                                                        │
│  ┌─────────────┐     OR      ┌─────────────┐                    │
│  │  Virtual    │◄───────────►│   grblHAL   │                    │
│  │ Controller  │  (Switch)   │   Driver    │                    │
│  └──────┬──────┘             └──────┬──────┘                    │
│         │                           │                            │
│         ▼                           ▼                            │
│  ┌─────────────┐             ┌─────────────┐                    │
│  │  Update UI  │             │   Motors    │                    │
│  │  (Simulated)│             │  (Physical) │                    │
│  └─────────────┘             └─────────────┘                    │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 2.3. Folder Structure (Additions to Phase 1)

```
📁 src/core/
├── 📁 src/
│   ├── 📁 state/                       # NEW - State Machine
│   │   ├── SystemStateManager.hpp
│   │   ├── SystemStateManager.cpp
│   │   ├── States.hpp                  # State definitions
│   │   ├── Events.hpp                  # Event definitions
│   │   └── Transitions.hpp             # Transition table
│   │
│   ├── 📁 safety/                      # NEW - Safety System
│   │   ├── SafetyManager.hpp
│   │   ├── SafetyManager.cpp
│   │   ├── SafetySignals.hpp
│   │   └── Interlock.hpp
│   │
│   ├── 📁 kinematics/                  # NEW - Kinematics
│   │   ├── Kinematics.hpp
│   │   ├── Kinematics.cpp
│   │   ├── ForwardKinematics.hpp
│   │   ├── InverseKinematics.hpp
│   │   └── DHParameters.hpp
│   │
│   ├── 📁 trajectory/                  # NEW - Trajectory
│   │   ├── TrajectoryGenerator.hpp
│   │   ├── TrajectoryGenerator.cpp
│   │   ├── RuckigWrapper.hpp
│   │   └── MotionProfile.hpp
│   │
│   ├── 📁 motion/                      # NEW - Motion Control
│   │   ├── MotionController.hpp
│   │   ├── MotionController.cpp
│   │   ├── JogController.hpp
│   │   ├── JogController.cpp
│   │   └── MotionQueue.hpp
│   │
│   ├── 📁 hardware/                    # NEW - Hardware Interface
│   │   ├── IHardwareInterface.hpp      # Abstract interface
│   │   ├── GrblHalDriver.hpp
│   │   ├── GrblHalDriver.cpp
│   │   ├── VirtualController.hpp       # Simulation
│   │   └── VirtualController.cpp
│   │
│   └── 📁 ipc/                         # Extended
│       └── (add new message handlers)
│
├── 📁 third_party/
│   ├── ruckig/                         # NEW - Ruckig library
│   └── rl/                             # NEW - Robotics Library (optional)
│
└── 📁 tests/
    ├── test_state_machine.cpp
    ├── test_kinematics.cpp
    ├── test_trajectory.cpp
    └── test_jog.cpp
```

---

## 3. TASK BREAKDOWN

### 3.1. Task Summary

| ID | Task | Priority | Status |
|----|------|----------|--------|
| P2-01 | State Definitions | P0 | Todo |
| P2-02 | Event Definitions | P0 | Todo |
| P2-03 | SystemStateManager Core | P0 | Todo |
| P2-04 | State Transition Logic | P0 | Todo |
| P2-05 | Safety Signal Manager | P0 | Todo |
| P2-06 | E-Stop Handling | P0 | Todo |
| P2-07 | Forward Kinematics | P0 | Todo |
| P2-08 | Inverse Kinematics | P0 | Todo |
| P2-09 | Ruckig Integration | P0 | Todo |
| P2-10 | Motion Controller | P0 | Todo |
| P2-11 | Jog Controller | P1 | Todo |
| P2-12 | Hardware Interface Abstraction | P1 | Todo |
| P2-13 | grblHAL Driver | P1 | Todo |
| P2-14 | VirtualController | P1 | Todo |
| P2-15 | Jog UI Panel | P1 | Todo |
| P2-16 | Status Display UI | P1 | Todo |
| P2-17 | Ghost Robot Visualization | P2 | Todo |
| P2-18 | Integration Testing | P2 | Todo |

### 3.2. Task Details

---

#### P2-01: State Definitions

| Attribute | Value |
|-----------|-------|
| **Priority** | P0 - Critical |
| **Dependencies** | Phase 1 |
| **Estimated Effort** | Small |
| **Reference** | Thiết Kế FSM Robot Công Nghiệp An Toàn.md |

**Description:**
Định nghĩa các trạng thái của robot theo chuẩn ISO 10218-1.

**State Definitions:**

```cpp
// States.hpp
#pragma once

#include <variant>
#include <chrono>
#include <string>

namespace robot_controller::state {

// State data structures
struct StateBoot {
    std::chrono::steady_clock::time_point start_time;
    int init_progress = 0;  // 0-100%
};

struct StateErrorLockout {
    uint32_t error_code;
    std::string error_message;
    std::chrono::steady_clock::time_point timestamp;
};

struct StateEstopActive {
    std::chrono::steady_clock::time_point activated_at;
    bool channel_a_triggered;
    bool channel_b_triggered;
};

struct StateEstopResetNeeded {
    // Waiting for manual reset after E-Stop released
};

struct StateIdle {
    // System ready, servos off, brakes engaged
};

struct StateArming {
    int arming_step = 0;  // 0: Check interlocks, 1: Enable drives, 2: Release brakes
    std::chrono::steady_clock::time_point step_start;
};

struct StateOperational {
    // Sub-states for operational mode
    enum class SubState {
        MANUAL_IDLE,        // Waiting for jog command
        MANUAL_JOGGING,     // Currently jogging
        AUTO_IDLE,          // Waiting for program start
        AUTO_RUNNING,       // Executing program
        AUTO_PAUSED         // Program paused
    };
    SubState sub_state = SubState::MANUAL_IDLE;
};

struct StateStopping {
    enum class StopCategory { CAT0, CAT1, CAT2 };
    StopCategory category;
    std::chrono::steady_clock::time_point initiated_at;
    double deceleration_progress = 0.0;  // 0-100%
};

// State variant - only one state active at a time
using SystemState = std::variant<
    StateBoot,
    StateErrorLockout,
    StateEstopActive,
    StateEstopResetNeeded,
    StateIdle,
    StateArming,
    StateOperational,
    StateStopping
>;

// Operating modes
enum class OperatingMode {
    T1,     // Manual reduced speed (250mm/s max)
    T2,     // Manual high speed
    AUTO    // Automatic mode
};

// Helper to get state name
std::string getStateName(const SystemState& state);

} // namespace robot_controller::state
```

**Acceptance Criteria:**
- [ ] Tất cả states định nghĩa theo ISO 10218-1
- [ ] State variant compile và type-safe
- [ ] Operating modes định nghĩa đúng

**Deliverables:**
- States.hpp
- Unit tests

---

#### P2-02: Event Definitions

| Attribute | Value |
|-----------|-------|
| **Priority** | P0 - Critical |
| **Dependencies** | P2-01 |
| **Estimated Effort** | Small |

**Description:**
Định nghĩa các events kích hoạt state transitions.

**Event Definitions:**

```cpp
// Events.hpp
#pragma once

#include <variant>
#include <array>

namespace robot_controller::state {

// Hardware events
struct EvEstopAsserted {};
struct EvEstopReleased {};
struct EvDeadmanPressed {};
struct EvDeadmanReleased {};
struct EvDeadmanPanic {};      // Squeezed too hard
struct EvSafeguardOpen {};     // Door/light curtain
struct EvSafeguardClosed {};

// System events
struct EvInitComplete {};
struct EvHardwareFault {
    uint32_t fault_code;
    std::string description;
};
struct EvServoReady {};
struct EvServoFault {
    int axis;
    uint32_t fault_code;
};

// Command events (from UI)
struct EvCmdReset {};
struct EvCmdServoOn {};
struct EvCmdServoOff {};
struct EvCmdStartProgram {};
struct EvCmdPauseProgram {};
struct EvCmdResumeProgram {};
struct EvCmdStopProgram {};
struct EvCmdJogStart {
    int axis;           // 0-5 for joints, or -1 for Cartesian
    int direction;      // +1 or -1
    double speed_ratio; // 0.0 - 1.0
};
struct EvCmdJogStop {};

// Mode change events
struct EvModeChanged {
    OperatingMode new_mode;
};

// Motion events
struct EvMotionComplete {};
struct EvMotionError {
    std::string error;
};
struct EvVelocityZero {};  // Robot has stopped moving

// Event variant
using Event = std::variant<
    // Hardware
    EvEstopAsserted,
    EvEstopReleased,
    EvDeadmanPressed,
    EvDeadmanReleased,
    EvDeadmanPanic,
    EvSafeguardOpen,
    EvSafeguardClosed,
    // System
    EvInitComplete,
    EvHardwareFault,
    EvServoReady,
    EvServoFault,
    // Commands
    EvCmdReset,
    EvCmdServoOn,
    EvCmdServoOff,
    EvCmdStartProgram,
    EvCmdPauseProgram,
    EvCmdResumeProgram,
    EvCmdStopProgram,
    EvCmdJogStart,
    EvCmdJogStop,
    // Mode
    EvModeChanged,
    // Motion
    EvMotionComplete,
    EvMotionError,
    EvVelocityZero
>;

// Helper to get event name
std::string getEventName(const Event& event);

} // namespace robot_controller::state
```

**Acceptance Criteria:**
- [ ] Tất cả events cần thiết đã định nghĩa
- [ ] Events có thể serialize cho logging
- [ ] Event names cho debugging

**Deliverables:**
- Events.hpp
- Unit tests

---

#### P2-03: SystemStateManager Core

| Attribute | Value |
|-----------|-------|
| **Priority** | P0 - Critical |
| **Dependencies** | P2-01, P2-02, P1-09 |
| **Estimated Effort** | Large |
| **Reference** | Thiết Kế FSM Robot Công Nghiệp An Toàn.md (Section 6) |

**Description:**
Implement core SystemStateManager class sử dụng std::variant và std::visit.

**Class Design:**

```cpp
// SystemStateManager.hpp
#pragma once

#include "States.hpp"
#include "Events.hpp"
#include "../logging/Logger.hpp"

#include <functional>
#include <mutex>
#include <queue>
#include <optional>

namespace robot_controller::state {

// Forward declarations
class SafetyManager;
class MotionController;

class SystemStateManager {
public:
    SystemStateManager();
    ~SystemStateManager();

    // Lifecycle
    void initialize();
    void shutdown();

    // Event processing
    void postEvent(const Event& event);
    void processEvents();  // Call in main loop

    // State queries
    const SystemState& currentState() const { return m_current_state; }
    OperatingMode currentMode() const { return m_operating_mode; }
    bool isOperational() const;
    bool isMovementAllowed() const;
    bool isJoggingAllowed() const;

    // Dependency injection
    void setSafetyManager(SafetyManager* safety) { m_safety = safety; }
    void setMotionController(MotionController* motion) { m_motion = motion; }

    // Callbacks for state changes
    using StateChangeCallback = std::function<void(const SystemState&, const SystemState&)>;
    void setStateChangeCallback(StateChangeCallback cb) { m_state_change_cb = cb; }

private:
    // Event handlers for each state (using visitor pattern)
    class StateEventHandler;

    // Process single event
    std::optional<SystemState> handleEvent(const Event& event);

    // Transition helpers
    void transitionTo(const SystemState& new_state);
    void onEnterState(const SystemState& state);
    void onExitState(const SystemState& state);

    // Current state
    SystemState m_current_state;
    OperatingMode m_operating_mode = OperatingMode::T1;

    // Event queue (thread-safe)
    std::queue<Event> m_event_queue;
    std::mutex m_queue_mutex;

    // Dependencies
    SafetyManager* m_safety = nullptr;
    MotionController* m_motion = nullptr;

    // Callbacks
    StateChangeCallback m_state_change_cb;
};

} // namespace robot_controller::state
```

**Implementation Pattern (std::visit):**

```cpp
// SystemStateManager.cpp (partial)

std::optional<SystemState> SystemStateManager::handleEvent(const Event& event) {
    // Visitor that handles events based on current state
    return std::visit([this, &event](auto&& current_state) -> std::optional<SystemState> {
        using StateT = std::decay_t<decltype(current_state)>;

        // E-Stop event handling - works from ANY state
        if (std::holds_alternative<EvEstopAsserted>(event)) {
            LOG_WARN("E-STOP ASSERTED!");
            return StateEstopActive{
                std::chrono::steady_clock::now(),
                true, true  // Both channels
            };
        }

        // State-specific event handling
        if constexpr (std::is_same_v<StateT, StateIdle>) {
            return handleIdleState(current_state, event);
        }
        else if constexpr (std::is_same_v<StateT, StateArming>) {
            return handleArmingState(current_state, event);
        }
        else if constexpr (std::is_same_v<StateT, StateOperational>) {
            return handleOperationalState(current_state, event);
        }
        else if constexpr (std::is_same_v<StateT, StateEstopActive>) {
            return handleEstopActiveState(current_state, event);
        }
        else if constexpr (std::is_same_v<StateT, StateEstopResetNeeded>) {
            return handleEstopResetNeededState(current_state, event);
        }
        else if constexpr (std::is_same_v<StateT, StateStopping>) {
            return handleStoppingState(current_state, event);
        }
        // ... other states

        return std::nullopt;  // No transition
    }, m_current_state);
}

std::optional<SystemState> SystemStateManager::handleIdleState(
    const StateIdle& state, const Event& event) {

    return std::visit([this](auto&& ev) -> std::optional<SystemState> {
        using EventT = std::decay_t<decltype(ev)>;

        if constexpr (std::is_same_v<EventT, EvCmdServoOn>) {
            // Guard: Check all interlocks before arming
            if (m_safety && !m_safety->areInterlocksOk()) {
                LOG_WARN("Cannot arm: Safety interlocks not satisfied");
                return std::nullopt;
            }

            // Check mode-specific requirements
            if (m_operating_mode != OperatingMode::AUTO) {
                // T1/T2 requires deadman switch
                if (m_safety && !m_safety->isDeadmanActive()) {
                    LOG_WARN("Cannot arm in manual mode: Deadman switch not pressed");
                    return std::nullopt;
                }
            }

            LOG_INFO("Starting arming sequence...");
            return StateArming{0, std::chrono::steady_clock::now()};
        }

        return std::nullopt;
    }, event);
}
```

**Acceptance Criteria:**
- [ ] State machine compile và chạy
- [ ] Events queue thread-safe
- [ ] State transitions logged
- [ ] E-Stop từ ANY state → EstopActive
- [ ] Callbacks khi state change

**Deliverables:**
- SystemStateManager.hpp
- SystemStateManager.cpp
- Unit tests cho state transitions

---

#### P2-04: State Transition Logic

| Attribute | Value |
|-----------|-------|
| **Priority** | P0 - Critical |
| **Dependencies** | P2-03 |
| **Estimated Effort** | Large |

**Description:**
Implement đầy đủ logic cho tất cả state transitions.

**Transition Table:**

| Current State | Event | Guard | Next State | Action |
|---------------|-------|-------|------------|--------|
| **ANY** | EvEstopAsserted | - | ESTOP_ACTIVE | Stop motion Cat 0 |
| BOOT | EvInitComplete | - | IDLE | Log ready |
| BOOT | EvHardwareFault | - | ERROR_LOCKOUT | Log error |
| IDLE | EvCmdServoOn | Interlocks OK, (Deadman if T1/T2) | ARMING | Start arming sequence |
| ARMING | EvServoReady | - | OPERATIONAL | Set sub_state = MANUAL_IDLE |
| ARMING | EvServoFault | - | IDLE | Log fault, disable drives |
| ARMING | EvDeadmanReleased | Mode = T1/T2 | IDLE | Abort arming |
| OPERATIONAL | EvCmdJogStart | IsJoggingAllowed | OPERATIONAL (JOGGING) | Start motion |
| OPERATIONAL | EvCmdJogStop | - | OPERATIONAL (IDLE) | Stop motion |
| OPERATIONAL | EvDeadmanReleased | Mode = T1/T2 | STOPPING (Cat 1) | Ramp down |
| OPERATIONAL | EvSafeguardOpen | Mode = AUTO | STOPPING (Cat 1) | Ramp down |
| OPERATIONAL | EvCmdServoOff | - | STOPPING (Cat 2) | Controlled stop |
| STOPPING | EvVelocityZero | - | IDLE | Disable drives |
| ESTOP_ACTIVE | EvEstopReleased | No discrepancy | ESTOP_RESET_NEEDED | Wait for reset |
| ESTOP_RESET_NEEDED | EvCmdReset | - | IDLE | Clear faults |

**Guard Conditions Implementation:**

```cpp
// Guards.hpp
namespace robot_controller::state {

class TransitionGuards {
public:
    TransitionGuards(SafetyManager* safety, MotionController* motion)
        : m_safety(safety), m_motion(motion) {}

    // Guard: Can we start arming?
    bool canArm(OperatingMode mode) const {
        if (!m_safety->areInterlocksOk()) return false;
        if (mode != OperatingMode::AUTO && !m_safety->isDeadmanActive()) return false;
        return true;
    }

    // Guard: Is jogging allowed?
    bool canJog(OperatingMode mode) const {
        if (mode == OperatingMode::AUTO) return false;  // No jog in AUTO
        if (!m_safety->isDeadmanActive()) return false;
        if (mode == OperatingMode::T1 && !m_motion->isVelocityLimited()) return false;
        return true;
    }

    // Guard: Can start program?
    bool canStartProgram(OperatingMode mode) const {
        if (mode != OperatingMode::AUTO) return false;
        if (!m_safety->isSafeguardClosed()) return false;
        return true;
    }

private:
    SafetyManager* m_safety;
    MotionController* m_motion;
};

} // namespace robot_controller::state
```

**Acceptance Criteria:**
- [ ] Tất cả transitions trong table hoạt động đúng
- [ ] Guard conditions được kiểm tra
- [ ] Invalid transitions bị reject
- [ ] Logs cho mỗi transition

**Deliverables:**
- Transitions.hpp
- Guards implementation
- Complete unit test coverage

---

#### P2-05: Safety Signal Manager

| Attribute | Value |
|-----------|-------|
| **Priority** | P0 - Critical |
| **Dependencies** | P1-05 (IPC), P2-01 |
| **Estimated Effort** | Medium |
| **Reference** | Thiết Kế FSM Robot Công Nghiệp An Toàn.md (Section 4) |

**Description:**
Quản lý các tín hiệu an toàn từ hardware với dual-channel monitoring.

**Class Design:**

```cpp
// SafetyManager.hpp
#pragma once

#include <atomic>
#include <chrono>
#include <functional>

namespace robot_controller::safety {

// Dual-channel safety signal
struct DualChannelSignal {
    bool channel_a = false;
    bool channel_b = false;
    std::chrono::steady_clock::time_point last_update;

    bool isActive() const { return channel_a && channel_b; }
    bool hasDiscrepancy() const { return channel_a != channel_b; }
};

// Safety signal types
struct SafetySignals {
    DualChannelSignal estop;           // Emergency stop
    DualChannelSignal safeguard;       // Door/light curtain
    DualChannelSignal deadman;         // Enable device (3-position)
    bool deadman_panic = false;        // Position 3 (squeezed)

    // Soft limits
    std::array<bool, 6> positive_limit;
    std::array<bool, 6> negative_limit;
};

class SafetyManager {
public:
    SafetyManager();

    // Update signals (call from hardware interface)
    void updateEstop(bool channel_a, bool channel_b);
    void updateSafeguard(bool channel_a, bool channel_b);
    void updateDeadman(bool channel_a, bool channel_b, bool panic);
    void updateLimits(int axis, bool positive, bool negative);

    // Queries
    bool isEstopActive() const;
    bool isEstopReleased() const;
    bool isSafeguardClosed() const;
    bool isDeadmanActive() const;
    bool isDeadmanPanic() const;
    bool hasDiscrepancy() const;  // Any dual-channel mismatch
    bool areInterlocksOk() const; // All safety conditions met
    bool isAxisAtLimit(int axis, int direction) const;

    // Discrepancy monitoring
    static constexpr auto DISCREPANCY_TIMEOUT = std::chrono::milliseconds(50);
    bool checkDiscrepancyTimeout() const;

    // Event generation
    using SafetyEventCallback = std::function<void(const state::Event&)>;
    void setEventCallback(SafetyEventCallback cb) { m_event_cb = cb; }

private:
    void checkForEvents();  // Generate events on signal changes

    SafetySignals m_signals;
    SafetySignals m_previous_signals;
    std::chrono::steady_clock::time_point m_discrepancy_start;
    SafetyEventCallback m_event_cb;
    mutable std::mutex m_mutex;
};

} // namespace robot_controller::safety
```

**Dual-Channel Logic:**

```cpp
// SafetyManager.cpp (partial)

void SafetyManager::updateEstop(bool channel_a, bool channel_b) {
    std::lock_guard<std::mutex> lock(m_mutex);

    m_previous_signals.estop = m_signals.estop;

    m_signals.estop.channel_a = channel_a;
    m_signals.estop.channel_b = channel_b;
    m_signals.estop.last_update = std::chrono::steady_clock::now();

    // Check for discrepancy
    if (m_signals.estop.hasDiscrepancy()) {
        if (!m_previous_signals.estop.hasDiscrepancy()) {
            m_discrepancy_start = std::chrono::steady_clock::now();
        }
        // If discrepancy persists too long, treat as fault
        if (checkDiscrepancyTimeout()) {
            LOG_ERROR("E-Stop discrepancy timeout - hardware fault!");
            if (m_event_cb) {
                m_event_cb(state::EvHardwareFault{
                    0x0001, "E-Stop dual channel discrepancy"
                });
            }
        }
    }

    checkForEvents();
}

void SafetyManager::checkForEvents() {
    // E-Stop pressed (either channel going active)
    if (!m_previous_signals.estop.isActive() && m_signals.estop.isActive()) {
        LOG_WARN("E-Stop ACTIVATED");
        if (m_event_cb) m_event_cb(state::EvEstopAsserted{});
    }

    // E-Stop released (both channels inactive)
    if (m_previous_signals.estop.isActive() &&
        !m_signals.estop.channel_a && !m_signals.estop.channel_b) {
        LOG_INFO("E-Stop released");
        if (m_event_cb) m_event_cb(state::EvEstopReleased{});
    }

    // Similar logic for deadman, safeguard...
}
```

**Acceptance Criteria:**
- [ ] Dual-channel signals monitored correctly
- [ ] Discrepancy detection within 50ms
- [ ] Events generated on signal changes
- [ ] Thread-safe signal updates
- [ ] All interlock queries work correctly

**Deliverables:**
- SafetyManager.hpp
- SafetyManager.cpp
- SafetySignals.hpp
- Unit tests

---

#### P2-06: E-Stop Handling

| Attribute | Value |
|-----------|-------|
| **Priority** | P0 - Critical |
| **Dependencies** | P2-03, P2-05 |
| **Estimated Effort** | Medium |

**Description:**
Implement complete E-Stop handling flow theo ISO 10218-1.

**E-Stop Categories:**

```cpp
// EstopHandler.hpp
namespace robot_controller::safety {

class EstopHandler {
public:
    // Stop categories
    enum class StopCategory {
        CAT0,  // Immediate power removal (uncontrolled)
        CAT1,  // Controlled stop, then power removal
        CAT2   // Controlled stop, power maintained
    };

    void executeStop(StopCategory category);

    // Cat 0: Immediate
    void executeCat0Stop() {
        LOG_WARN("Executing Category 0 Stop (IMMEDIATE)");
        // 1. Disable all servo drives immediately
        m_hardware->disableAllDrives();
        // 2. Engage mechanical brakes
        m_hardware->engageBrakes();
        // 3. Cut power to motors (contactor)
        m_hardware->openMainContactor();
    }

    // Cat 1: Controlled then power off
    void executeCat1Stop() {
        LOG_WARN("Executing Category 1 Stop (CONTROLLED)");
        // 1. Command zero velocity to all axes
        m_motion->commandVelocity({0, 0, 0, 0, 0, 0});
        // 2. Monitor until velocity reaches zero (with timeout)
        m_motion->setStoppingCallback([this]() {
            // 3. Once stopped, engage brakes and cut power
            m_hardware->engageBrakes();
            m_hardware->disableAllDrives();
        });
    }

private:
    IHardwareInterface* m_hardware;
    MotionController* m_motion;
};

} // namespace robot_controller::safety
```

**Reset Sequence:**

```cpp
// Reset requires explicit operator action (ISO 13849-1)
bool SystemStateManager::handleResetCommand() {
    // Only valid in ESTOP_RESET_NEEDED state
    if (!std::holds_alternative<StateEstopResetNeeded>(m_current_state)) {
        LOG_WARN("Reset command ignored - not in reset-needed state");
        return false;
    }

    // Verify E-Stop is actually released
    if (m_safety->isEstopActive()) {
        LOG_WARN("Cannot reset - E-Stop still active");
        return false;
    }

    // Check for any persistent faults
    if (m_safety->hasDiscrepancy()) {
        LOG_ERROR("Cannot reset - safety discrepancy detected");
        return false;
    }

    // Transition to IDLE
    LOG_INFO("System reset - transitioning to IDLE");
    transitionTo(StateIdle{});
    return true;
}
```

**Acceptance Criteria:**
- [ ] Cat 0 stop: Power cut < 10ms
- [ ] Cat 1 stop: Controlled deceleration
- [ ] Reset requires manual action
- [ ] Reset blocked if E-Stop still pressed
- [ ] Reset blocked if hardware fault exists

**Deliverables:**
- EstopHandler implementation
- Reset sequence implementation
- Tests for all stop categories

---

#### P2-07: Forward Kinematics

| Attribute | Value |
|-----------|-------|
| **Priority** | P0 - Critical |
| **Dependencies** | P1-07 (Config) |
| **Estimated Effort** | Medium |
| **Reference** | Robotics Library Robot Tùy Chỉnh & IK.md |

**Description:**
Implement Forward Kinematics sử dụng DH parameters.

**DH Convention (Modified):**

```
T_i = Rz(θi) * Tz(di) * Tx(ai) * Rx(αi)
```

**Implementation:**

```cpp
// ForwardKinematics.hpp
#pragma once

#include <Eigen/Dense>
#include <array>

namespace robot_controller::kinematics {

using Matrix4d = Eigen::Matrix4d;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using JointAngles = std::array<double, 6>;  // radians

struct DHParameter {
    double a;       // Link length (mm)
    double alpha;   // Link twist (rad)
    double d;       // Link offset (mm)
    double theta_offset;  // Joint angle offset (rad)
};

class ForwardKinematics {
public:
    ForwardKinematics(const std::array<DHParameter, 6>& dh_params);

    // Compute transform from base to each link
    std::array<Matrix4d, 7> computeAllTransforms(const JointAngles& q) const;

    // Compute transform from base to TCP
    Matrix4d computeTCP(const JointAngles& q) const;

    // Extract position and orientation from transform
    struct Pose {
        Eigen::Vector3d position;    // X, Y, Z (mm)
        Eigen::Vector3d orientation; // Rx, Ry, Rz (rad) - ZYX Euler
    };
    Pose transformToPose(const Matrix4d& T) const;

    // Jacobian for velocity mapping
    Eigen::Matrix<double, 6, 6> computeJacobian(const JointAngles& q) const;

private:
    // Single DH transform
    Matrix4d dhTransform(double theta, double d, double a, double alpha) const;

    std::array<DHParameter, 6> m_dh_params;
    Matrix4d m_tcp_offset = Matrix4d::Identity();
};

} // namespace robot_controller::kinematics
```

**DH Transform Implementation:**

```cpp
// ForwardKinematics.cpp

Matrix4d ForwardKinematics::dhTransform(double theta, double d, double a, double alpha) const {
    Matrix4d T;

    double ct = std::cos(theta);
    double st = std::sin(theta);
    double ca = std::cos(alpha);
    double sa = std::sin(alpha);

    T << ct, -st * ca,  st * sa, a * ct,
         st,  ct * ca, -ct * sa, a * st,
          0,      sa,       ca,      d,
          0,       0,        0,      1;

    return T;
}

Matrix4d ForwardKinematics::computeTCP(const JointAngles& q) const {
    Matrix4d T = Matrix4d::Identity();

    for (int i = 0; i < 6; ++i) {
        double theta = q[i] + m_dh_params[i].theta_offset;
        T = T * dhTransform(
            theta,
            m_dh_params[i].d,
            m_dh_params[i].a,
            m_dh_params[i].alpha
        );
    }

    return T * m_tcp_offset;
}

ForwardKinematics::Pose ForwardKinematics::transformToPose(const Matrix4d& T) const {
    Pose pose;

    // Position
    pose.position = T.block<3, 1>(0, 3);

    // Orientation (ZYX Euler angles)
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);

    // Extract Euler angles (ZYX convention)
    pose.orientation.y() = std::asin(-R(2, 0));  // Ry (pitch)

    if (std::abs(std::cos(pose.orientation.y())) > 1e-6) {
        pose.orientation.x() = std::atan2(R(2, 1), R(2, 2));  // Rx (roll)
        pose.orientation.z() = std::atan2(R(1, 0), R(0, 0));  // Rz (yaw)
    } else {
        // Gimbal lock
        pose.orientation.x() = 0;
        pose.orientation.z() = std::atan2(-R(0, 1), R(1, 1));
    }

    return pose;
}
```

**Acceptance Criteria:**
- [ ] FK tính đúng cho home position
- [ ] FK tính đúng cho các test configurations
- [ ] Jacobian tính đúng
- [ ] Matches với Robotics Library (nếu dùng)

**Deliverables:**
- ForwardKinematics.hpp
- ForwardKinematics.cpp
- Test cases với known configurations

---

#### P2-08: Inverse Kinematics

| Attribute | Value |
|-----------|-------|
| **Priority** | P0 - Critical |
| **Dependencies** | P2-07 |
| **Estimated Effort** | Large |
| **Reference** | Robotics Library Robot Tùy Chỉnh & IK.md |

**Description:**
Implement Inverse Kinematics (analytical solution cho 6-DOF standard).

**IK Design:**

```cpp
// InverseKinematics.hpp
#pragma once

#include "ForwardKinematics.hpp"
#include <optional>
#include <vector>

namespace robot_controller::kinematics {

// IK Configuration (8 possible solutions)
struct IKConfig {
    bool shoulder_left;   // vs shoulder_right
    bool elbow_up;        // vs elbow_down
    bool wrist_flip;      // vs wrist_no_flip
};

class InverseKinematics {
public:
    InverseKinematics(const std::array<DHParameter, 6>& dh_params,
                      const std::array<JointLimit, 6>& limits);

    // Compute all valid IK solutions
    std::vector<JointAngles> computeAll(const Matrix4d& target_pose) const;

    // Compute closest solution to current configuration
    std::optional<JointAngles> computeClosest(
        const Matrix4d& target_pose,
        const JointAngles& current_q) const;

    // Compute solution with specific configuration
    std::optional<JointAngles> computeWithConfig(
        const Matrix4d& target_pose,
        const IKConfig& config) const;

    // Check if solution is valid (within limits)
    bool isValidSolution(const JointAngles& q) const;

    // Check if pose is reachable
    bool isReachable(const Matrix4d& target_pose) const;

private:
    // Analytical IK steps
    std::array<double, 2> solveJoint1(const Eigen::Vector3d& wrist_center) const;
    std::array<double, 2> solveJoint2(double q1, const Eigen::Vector3d& wrist_center) const;
    std::array<double, 2> solveJoint3(double q1, double q2, const Eigen::Vector3d& wrist_center) const;
    std::array<JointAngles, 2> solveWrist(const Matrix4d& R0_3, const Matrix4d& target_R) const;

    // Wrist center calculation
    Eigen::Vector3d computeWristCenter(const Matrix4d& target_pose) const;

    // Angle normalization
    double normalizeAngle(double angle) const;

    std::array<DHParameter, 6> m_dh_params;
    std::array<JointLimit, 6> m_limits;
    double m_d6;  // Wrist to flange distance
};

} // namespace robot_controller::kinematics
```

**Closest Solution Selection:**

```cpp
std::optional<JointAngles> InverseKinematics::computeClosest(
    const Matrix4d& target_pose,
    const JointAngles& current_q) const {

    auto solutions = computeAll(target_pose);

    if (solutions.empty()) {
        return std::nullopt;
    }

    // Find solution with minimum joint movement
    double min_distance = std::numeric_limits<double>::max();
    const JointAngles* best_solution = nullptr;

    for (const auto& sol : solutions) {
        double distance = 0;
        for (int i = 0; i < 6; ++i) {
            double diff = std::abs(normalizeAngle(sol[i] - current_q[i]));
            // Weight joints differently (base joints more important)
            double weight = (i < 3) ? 1.0 : 0.5;
            distance += weight * diff * diff;
        }

        if (distance < min_distance) {
            min_distance = distance;
            best_solution = &sol;
        }
    }

    return *best_solution;
}
```

**Acceptance Criteria:**
- [ ] IK returns correct solutions for test poses
- [ ] All 8 configurations handled
- [ ] Joint limits checked
- [ ] Closest solution selection works
- [ ] Singular configurations handled gracefully

**Deliverables:**
- InverseKinematics.hpp
- InverseKinematics.cpp
- Comprehensive unit tests

---

#### P2-09: Ruckig Integration

| Attribute | Value |
|-----------|-------|
| **Priority** | P0 - Critical |
| **Dependencies** | P2-08 |
| **Estimated Effort** | Medium |
| **Reference** | Tích hợp Ruckig cho Robot Hàn 6-DOF.md |

**Description:**
Tích hợp Ruckig Online Trajectory Generation cho smooth motion.

**Ruckig Wrapper:**

```cpp
// RuckigWrapper.hpp
#pragma once

#include <ruckig/ruckig.hpp>
#include <array>

namespace robot_controller::trajectory {

class TrajectoryGenerator {
public:
    static constexpr int DOF = 6;
    static constexpr double CYCLE_TIME = 0.001;  // 1ms

    TrajectoryGenerator();

    // Set motion limits (from config)
    void setLimits(
        const std::array<double, DOF>& max_velocity,     // rad/s
        const std::array<double, DOF>& max_acceleration, // rad/s^2
        const std::array<double, DOF>& max_jerk);        // rad/s^3

    // Set current state
    void setCurrentState(
        const std::array<double, DOF>& position,
        const std::array<double, DOF>& velocity = {},
        const std::array<double, DOF>& acceleration = {});

    // Set target (for point-to-point motion)
    void setTarget(
        const std::array<double, DOF>& target_position,
        const std::array<double, DOF>& target_velocity = {});

    // Update trajectory (call every cycle)
    enum class Result {
        Working,    // Still generating
        Finished,   // Reached target
        Error       // Invalid input
    };
    Result update();

    // Get current output
    const std::array<double, DOF>& getPosition() const { return m_output.new_position; }
    const std::array<double, DOF>& getVelocity() const { return m_output.new_velocity; }
    const std::array<double, DOF>& getAcceleration() const { return m_output.new_acceleration; }

    // Get trajectory info
    double getRemainingTime() const;
    bool isMoving() const;

    // Emergency stop (instant target = current)
    void emergencyStop();

    // Jog mode (continuous velocity)
    void setJogVelocity(const std::array<double, DOF>& velocity);
    void stopJog();

private:
    ruckig::Ruckig<DOF> m_ruckig;
    ruckig::InputParameter<DOF> m_input;
    ruckig::OutputParameter<DOF> m_output;
    ruckig::Result m_last_result;

    bool m_is_jogging = false;
};

} // namespace robot_controller::trajectory
```

**Implementation:**

```cpp
// RuckigWrapper.cpp

TrajectoryGenerator::TrajectoryGenerator()
    : m_ruckig(CYCLE_TIME) {

    // Default limits (will be overwritten from config)
    for (int i = 0; i < DOF; ++i) {
        m_input.max_velocity[i] = 2.0;       // rad/s
        m_input.max_acceleration[i] = 10.0;  // rad/s^2
        m_input.max_jerk[i] = 50.0;          // rad/s^3

        m_input.current_position[i] = 0;
        m_input.current_velocity[i] = 0;
        m_input.current_acceleration[i] = 0;

        m_input.target_position[i] = 0;
        m_input.target_velocity[i] = 0;
    }
}

void TrajectoryGenerator::setLimits(
    const std::array<double, DOF>& max_vel,
    const std::array<double, DOF>& max_acc,
    const std::array<double, DOF>& max_jerk) {

    for (int i = 0; i < DOF; ++i) {
        m_input.max_velocity[i] = max_vel[i];
        m_input.max_acceleration[i] = max_acc[i];
        m_input.max_jerk[i] = max_jerk[i];
    }
}

TrajectoryGenerator::Result TrajectoryGenerator::update() {
    m_last_result = m_ruckig.update(m_input, m_output);

    // Update input for next cycle
    m_output.pass_to_input(m_input);

    switch (m_last_result) {
        case ruckig::Result::Working:
            return Result::Working;
        case ruckig::Result::Finished:
            return Result::Finished;
        default:
            LOG_ERROR("Ruckig error: {}", static_cast<int>(m_last_result));
            return Result::Error;
    }
}

void TrajectoryGenerator::setJogVelocity(const std::array<double, DOF>& velocity) {
    m_is_jogging = true;

    // In jog mode, we set target velocity, not position
    // Ruckig will smoothly accelerate to target velocity
    for (int i = 0; i < DOF; ++i) {
        m_input.target_velocity[i] = velocity[i];
        // Target position far away in direction of velocity
        m_input.target_position[i] = m_input.current_position[i]
            + std::copysign(1e6, velocity[i]);
    }
}

void TrajectoryGenerator::stopJog() {
    m_is_jogging = false;

    // Set target to current position with zero velocity
    for (int i = 0; i < DOF; ++i) {
        m_input.target_position[i] = m_input.current_position[i];
        m_input.target_velocity[i] = 0;
    }
}

void TrajectoryGenerator::emergencyStop() {
    // Immediate stop - set current as target
    for (int i = 0; i < DOF; ++i) {
        m_input.target_position[i] = m_output.new_position[i];
        m_input.target_velocity[i] = 0;
        m_input.current_velocity[i] = 0;
        m_input.current_acceleration[i] = 0;
    }
    m_is_jogging = false;
}
```

**Acceptance Criteria:**
- [ ] Ruckig library compiles và links
- [ ] Smooth acceleration/deceleration profiles
- [ ] Jog mode velocity control works
- [ ] Emergency stop works instantly
- [ ] No jerk violations

**Deliverables:**
- RuckigWrapper.hpp
- RuckigWrapper.cpp
- Motion profile tests

---

#### P2-10: Motion Controller

| Attribute | Value |
|-----------|-------|
| **Priority** | P0 - Critical |
| **Dependencies** | P2-07, P2-08, P2-09 |
| **Estimated Effort** | Large |

**Description:**
Motion Controller tích hợp Kinematics và Trajectory, quản lý motion loop.

**Class Design:**

```cpp
// MotionController.hpp
#pragma once

#include "kinematics/ForwardKinematics.hpp"
#include "kinematics/InverseKinematics.hpp"
#include "trajectory/RuckigWrapper.hpp"
#include "hardware/IHardwareInterface.hpp"

#include <thread>
#include <atomic>
#include <mutex>

namespace robot_controller::motion {

class MotionController {
public:
    MotionController(
        std::unique_ptr<kinematics::ForwardKinematics> fk,
        std::unique_ptr<kinematics::InverseKinematics> ik,
        std::unique_ptr<trajectory::TrajectoryGenerator> traj,
        IHardwareInterface* hardware);

    ~MotionController();

    // Lifecycle
    void start();
    void stop();
    bool isRunning() const { return m_running; }

    // Motion commands
    bool moveToJoint(const JointAngles& target);
    bool moveToCartesian(const CartesianPose& target);
    bool jogJoint(int axis, double velocity);
    bool jogCartesian(int axis, double velocity);  // axis: 0-2=XYZ, 3-5=RxRyRz
    void stopMotion();
    void emergencyStop();

    // State queries
    JointAngles getCurrentJoints() const;
    CartesianPose getCurrentPose() const;
    bool isMoving() const;
    bool isAtTarget() const;
    double getVelocityMagnitude() const;

    // Limits
    void setVelocityLimit(double ratio);  // 0.0 - 1.0
    bool isVelocityLimited() const { return m_velocity_limit < 1.0; }

    // Mode
    void enableSimulation(bool enable);
    bool isSimulation() const { return m_simulation_mode; }

private:
    void motionLoop();  // Main control loop (1kHz)

    // Components
    std::unique_ptr<kinematics::ForwardKinematics> m_fk;
    std::unique_ptr<kinematics::InverseKinematics> m_ik;
    std::unique_ptr<trajectory::TrajectoryGenerator> m_traj;
    IHardwareInterface* m_hardware;

    // Thread
    std::thread m_motion_thread;
    std::atomic<bool> m_running{false};

    // State
    JointAngles m_current_joints{};
    JointAngles m_target_joints{};
    std::atomic<bool> m_is_moving{false};
    std::atomic<double> m_velocity_limit{1.0};
    std::atomic<bool> m_simulation_mode{false};

    mutable std::mutex m_state_mutex;

    // Timing
    static constexpr double CYCLE_TIME_MS = 1.0;
};

} // namespace robot_controller::motion
```

**Motion Loop Implementation:**

```cpp
// MotionController.cpp

void MotionController::motionLoop() {
    LOG_INFO("Motion loop started @ {}Hz", 1000.0 / CYCLE_TIME_MS);

    using Clock = std::chrono::steady_clock;
    auto next_cycle = Clock::now();
    const auto cycle_duration = std::chrono::microseconds(
        static_cast<int>(CYCLE_TIME_MS * 1000));

    while (m_running) {
        // Wait for next cycle
        std::this_thread::sleep_until(next_cycle);
        next_cycle += cycle_duration;

        // Read current joint positions from hardware
        if (!m_simulation_mode) {
            auto hw_joints = m_hardware->readJointPositions();
            std::lock_guard<std::mutex> lock(m_state_mutex);
            m_current_joints = hw_joints;
        }

        // Update trajectory
        auto result = m_traj->update();

        // Get commanded positions
        auto cmd_joints = m_traj->getPosition();

        // Send to hardware (or update simulation)
        if (m_simulation_mode) {
            std::lock_guard<std::mutex> lock(m_state_mutex);
            m_current_joints = cmd_joints;
        } else {
            m_hardware->sendJointPositions(cmd_joints);
        }

        // Update state
        m_is_moving = (result == trajectory::TrajectoryGenerator::Result::Working);
    }

    LOG_INFO("Motion loop stopped");
}

bool MotionController::jogJoint(int axis, double velocity) {
    if (axis < 0 || axis >= 6) return false;

    std::array<double, 6> vel{};
    vel[axis] = velocity * m_velocity_limit;

    m_traj->setJogVelocity(vel);
    m_is_moving = true;
    return true;
}

bool MotionController::jogCartesian(int axis, double velocity) {
    if (axis < 0 || axis >= 6) return false;

    // Get current pose
    auto current_pose = getCurrentPose();

    // Compute velocity in Cartesian space
    Eigen::Matrix<double, 6, 1> cart_vel = Eigen::Matrix<double, 6, 1>::Zero();
    cart_vel[axis] = velocity;

    // Transform to joint velocities via Jacobian inverse
    auto J = m_fk->computeJacobian(getCurrentJoints());
    Eigen::Matrix<double, 6, 1> joint_vel = J.inverse() * cart_vel;

    // Apply velocity limit
    std::array<double, 6> vel;
    for (int i = 0; i < 6; ++i) {
        vel[i] = joint_vel[i] * m_velocity_limit;
    }

    m_traj->setJogVelocity(vel);
    m_is_moving = true;
    return true;
}
```

**Acceptance Criteria:**
- [ ] Motion loop runs at stable 1kHz
- [ ] Joint jog works for all axes
- [ ] Cartesian jog works for all axes
- [ ] Stop motion works smoothly
- [ ] Emergency stop works immediately
- [ ] Simulation mode works correctly

**Deliverables:**
- MotionController.hpp
- MotionController.cpp
- Integration tests

---

#### P2-11: Jog Controller

| Attribute | Value |
|-----------|-------|
| **Priority** | P1 - High |
| **Dependencies** | P2-10 |
| **Estimated Effort** | Medium |

**Description:**
High-level Jog Controller xử lý jog commands từ UI.

```cpp
// JogController.hpp
#pragma once

namespace robot_controller::motion {

enum class JogMode {
    JOINT,      // Jog individual joints
    WORLD,      // Jog in world coordinates (X, Y, Z, Rx, Ry, Rz)
    TOOL        // Jog in tool coordinates
};

enum class JogAxis {
    AXIS_1, AXIS_2, AXIS_3, AXIS_4, AXIS_5, AXIS_6,  // Joint mode
    X, Y, Z, RX, RY, RZ                               // Cartesian mode
};

class JogController {
public:
    JogController(MotionController* motion, state::SystemStateManager* state);

    // Jog commands
    bool startJog(JogMode mode, JogAxis axis, int direction, double speed_ratio);
    void stopJog();

    // Settings
    void setJogMode(JogMode mode) { m_mode = mode; }
    JogMode getJogMode() const { return m_mode; }

    void setSpeedRatio(double ratio);  // 0.0 - 1.0
    double getSpeedRatio() const { return m_speed_ratio; }

    // T1 mode speed limit (250 mm/s)
    static constexpr double T1_MAX_TCP_SPEED = 250.0;  // mm/s

private:
    bool validateJogRequest(JogMode mode, JogAxis axis) const;
    double computeJointVelocity(int axis, int direction) const;
    double computeCartesianVelocity(int axis, int direction) const;

    MotionController* m_motion;
    state::SystemStateManager* m_state;

    JogMode m_mode = JogMode::JOINT;
    double m_speed_ratio = 0.5;
    bool m_is_jogging = false;
};

} // namespace robot_controller::motion
```

**Acceptance Criteria:**
- [ ] Joint jog hoạt động với UI
- [ ] Cartesian jog hoạt động
- [ ] Speed ratio điều chỉnh được
- [ ] T1 mode limit 250mm/s được enforce

**Deliverables:**
- JogController.hpp
- JogController.cpp

---

#### P2-12: Hardware Interface Abstraction

| Attribute | Value |
|-----------|-------|
| **Priority** | P1 - High |
| **Dependencies** | P2-10 |
| **Estimated Effort** | Medium |

**Description:**
Abstract interface cho hardware, cho phép switch giữa real và simulation.

```cpp
// IHardwareInterface.hpp
#pragma once

#include <array>

namespace robot_controller {

class IHardwareInterface {
public:
    virtual ~IHardwareInterface() = default;

    // Connection
    virtual bool connect() = 0;
    virtual void disconnect() = 0;
    virtual bool isConnected() const = 0;

    // Motion
    virtual void sendJointPositions(const std::array<double, 6>& positions) = 0;
    virtual std::array<double, 6> readJointPositions() const = 0;
    virtual std::array<double, 6> readJointVelocities() const = 0;

    // Servo control
    virtual void enableDrives() = 0;
    virtual void disableAllDrives() = 0;
    virtual bool areAllDrivesReady() const = 0;

    // Safety
    virtual void engageBrakes() = 0;
    virtual void releaseBrakes() = 0;
    virtual void openMainContactor() = 0;
    virtual void closeMainContactor() = 0;

    // I/O
    virtual void setDigitalOutput(int channel, bool value) = 0;
    virtual bool readDigitalInput(int channel) const = 0;
    virtual void setAnalogOutput(int channel, double value) = 0;
    virtual double readAnalogInput(int channel) const = 0;

    // Status
    virtual std::string getStatusString() const = 0;
};

} // namespace robot_controller
```

**Acceptance Criteria:**
- [ ] Interface đủ cho motion control
- [ ] Interface đủ cho safety control
- [ ] Interface đủ cho I/O

**Deliverables:**
- IHardwareInterface.hpp

---

#### P2-13: grblHAL Driver

| Attribute | Value |
|-----------|-------|
| **Priority** | P1 - High |
| **Dependencies** | P2-12 |
| **Estimated Effort** | Large |
| **Reference** | Tối ưu grblHAL cho Robot 6-DOF.md |

**Description:**
Implement hardware interface cho grblHAL firmware.

```cpp
// GrblHalDriver.hpp
#pragma once

#include "IHardwareInterface.hpp"
#include <serial/serial.h>  // Serial library

namespace robot_controller::hardware {

class GrblHalDriver : public IHardwareInterface {
public:
    GrblHalDriver(const std::string& port, uint32_t baud = 115200);
    ~GrblHalDriver();

    // Connection
    bool connect() override;
    void disconnect() override;
    bool isConnected() const override;

    // Motion (G-code based)
    void sendJointPositions(const std::array<double, 6>& positions) override;
    std::array<double, 6> readJointPositions() const override;
    std::array<double, 6> readJointVelocities() const override;

    // ... other interface methods

private:
    void sendGcode(const std::string& cmd);
    std::string readResponse();
    void parseStatusReport(const std::string& report);

    // Convert joint angles to machine coordinates
    std::string jointsToGcode(const std::array<double, 6>& joints);

    serial::Serial m_serial;
    std::string m_port;
    uint32_t m_baud;

    // Cached state
    std::array<double, 6> m_current_position{};
    std::array<double, 6> m_current_velocity{};
    std::string m_machine_state;

    // Status polling
    std::thread m_status_thread;
    std::atomic<bool> m_polling{false};
};

} // namespace robot_controller::hardware
```

**Acceptance Criteria:**
- [ ] Connect/disconnect to grblHAL
- [ ] Send joint commands via G-code
- [ ] Read position feedback
- [ ] Status polling hoạt động
- [ ] Error handling cho serial errors

**Deliverables:**
- GrblHalDriver.hpp
- GrblHalDriver.cpp
- Tests với actual hardware (hoặc mock)

---

#### P2-14: VirtualController

| Attribute | Value |
|-----------|-------|
| **Priority** | P1 - High |
| **Dependencies** | P2-12, P2-09 |
| **Estimated Effort** | Medium |
| **Reference** | Thiết Kế Mô Phỏng Robot WPF Helix.md |

**Description:**
Simulation implementation của hardware interface.

```cpp
// VirtualController.hpp
#pragma once

#include "IHardwareInterface.hpp"
#include "../trajectory/RuckigWrapper.hpp"

namespace robot_controller::hardware {

class VirtualController : public IHardwareInterface {
public:
    VirtualController();
    ~VirtualController();

    // Always "connected" in simulation
    bool connect() override { return true; }
    void disconnect() override {}
    bool isConnected() const override { return true; }

    // Simulated motion
    void sendJointPositions(const std::array<double, 6>& positions) override;
    std::array<double, 6> readJointPositions() const override;
    std::array<double, 6> readJointVelocities() const override;

    // Simulated servo
    void enableDrives() override;
    void disableAllDrives() override;
    bool areAllDrivesReady() const override;

    // Simulated safety (always OK in simulation, unless forced)
    void engageBrakes() override;
    void releaseBrakes() override;
    void openMainContactor() override;
    void closeMainContactor() override;

    // Simulated I/O
    void setDigitalOutput(int channel, bool value) override;
    bool readDigitalInput(int channel) const override;
    void setAnalogOutput(int channel, double value) override;
    double readAnalogInput(int channel) const override;

    std::string getStatusString() const override;

    // Simulation controls
    void setPosition(const std::array<double, 6>& pos);
    void injectFault(const std::string& fault_type);
    void clearFaults();

private:
    std::array<double, 6> m_position{};
    std::array<double, 6> m_velocity{};
    std::array<double, 6> m_target{};

    bool m_drives_enabled = false;
    bool m_brakes_engaged = true;
    bool m_contactor_closed = false;

    std::array<bool, 32> m_digital_outputs{};
    std::array<bool, 32> m_digital_inputs{};
    std::array<double, 8> m_analog_outputs{};
    std::array<double, 8> m_analog_inputs{};

    mutable std::mutex m_mutex;
};

} // namespace robot_controller::hardware
```

**Acceptance Criteria:**
- [ ] Simulates joint motion
- [ ] Simulates servo enable/disable
- [ ] Simulates I/O
- [ ] Can inject faults for testing
- [ ] Thread-safe

**Deliverables:**
- VirtualController.hpp
- VirtualController.cpp

---

#### P2-15: Jog UI Panel

| Attribute | Value |
|-----------|-------|
| **Priority** | P1 - High |
| **Dependencies** | P1-11, P2-11 |
| **Estimated Effort** | Medium |

**Description:**
WPF UI panel cho jog control.

**XAML Layout:**

```xml
<!-- JogPanel.xaml -->
<UserControl x:Class="RobotController.UI.Views.JogPanel">
    <Grid>
        <Grid.RowDefinitions>
            <RowDefinition Height="Auto"/>
            <RowDefinition Height="*"/>
            <RowDefinition Height="Auto"/>
        </Grid.RowDefinitions>

        <!-- Mode Selection -->
        <StackPanel Grid.Row="0" Orientation="Horizontal" Margin="5">
            <RadioButton Content="Joint" IsChecked="{Binding IsJointMode}"
                         GroupName="JogMode" Margin="5"/>
            <RadioButton Content="World" IsChecked="{Binding IsWorldMode}"
                         GroupName="JogMode" Margin="5"/>
            <RadioButton Content="Tool" IsChecked="{Binding IsToolMode}"
                         GroupName="JogMode" Margin="5"/>
        </StackPanel>

        <!-- Jog Buttons -->
        <UniformGrid Grid.Row="1" Rows="6" Columns="3" Margin="10">
            <!-- Axis 1 / X -->
            <TextBlock Text="{Binding Axis1Label}" VerticalAlignment="Center"/>
            <Button Content="-" Command="{Binding JogNegativeCommand}"
                    CommandParameter="0"
                    PreviewMouseDown="JogButton_MouseDown"
                    PreviewMouseUp="JogButton_MouseUp"/>
            <Button Content="+" Command="{Binding JogPositiveCommand}"
                    CommandParameter="0"
                    PreviewMouseDown="JogButton_MouseDown"
                    PreviewMouseUp="JogButton_MouseUp"/>

            <!-- Repeat for other axes... -->
        </UniformGrid>

        <!-- Speed Slider -->
        <StackPanel Grid.Row="2" Orientation="Horizontal" Margin="5">
            <TextBlock Text="Speed:" VerticalAlignment="Center"/>
            <Slider Minimum="1" Maximum="100" Value="{Binding SpeedPercent}"
                    Width="200" Margin="10,0"/>
            <TextBlock Text="{Binding SpeedPercent, StringFormat={}{0}%}"
                       VerticalAlignment="Center"/>
        </StackPanel>
    </Grid>
</UserControl>
```

**Acceptance Criteria:**
- [ ] Mode switching works
- [ ] Jog buttons responsive
- [ ] Hold-to-jog behavior
- [ ] Speed slider works
- [ ] Keyboard shortcuts (optional)

**Deliverables:**
- JogPanel.xaml
- JogPanelViewModel.cs

---

#### P2-16: Status Display UI

| Attribute | Value |
|-----------|-------|
| **Priority** | P1 - High |
| **Dependencies** | P1-11, P2-03 |
| **Estimated Effort** | Medium |

**Description:**
UI hiển thị robot status, joint positions, TCP position.

**Features:**
- State indicator (color-coded)
- Mode indicator (T1/T2/AUTO)
- Joint positions (6 values)
- TCP position (X, Y, Z, Rx, Ry, Rz)
- Velocity display
- Error messages

**Acceptance Criteria:**
- [ ] Real-time position update
- [ ] State displayed with color
- [ ] Errors displayed prominently
- [ ] Updates at 10Hz+ without UI lag

**Deliverables:**
- StatusPanel.xaml
- StatusPanelViewModel.cs

---

#### P2-17: Ghost Robot Visualization

| Attribute | Value |
|-----------|-------|
| **Priority** | P2 - Medium |
| **Dependencies** | P1-14, P2-10 |
| **Estimated Effort** | Medium |
| **Reference** | Thiết Kế Mô Phỏng Robot WPF Helix.md (Section 3) |

**Description:**
Hiển thị Ghost Robot (transparent) cho target position trong jog mode.

**Implementation:**
- Duplicate robot model với transparent material
- Update Ghost position to target
- Use SortingVisual3D for correct transparency

**Acceptance Criteria:**
- [ ] Ghost Robot visible
- [ ] Correct transparency rendering
- [ ] Updates with target position
- [ ] Toggle on/off

**Deliverables:**
- GhostRobotVisualizer.cs
- Material configuration

---

#### P2-18: Integration Testing

| Attribute | Value |
|-----------|-------|
| **Priority** | P2 - Medium |
| **Dependencies** | All P2 tasks |
| **Estimated Effort** | Large |

**Description:**
End-to-end testing của Phase 2 components.

**Test Cases:**

| ID | Test Case | Expected |
|----|-----------|----------|
| MT-01 | Boot → Idle | System starts in Idle |
| MT-02 | Idle → Servo On → Operational | Arming sequence works |
| MT-03 | Jog Joint + (all axes) | Each axis moves positive |
| MT-04 | Jog Joint - (all axes) | Each axis moves negative |
| MT-05 | Jog World X/Y/Z | TCP moves correctly |
| MT-06 | Stop Jog | Smooth deceleration |
| MT-07 | E-Stop during jog | Immediate stop |
| MT-08 | Deadman release during jog | Controlled stop |
| MT-09 | Joint limit approach | Stops at limit |
| MT-10 | Simulation mode toggle | Switches correctly |
| MT-11 | Real hardware connection | Connects, jogs work |

**Acceptance Criteria:**
- [ ] All test cases pass
- [ ] No memory leaks
- [ ] Cycle time stable (< 1ms jitter)
- [ ] No deadlocks

**Deliverables:**
- Integration test suite
- Test report

---

## 4. TECH STACK SUMMARY

### 4.1. New Dependencies (Phase 2)

| Component | Library | Version | Purpose |
|-----------|---------|---------|---------|
| Trajectory | Ruckig | 0.9+ | OTG jerk-limited |
| Linear Algebra | Eigen | 3.4+ | Matrix operations |
| Kinematics | Custom / RL | - | IK/FK |
| Serial | serial | 1.2+ | Hardware comm |

### 4.2. CMake Updates

```cmake
# Add to CMakeLists.txt

# Eigen
find_package(Eigen3 REQUIRED)

# Ruckig (header-only or submodule)
add_subdirectory(third_party/ruckig)

# Serial library
find_package(serial REQUIRED)

# Link to main executable
target_link_libraries(robot_core
    PRIVATE
        Eigen3::Eigen
        ruckig
        serial
)
```

---

## 5. DEFINITION OF DONE

### Phase 2 is DONE when:

- [ ] **All Tasks Completed**: P2-01 through P2-18
- [ ] **Milestone Achieved**: "Robot Moves"
  - [ ] State Machine works correctly
  - [ ] E-Stop works
  - [ ] Jog Joint mode works
  - [ ] Jog Cartesian mode works
  - [ ] Simulation Mode works
- [ ] **Safety**:
  - [ ] E-Stop latency < 10ms
  - [ ] T1 speed limit enforced
- [ ] **Performance**:
  - [ ] Motion loop @ 1kHz stable
  - [ ] UI update @ 10Hz stable
- [ ] **Testing**:
  - [ ] All unit tests pass
  - [ ] Integration tests pass
- [ ] **Documentation**:
  - [ ] API documented
  - [ ] User guide updated

---

## 6. RISKS & MITIGATIONS

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Ruckig performance issues | High | Low | Profile early, optimize if needed |
| IK singularities | Medium | Medium | Implement singularity avoidance |
| grblHAL communication errors | Medium | Medium | Add retry logic, watchdog |
| Real-time jitter on Windows | High | Medium | Use high-priority threads |
| Motion loop timing | High | Low | Use dedicated thread, spin-wait |

---

## 7. DEPENDENCIES ON OTHER PHASES

| From Phase | Dependency | Type |
|------------|------------|------|
| Phase 1 | IPC, Config, Logger, HMI | Required |
| Phase 1 | Robot Model, 3D Viewport | Required |
| → Phase 3 | StateManager, MotionController | Provides |
| → Phase 3 | Safety system | Provides |
| → Phase 4 | All Phase 2 components | Provides |

---

## APPENDIX

### A. State Diagram (Visual)

```
                                    ┌─────────────────┐
                                    │      BOOT       │
                                    └────────┬────────┘
                                             │ EvInitComplete
                                             ▼
                    ┌───────────────────────────────────────────────┐
                    │                                               │
         ┌──────────┴──────────┐                                   │
         │                     │                                   │
         ▼                     │                                   │
  ┌─────────────┐              │                                   │
  │    IDLE     │◄─────────────┼───────────────────────────────────┤
  └──────┬──────┘              │                                   │
         │ EvCmdServoOn        │                                   │
         │ [Interlocks OK]     │                                   │
         ▼                     │                                   │
  ┌─────────────┐              │                                   │
  │   ARMING    │──────────────┤ EvServoFault                      │
  └──────┬──────┘              │                                   │
         │ EvServoReady        │                                   │
         ▼                     │                                   │
  ┌─────────────┐              │                                   │
  │ OPERATIONAL │              │                                   │
  │  ┌───────┐  │              │                                   │
  │  │MANUAL │  │              │                                   │
  │  │ IDLE  │◄─┼──────────────┼───────────┐                       │
  │  └───┬───┘  │              │           │                       │
  │      │ Jog  │              │           │                       │
  │      ▼      │              │           │                       │
  │  ┌───────┐  │              │           │                       │
  │  │JOGGING│  │              │           │                       │
  │  └───────┘  │              │           │                       │
  └──────┬──────┘              │           │                       │
         │ Deadman OFF         │           │                       │
         │ Safeguard OPEN      │           │ EvVelocityZero        │
         ▼                     │           │                       │
  ┌─────────────┐              │           │                       │
  │  STOPPING   │──────────────┴───────────┘                       │
  │  (Cat 1/2)  │                                                  │
  └─────────────┘                                                  │
                                                                   │
  ════════════════════════════════════════════════════════════════ │
  ANY STATE                                                        │
       │                                                           │
       │ EvEstopAsserted                                           │
       ▼                                                           │
  ┌─────────────┐         ┌─────────────────┐                      │
  │   ESTOP     │────────►│  ESTOP_RESET    │──────────────────────┘
  │   ACTIVE    │ Released│    NEEDED       │  EvCmdReset
  └─────────────┘         └─────────────────┘
```

### B. Checklist

```
Phase 2 Completion Checklist:

[ ] State definitions complete
[ ] Event definitions complete
[ ] SystemStateManager works
[ ] All transitions implemented
[ ] Safety Manager works
[ ] E-Stop handling works
[ ] Forward Kinematics works
[ ] Inverse Kinematics works
[ ] Ruckig integrated
[ ] Motion Controller works
[ ] Jog Controller works
[ ] Hardware interface abstraction
[ ] grblHAL driver works
[ ] VirtualController works
[ ] Jog UI panel works
[ ] Status display works
[ ] Ghost Robot works
[ ] Integration tests pass
```

---

*Document Version: 1.0 | Last Updated: 2026-02-01*
