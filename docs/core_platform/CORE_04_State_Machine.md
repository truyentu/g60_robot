# CORE MODULE: SystemStateManager (FSM)

## Document Info
| Item | Value |
|------|-------|
| **Module** | SystemStateManager |
| **Layer** | Core Logic (C++ 17/20) |
| **Standard** | ISO 10218-1, ISO 13849-1 |
| **Version** | 1.0 |
| **Last Updated** | 2026-02-01 |

---

## 1. Overview

### 1.1. Purpose
Module SystemStateManager là trình quản lý trạng thái trung tâm (FSM - Finite State Machine) cho bộ điều khiển robot 6-DOF. Module này đóng vai trò là "cơ quan quyết định" trung tâm, quản lý mọi hành vi vận hành của robot, tuân thủ nghiêm ngặt các tiêu chuẩn an toàn công nghiệp ISO 10218-1 và ISO 13849-1.

### 1.2. Key Features
- **Deterministic FSM**: Máy trạng thái hữu hạn với hành vi xác định
- **Safety-Critical**: Tuân thủ ISO 10218-1 (Robot Safety) và ISO 13849-1 (PL-d)
- **Table-Driven Design**: Kiến trúc dựa trên bảng, không switch-case lồng nhau
- **Dual-Channel Safety**: Giám sát tín hiệu an toàn kênh đôi
- **Stop Categories**: Hỗ trợ Cat 0, Cat 1, Cat 2 theo IEC 60204-1
- **Modern C++17/20**: Sử dụng std::variant và std::visit

### 1.3. Safety Compliance

```
┌─────────────────────────────────────────────────────────────────┐
│                    SAFETY STANDARDS COMPLIANCE                   │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  ISO 10218-1:2011 - Safety of Industrial Robots            │ │
│  │  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  │ │
│  │  • Operational Modes (AUTO, T1, T2)                        │ │
│  │  • Stop Categories (Cat 0, 1, 2)                           │ │
│  │  • Enabling Device (Deadman Switch)                        │ │
│  │  • Speed Limitation (250 mm/s in T1)                       │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  ISO 13849-1 - Performance Level d (PL-d)                  │ │
│  │  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  │ │
│  │  • Category 3 Architecture                                 │ │
│  │  • Dual-Channel Monitoring                                 │ │
│  │  • Diagnostic Coverage (DC)                                │ │
│  │  • Manual Reset Requirement                                │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  PLCopen Motion Control State Diagram                      │ │
│  │  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  │ │
│  │  • StandStill, DiscreteMotion, ContinuousMotion            │ │
│  │  • ErrorStop, Stopping                                      │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Architecture

### 2.1. System Position

```
┌─────────────────────────────────────────────────────────────────┐
│                    SYSTEMSTATEMANAGER POSITION                   │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                      C# UI (WPF)                           │ │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐      │ │
│  │  │ Control  │ │  Mode    │ │  Jog     │ │ Program  │      │ │
│  │  │ Buttons  │ │ Selector │ │ Buttons  │ │ Start    │      │ │
│  │  └────┬─────┘ └────┬─────┘ └────┬─────┘ └────┬─────┘      │ │
│  └───────┼────────────┼────────────┼────────────┼────────────┘ │
│          │            │            │            │               │
│          └────────────┴────────────┴────────────┘               │
│                              │                                   │
│                      ZeroMQ Commands                             │
│                              │                                   │
│  ┌───────────────────────────▼──────────────────────────────────┐
│  │                                                              │
│  │  ╔════════════════════════════════════════════════════════╗ │
│  │  ║           SYSTEMSTATEMANAGER (FSM Core)                ║ │
│  │  ║                                                        ║ │
│  │  ║  • Command Validation                                  ║ │
│  │  ║  • State Transitions                                   ║ │
│  │  ║  • Safety Interlocks                                   ║ │
│  │  ║  • Mode Management                                     ║ │
│  │  ╚════════════════════════════════════════════════════════╝ │
│  │                              │                               │
│  │     ┌────────────────────────┴────────────────────┐         │
│  │     │                        │                    │         │
│  │     ▼                        ▼                    ▼         │
│  │ ┌──────────┐          ┌──────────┐          ┌──────────┐   │
│  │ │ Motion   │          │ Welding  │          │ Vision   │   │
│  │ │ Core     │          │Sequencer │          │ Pipeline │   │
│  │ │          │          │          │          │          │   │
│  │ └────┬─────┘          └────┬─────┘          └────┬─────┘   │
│  │      │                     │                     │          │
│  │      C++ Core Logic Layer                                   │
│  └──────┴─────────────────────┴─────────────────────┴──────────┘
│                              │                                   │
│                       Serial / USB                               │
│                              │                                   │
│  ┌───────────────────────────▼──────────────────────────────────┐
│  │                    FIRMWARE (Teensy 4.1)                     │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐        │
│  │  │ grblHAL  │ │ Safety   │ │ Servo    │ │ E-Stop   │        │
│  │  │          │ │ I/O      │ │ Enable   │ │ Relay    │        │
│  │  └──────────┘ └──────────┘ └──────────┘ └──────────┘        │
│  └──────────────────────────────────────────────────────────────┘
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2. Hierarchical State Machine

```
┌─────────────────────────────────────────────────────────────────┐
│              HIERARCHICAL STATE MACHINE (HSM)                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│                      ┌─────────────┐                            │
│                      │  STATE_BOOT │                            │
│                      │ (Initialize)│                            │
│                      └──────┬──────┘                            │
│                             │ Init Complete                     │
│                             ▼                                    │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                   NORMAL OPERATION                        │   │
│  │                                                           │   │
│  │   ┌─────────┐     ┌─────────┐     ┌──────────────────┐   │   │
│  │   │  IDLE   │────►│ ARMING  │────►│   OPERATIONAL    │   │   │
│  │   │         │     │         │     │                  │   │   │
│  │   │ Servo   │     │ Servo   │     │ ┌──────────────┐ │   │   │
│  │   │ Off     │     │ Enabling│     │ │  AUTO Mode   │ │   │   │
│  │   │ Brakes  │     │         │     │ │ ┌──────────┐ │ │   │   │
│  │   │ Engaged │     │         │     │ │ │AUTO_IDLE │ │ │   │   │
│  │   │         │     │         │     │ │ └────┬─────┘ │ │   │   │
│  │   └────▲────┘     └─────────┘     │ │      │       │ │   │   │
│  │        │                          │ │      ▼       │ │   │   │
│  │        │                          │ │ ┌──────────┐ │ │   │   │
│  │        │                          │ │ │AUTO_RUN  │ │ │   │   │
│  │        │                          │ │ └──────────┘ │ │   │   │
│  │        │                          │ └──────────────┘ │   │   │
│  │        │                          │                  │   │   │
│  │        │                          │ ┌──────────────┐ │   │   │
│  │        │◄─────────────────────────┤ │ MANUAL Mode  │ │   │   │
│  │        │                          │ │ ┌──────────┐ │ │   │   │
│  │        │                          │ │ │MAN_IDLE  │ │ │   │   │
│  │        │                          │ │ └────┬─────┘ │ │   │   │
│  │        │                          │ │      │       │ │   │   │
│  │        │                          │ │      ▼       │ │   │   │
│  │        │                          │ │ ┌──────────┐ │ │   │   │
│  │        │                          │ │ │MAN_JOG   │ │ │   │   │
│  │        │                          │ │ └──────────┘ │ │   │   │
│  │        │                          │ └──────────────┘ │   │   │
│  │        │                          └──────────────────┘   │   │
│  │        │                                    │             │   │
│  │        │◄───────────────────────────────────┘             │   │
│  │        │           STOPPING                               │   │
│  └────────┴──────────────────────────────────────────────────┘   │
│                             │                                    │
│     ════════════════════════╪════════════════════════════════   │
│                             │ E-STOP ASSERTED (ANY STATE)       │
│                             ▼                                    │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                    SAFETY STATES                          │   │
│  │                                                           │   │
│  │   ┌─────────────┐     ┌─────────────────┐                │   │
│  │   │ESTOP_ACTIVE │────►│ ESTOP_RESET_    │                │   │
│  │   │             │     │ NEEDED          │───► IDLE       │   │
│  │   │  Cat 0 Stop │     │                 │   (Reset Cmd)  │   │
│  │   │  Power Off  │     │ Waiting Manual  │                │   │
│  │   └─────────────┘     │ Reset           │                │   │
│  │        ▲              └─────────────────┘                │   │
│  │        │                                                  │   │
│  │   ┌────┴────────┐                                        │   │
│  │   │ERROR_LOCKOUT│  Hardware Fault                        │   │
│  │   │             │  Requires Power Cycle                  │   │
│  │   └─────────────┘                                        │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 3. Stop Categories

### 3.1. IEC 60204-1 Stop Categories

```
┌─────────────────────────────────────────────────────────────────┐
│                    STOP CATEGORIES                               │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  CATEGORY 0: Uncontrolled Stop                                  │
│  ════════════════════════════════                               │
│                                                                  │
│     Velocity                                                     │
│        ▲                                                         │
│        │ ━━━━━━━━━┐                                             │
│        │          ┃ ← Immediate power cut                       │
│        │          ┃   Mechanical brakes engage                  │
│        │          ┃   Risk of mechanical damage at high speed   │
│        │          ┃                                              │
│        └──────────╂─────────────────────► Time                  │
│                   │                                              │
│     Triggers: E-Stop pressed, Power failure                     │
│                                                                  │
│  ─────────────────────────────────────────────────────────────  │
│                                                                  │
│  CATEGORY 1: Controlled Stop, then Power Off                    │
│  ═══════════════════════════════════════════                    │
│                                                                  │
│     Velocity                                                     │
│        ▲                                                         │
│        │ ━━━━━━━━━╲                                             │
│        │           ╲ ← Controlled deceleration                  │
│        │            ╲   (Ruckig ramp-down)                      │
│        │             ╲                                           │
│        │              ╲▂▂▂▂▂▂ Power off when v=0                │
│        └──────────────────────────────────► Time                │
│                                                                  │
│     Triggers: Light curtain violation, Deadman released         │
│                                                                  │
│  ─────────────────────────────────────────────────────────────  │
│                                                                  │
│  CATEGORY 2: Controlled Stop, Power Maintained                  │
│  ═════════════════════════════════════════════                  │
│                                                                  │
│     Velocity                                                     │
│        ▲                                                         │
│        │ ━━━━━━━━━╲                                             │
│        │           ╲ ← Controlled deceleration                  │
│        │            ╲                                            │
│        │             ╲▂▂▂▂▂▂▂▂▂▂▂▂▂▂▂ Servo holds position     │
│        └──────────────────────────────────► Time                │
│                                                                  │
│     Triggers: Pause command, Program hold                       │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 4. Operational Modes

### 4.1. Mode Comparison

| Mode | Description | Speed Limit | Deadman Required | Gate Required |
|------|-------------|-------------|------------------|---------------|
| **AUTO** | Automatic production | Full speed | No | Yes (Closed) |
| **T1** | Manual Reduced Speed | 250 mm/s | Yes | No (Bypass) |
| **T2** | Manual High Speed | Full speed | Yes | No (Bypass) |

### 4.2. Mode Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                    OPERATIONAL MODES                             │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│     ┌───────────────────────────────────────────────────────┐   │
│     │                   MODE: AUTO                          │   │
│     │                                                       │   │
│     │   • Robot runs pre-programmed tasks                   │   │
│     │   • Full speed allowed                                │   │
│     │   • Safety fence MUST be active                       │   │
│     │   • Gate open → Immediate Cat 1 Stop                  │   │
│     │   • No manual jogging allowed                         │   │
│     │                                                       │   │
│     │   FSM rejects: CMD_JOG_START                          │   │
│     │   FSM requires: Gate == CLOSED for CMD_START_AUTO     │   │
│     │                                                       │   │
│     └───────────────────────────────────────────────────────┘   │
│                                                                  │
│     ┌───────────────────────────────────────────────────────┐   │
│     │                   MODE: T1 (Teaching)                 │   │
│     │                                                       │   │
│     │   • Teaching and programming mode                     │   │
│     │   • TCP speed LIMITED to 250 mm/s                     │   │
│     │   • Deadman switch REQUIRED for motion                │   │
│     │   • Operator allowed inside workspace                 │   │
│     │   • Gate signal bypassed                              │   │
│     │                                                       │   │
│     │   FSM requires: Deadman == ON for any motion          │   │
│     │   FSM monitors: Speed violation → Emergency Stop      │   │
│     │                                                       │   │
│     └───────────────────────────────────────────────────────┘   │
│                                                                  │
│     ┌───────────────────────────────────────────────────────┐   │
│     │                   MODE: T2 (Test Run)                 │   │
│     │                                                       │   │
│     │   • Program verification at full speed                │   │
│     │   • HIGHEST RISK mode                                 │   │
│     │   • Deadman switch REQUIRED                           │   │
│     │   • Additional confirmation required                   │   │
│     │   • Strict guard conditions in FSM                    │   │
│     │                                                       │   │
│     │   FSM requires: Deadman == ON + Confirmation action   │   │
│     │                                                       │   │
│     └───────────────────────────────────────────────────────┘   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 5. Safety Signal Processing

### 5.1. Dual-Channel Monitoring

```
┌─────────────────────────────────────────────────────────────────┐
│              DUAL-CHANNEL SAFETY MONITORING (PL-d)               │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│     Physical E-Stop Button                                       │
│            │                                                     │
│     ┌──────┴──────┐                                             │
│     │             │                                              │
│     ▼             ▼                                              │
│  ┌──────┐     ┌──────┐                                          │
│  │Ch A  │     │Ch B  │    (NC Contacts)                         │
│  │Input │     │Input │                                          │
│  └──┬───┘     └──┬───┘                                          │
│     │            │                                               │
│     ▼            ▼                                               │
│  ┌────────────────────────────────────────────────┐             │
│  │         SafetySignalManager                     │             │
│  │                                                 │             │
│  │   ┌─────────────────────────────────────────┐  │             │
│  │   │     Discrepancy Check Logic              │  │             │
│  │   │                                          │  │             │
│  │   │  if (Ch_A != Ch_B) {                    │  │             │
│  │   │      mismatch_timer++;                  │  │             │
│  │   │      if (mismatch_timer > 50ms) {       │  │             │
│  │   │          → HARDWARE_FAULT               │  │             │
│  │   │      }                                   │  │             │
│  │   │  } else {                               │  │             │
│  │   │      mismatch_timer = 0;                │  │             │
│  │   │      safe_state = (Ch_A && Ch_B);       │  │             │
│  │   │  }                                       │  │             │
│  │   └─────────────────────────────────────────┘  │             │
│  │                                                 │             │
│  │   Output: SafetySignal { .ok, .fault, .state } │             │
│  └─────────────────────────────────────────────────┘             │
│                                                                  │
│  Detected Faults:                                               │
│  • Wire broken (one channel stuck HIGH)                         │
│  • Contact welded (one channel stuck LOW)                       │
│  • Signal mismatch > discrepancy time                           │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 5.2. Safety Signal List

| Signal | Channels | Type | Function |
|--------|----------|------|----------|
| **E-Stop** | 2 | NC | Emergency stop (Cat 0) |
| **Gate/Safeguard** | 2 | NC | Safety fence interlock |
| **Deadman** | 3-pos | Special | Enabling device |
| **Mode Selector** | 3 | Exclusive | T1/T2/AUTO selection |
| **Servo Ready** | Per axis | Status | Drive ready signal |
| **Drive Fault** | Per axis | Status | Drive error signal |

---

## 6. Task Breakdown

### 6.1. Task List

| ID | Task | Description | Priority | Dependencies |
|----|------|-------------|----------|--------------|
| **T-01** | State Definitions | Định nghĩa các trạng thái với std::variant | P0 | None |
| **T-02** | Event Definitions | Định nghĩa các sự kiện | P0 | None |
| **T-03** | SafetySignalManager | Xử lý tín hiệu an toàn dual-channel | P0 | HAL |
| **T-04** | TransitionTable | Bảng chuyển đổi trạng thái | P0 | T-01, T-02 |
| **T-05** | StateHandlers | Xử lý logic cho từng trạng thái | P0 | T-01 |
| **T-06** | CommandValidator | Thẩm định lệnh từ UI | P0 | T-04 |
| **T-07** | StopManager | Quản lý Cat 0/1/2 stops | P0 | T-05 |
| **T-08** | ModeManager | Quản lý chế độ T1/T2/AUTO | P0 | T-03 |
| **T-09** | IPC Integration | Kết nối ZeroMQ command queue | P0 | IPC Layer |
| **T-10** | MotionCore Integration | Kết nối Motion Planner | P0 | Trajectory |
| **T-11** | DiagnosticLogger | Log trạng thái cho audit | P1 | Logger |
| **T-12** | ResetSequence | Chuỗi reset sau E-Stop | P0 | T-07 |
| **T-13** | ArmingSequence | Chuỗi khởi động servo | P0 | T-05 |
| **T-14** | ConfigLoader | Load cấu hình từ YAML | P0 | Config System |
| **T-15** | UnitTests | Test coverage cho FSM | P0 | T-04, T-05 |

---

## 7. Implementation

### 7.1. State and Event Definitions

```cpp
// state_manager_types.hpp

#pragma once
#include <variant>
#include <cstdint>
#include <chrono>

namespace robot_controller {
namespace state {

//=============================================================================
// EVENTS
//=============================================================================

/**
 * @brief All possible events that can trigger state transitions
 */
enum class Event : uint16_t {
    // System events
    EV_BOOT_COMPLETE,
    EV_SELF_TEST_PASS,
    EV_SELF_TEST_FAIL,

    // Safety events (Hardware)
    EV_ESTOP_ASSERTED,
    EV_ESTOP_RELEASED,
    EV_GATE_OPEN,
    EV_GATE_CLOSED,
    EV_DEADMAN_ON,
    EV_DEADMAN_OFF,
    EV_DEADMAN_PANIC,  // 3rd position (squeeze)
    EV_SAFETY_MISMATCH,

    // Servo events
    EV_SERVO_READY,
    EV_SERVO_FAULT,
    EV_SERVO_TIMEOUT,

    // Motion events
    EV_MOTION_COMPLETE,
    EV_MOTION_ERROR,
    EV_VELOCITY_ZERO,

    // User commands
    CMD_SYS_RESET,
    CMD_SERVO_ON,
    CMD_SERVO_OFF,
    CMD_START_AUTO,
    CMD_STOP,
    CMD_PAUSE,
    CMD_RESUME,
    CMD_JOG_START,
    CMD_JOG_STOP,
    CMD_MODE_CHANGE,

    // Internal
    EV_TIMEOUT,
    EV_TICK,  // Periodic tick for housekeeping
};

/**
 * @brief Operational modes according to ISO 10218-1
 */
enum class OperationalMode : uint8_t {
    AUTO,   // Automatic production mode
    T1,     // Manual reduced speed (250 mm/s limit)
    T2,     // Manual high speed (test run)
};

/**
 * @brief Stop category according to IEC 60204-1
 */
enum class StopCategory : uint8_t {
    CAT_0,  // Uncontrolled stop (immediate power cut)
    CAT_1,  // Controlled stop, then power off
    CAT_2,  // Controlled stop, power maintained
};

//=============================================================================
// STATES (using std::variant for type-safe state data)
//=============================================================================

/**
 * @brief System booting state
 */
struct StateBoot {
    uint32_t progress_percent = 0;
    bool self_test_complete = false;
};

/**
 * @brief Unrecoverable error state (requires power cycle)
 */
struct StateErrorLockout {
    uint32_t error_code = 0;
    std::string error_message;
    std::chrono::steady_clock::time_point timestamp;
};

/**
 * @brief E-Stop is currently asserted
 */
struct StateEstopActive {
    std::chrono::steady_clock::time_point activated_at;
    bool contactor_opened = false;
    bool brakes_engaged = false;
};

/**
 * @brief E-Stop released, waiting for manual reset
 */
struct StateEstopResetNeeded {
    std::chrono::steady_clock::time_point estop_released_at;
};

/**
 * @brief System idle, servo off, brakes engaged
 */
struct StateIdle {
    OperationalMode selected_mode = OperationalMode::T1;
    bool faults_present = false;
};

/**
 * @brief Transitioning to operational (enabling servos)
 */
struct StateArming {
    uint32_t step = 0;  // Current step in arming sequence
    // Steps: 0=Check interlocks, 1=Close contactor, 2=Enable servos,
    //        3=Release brakes, 4=Wait servo ready
    std::chrono::steady_clock::time_point step_started;
    static constexpr uint32_t TOTAL_STEPS = 5;
};

/**
 * @brief System operational, servo on, ready for motion
 */
struct StateOperational {
    OperationalMode mode;
    bool program_running = false;
    bool jogging = false;
};

/**
 * @brief Stopping (Cat 1 or Cat 2 ramp-down in progress)
 */
struct StateStopping {
    StopCategory category;
    std::chrono::steady_clock::time_point stop_initiated;
    bool velocity_zero = false;
};

/**
 * @brief Variant holding all possible states
 */
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

/**
 * @brief State ID for debugging and logging
 */
enum class StateId : uint8_t {
    BOOT,
    ERROR_LOCKOUT,
    ESTOP_ACTIVE,
    ESTOP_RESET_NEEDED,
    IDLE,
    ARMING,
    OPERATIONAL,
    STOPPING,
};

/**
 * @brief Get state ID from variant
 */
inline StateId getStateId(const SystemState& state) {
    return std::visit([](const auto& s) -> StateId {
        using T = std::decay_t<decltype(s)>;
        if constexpr (std::is_same_v<T, StateBoot>) return StateId::BOOT;
        else if constexpr (std::is_same_v<T, StateErrorLockout>) return StateId::ERROR_LOCKOUT;
        else if constexpr (std::is_same_v<T, StateEstopActive>) return StateId::ESTOP_ACTIVE;
        else if constexpr (std::is_same_v<T, StateEstopResetNeeded>) return StateId::ESTOP_RESET_NEEDED;
        else if constexpr (std::is_same_v<T, StateIdle>) return StateId::IDLE;
        else if constexpr (std::is_same_v<T, StateArming>) return StateId::ARMING;
        else if constexpr (std::is_same_v<T, StateOperational>) return StateId::OPERATIONAL;
        else if constexpr (std::is_same_v<T, StateStopping>) return StateId::STOPPING;
    }, state);
}

/**
 * @brief Get state name for logging
 */
inline const char* getStateName(StateId id) {
    switch (id) {
        case StateId::BOOT: return "BOOT";
        case StateId::ERROR_LOCKOUT: return "ERROR_LOCKOUT";
        case StateId::ESTOP_ACTIVE: return "ESTOP_ACTIVE";
        case StateId::ESTOP_RESET_NEEDED: return "ESTOP_RESET_NEEDED";
        case StateId::IDLE: return "IDLE";
        case StateId::ARMING: return "ARMING";
        case StateId::OPERATIONAL: return "OPERATIONAL";
        case StateId::STOPPING: return "STOPPING";
        default: return "UNKNOWN";
    }
}

} // namespace state
} // namespace robot_controller
```

### 7.2. Safety Signal Manager

```cpp
// safety_signal_manager.hpp

#pragma once
#include <array>
#include <cstdint>

namespace robot_controller {
namespace state {

/**
 * @brief Result of dual-channel signal validation
 */
struct SafetySignalStatus {
    bool ok;              // Signal is valid and safe
    bool fault;           // Hardware fault detected (mismatch)
    bool asserted;        // Signal is asserted (e.g., E-Stop pressed)
    uint32_t mismatch_ms; // Duration of mismatch
};

/**
 * @brief 3-position Deadman switch state
 */
enum class DeadmanState {
    RELEASED,   // Position 1: Released (Off)
    HELD,       // Position 2: Held in middle (On)
    PANIC,      // Position 3: Squeezed tight (Off - panic)
};

/**
 * @brief Processes dual-channel safety signals with discrepancy checking
 *
 * Implements ISO 13849-1 Category 3 architecture requirements:
 * - Dual-channel monitoring
 * - Cross-channel discrepancy detection
 * - Diagnostic coverage
 */
class SafetySignalManager {
public:
    /**
     * @param discrepancy_time_ms Maximum allowed discrepancy time
     */
    explicit SafetySignalManager(uint32_t discrepancy_time_ms = 50);

    /**
     * @brief Update E-Stop signal from hardware
     * @param channel_a First channel (NC contact)
     * @param channel_b Second channel (NC contact)
     */
    void updateEstop(bool channel_a, bool channel_b);

    /**
     * @brief Update Gate/Safeguard signal
     */
    void updateGate(bool channel_a, bool channel_b);

    /**
     * @brief Update Deadman switch
     * @param pos1 Position 1 contact (released)
     * @param pos2 Position 2 contact (held)
     * @param pos3 Position 3 contact (panic)
     */
    void updateDeadman(bool pos1, bool pos2, bool pos3);

    /**
     * @brief Update Mode selector
     */
    void updateModeSelector(bool mode_auto, bool mode_t1, bool mode_t2);

    /**
     * @brief Process all signals (call every cycle)
     * @param dt_ms Time since last call in milliseconds
     */
    void process(uint32_t dt_ms);

    // Getters
    SafetySignalStatus getEstopStatus() const { return estop_status_; }
    SafetySignalStatus getGateStatus() const { return gate_status_; }
    DeadmanState getDeadmanState() const { return deadman_state_; }
    OperationalMode getSelectedMode() const { return selected_mode_; }

    /**
     * @brief Check if any safety fault is present
     */
    bool hasAnyFault() const;

    /**
     * @brief Check if E-Stop is currently asserted
     */
    bool isEstopAsserted() const { return estop_status_.asserted; }

    /**
     * @brief Check if gate is open
     */
    bool isGateOpen() const { return gate_status_.asserted; }

    /**
     * @brief Check if deadman is enabling
     */
    bool isDeadmanEnabled() const { return deadman_state_ == DeadmanState::HELD; }

private:
    // Configuration
    uint32_t discrepancy_time_ms_;

    // Raw inputs
    bool estop_a_, estop_b_;
    bool gate_a_, gate_b_;
    bool deadman_pos1_, deadman_pos2_, deadman_pos3_;
    bool mode_auto_, mode_t1_, mode_t2_;

    // Processed outputs
    SafetySignalStatus estop_status_;
    SafetySignalStatus gate_status_;
    DeadmanState deadman_state_;
    OperationalMode selected_mode_;

    // Mismatch timers
    uint32_t estop_mismatch_timer_;
    uint32_t gate_mismatch_timer_;

    // Internal processing
    SafetySignalStatus processDualChannel(bool ch_a, bool ch_b, uint32_t& timer, uint32_t dt);
    DeadmanState processDeadman();
    OperationalMode processMode();
};

} // namespace state
} // namespace robot_controller
```

### 7.3. SafetySignalManager Implementation

```cpp
// safety_signal_manager.cpp

#include "safety_signal_manager.hpp"

namespace robot_controller {
namespace state {

SafetySignalManager::SafetySignalManager(uint32_t discrepancy_time_ms)
    : discrepancy_time_ms_(discrepancy_time_ms)
    , estop_a_(false), estop_b_(false)
    , gate_a_(false), gate_b_(false)
    , deadman_pos1_(true), deadman_pos2_(false), deadman_pos3_(false)
    , mode_auto_(false), mode_t1_(true), mode_t2_(false)
    , deadman_state_(DeadmanState::RELEASED)
    , selected_mode_(OperationalMode::T1)
    , estop_mismatch_timer_(0)
    , gate_mismatch_timer_(0)
{
    // Initialize status
    estop_status_ = {false, false, true, 0};  // E-Stop assumed asserted at startup
    gate_status_ = {false, false, false, 0};
}

void SafetySignalManager::updateEstop(bool channel_a, bool channel_b) {
    // For NC contacts: LOW = pressed, HIGH = released
    // We invert so that 'asserted' means button is pressed
    estop_a_ = !channel_a;  // Pressed = true
    estop_b_ = !channel_b;
}

void SafetySignalManager::updateGate(bool channel_a, bool channel_b) {
    // NC contacts: LOW = gate open (unsafe), HIGH = gate closed (safe)
    gate_a_ = !channel_a;  // Open = true
    gate_b_ = !channel_b;
}

void SafetySignalManager::updateDeadman(bool pos1, bool pos2, bool pos3) {
    deadman_pos1_ = pos1;
    deadman_pos2_ = pos2;
    deadman_pos3_ = pos3;
}

void SafetySignalManager::updateModeSelector(bool mode_auto, bool mode_t1, bool mode_t2) {
    mode_auto_ = mode_auto;
    mode_t1_ = mode_t1;
    mode_t2_ = mode_t2;
}

void SafetySignalManager::process(uint32_t dt_ms) {
    // Process dual-channel signals
    estop_status_ = processDualChannel(estop_a_, estop_b_, estop_mismatch_timer_, dt_ms);
    gate_status_ = processDualChannel(gate_a_, gate_b_, gate_mismatch_timer_, dt_ms);

    // Process deadman
    deadman_state_ = processDeadman();

    // Process mode
    selected_mode_ = processMode();
}

SafetySignalStatus SafetySignalManager::processDualChannel(
    bool ch_a, bool ch_b, uint32_t& timer, uint32_t dt) {

    SafetySignalStatus status;

    if (ch_a != ch_b) {
        // Channels disagree - potential fault
        timer += dt;
        status.mismatch_ms = timer;

        if (timer >= discrepancy_time_ms_) {
            // Discrepancy too long - hardware fault!
            status.ok = false;
            status.fault = true;
            status.asserted = true;  // Fail-safe: assume asserted
        } else {
            // Still within tolerance, use logical OR (fail-safe)
            status.ok = true;
            status.fault = false;
            status.asserted = ch_a || ch_b;
        }
    } else {
        // Channels agree
        timer = 0;
        status.ok = true;
        status.fault = false;
        status.asserted = ch_a && ch_b;
        status.mismatch_ms = 0;
    }

    return status;
}

DeadmanState SafetySignalManager::processDeadman() {
    // 3-position switch logic:
    // Position 1 (Released): Only pos1 active
    // Position 2 (Held):     Only pos2 active
    // Position 3 (Panic):    Only pos3 active (or pos1 + pos3 in some designs)

    int active_count = (deadman_pos1_ ? 1 : 0) +
                       (deadman_pos2_ ? 1 : 0) +
                       (deadman_pos3_ ? 1 : 0);

    if (active_count != 1) {
        // Invalid state (hardware fault or transition) - fail-safe
        return DeadmanState::RELEASED;
    }

    if (deadman_pos2_) return DeadmanState::HELD;
    if (deadman_pos3_) return DeadmanState::PANIC;
    return DeadmanState::RELEASED;
}

OperationalMode SafetySignalManager::processMode() {
    // Mode selector must have exactly one active
    int active_count = (mode_auto_ ? 1 : 0) +
                       (mode_t1_ ? 1 : 0) +
                       (mode_t2_ ? 1 : 0);

    if (active_count != 1) {
        // Invalid - default to safest mode (T1)
        return OperationalMode::T1;
    }

    if (mode_auto_) return OperationalMode::AUTO;
    if (mode_t2_) return OperationalMode::T2;
    return OperationalMode::T1;
}

bool SafetySignalManager::hasAnyFault() const {
    return estop_status_.fault || gate_status_.fault;
}

} // namespace state
} // namespace robot_controller
```

### 7.4. SystemStateManager

```cpp
// system_state_manager.hpp

#pragma once
#include "state_manager_types.hpp"
#include "safety_signal_manager.hpp"
#include <memory>
#include <functional>
#include <queue>
#include <mutex>

namespace robot_controller {

// Forward declarations
namespace motion { class MotionEngine; }
namespace hal { class HardwareInterface; }

namespace state {

/**
 * @brief Command from UI with validation result
 */
struct CommandResult {
    bool accepted;
    uint32_t error_code;
    std::string error_message;
};

/**
 * @brief Central state manager implementing safety-critical FSM
 *
 * Design principles:
 * - Table-driven transitions (no nested switch-case)
 * - std::variant for type-safe states
 * - Dual-channel safety monitoring
 * - ISO 10218-1 / ISO 13849-1 compliance
 */
class SystemStateManager {
public:
    /**
     * @brief Constructor
     * @param hw Hardware abstraction layer
     * @param motion Motion engine for stop commands
     */
    SystemStateManager(std::shared_ptr<hal::HardwareInterface> hw,
                       std::shared_ptr<motion::MotionEngine> motion);

    /**
     * @brief Initialize state manager
     */
    void initialize();

    /**
     * @brief Process one cycle (call at 1kHz)
     * @param dt_ms Time since last call
     */
    void update(uint32_t dt_ms);

    /**
     * @brief Queue a command from UI
     * @param event Command event
     * @return Result indicating if command was accepted
     */
    CommandResult queueCommand(Event event);

    /**
     * @brief Get current state
     */
    const SystemState& getCurrentState() const { return current_state_; }

    /**
     * @brief Get current state ID
     */
    StateId getCurrentStateId() const { return getStateId(current_state_); }

    /**
     * @brief Get current operational mode
     */
    OperationalMode getCurrentMode() const;

    /**
     * @brief Check if motion is allowed
     */
    bool isMotionAllowed() const;

    /**
     * @brief Check if jogging is allowed
     */
    bool isJoggingAllowed() const;

    /**
     * @brief Check if program execution is allowed
     */
    bool isProgramExecutionAllowed() const;

    /**
     * @brief Get safety signal manager for status queries
     */
    const SafetySignalManager& getSafetyManager() const { return *safety_manager_; }

    /**
     * @brief State change callback type
     */
    using StateChangeCallback = std::function<void(StateId old_state, StateId new_state)>;

    /**
     * @brief Register state change callback
     */
    void onStateChange(StateChangeCallback callback);

private:
    // Components
    std::shared_ptr<hal::HardwareInterface> hw_;
    std::shared_ptr<motion::MotionEngine> motion_;
    std::unique_ptr<SafetySignalManager> safety_manager_;

    // State
    SystemState current_state_;
    OperationalMode current_mode_;

    // Command queue (lock-free in production)
    std::queue<Event> command_queue_;
    std::mutex queue_mutex_;

    // Callbacks
    std::vector<StateChangeCallback> state_callbacks_;

    // Internal methods
    void readSafetyInputs();
    void processEvents();
    void processStateLogic(uint32_t dt_ms);
    void handleEvent(Event event);

    // Transition helpers
    void transitionTo(SystemState new_state);
    bool checkGuard(Event event);

    // State-specific handlers (using std::visit pattern)
    struct EventHandler;
    void dispatchEvent(Event event);

    // Stop control
    void initiateStop(StopCategory category);
    void executeEmergencyStop();
    void executeControlledStop();

    // Arming sequence
    void processArmingSequence(uint32_t dt_ms);

    // Mode-specific checks
    bool checkAutoModePrerequisites() const;
    bool checkManualModePrerequisites() const;
};

} // namespace state
} // namespace robot_controller
```

### 7.5. SystemStateManager Implementation

```cpp
// system_state_manager.cpp

#include "system_state_manager.hpp"
#include "motion/motion_engine.hpp"
#include "hal/hardware_interface.hpp"
#include <spdlog/spdlog.h>

namespace robot_controller {
namespace state {

SystemStateManager::SystemStateManager(
    std::shared_ptr<hal::HardwareInterface> hw,
    std::shared_ptr<motion::MotionEngine> motion)
    : hw_(hw)
    , motion_(motion)
    , current_state_(StateBoot{})
    , current_mode_(OperationalMode::T1)
{
    safety_manager_ = std::make_unique<SafetySignalManager>(50); // 50ms discrepancy
}

void SystemStateManager::initialize() {
    spdlog::info("SystemStateManager initializing...");

    // Start in BOOT state
    current_state_ = StateBoot{};

    // Perform self-test
    bool self_test_ok = true;  // Simplified - real implementation checks hardware

    if (self_test_ok) {
        // Check E-Stop status
        readSafetyInputs();

        if (safety_manager_->isEstopAsserted()) {
            transitionTo(StateEstopActive{
                .activated_at = std::chrono::steady_clock::now(),
                .contactor_opened = true,
                .brakes_engaged = true
            });
        } else {
            transitionTo(StateIdle{
                .selected_mode = safety_manager_->getSelectedMode(),
                .faults_present = false
            });
        }
    } else {
        transitionTo(StateErrorLockout{
            .error_code = 1,
            .error_message = "Self-test failed",
            .timestamp = std::chrono::steady_clock::now()
        });
    }
}

void SystemStateManager::update(uint32_t dt_ms) {
    // Step 1: Read safety inputs
    readSafetyInputs();

    // Step 2: Check for E-Stop (highest priority, from ANY state)
    if (safety_manager_->isEstopAsserted() &&
        !std::holds_alternative<StateEstopActive>(current_state_) &&
        !std::holds_alternative<StateErrorLockout>(current_state_)) {

        spdlog::warn("E-STOP ASSERTED!");
        executeEmergencyStop();
        transitionTo(StateEstopActive{
            .activated_at = std::chrono::steady_clock::now(),
            .contactor_opened = true,
            .brakes_engaged = true
        });
        return;  // Skip other processing
    }

    // Step 3: Check for hardware faults
    if (safety_manager_->hasAnyFault()) {
        spdlog::error("Safety signal fault detected!");
        executeEmergencyStop();
        transitionTo(StateErrorLockout{
            .error_code = 100,
            .error_message = "Safety signal discrepancy",
            .timestamp = std::chrono::steady_clock::now()
        });
        return;
    }

    // Step 4: Process queued commands
    processEvents();

    // Step 5: State-specific logic
    processStateLogic(dt_ms);

    // Step 6: Update current mode
    current_mode_ = safety_manager_->getSelectedMode();
}

void SystemStateManager::readSafetyInputs() {
    // Read from HAL
    bool estop_a, estop_b;
    hw_->readEstop(estop_a, estop_b);
    safety_manager_->updateEstop(estop_a, estop_b);

    bool gate_a, gate_b;
    hw_->readGate(gate_a, gate_b);
    safety_manager_->updateGate(gate_a, gate_b);

    bool dm1, dm2, dm3;
    hw_->readDeadman(dm1, dm2, dm3);
    safety_manager_->updateDeadman(dm1, dm2, dm3);

    bool auto_sel, t1_sel, t2_sel;
    hw_->readModeSelector(auto_sel, t1_sel, t2_sel);
    safety_manager_->updateModeSelector(auto_sel, t1_sel, t2_sel);

    // Process with 1ms delta (assuming 1kHz loop)
    safety_manager_->process(1);
}

void SystemStateManager::processEvents() {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    while (!command_queue_.empty()) {
        Event event = command_queue_.front();
        command_queue_.pop();
        handleEvent(event);
    }
}

void SystemStateManager::handleEvent(Event event) {
    // Use std::visit to dispatch based on current state
    std::visit([this, event](auto& state) {
        using T = std::decay_t<decltype(state)>;

        if constexpr (std::is_same_v<T, StateEstopActive>) {
            // Only EV_ESTOP_RELEASED is valid
            if (event == Event::EV_ESTOP_RELEASED) {
                transitionTo(StateEstopResetNeeded{
                    .estop_released_at = std::chrono::steady_clock::now()
                });
            }
        }
        else if constexpr (std::is_same_v<T, StateEstopResetNeeded>) {
            // Only CMD_SYS_RESET is valid
            if (event == Event::CMD_SYS_RESET) {
                transitionTo(StateIdle{
                    .selected_mode = current_mode_,
                    .faults_present = false
                });
            }
        }
        else if constexpr (std::is_same_v<T, StateIdle>) {
            if (event == Event::CMD_SERVO_ON) {
                if (checkGuard(event)) {
                    transitionTo(StateArming{
                        .step = 0,
                        .step_started = std::chrono::steady_clock::now()
                    });
                }
            }
        }
        else if constexpr (std::is_same_v<T, StateArming>) {
            // Arming handled in processStateLogic
        }
        else if constexpr (std::is_same_v<T, StateOperational>) {
            handleOperationalEvent(state, event);
        }
        else if constexpr (std::is_same_v<T, StateStopping>) {
            // Wait for velocity zero
        }
    }, current_state_);
}

void SystemStateManager::handleOperationalEvent(StateOperational& state, Event event) {
    switch (event) {
        case Event::CMD_START_AUTO:
            if (current_mode_ == OperationalMode::AUTO &&
                !safety_manager_->isGateOpen()) {
                state.program_running = true;
                motion_->startProgram();
                spdlog::info("AUTO program started");
            }
            break;

        case Event::CMD_STOP:
            initiateStop(StopCategory::CAT_1);
            break;

        case Event::CMD_PAUSE:
            initiateStop(StopCategory::CAT_2);
            break;

        case Event::CMD_JOG_START:
            if ((current_mode_ == OperationalMode::T1 ||
                 current_mode_ == OperationalMode::T2) &&
                safety_manager_->isDeadmanEnabled()) {
                state.jogging = true;
                spdlog::info("Jog started");
            }
            break;

        case Event::CMD_JOG_STOP:
            state.jogging = false;
            motion_->stopJog();
            break;

        case Event::CMD_SERVO_OFF:
            initiateStop(StopCategory::CAT_1);
            break;

        case Event::EV_GATE_OPEN:
            if (current_mode_ == OperationalMode::AUTO) {
                spdlog::warn("Gate opened in AUTO mode - Cat 1 stop");
                initiateStop(StopCategory::CAT_1);
            }
            break;

        case Event::EV_DEADMAN_OFF:
        case Event::EV_DEADMAN_PANIC:
            if (current_mode_ != OperationalMode::AUTO) {
                spdlog::warn("Deadman released - Cat 1 stop");
                initiateStop(StopCategory::CAT_1);
            }
            break;

        default:
            break;
    }
}

void SystemStateManager::processStateLogic(uint32_t dt_ms) {
    std::visit([this, dt_ms](auto& state) {
        using T = std::decay_t<decltype(state)>;

        if constexpr (std::is_same_v<T, StateArming>) {
            processArmingSequence(dt_ms);
        }
        else if constexpr (std::is_same_v<T, StateOperational>) {
            // Check deadman in manual modes
            if (state.jogging &&
                current_mode_ != OperationalMode::AUTO &&
                !safety_manager_->isDeadmanEnabled()) {
                spdlog::warn("Deadman released during jog - stopping");
                state.jogging = false;
                motion_->stopJog();
            }
        }
        else if constexpr (std::is_same_v<T, StateStopping>) {
            // Check if velocity is zero
            if (motion_->isVelocityZero()) {
                state.velocity_zero = true;

                if (state.category == StopCategory::CAT_1) {
                    // Power off servos
                    hw_->setServoEnable(false);
                    hw_->engageBrakes(true);
                    transitionTo(StateIdle{
                        .selected_mode = current_mode_,
                        .faults_present = false
                    });
                } else {
                    // Cat 2 - stay operational but stopped
                    transitionTo(StateOperational{
                        .mode = current_mode_,
                        .program_running = false,
                        .jogging = false
                    });
                }
            }
        }
    }, current_state_);
}

void SystemStateManager::processArmingSequence(uint32_t dt_ms) {
    auto& arming = std::get<StateArming>(current_state_);

    auto now = std::chrono::steady_clock::now();
    auto step_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - arming.step_started).count();

    switch (arming.step) {
        case 0:  // Check interlocks
            if (!checkManualModePrerequisites() && current_mode_ != OperationalMode::AUTO) {
                spdlog::error("Interlock check failed - cannot arm");
                transitionTo(StateIdle{.selected_mode = current_mode_, .faults_present = true});
                return;
            }
            if (current_mode_ == OperationalMode::AUTO && !checkAutoModePrerequisites()) {
                spdlog::error("AUTO mode prerequisites not met");
                transitionTo(StateIdle{.selected_mode = current_mode_, .faults_present = true});
                return;
            }
            arming.step = 1;
            arming.step_started = now;
            break;

        case 1:  // Close main contactor
            hw_->setMainContactor(true);
            if (step_duration > 100) {  // Wait 100ms for contactor
                arming.step = 2;
                arming.step_started = now;
            }
            break;

        case 2:  // Enable servo drives
            hw_->setServoEnable(true);
            if (step_duration > 50) {
                arming.step = 3;
                arming.step_started = now;
            }
            break;

        case 3:  // Release brakes
            hw_->engageBrakes(false);
            if (step_duration > 200) {  // Wait 200ms for brakes
                arming.step = 4;
                arming.step_started = now;
            }
            break;

        case 4:  // Wait for servo ready
            if (hw_->areServosReady()) {
                spdlog::info("Arming complete - system operational");
                transitionTo(StateOperational{
                    .mode = current_mode_,
                    .program_running = false,
                    .jogging = false
                });
            } else if (step_duration > 2000) {  // 2 second timeout
                spdlog::error("Servo ready timeout");
                hw_->setServoEnable(false);
                hw_->engageBrakes(true);
                transitionTo(StateIdle{.selected_mode = current_mode_, .faults_present = true});
            }
            break;
    }
}

CommandResult SystemStateManager::queueCommand(Event event) {
    // Validate command against current state
    CommandResult result{true, 0, ""};

    // Basic validation
    if (std::holds_alternative<StateErrorLockout>(current_state_)) {
        return {false, 1, "System in error lockout - power cycle required"};
    }

    if (std::holds_alternative<StateEstopActive>(current_state_) &&
        event != Event::EV_ESTOP_RELEASED) {
        return {false, 2, "E-Stop active - release E-Stop first"};
    }

    if (std::holds_alternative<StateEstopResetNeeded>(current_state_) &&
        event != Event::CMD_SYS_RESET) {
        return {false, 3, "Reset required after E-Stop"};
    }

    // Command-specific validation
    if (event == Event::CMD_JOG_START) {
        if (current_mode_ == OperationalMode::AUTO) {
            return {false, 10, "Jog not allowed in AUTO mode"};
        }
        if (!safety_manager_->isDeadmanEnabled()) {
            return {false, 11, "Deadman switch must be held"};
        }
    }

    if (event == Event::CMD_START_AUTO) {
        if (current_mode_ != OperationalMode::AUTO) {
            return {false, 20, "Must be in AUTO mode"};
        }
        if (safety_manager_->isGateOpen()) {
            return {false, 21, "Safety gate must be closed"};
        }
    }

    // Queue the command
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        command_queue_.push(event);
    }

    return result;
}

bool SystemStateManager::checkGuard(Event event) {
    switch (event) {
        case Event::CMD_SERVO_ON:
            // Check mode-specific guards
            if (current_mode_ == OperationalMode::AUTO) {
                return !safety_manager_->isGateOpen();
            } else {
                return safety_manager_->isDeadmanEnabled();
            }

        default:
            return true;
    }
}

void SystemStateManager::transitionTo(SystemState new_state) {
    StateId old_id = getStateId(current_state_);
    StateId new_id = getStateId(new_state);

    spdlog::info("State transition: {} -> {}",
                 getStateName(old_id), getStateName(new_id));

    current_state_ = new_state;

    // Notify callbacks
    for (const auto& callback : state_callbacks_) {
        callback(old_id, new_id);
    }
}

void SystemStateManager::initiateStop(StopCategory category) {
    spdlog::info("Initiating stop: Cat {}", static_cast<int>(category));

    if (category == StopCategory::CAT_0) {
        executeEmergencyStop();
    } else {
        motion_->initiateRampDown();
        transitionTo(StateStopping{
            .category = category,
            .stop_initiated = std::chrono::steady_clock::now(),
            .velocity_zero = false
        });
    }
}

void SystemStateManager::executeEmergencyStop() {
    spdlog::warn("EXECUTING EMERGENCY STOP (Cat 0)");

    // Immediate actions - no deceleration
    hw_->setServoEnable(false);  // Cut servo power
    hw_->engageBrakes(true);     // Engage mechanical brakes
    hw_->setMainContactor(false); // Open main contactor

    motion_->emergencyStop();  // Tell motion to abort
}

void SystemStateManager::executeControlledStop() {
    motion_->initiateRampDown();
}

bool SystemStateManager::checkAutoModePrerequisites() const {
    return !safety_manager_->isGateOpen() &&
           !safety_manager_->hasAnyFault();
}

bool SystemStateManager::checkManualModePrerequisites() const {
    return safety_manager_->isDeadmanEnabled() &&
           !safety_manager_->hasAnyFault();
}

OperationalMode SystemStateManager::getCurrentMode() const {
    return current_mode_;
}

bool SystemStateManager::isMotionAllowed() const {
    return std::holds_alternative<StateOperational>(current_state_);
}

bool SystemStateManager::isJoggingAllowed() const {
    if (!std::holds_alternative<StateOperational>(current_state_)) {
        return false;
    }

    if (current_mode_ == OperationalMode::AUTO) {
        return false;
    }

    return safety_manager_->isDeadmanEnabled();
}

bool SystemStateManager::isProgramExecutionAllowed() const {
    if (!std::holds_alternative<StateOperational>(current_state_)) {
        return false;
    }

    if (current_mode_ != OperationalMode::AUTO) {
        return false;
    }

    return !safety_manager_->isGateOpen();
}

void SystemStateManager::onStateChange(StateChangeCallback callback) {
    state_callbacks_.push_back(callback);
}

} // namespace state
} // namespace robot_controller
```

---

## 8. Configuration

### 8.1. YAML Configuration

```yaml
# config/state_manager_config.yaml

state_manager:
  # Safety signal timing
  safety:
    discrepancy_time_ms: 50      # Max allowed mismatch between dual channels
    estop_debounce_ms: 10        # E-Stop signal debounce time
    gate_debounce_ms: 20         # Gate signal debounce time

  # Arming sequence timeouts
  arming:
    contactor_delay_ms: 100      # Wait for contactor to close
    servo_enable_delay_ms: 50    # Wait after enabling servos
    brake_release_delay_ms: 200  # Wait for brake release
    servo_ready_timeout_ms: 2000 # Timeout waiting for servo ready

  # Stop behavior
  stopping:
    cat1_max_time_ms: 5000       # Maximum Cat 1 stop duration
    cat2_settle_time_ms: 100     # Time to confirm stopped

  # Speed limits by mode
  speed_limits:
    T1_max_tcp_speed_mm_s: 250   # ISO 10218-1 requirement
    T2_max_tcp_speed_mm_s: 1000  # Full speed in T2
    AUTO_max_tcp_speed_mm_s: 2000 # Production speed

  # Logging
  logging:
    log_state_transitions: true
    log_commands: true
    log_safety_events: true
```

---

## 9. Testing

### 9.1. Unit Test Cases

```cpp
// test_state_manager.cpp

#include <gtest/gtest.h>
#include "state/system_state_manager.hpp"
#include "test_mocks.hpp"

using namespace robot_controller::state;

class StateManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        hw_mock_ = std::make_shared<MockHardwareInterface>();
        motion_mock_ = std::make_shared<MockMotionEngine>();
        state_manager_ = std::make_unique<SystemStateManager>(hw_mock_, motion_mock_);

        // Default: E-Stop released, gate closed, deadman held, T1 mode
        hw_mock_->setEstop(true, true);      // NC released = high
        hw_mock_->setGate(true, true);       // NC closed = high
        hw_mock_->setDeadman(false, true, false);  // Position 2 (held)
        hw_mock_->setMode(false, true, false);     // T1
    }

    std::shared_ptr<MockHardwareInterface> hw_mock_;
    std::shared_ptr<MockMotionEngine> motion_mock_;
    std::unique_ptr<SystemStateManager> state_manager_;
};

// Test: Boot to Idle
TEST_F(StateManagerTest, BootToIdle) {
    state_manager_->initialize();
    EXPECT_EQ(state_manager_->getCurrentStateId(), StateId::IDLE);
}

// Test: Boot with E-Stop active
TEST_F(StateManagerTest, BootWithEstop) {
    hw_mock_->setEstop(false, false);  // E-Stop pressed
    state_manager_->initialize();
    EXPECT_EQ(state_manager_->getCurrentStateId(), StateId::ESTOP_ACTIVE);
}

// Test: E-Stop transition from any state
TEST_F(StateManagerTest, EstopFromOperational) {
    state_manager_->initialize();
    state_manager_->queueCommand(Event::CMD_SERVO_ON);

    // Simulate arming complete
    for (int i = 0; i < 100; i++) {
        state_manager_->update(1);
    }
    hw_mock_->setServosReady(true);
    for (int i = 0; i < 100; i++) {
        state_manager_->update(1);
    }

    EXPECT_EQ(state_manager_->getCurrentStateId(), StateId::OPERATIONAL);

    // Now press E-Stop
    hw_mock_->setEstop(false, false);
    state_manager_->update(1);

    EXPECT_EQ(state_manager_->getCurrentStateId(), StateId::ESTOP_ACTIVE);
}

// Test: E-Stop recovery requires reset
TEST_F(StateManagerTest, EstopRecoveryRequiresReset) {
    hw_mock_->setEstop(false, false);
    state_manager_->initialize();
    EXPECT_EQ(state_manager_->getCurrentStateId(), StateId::ESTOP_ACTIVE);

    // Release E-Stop
    hw_mock_->setEstop(true, true);
    state_manager_->update(1);

    EXPECT_EQ(state_manager_->getCurrentStateId(), StateId::ESTOP_RESET_NEEDED);

    // Try to servo on without reset - should fail
    auto result = state_manager_->queueCommand(Event::CMD_SERVO_ON);
    EXPECT_FALSE(result.accepted);

    // Now reset
    state_manager_->queueCommand(Event::CMD_SYS_RESET);
    state_manager_->update(1);

    EXPECT_EQ(state_manager_->getCurrentStateId(), StateId::IDLE);
}

// Test: Jog blocked in AUTO mode
TEST_F(StateManagerTest, JogBlockedInAutoMode) {
    hw_mock_->setMode(true, false, false);  // AUTO mode
    state_manager_->initialize();

    // Arm system
    hw_mock_->setGate(true, true);  // Gate closed for AUTO
    state_manager_->queueCommand(Event::CMD_SERVO_ON);

    for (int i = 0; i < 500; i++) {
        state_manager_->update(1);
        if (i == 100) hw_mock_->setServosReady(true);
    }

    // Try to jog - should fail
    auto result = state_manager_->queueCommand(Event::CMD_JOG_START);
    EXPECT_FALSE(result.accepted);
    EXPECT_EQ(result.error_code, 10);  // "Jog not allowed in AUTO mode"
}

// Test: Jog requires deadman in T1
TEST_F(StateManagerTest, JogRequiresDeadman) {
    state_manager_->initialize();

    // Arm and get to operational
    state_manager_->queueCommand(Event::CMD_SERVO_ON);
    for (int i = 0; i < 500; i++) {
        state_manager_->update(1);
        if (i == 100) hw_mock_->setServosReady(true);
    }

    // Release deadman
    hw_mock_->setDeadman(true, false, false);  // Released

    auto result = state_manager_->queueCommand(Event::CMD_JOG_START);
    EXPECT_FALSE(result.accepted);
    EXPECT_EQ(result.error_code, 11);  // "Deadman switch must be held"
}

// Test: Dual-channel discrepancy detection
TEST_F(StateManagerTest, DualChannelDiscrepancy) {
    state_manager_->initialize();

    // Create E-Stop mismatch (one channel stuck)
    hw_mock_->setEstop(true, false);  // Mismatch!

    // Run until discrepancy time exceeded
    for (int i = 0; i < 100; i++) {
        state_manager_->update(1);
    }

    // Should enter ERROR_LOCKOUT due to safety fault
    EXPECT_EQ(state_manager_->getCurrentStateId(), StateId::ERROR_LOCKOUT);
}

// Test: Gate open stops AUTO program
TEST_F(StateManagerTest, GateOpenStopsAuto) {
    hw_mock_->setMode(true, false, false);  // AUTO
    state_manager_->initialize();

    // Get to operational and start program
    state_manager_->queueCommand(Event::CMD_SERVO_ON);
    for (int i = 0; i < 500; i++) {
        state_manager_->update(1);
        if (i == 100) hw_mock_->setServosReady(true);
    }

    state_manager_->queueCommand(Event::CMD_START_AUTO);
    state_manager_->update(1);

    EXPECT_TRUE(motion_mock_->isProgramRunning());

    // Open gate
    hw_mock_->setGate(false, false);  // Gate open
    state_manager_->update(1);

    // Should transition to STOPPING (Cat 1)
    EXPECT_EQ(state_manager_->getCurrentStateId(), StateId::STOPPING);
    EXPECT_TRUE(motion_mock_->isRampingDown());
}
```

---

## 10. Integration

### 10.1. Usage Example

```cpp
// main.cpp (simplified)

#include "state/system_state_manager.hpp"
#include "hal/teensy_hal.hpp"
#include "motion/motion_engine.hpp"
#include <thread>

int main() {
    // Initialize components
    auto hw = std::make_shared<TeensyHAL>("/dev/ttyUSB0");
    auto motion = std::make_shared<MotionEngine>();
    auto state_manager = std::make_shared<SystemStateManager>(hw, motion);

    // Register state change callback
    state_manager->onStateChange([](StateId old_s, StateId new_s) {
        spdlog::info("State: {} -> {}", getStateName(old_s), getStateName(new_s));
        // Send to UI via IPC...
    });

    // Initialize
    state_manager->initialize();

    // Real-time control loop (simplified - in production use RT scheduler)
    while (true) {
        auto start = std::chrono::high_resolution_clock::now();

        // 1ms control cycle
        state_manager->update(1);

        if (state_manager->isMotionAllowed()) {
            motion->update(1);
        }

        // Wait for next cycle
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

        if (elapsed.count() < 1000) {
            std::this_thread::sleep_for(std::chrono::microseconds(1000 - elapsed.count()));
        }
    }

    return 0;
}
```

---

## 11. References

### 11.1. Research Documents
| Document | Path |
|----------|------|
| Thiết Kế FSM Robot Công Nghiệp An Toàn | ressearch_doc_md/ |

### 11.2. External Standards
| Standard | Description |
|----------|-------------|
| ISO 10218-1:2011 | Safety requirements for industrial robots |
| ISO 13849-1 | Safety-related parts of control systems - PL |
| IEC 60204-1 | Safety of machinery - Electrical equipment |
| PLCopen | Motion control state diagram |

---

## APPENDIX

### A. State Transition Quick Reference

| From State | Event | Guard | To State |
|------------|-------|-------|----------|
| ANY | EV_ESTOP_ASSERTED | - | ESTOP_ACTIVE |
| ANY | EV_SAFETY_MISMATCH | - | ERROR_LOCKOUT |
| ESTOP_ACTIVE | EV_ESTOP_RELEASED | - | ESTOP_RESET_NEEDED |
| ESTOP_RESET_NEEDED | CMD_SYS_RESET | - | IDLE |
| IDLE | CMD_SERVO_ON | Mode guard | ARMING |
| ARMING | EV_SERVO_READY | - | OPERATIONAL |
| OPERATIONAL | CMD_START_AUTO | AUTO + Gate closed | Running |
| OPERATIONAL | CMD_JOG_START | Manual + Deadman | Jogging |
| OPERATIONAL (AUTO) | EV_GATE_OPEN | - | STOPPING (Cat 1) |
| OPERATIONAL (Manual) | EV_DEADMAN_OFF | - | STOPPING (Cat 1) |

### B. Glossary

| Term | Definition |
|------|------------|
| **FSM** | Finite State Machine - Máy trạng thái hữu hạn |
| **PL-d** | Performance Level d - Mức hiệu suất an toàn |
| **Cat 0/1/2** | Stop Categories - Danh mục dừng |
| **NC** | Normally Closed - Tiếp điểm thường đóng |
| **Deadman** | Enabling Device - Thiết bị cho phép 3 vị trí |
| **Discrepancy** | Sự không khớp giữa 2 kênh an toàn |

### C. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-02-01 | Initial version |

---

*Document generated as part of Robot Controller development project.*
