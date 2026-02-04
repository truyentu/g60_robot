# MODE_01: Welding Mode (MIG/MAG)

## Document Info
| Item | Value |
|------|-------|
| **Mode** | Welding (MIG/MAG Arc Welding) |
| **Priority** | P0 - Primary Mode |
| **Hardware** | MIG/MAG Power Source, Laser Profiler (optional) |
| **Dependencies** | StateManager, Trajectory, grblHAL, I/O Interface |
| **Last Updated** | 2026-02-01 |

---

## 1. Overview

### 1.1. Purpose
Cung cấp khả năng hàn MIG/MAG (Metal Inert Gas / Metal Active Gas) tự động với Robot 6-DOF. Robot đóng vai trò **Master** điều phối trình tự thời gian (timing sequence), trong khi nguồn hàn đóng vai trò **Slave** quản lý năng lượng hàn (Synergic curves).

### 1.2. Key Features
| Feature | Description |
|---------|-------------|
| **Arc Sequence Control** | Pre-flow → Ignition → Weld → Crater → Burnback → Post-flow |
| **Synergic Integration** | Robot gửi setpoint, nguồn hàn tính toán V/A |
| **Weaving Patterns** | Zigzag, Sine, Triangle, Crescent oscillation |
| **Seam Tracking** | Real-time path correction với Laser Profiler (P2) |
| **Multi-pass Welding** | Hỗ trợ hàn nhiều lớp (root, fill, cap) |

### 1.3. Architecture Context

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     WELDING MODE ARCHITECTURE                            │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │  ROBOT CONTROLLER (Master)                                        │   │
│  │                                                                    │   │
│  │  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐          │   │
│  │  │  Welding     │   │   Motion     │   │    I/O       │          │   │
│  │  │  Sequencer   │──▶│  Planner     │──▶│  Interface   │          │   │
│  │  │   (FSM)      │   │  (Ruckig)    │   │  (Digital)   │          │   │
│  │  └──────┬───────┘   └──────────────┘   └──────┬───────┘          │   │
│  │         │                                      │                  │   │
│  │         │  State Commands                      │  DO/DI Signals   │   │
│  │         │                                      │                  │   │
│  └─────────┼──────────────────────────────────────┼──────────────────┘   │
│            │                                      │                      │
│            ▼                                      ▼                      │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │  WELDING POWER SOURCE (Slave)                                     │   │
│  │                                                                    │   │
│  │  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐          │   │
│  │  │   Synergic   │   │    Wire      │   │    Arc       │          │   │
│  │  │    Core      │   │   Feeder     │   │  Generator   │          │   │
│  │  │  (V/A Calc)  │   │              │   │              │          │   │
│  │  └──────────────┘   └──────────────┘   └──────────────┘          │   │
│  │                                                                    │   │
│  │  Inputs: DO_ARC_START, AO_WFS_REF, AO_VOLT_REF                   │   │
│  │  Outputs: DI_ARC_OK, DI_READY, DI_ERROR                          │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Welding Sequencer FSM

### 2.1. State Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    WELDING SEQUENCER STATE MACHINE                       │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│                            ┌─────────┐                                  │
│                            │  IDLE   │◀─────────────────────────┐       │
│                            │  (0)    │                          │       │
│                            └────┬────┘                          │       │
│                                 │ CMD_START                     │       │
│                                 ▼                               │       │
│                            ┌─────────┐                          │       │
│                            │PREFLOW  │                          │       │
│                            │  (1)    │ DO_GAS_VALVE = ON        │       │
│                            └────┬────┘                          │       │
│                                 │ t >= PreFlowTime              │       │
│                                 ▼                               │       │
│                            ┌─────────┐                          │       │
│                       ┌───▶│IGNITION │                          │       │
│                       │    │  (2)    │ DO_ARC_START = ON        │       │
│                       │    └────┬────┘                          │       │
│              Retry    │         │ DI_ARC_OK = HIGH              │       │
│              (max 3)  │         ▼                               │       │
│                       │    ┌─────────┐                          │       │
│                       │    │STABILIZE│                          │       │
│                       │    │  (3)    │ Wait for stable arc      │       │
│                       │    └────┬────┘                          │       │
│                       │         │ t >= StabilizeTime            │       │
│                       │         ▼                               │       │
│                       │    ┌─────────┐                          │       │
│  Timeout ─────────────┴───▶│  WELD   │ MOTION_PERMIT = TRUE     │       │
│  (Ignition Failure)        │  (4)    │◀─────────────┐           │       │
│           │                └────┬────┘              │           │       │
│           │                     │ CMD_STOP          │ Arc Lost  │       │
│           │                     ▼                   │ (Re-strike)│      │
│           │                ┌─────────┐              │           │       │
│           │                │ CRATER  │              │           │       │
│           │                │  (5)    │ Reduce current           │       │
│           │                └────┬────┘              │           │       │
│           │                     │ t >= CraterTime   │           │       │
│           │                     ▼                   │           │       │
│           │                ┌─────────┐              │           │       │
│           │                │BURNBACK │              │           │       │
│           │                │  (6)    │ Stop wire, keep arc      │       │
│           │                └────┬────┘              │           │       │
│           │                     │ t >= BurnbackTime │           │       │
│           │                     ▼                   │           │       │
│           │                ┌─────────┐              │           │       │
│           │                │POSTFLOW │ DO_ARC_START = OFF       │       │
│           │                │  (7)    │ DO_GAS_VALVE = ON        │       │
│           │                └────┬────┘              │           │       │
│           │                     │ t >= PostFlowTime │           │       │
│           │                     └───────────────────┴───────────┘       │
│           │                                                             │
│           ▼                                                             │
│      ┌─────────┐                                                        │
│      │  FAULT  │ All outputs OFF, MOTION_INHIBIT                       │
│      │  (99)   │◀─── DI_ERROR / Arc Lost (no restrike) / Wire Stuck    │
│      └────┬────┘                                                        │
│           │ Reset + Fault Cleared                                       │
│           └─────────────────────────────────────────────────────────────┘
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 2.2. State Definitions

| State | ID | Description | Outputs | Duration |
|-------|-----|-------------|---------|----------|
| **IDLE** | 0 | Waiting for start command | All OFF | - |
| **PREFLOW** | 1 | Gas purging | DO_GAS = ON | 0.3-0.5s |
| **IGNITION** | 2 | Arc striking | DO_ARC_START = ON | Until ARC_OK |
| **STABILIZE** | 3 | Arc stabilization | Hold position | 0.1-0.2s |
| **WELD** | 4 | Welding in progress | MOTION_PERMIT = ON | Until CMD_STOP |
| **CRATER** | 5 | Crater fill | Reduce WFS/V | 0.5-0.8s |
| **BURNBACK** | 6 | Wire retraction | Stop wire, keep arc | 0.05-0.12s |
| **POSTFLOW** | 7 | Post-weld gas shield | DO_GAS = ON | 1.0-3.0s |
| **FAULT** | 99 | Error state | All OFF | Until reset |

### 2.3. Timing Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      WELDING TIMING DIAGRAM                              │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  Time ─────────────────────────────────────────────────────────────────▶│
│                                                                          │
│  CMD_START ─────┐                                                        │
│                 └───────────────────────────────────────────────────────│
│                                                                          │
│  DO_GAS_VALVE ──┐                                                  ┌────│
│                 └──────────────────────────────────────────────────┘    │
│                 │◀─Pre─▶│                              │◀──Post──▶│     │
│                                                                          │
│  DO_ARC_START ──────────┐                              ┌────────────────│
│                         └──────────────────────────────┘                │
│                         │◀──────── Weld Time ─────────▶│                │
│                                                                          │
│  DI_ARC_OK ─────────────────┐                      ┌────────────────────│
│                             └──────────────────────┘                    │
│                             │◀─Stab─▶│◀──Weld──▶│◀Crat▶│                │
│                                                                          │
│  MOTION_PERMIT ────────────────────┐           ┌────────────────────────│
│                                    └───────────┘                        │
│                                    │◀──Robot Moving──▶│                 │
│                                                                          │
│  Wire Feed ────────────────────────┬───────────┬────┬───────────────────│
│                                    │ Full WFS  │Crat│BB                 │
│                                    │           │ er │                   │
│                                                                          │
│  States:    IDLE │PRE│ IGN │STAB│    WELD     │CRAT│BB│  POSTFLOW │IDLE │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Hardware Interface (I/O Map)

### 3.1. Digital Outputs (Robot → Power Source)

| Signal | Type | Function | Active | Notes |
|--------|------|----------|--------|-------|
| **DO_GAS_VALVE** | Digital | Gas solenoid control | HIGH = Open | Separate from Arc for Pre-flow |
| **DO_ARC_START** | Digital | Main contactor command | HIGH = Weld | Triggers wire feed + power |
| **DO_WIRE_INCH** | Digital | Wire inching (no weld) | HIGH = Feed | For spool change/test |
| **DO_WIRE_RETRACT** | Digital | Wire retraction | HIGH = Retract | For stick-out adjustment |
| **DO_ERROR_RESET** | Digital | Clear welder faults | Pulse | After fault recovery |
| **DO_JOB_BIT0..3** | Digital | Job/Program select | Binary | Select Synergic curve |

### 3.2. Digital Inputs (Power Source → Robot)

| Signal | Type | Function | Active | Notes |
|--------|------|----------|--------|-------|
| **DI_ARC_OK** | Digital | Arc established | HIGH = OK | **Critical** - enables motion |
| **DI_READY** | Digital | Welder ready | HIGH = Ready | Power on, no faults |
| **DI_WELD_ERROR** | Digital | General fault | HIGH = Fault | Overtemp, phase loss, etc. |
| **DI_WIRE_STUCK** | Digital | Anti-stick triggered | HIGH = Stuck | Wire stuck in puddle |
| **DI_GAS_FLOW** | Digital | Gas flow sensor | HIGH = Flowing | Recommended for safety |

### 3.3. Analog Interface (0-10V)

| Signal | Type | Range | Function | Notes |
|--------|------|-------|----------|-------|
| **AO_WFS_REF** | Output | 0-10V | Wire Feed Speed / Current | 5V = 150A typical |
| **AO_VOLT_REF** | Output | 0-10V | Voltage / Arc Length Trim | 5V = 0 trim |
| **AI_CURRENT** | Input | 0-10V | Actual welding current | For monitoring |
| **AI_VOLTAGE** | Input | 0-10V | Actual arc voltage | For monitoring |

### 3.4. I/O Wiring Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        I/O WIRING DIAGRAM                                │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  TEENSY 4.1                              POWER SOURCE                   │
│  ┌─────────────┐                         ┌─────────────┐                │
│  │             │                         │             │                │
│  │  DO (24V)   │                         │   DI (24V)  │                │
│  │  ┌───────┐  │      Opto-Isolator      │  ┌───────┐  │                │
│  │  │ Pin 2 │──┼────────[▷|◁]────────────┼──│ X1.1  │  │  Gas Valve     │
│  │  │ Pin 3 │──┼────────[▷|◁]────────────┼──│ X1.2  │  │  Arc Start     │
│  │  │ Pin 4 │──┼────────[▷|◁]────────────┼──│ X1.3  │  │  Wire Inch     │
│  │  │ Pin 5 │──┼────────[▷|◁]────────────┼──│ X1.4  │  │  Wire Retract  │
│  │  │ Pin 6 │──┼────────[▷|◁]────────────┼──│ X1.5  │  │  Error Reset   │
│  │  └───────┘  │                         │  └───────┘  │                │
│  │             │                         │             │                │
│  │  DI (24V)   │                         │   DO (24V)  │                │
│  │  ┌───────┐  │      Opto-Isolator      │  ┌───────┐  │                │
│  │  │ Pin 10│──┼────────[▷|◁]────────────┼──│ X2.1  │  │  Arc OK        │
│  │  │ Pin 11│──┼────────[▷|◁]────────────┼──│ X2.2  │  │  Ready         │
│  │  │ Pin 12│──┼────────[▷|◁]────────────┼──│ X2.3  │  │  Error         │
│  │  │ Pin 13│──┼────────[▷|◁]────────────┼──│ X2.4  │  │  Wire Stuck    │
│  │  └───────┘  │                         │  └───────┘  │                │
│  │             │                         │             │                │
│  │  AO (0-10V) │      Shielded Cable     │   AI       │                │
│  │  ┌───────┐  │      (Twisted Pair)     │  ┌───────┐  │                │
│  │  │DAC0   │──┼─────────────────────────┼──│ A1    │  │  WFS Reference │
│  │  │DAC1   │──┼─────────────────────────┼──│ A2    │  │  Volt Reference│
│  │  │GND    │──┼─────────────────────────┼──│ GND   │  │                │
│  │  └───────┘  │   Shield grounded       │  └───────┘  │                │
│  │             │   at robot side only    │             │                │
│  └─────────────┘                         └─────────────┘                │
│                                                                          │
│  Notes:                                                                  │
│  - All digital signals through opto-isolators for EMI protection        │
│  - Analog cables: shielded twisted pair, shield grounded at one end    │
│  - 24VDC power supply for I/O isolation                                 │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 4. WeldingSequencer Implementation

### 4.1. Class Structure (C++)

```cpp
// welding/welding_sequencer.hpp
#pragma once

#include <chrono>
#include <functional>
#include <optional>

namespace robot::welding {

enum class WeldState : uint8_t {
    Idle = 0,
    PreFlow = 1,
    Ignition = 2,
    Stabilize = 3,
    Weld = 4,
    Crater = 5,
    Burnback = 6,
    PostFlow = 7,
    Fault = 99
};

enum class WeldFault : uint8_t {
    None = 0,
    IgnitionTimeout = 1,
    ArcLost = 2,
    WireStuck = 3,
    PowerSourceError = 4,
    GasFlowError = 5
};

struct WeldParameters {
    // Timing parameters (seconds)
    double preFlowTime = 0.3;
    double ignitionTimeout = 2.0;
    double stabilizeTime = 0.2;
    double craterTime = 0.5;
    double burnbackTime = 0.08;
    double postFlowTime = 1.0;

    // Weld parameters
    double weldWFS = 8.0;      // m/min
    double weldVoltage = 22.0; // V (or trim 0-10)
    double craterWFS = 5.0;    // m/min (60-80% of weld)
    double craterVoltage = 18.0;

    // Arc recovery
    int maxIgnitionRetries = 3;
    double arcLostTimeout = 0.2;  // seconds before fault

    // Material presets
    static WeldParameters Steel();
    static WeldParameters Aluminum();
    static WeldParameters StainlessSteel();
};

struct WeldIO {
    // Digital Outputs
    bool gasValve = false;
    bool arcStart = false;
    bool wireInch = false;
    bool wireRetract = false;
    bool errorReset = false;
    uint8_t jobSelect = 0;

    // Analog Outputs (0.0 - 10.0 V)
    double wfsReference = 0.0;
    double voltReference = 5.0;  // 5V = neutral trim

    // Digital Inputs
    bool arcOK = false;
    bool ready = false;
    bool error = false;
    bool wireStuck = false;
    bool gasFlow = false;

    // Analog Inputs
    double actualCurrent = 0.0;
    double actualVoltage = 0.0;
};

class WeldingSequencer {
public:
    using StateCallback = std::function<void(WeldState, WeldState)>;
    using FaultCallback = std::function<void(WeldFault)>;
    using MotionPermitCallback = std::function<void(bool)>;

    WeldingSequencer();

    // Configuration
    void setParameters(const WeldParameters& params);
    WeldParameters getParameters() const;

    // Callbacks
    void onStateChange(StateCallback callback);
    void onFault(FaultCallback callback);
    void onMotionPermit(MotionPermitCallback callback);

    // Commands
    void start();
    void stop();
    void reset();
    void emergencyStop();

    // Update loop (call at 100Hz / 10ms)
    void update(const WeldIO& inputs, WeldIO& outputs);

    // Status
    WeldState currentState() const { return state_; }
    WeldFault currentFault() const { return fault_; }
    bool isWelding() const { return state_ == WeldState::Weld; }
    bool motionPermitted() const { return motionPermit_; }
    double stateElapsedTime() const;

private:
    void transitionTo(WeldState newState);
    void handleIdle(const WeldIO& inputs, WeldIO& outputs);
    void handlePreFlow(const WeldIO& inputs, WeldIO& outputs);
    void handleIgnition(const WeldIO& inputs, WeldIO& outputs);
    void handleStabilize(const WeldIO& inputs, WeldIO& outputs);
    void handleWeld(const WeldIO& inputs, WeldIO& outputs);
    void handleCrater(const WeldIO& inputs, WeldIO& outputs);
    void handleBurnback(const WeldIO& inputs, WeldIO& outputs);
    void handlePostFlow(const WeldIO& inputs, WeldIO& outputs);
    void handleFault(const WeldIO& inputs, WeldIO& outputs);

    bool debounceArcOK(bool rawSignal);
    void setFault(WeldFault fault);
    void clearOutputs(WeldIO& outputs);

    WeldState state_ = WeldState::Idle;
    WeldFault fault_ = WeldFault::None;
    WeldParameters params_;

    std::chrono::steady_clock::time_point stateEntryTime_;
    bool motionPermit_ = false;
    bool startRequested_ = false;
    bool stopRequested_ = false;

    // Arc OK debouncing
    int arcOKCounter_ = 0;
    static constexpr int ARC_OK_ON_THRESHOLD = 2;   // 20ms @ 10ms cycle
    static constexpr int ARC_OK_OFF_THRESHOLD = 5;  // 50ms @ 10ms cycle

    // Retry counter
    int ignitionRetryCount_ = 0;

    // Callbacks
    StateCallback stateCallback_;
    FaultCallback faultCallback_;
    MotionPermitCallback motionPermitCallback_;
};

} // namespace robot::welding
```

### 4.2. State Machine Implementation

```cpp
// welding/welding_sequencer.cpp
#include "welding_sequencer.hpp"
#include <spdlog/spdlog.h>

namespace robot::welding {

void WeldingSequencer::update(const WeldIO& inputs, WeldIO& outputs)
{
    // Check for global fault conditions
    if (inputs.error && state_ != WeldState::Fault && state_ != WeldState::Idle) {
        setFault(WeldFault::PowerSourceError);
        return;
    }

    if (inputs.wireStuck && state_ != WeldState::Fault) {
        setFault(WeldFault::WireStuck);
        return;
    }

    // State-specific handling
    switch (state_) {
        case WeldState::Idle:      handleIdle(inputs, outputs); break;
        case WeldState::PreFlow:   handlePreFlow(inputs, outputs); break;
        case WeldState::Ignition:  handleIgnition(inputs, outputs); break;
        case WeldState::Stabilize: handleStabilize(inputs, outputs); break;
        case WeldState::Weld:      handleWeld(inputs, outputs); break;
        case WeldState::Crater:    handleCrater(inputs, outputs); break;
        case WeldState::Burnback:  handleBurnback(inputs, outputs); break;
        case WeldState::PostFlow:  handlePostFlow(inputs, outputs); break;
        case WeldState::Fault:     handleFault(inputs, outputs); break;
    }
}

void WeldingSequencer::handleIdle(const WeldIO& inputs, WeldIO& outputs)
{
    clearOutputs(outputs);

    if (startRequested_ && inputs.ready) {
        startRequested_ = false;
        ignitionRetryCount_ = 0;
        transitionTo(WeldState::PreFlow);
    }
}

void WeldingSequencer::handlePreFlow(const WeldIO& inputs, WeldIO& outputs)
{
    outputs.gasValve = true;
    outputs.arcStart = false;

    if (stateElapsedTime() >= params_.preFlowTime) {
        transitionTo(WeldState::Ignition);
    }
}

void WeldingSequencer::handleIgnition(const WeldIO& inputs, WeldIO& outputs)
{
    outputs.gasValve = true;
    outputs.arcStart = true;

    // Set initial weld parameters
    outputs.wfsReference = wfsToVoltage(params_.weldWFS);
    outputs.voltReference = params_.weldVoltage / 4.0;  // Scale to 0-10V

    // Check for arc establishment (with debouncing)
    if (debounceArcOK(inputs.arcOK)) {
        transitionTo(WeldState::Stabilize);
        return;
    }

    // Ignition timeout
    if (stateElapsedTime() >= params_.ignitionTimeout) {
        ignitionRetryCount_++;

        if (ignitionRetryCount_ < params_.maxIgnitionRetries) {
            spdlog::warn("Ignition failed, retry {}/{}",
                        ignitionRetryCount_, params_.maxIgnitionRetries);

            // Retry sequence: retract wire, wait, try again
            outputs.arcStart = false;
            outputs.wireRetract = true;
            transitionTo(WeldState::PreFlow);  // Restart from PreFlow
        } else {
            setFault(WeldFault::IgnitionTimeout);
        }
    }
}

void WeldingSequencer::handleStabilize(const WeldIO& inputs, WeldIO& outputs)
{
    outputs.gasValve = true;
    outputs.arcStart = true;
    outputs.wfsReference = wfsToVoltage(params_.weldWFS);
    outputs.voltReference = params_.weldVoltage / 4.0;

    // Check arc still OK
    if (!debounceArcOK(inputs.arcOK)) {
        // Arc lost during stabilization - retry
        transitionTo(WeldState::Ignition);
        return;
    }

    if (stateElapsedTime() >= params_.stabilizeTime) {
        motionPermit_ = true;
        if (motionPermitCallback_) {
            motionPermitCallback_(true);
        }
        transitionTo(WeldState::Weld);
    }
}

void WeldingSequencer::handleWeld(const WeldIO& inputs, WeldIO& outputs)
{
    outputs.gasValve = true;
    outputs.arcStart = true;
    outputs.wfsReference = wfsToVoltage(params_.weldWFS);
    outputs.voltReference = params_.weldVoltage / 4.0;

    // Monitor arc
    if (!debounceArcOK(inputs.arcOK)) {
        // Arc lost - attempt re-strike
        motionPermit_ = false;
        if (motionPermitCallback_) {
            motionPermitCallback_(false);
        }

        if (stateElapsedTime() >= params_.arcLostTimeout) {
            setFault(WeldFault::ArcLost);
        }
        // Stay in Weld state, arc might recover
        return;
    }

    // Check for stop command
    if (stopRequested_) {
        stopRequested_ = false;
        motionPermit_ = false;
        if (motionPermitCallback_) {
            motionPermitCallback_(false);
        }
        transitionTo(WeldState::Crater);
    }
}

void WeldingSequencer::handleCrater(const WeldIO& inputs, WeldIO& outputs)
{
    outputs.gasValve = true;
    outputs.arcStart = true;

    // Reduced parameters for crater fill
    outputs.wfsReference = wfsToVoltage(params_.craterWFS);
    outputs.voltReference = params_.craterVoltage / 4.0;

    if (stateElapsedTime() >= params_.craterTime) {
        transitionTo(WeldState::Burnback);
    }
}

void WeldingSequencer::handleBurnback(const WeldIO& inputs, WeldIO& outputs)
{
    outputs.gasValve = true;
    outputs.arcStart = true;  // Keep power on
    outputs.wfsReference = 0.0;  // Stop wire feed

    if (stateElapsedTime() >= params_.burnbackTime) {
        outputs.arcStart = false;  // Now turn off arc
        transitionTo(WeldState::PostFlow);
    }
}

void WeldingSequencer::handlePostFlow(const WeldIO& inputs, WeldIO& outputs)
{
    outputs.gasValve = true;
    outputs.arcStart = false;
    outputs.wfsReference = 0.0;
    outputs.voltReference = 5.0;  // Neutral

    if (stateElapsedTime() >= params_.postFlowTime) {
        transitionTo(WeldState::Idle);
    }
}

void WeldingSequencer::handleFault(const WeldIO& inputs, WeldIO& outputs)
{
    clearOutputs(outputs);
    motionPermit_ = false;
    // Wait for reset() to be called
}

bool WeldingSequencer::debounceArcOK(bool rawSignal)
{
    if (rawSignal) {
        arcOKCounter_ = std::min(arcOKCounter_ + 1, ARC_OK_ON_THRESHOLD + 1);
    } else {
        arcOKCounter_ = std::max(arcOKCounter_ - 1, -(ARC_OK_OFF_THRESHOLD + 1));
    }

    // Hysteresis: ON if counter >= threshold, OFF if counter <= -threshold
    if (arcOKCounter_ >= ARC_OK_ON_THRESHOLD) {
        return true;
    } else if (arcOKCounter_ <= -ARC_OK_OFF_THRESHOLD) {
        return false;
    }

    // In hysteresis zone, maintain previous state
    return arcOKCounter_ > 0;
}

double WeldingSequencer::wfsToVoltage(double wfsMetersPerMin)
{
    // Convert WFS (m/min) to 0-10V signal
    // Assuming 0V = 0 m/min, 10V = 20 m/min
    return std::clamp(wfsMetersPerMin / 2.0, 0.0, 10.0);
}

} // namespace robot::welding
```

---

## 5. Weaving Patterns

### 5.1. Weaving Types

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        WEAVING PATTERNS                                  │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  1. ZIGZAG (Triangle)                                                   │
│     ────/\────/\────/\────                                              │
│         \/    \/    \/                                                  │
│                                                                          │
│  2. SINE WAVE                                                           │
│     ────~~~~~────~~~~~────                                              │
│          ︶︶︶    ︶︶︶                                                    │
│                                                                          │
│  3. SQUARE                                                              │
│     ────┌─┐────┌─┐────┌─┐                                               │
│         └─┘    └─┘    └─┘                                               │
│                                                                          │
│  4. CRESCENT (Lưỡi liềm)                                                │
│     ────⌒────⌒────⌒────                                                 │
│                                                                          │
│  5. FIGURE-8                                                            │
│     ────∞────∞────∞────                                                 │
│                                                                          │
│  Parameters:                                                             │
│  ├── Amplitude (A): Width of oscillation (mm)                           │
│  ├── Frequency (f): Oscillations per second (Hz)                        │
│  ├── Dwell Time: Pause at edges (ms)                                    │
│  └── Phase: Starting phase offset (degrees)                             │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 5.2. Weaving Algorithm

```cpp
// welding/weaving.hpp
#pragma once

#include <Eigen/Core>
#include <cmath>

namespace robot::welding {

enum class WeavePattern {
    None,
    Zigzag,
    Sine,
    Square,
    Crescent,
    Figure8
};

struct WeaveParameters {
    WeavePattern pattern = WeavePattern::None;
    double amplitude = 3.0;      // mm (half-width)
    double frequency = 2.0;      // Hz
    double dwellLeft = 0.0;      // seconds
    double dwellRight = 0.0;     // seconds
    double phase = 0.0;          // radians
};

class WeavingGenerator {
public:
    WeavingGenerator() = default;

    void setParameters(const WeaveParameters& params) { params_ = params; }

    // Calculate lateral offset at time t
    // Returns offset perpendicular to weld direction
    Eigen::Vector3d calculateOffset(
        double t,
        const Eigen::Vector3d& tangent,     // Weld direction (normalized)
        const Eigen::Vector3d& toolAxis     // Tool Z axis (normalized)
    ) const
    {
        if (params_.pattern == WeavePattern::None) {
            return Eigen::Vector3d::Zero();
        }

        // Calculate binormal (lateral direction)
        // B = T × Z (cross product)
        Eigen::Vector3d binormal = tangent.cross(toolAxis).normalized();

        // Calculate oscillation value [-1, 1]
        double oscillation = calculateOscillation(t);

        // Apply amplitude
        return binormal * (oscillation * params_.amplitude);
    }

private:
    double calculateOscillation(double t) const
    {
        double omega = 2.0 * M_PI * params_.frequency;
        double phase = omega * t + params_.phase;

        switch (params_.pattern) {
            case WeavePattern::Sine:
                return std::sin(phase);

            case WeavePattern::Zigzag: {
                // Triangle wave
                double normalized = std::fmod(phase / (2.0 * M_PI), 1.0);
                if (normalized < 0.25) {
                    return 4.0 * normalized;
                } else if (normalized < 0.75) {
                    return 2.0 - 4.0 * normalized;
                } else {
                    return -4.0 + 4.0 * normalized;
                }
            }

            case WeavePattern::Square: {
                return std::sin(phase) >= 0 ? 1.0 : -1.0;
            }

            case WeavePattern::Crescent: {
                // Asymmetric sine - more time on one side
                double s = std::sin(phase);
                return (s >= 0) ? std::sqrt(s) : -std::sqrt(-s);
            }

            case WeavePattern::Figure8: {
                // Lissajous-like pattern
                double x = std::sin(phase);
                double y = std::sin(2.0 * phase);
                return x * (1.0 + 0.5 * y);
            }

            default:
                return 0.0;
        }
    }

    WeaveParameters params_;
};

} // namespace robot::welding
```

---

## 6. Weld Program Structure

### 6.1. Program Command Types

```cpp
// welding/weld_program.hpp

enum class WeldCommandType {
    // Motion commands
    MoveJ,      // Joint move (approach)
    MoveL,      // Linear move (weld path)
    MoveC,      // Circular move (curved weld)

    // Welding commands
    ArcStart,   // Start welding
    ArcEnd,     // End welding
    WeldParams, // Change weld parameters mid-weld

    // Weaving commands
    WeaveOn,    // Enable weaving
    WeaveOff,   // Disable weaving

    // Logic commands
    SetIO,      // Set digital output
    WaitIO,     // Wait for input condition
    Delay,      // Time delay
    Comment     // Comment/label
};

struct WeldCommand {
    WeldCommandType type;
    std::string label;

    // Motion data
    CartesianPose targetPose;
    double speed;           // mm/s for linear, %max for joint
    double acceleration;    // mm/s²

    // Weld data
    WeldParameters weldParams;
    WeaveParameters weaveParams;

    // I/O data
    int ioPort;
    bool ioValue;
    double timeout;
};
```

### 6.2. Example Weld Program

```
; Example Weld Program - Fillet Weld
; Material: Steel, 6mm plate, 1.2mm wire

; Approach position
1: MOVJ P[1] 100% FINE           ; Move to start approach
2: MOVL P[2] 500mm/s FINE        ; Approach weld start

; Weld parameters
3: WELD_PARAMS WFS=8.0 V=22.0    ; Set weld parameters
4: WEAVE_ON AMP=3.0 FREQ=2.0     ; Enable zigzag weave

; Start welding
5: ARC_START                      ; Pre-flow, ignition, stabilize

; Weld path
6: MOVL P[3] 400mm/s CNT100      ; First segment
7: MOVL P[4] 400mm/s CNT100      ; Second segment
8: WELD_PARAMS WFS=6.0 V=20.0    ; Reduce for corner
9: MOVL P[5] 300mm/s CNT50       ; Corner segment
10: WELD_PARAMS WFS=8.0 V=22.0   ; Resume normal
11: MOVL P[6] 400mm/s FINE       ; Final segment

; End welding
12: ARC_END                       ; Crater, burnback, postflow

; Retract
13: WEAVE_OFF
14: MOVL P[7] 500mm/s FINE       ; Retract from weld
15: MOVJ P[8] 100% FINE          ; Return to home
```

---

## 7. Welding UI Components

### 7.1. WeldingPanelView

```xml
<!-- Views/Welding/WeldingPanelView.xaml -->
<UserControl x:Class="RobotUI.Views.WeldingPanelView">
    <Grid>
        <Grid.RowDefinitions>
            <RowDefinition Height="Auto"/>  <!-- Status -->
            <RowDefinition Height="*"/>      <!-- Parameters -->
            <RowDefinition Height="Auto"/>  <!-- Controls -->
        </Grid.RowDefinitions>

        <!-- Weld Status Indicator -->
        <Border Grid.Row="0" Background="{Binding StatusColor}" Padding="20">
            <StackPanel>
                <TextBlock Text="{Binding CurrentState}"
                           Style="{StaticResource HeaderTextStyle}"/>
                <TextBlock Text="{Binding StatusMessage}"
                           Style="{StaticResource BodyTextStyle}"/>
            </StackPanel>
        </Border>

        <!-- Parameters Panel -->
        <ScrollViewer Grid.Row="1">
            <StackPanel Margin="20">
                <!-- Timing Parameters -->
                <GroupBox Header="Timing">
                    <Grid>
                        <Grid.ColumnDefinitions>
                            <ColumnDefinition Width="*"/>
                            <ColumnDefinition Width="100"/>
                        </Grid.ColumnDefinitions>
                        <Grid.RowDefinitions>
                            <RowDefinition Height="40"/>
                            <RowDefinition Height="40"/>
                            <RowDefinition Height="40"/>
                            <RowDefinition Height="40"/>
                        </Grid.RowDefinitions>

                        <TextBlock Grid.Row="0" Text="Pre-flow Time (s)"/>
                        <TextBox Grid.Row="0" Grid.Column="1"
                                 Text="{Binding PreFlowTime}"/>

                        <TextBlock Grid.Row="1" Text="Crater Time (s)"/>
                        <TextBox Grid.Row="1" Grid.Column="1"
                                 Text="{Binding CraterTime}"/>

                        <TextBlock Grid.Row="2" Text="Burnback Time (s)"/>
                        <TextBox Grid.Row="2" Grid.Column="1"
                                 Text="{Binding BurnbackTime}"/>

                        <TextBlock Grid.Row="3" Text="Post-flow Time (s)"/>
                        <TextBox Grid.Row="3" Grid.Column="1"
                                 Text="{Binding PostFlowTime}"/>
                    </Grid>
                </GroupBox>

                <!-- Weld Parameters -->
                <GroupBox Header="Weld Parameters" Margin="0,10,0,0">
                    <Grid>
                        <Grid.ColumnDefinitions>
                            <ColumnDefinition Width="*"/>
                            <ColumnDefinition Width="100"/>
                        </Grid.ColumnDefinitions>
                        <Grid.RowDefinitions>
                            <RowDefinition Height="40"/>
                            <RowDefinition Height="40"/>
                        </Grid.RowDefinitions>

                        <TextBlock Grid.Row="0" Text="Wire Feed Speed (m/min)"/>
                        <TextBox Grid.Row="0" Grid.Column="1"
                                 Text="{Binding WeldWFS}"/>

                        <TextBlock Grid.Row="1" Text="Voltage (V)"/>
                        <TextBox Grid.Row="1" Grid.Column="1"
                                 Text="{Binding WeldVoltage}"/>
                    </Grid>
                </GroupBox>

                <!-- Weave Parameters -->
                <GroupBox Header="Weaving" Margin="0,10,0,0">
                    <StackPanel>
                        <ComboBox ItemsSource="{Binding WeavePatterns}"
                                  SelectedItem="{Binding SelectedPattern}"/>
                        <Grid Margin="0,10,0,0">
                            <Grid.ColumnDefinitions>
                                <ColumnDefinition Width="*"/>
                                <ColumnDefinition Width="100"/>
                            </Grid.ColumnDefinitions>
                            <Grid.RowDefinitions>
                                <RowDefinition Height="40"/>
                                <RowDefinition Height="40"/>
                            </Grid.RowDefinitions>

                            <TextBlock Grid.Row="0" Text="Amplitude (mm)"/>
                            <TextBox Grid.Row="0" Grid.Column="1"
                                     Text="{Binding WeaveAmplitude}"/>

                            <TextBlock Grid.Row="1" Text="Frequency (Hz)"/>
                            <TextBox Grid.Row="1" Grid.Column="1"
                                     Text="{Binding WeaveFrequency}"/>
                        </Grid>
                    </StackPanel>
                </GroupBox>
            </StackPanel>
        </ScrollViewer>

        <!-- Control Buttons -->
        <StackPanel Grid.Row="2" Orientation="Horizontal"
                    HorizontalAlignment="Center" Margin="20">
            <Button Content="GAS TEST" Command="{Binding GasTestCommand}"
                    Style="{StaticResource SmartKeyButtonStyle}"/>
            <Button Content="WIRE INCH" Command="{Binding WireInchCommand}"
                    Style="{StaticResource SmartKeyButtonStyle}"/>
            <Button Content="ARC START" Command="{Binding ArcStartCommand}"
                    Style="{StaticResource SmartKeyButtonStyle}"
                    Background="{StaticResource StatusGreenBrush}"/>
            <Button Content="ARC STOP" Command="{Binding ArcStopCommand}"
                    Style="{StaticResource SmartKeyButtonStyle}"
                    Background="{StaticResource StatusRedBrush}"/>
        </StackPanel>
    </Grid>
</UserControl>
```

### 7.2. WeldScopeView (Real-time Monitoring)

```xml
<!-- Views/Welding/WeldScopeView.xaml -->
<UserControl x:Class="RobotUI.Views.WeldScopeView">
    <Grid>
        <Grid.RowDefinitions>
            <RowDefinition Height="*"/>
            <RowDefinition Height="*"/>
        </Grid.RowDefinitions>

        <!-- Current Chart -->
        <oxy:PlotView Grid.Row="0" Model="{Binding CurrentPlotModel}">
            <oxy:PlotView.Axes>
                <oxy:LinearAxis Position="Left" Title="Current (A)"
                                Minimum="0" Maximum="400"/>
                <oxy:LinearAxis Position="Bottom" Title="Time (s)"/>
            </oxy:PlotView.Axes>
        </oxy:PlotView>

        <!-- Voltage Chart -->
        <oxy:PlotView Grid.Row="1" Model="{Binding VoltagePlotModel}">
            <oxy:PlotView.Axes>
                <oxy:LinearAxis Position="Left" Title="Voltage (V)"
                                Minimum="0" Maximum="40"/>
                <oxy:LinearAxis Position="Bottom" Title="Time (s)"/>
            </oxy:PlotView.Axes>
        </oxy:PlotView>
    </Grid>
</UserControl>
```

---

## 8. Weld Parameter Presets

### 8.1. Material Presets

| Parameter | Steel | Aluminum | Stainless Steel |
|-----------|-------|----------|-----------------|
| **Pre-flow** | 0.3s | 0.5s | 0.4s |
| **Ignition Timeout** | 2.0s | 2.0s | 2.0s |
| **Stabilize** | 0.2s | 0.1s | 0.2s |
| **Crater Time** | 0.5s | 0.8s | 0.6s |
| **Burnback** | 0.08s | 0.12s | 0.08s |
| **Post-flow** | 1.0s | 3.0s | 2.0s |

### 8.2. Joint Type Presets

| Joint Type | WFS (m/min) | Voltage (V) | Speed (mm/s) | Weave |
|------------|-------------|-------------|--------------|-------|
| **Butt** | 6-8 | 20-24 | 400-600 | Optional |
| **Fillet** | 8-10 | 22-26 | 300-500 | Recommended |
| **Lap** | 7-9 | 21-25 | 350-550 | None |
| **Corner** | 6-8 | 20-24 | 300-450 | None |

---

## 9. Error Handling & Recovery

### 9.1. Fault Recovery Procedures

| Fault | Detection | Immediate Action | Recovery |
|-------|-----------|------------------|----------|
| **Ignition Failure** | Timeout | Stop arc, retract wire | Retry 3x, then operator |
| **Arc Lost** | DI_ARC_OK = LOW | Stop motion | Re-strike within 200ms |
| **Wire Stuck** | DI_WIRE_STUCK | Lock motion | Operator intervention |
| **Power Source Error** | DI_ERROR | Emergency stop | Reset after clear |
| **Gas Flow Error** | DI_GAS_FLOW = LOW | Stop weld | Check gas supply |

### 9.2. Recovery Algorithm

```cpp
void WeldingSequencer::attemptRecovery(WeldFault fault)
{
    switch (fault) {
        case WeldFault::IgnitionTimeout:
            if (ignitionRetryCount_ < params_.maxIgnitionRetries) {
                // Retract wire
                outputs_.wireRetract = true;
                std::this_thread::sleep_for(500ms);
                outputs_.wireRetract = false;

                // Lift Z
                if (liftForRetry_) {
                    liftForRetry_(10.0);  // mm
                }

                // Retry from PreFlow
                transitionTo(WeldState::PreFlow);
            } else {
                // Give up, require operator
                setFault(WeldFault::IgnitionTimeout);
            }
            break;

        case WeldFault::ArcLost:
            // Attempt re-strike without moving
            // Stay in current state, sequencer will retry
            break;

        default:
            // Unrecoverable, require operator
            setFault(fault);
            break;
    }
}
```

---

## 10. Tasks Breakdown

### 10.1. Implementation Tasks

| ID | Task | Description | Priority |
|----|------|-------------|----------|
| T01 | WeldingSequencer FSM | Core state machine | P0 |
| T02 | I/O Interface | Digital/Analog I/O handling | P0 |
| T03 | Arc OK debouncing | Signal filtering | P0 |
| T04 | Timing management | State timeouts | P0 |
| T05 | Weaving generator | Pattern algorithms | P1 |
| T06 | Weld program parser | Program file loader | P1 |
| T07 | WeldingPanel UI | Parameter editing | P1 |
| T08 | WeldScope UI | Real-time charts | P2 |
| T09 | Error recovery | Retry logic | P1 |
| T10 | Material presets | Preset management | P2 |

### 10.2. Testing Checklist

```
[ ] Pre-flow timing accurate (±50ms)
[ ] Arc OK detection reliable (no false triggers)
[ ] Motion blocked until arc stable
[ ] Crater fill reduces current correctly
[ ] Burnback prevents wire stick
[ ] Post-flow protects weld pool
[ ] E-Stop immediately kills arc
[ ] Ignition retry works (max 3)
[ ] Arc lost detection < 200ms
[ ] Weaving amplitude correct (±0.5mm)
```

---

## 11. References

### 11.1. Research Documents
- Thiết Kế Module Điều Khiển Hàn MIG_MAG.md

### 11.2. External Resources
| Resource | URL |
|----------|-----|
| Miller Robotic Interface | https://www.millerwelds.com/files/owners-manuals/o172324d_mil.pdf |
| Fronius Robot Interface | https://www.fronius.com |
| AWS D1.1 Welding Code | https://www.aws.org |

### 11.3. Related Documents
| Document | Description |
|----------|-------------|
| [CORE_StateManager.md](../core/CORE_StateManager.md) | System state machine |
| [CORE_Trajectory.md](../core/CORE_Trajectory.md) | Motion planning |
| [PHASE_3_Welding.md](../phases/PHASE_3_Welding.md) | Phase 3 details |

---

*Document version: 1.0 | Last updated: 2026-02-01*
