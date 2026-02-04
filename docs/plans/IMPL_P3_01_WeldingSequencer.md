# IMPL_P3_01: Welding Sequencer

| Metadata | Value |
|----------|-------|
| Plan ID | IMPL_P3_01 |
| Phase | 3 - Welding Integration |
| Priority | P0 (Critical) |
| Depends On | IMPL_P2_05 (Motion HMI) |
| Status | Ready for Implementation |

---

## Required Reading (ĐỌC TRƯỚC KHI CODE)

| Priority | Document | Lý do |
|----------|----------|-------|
| P0 | `ressearch_doc_md/Thiết Kế Module Điều Khiển Hàn MIG_MAG.md` | MIG/MAG process, arc control, timing, gas flow |
| P1 | `ressearch_doc_md/Thiết Kế FSM Robot Công Nghiệp An Toàn.md` | FSM patterns cho welding states |

---

## Overview

Implementation plan cho Welding Sequencer - điều khiển quy trình hàn MIG/MAG:
- **Welding FSM:** State machine cho quy trình hàn
- **Timing Control:** Gas pre-flow, arc start/end, post-flow
- **Arc Control:** Arc on/off, crater fill
- **Wire Feed:** Wire speed control, burn-back
- **Parameters:** Current, voltage, wire speed profiles
- **Safety:** Interlocks, fault detection

---

## Welding Process Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        MIG/MAG Welding Sequence                          │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌──────┐  ┌─────────┐  ┌──────────┐  ┌─────────┐  ┌─────────┐         │
│  │ IDLE │─►│PRE-FLOW │─►│ARC START │─►│ WELDING │─►│ARC END  │─►...    │
│  └──────┘  └─────────┘  └──────────┘  └─────────┘  └─────────┘         │
│                                                           │              │
│     ┌──────────┐  ┌───────────┐  ┌──────┐                │              │
│ ...─│BURN-BACK │─►│ POST-FLOW │─►│ IDLE │◄───────────────┘              │
│     └──────────┘  └───────────┘  └──────┘                               │
│                                                                          │
├─────────────────────────────────────────────────────────────────────────┤
│  Timeline:                                                               │
│                                                                          │
│  ─────┬────────┬───────────┬─────────────────────┬──────────┬─────────  │
│       │        │           │                     │          │           │
│       │◄──────►│◄─────────►│◄───────────────────►│◄────────►│◄────────► │
│       │Pre-flow│ Arc Start │      Welding        │ Arc End  │Post-flow  │
│       │ 0.3s   │   0.5s    │     Variable        │  0.5s    │  0.5s     │
│                                                                          │
│  Gas: ████████████████████████████████████████████████████████████████  │
│  Arc:          ████████████████████████████████████████████             │
│  Wire:              ████████████████████████████████████                │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Prerequisites

- [ ] IMPL_P2_05 (Motion HMI) đã hoàn thành
- [ ] Hardware I/O interface defined
- [ ] Welding power source communication protocol known
- [ ] Safety requirements documented

---

## Step 1: Create Welding Types

### 1.1 Create WeldingTypes.hpp

**File:** `src/cpp/include/welding/WeldingTypes.hpp`

```cpp
#pragma once

#include <cstdint>
#include <string>
#include <array>
#include <chrono>

namespace robotics {
namespace welding {

// ============================================================================
// Welding Process Types
// ============================================================================

/**
 * Welding process type
 */
enum class WeldingProcess : uint8_t {
    MIG_MAG = 0,        // Gas Metal Arc Welding (GMAW)
    TIG = 1,            // Gas Tungsten Arc Welding (GTAW)
    SPOT = 2,           // Spot welding
    PLASMA = 3,         // Plasma cutting/welding
    LASER = 4           // Laser welding
};

/**
 * Transfer mode for MIG/MAG
 */
enum class TransferMode : uint8_t {
    SHORT_ARC = 0,      // Short circuit transfer
    GLOBULAR = 1,       // Globular transfer
    SPRAY = 2,          // Spray transfer
    PULSE = 3,          // Pulse spray transfer
    CMT = 4,            // Cold Metal Transfer
    STT = 5             // Surface Tension Transfer
};

/**
 * Shielding gas type
 */
enum class GasType : uint8_t {
    ARGON = 0,          // Pure Argon
    CO2 = 1,            // Pure CO2
    ARGON_CO2 = 2,      // Ar + CO2 mix (common: 82/18, 75/25)
    ARGON_O2 = 3,       // Ar + O2 mix
    HELIUM = 4,         // Pure Helium
    ARGON_HELIUM = 5    // Ar + He mix
};

/**
 * Wire type
 */
enum class WireType : uint8_t {
    SOLID = 0,          // Solid wire
    FLUX_CORED = 1,     // Flux-cored wire
    METAL_CORED = 2     // Metal-cored wire
};

// ============================================================================
// Welding State Machine
// ============================================================================

/**
 * Welding sequence states
 */
enum class WeldingState : uint8_t {
    IDLE = 0,           // Ready to weld
    PRE_FLOW = 1,       // Gas pre-flow before arc
    ARC_START = 2,      // Arc ignition
    WELDING = 3,        // Active welding
    CRATER_FILL = 4,    // End crater filling
    ARC_END = 5,        // Arc extinguishing
    BURN_BACK = 6,      // Wire burn-back
    POST_FLOW = 7,      // Gas post-flow after weld
    ERROR = 8,          // Fault state
    EMERGENCY_STOP = 9  // E-stop active
};

/**
 * Welding events
 */
enum class WeldingEvent : uint8_t {
    START_WELD = 0,     // Trigger weld start
    STOP_WELD = 1,      // Trigger weld stop
    ABORT = 2,          // Abort immediately
    TIMER_EXPIRED = 3,  // Phase timer completed
    ARC_DETECTED = 4,   // Arc established
    ARC_LOST = 5,       // Arc lost unexpectedly
    FAULT = 6,          // Fault detected
    RESET = 7,          // Reset from error
    EMERGENCY_STOP = 8  // E-stop triggered
};

/**
 * Welding fault codes
 */
enum class WeldingFault : uint8_t {
    NONE = 0,
    NO_GAS = 1,             // Gas flow not detected
    NO_WIRE = 2,            // Wire not feeding
    ARC_FAIL = 3,           // Arc failed to start
    ARC_LOST = 4,           // Arc lost during weld
    OVER_CURRENT = 5,       // Current exceeded limit
    OVER_VOLTAGE = 6,       // Voltage exceeded limit
    STICK_WIRE = 7,         // Wire stuck to workpiece
    POWER_SUPPLY_FAULT = 8, // Power source error
    COMMUNICATION_FAULT = 9,// Communication lost
    SAFETY_FAULT = 10,      // Safety interlock open
    THERMAL_FAULT = 11      // Overheating
};

// ============================================================================
// Welding Parameters
// ============================================================================

/**
 * Timing parameters for welding sequence (in milliseconds)
 */
struct WeldingTimings {
    uint32_t preFlowTime;       // Gas pre-flow time (ms)
    uint32_t arcStartTime;      // Max time for arc to establish (ms)
    uint32_t hotStartTime;      // Hot start duration (ms)
    uint32_t craterFillTime;    // Crater fill time (ms)
    uint32_t burnBackTime;      // Wire burn-back time (ms)
    uint32_t postFlowTime;      // Gas post-flow time (ms)

    // Arc start parameters
    uint32_t arcRetryDelay;     // Delay between retry attempts (ms)
    uint8_t arcRetryCount;      // Number of arc start retries

    WeldingTimings()
        : preFlowTime(300),
          arcStartTime(500),
          hotStartTime(200),
          craterFillTime(500),
          burnBackTime(50),
          postFlowTime(500),
          arcRetryDelay(200),
          arcRetryCount(3) {}
};

/**
 * Welding electrical parameters
 */
struct WeldingParams {
    // Main parameters
    float current;              // Welding current (A)
    float voltage;              // Arc voltage (V)
    float wireSpeed;            // Wire feed speed (m/min)
    float travelSpeed;          // Robot travel speed (mm/s)

    // Hot start (beginning of weld)
    float hotStartCurrent;      // Initial current boost (A)
    float hotStartVoltage;      // Initial voltage (V)
    float hotStartWireSpeed;    // Initial wire speed (m/min)

    // Crater fill (end of weld)
    float craterFillCurrent;    // Crater current (A)
    float craterFillVoltage;    // Crater voltage (V)
    float craterFillWireSpeed;  // Crater wire speed (m/min)

    // Limits
    float maxCurrent;           // Maximum allowed current (A)
    float maxVoltage;           // Maximum allowed voltage (V)
    float minVoltage;           // Minimum allowed voltage (V)

    WeldingParams()
        : current(180.0f),
          voltage(22.0f),
          wireSpeed(8.0f),
          travelSpeed(10.0f),
          hotStartCurrent(200.0f),
          hotStartVoltage(24.0f),
          hotStartWireSpeed(10.0f),
          craterFillCurrent(120.0f),
          craterFillVoltage(18.0f),
          craterFillWireSpeed(5.0f),
          maxCurrent(400.0f),
          maxVoltage(40.0f),
          minVoltage(10.0f) {}
};

/**
 * Gas parameters
 */
struct GasParams {
    GasType type;
    float flowRate;             // Flow rate (L/min)
    float minFlowRate;          // Minimum acceptable (L/min)

    GasParams()
        : type(GasType::ARGON_CO2),
          flowRate(15.0f),
          minFlowRate(5.0f) {}
};

/**
 * Wire parameters
 */
struct WireParams {
    WireType type;
    float diameter;             // Wire diameter (mm)
    std::string material;       // e.g., "ER70S-6"

    WireParams()
        : type(WireType::SOLID),
          diameter(1.0f),
          material("ER70S-6") {}
};

/**
 * Complete welding job configuration
 */
struct WeldingJob {
    std::string name;
    WeldingProcess process;
    TransferMode transferMode;

    WeldingParams params;
    WeldingTimings timings;
    GasParams gas;
    WireParams wire;

    // Synergic control (power source calculates params from wire speed)
    bool synergicMode;
    uint16_t synergicProgram;   // Power source program number

    WeldingJob()
        : name("Default"),
          process(WeldingProcess::MIG_MAG),
          transferMode(TransferMode::SHORT_ARC),
          synergicMode(true),
          synergicProgram(1) {}
};

// ============================================================================
// Real-time Welding Data
// ============================================================================

/**
 * Current welding measurements (feedback from power source)
 */
struct WeldingFeedback {
    float actualCurrent;        // Measured current (A)
    float actualVoltage;        // Measured voltage (V)
    float actualWireSpeed;      // Measured wire speed (m/min)
    float actualPower;          // Calculated power (kW)

    float arcLength;            // Estimated arc length (mm)
    float heatInput;            // Heat input (kJ/mm)

    bool arcPresent;            // Arc detected
    bool gasFlowOk;             // Gas flow detected
    bool wireFeeding;           // Wire is feeding

    uint32_t weldTime;          // Time welding (ms)
    float weldDistance;         // Distance welded (mm)
    float wireConsumed;         // Wire consumed (m)

    WeldingFeedback()
        : actualCurrent(0), actualVoltage(0), actualWireSpeed(0), actualPower(0),
          arcLength(0), heatInput(0),
          arcPresent(false), gasFlowOk(false), wireFeeding(false),
          weldTime(0), weldDistance(0), wireConsumed(0) {}
};

/**
 * Complete welding status
 */
struct WeldingStatus {
    WeldingState state;
    WeldingFault fault;
    WeldingFeedback feedback;

    bool isReady;               // Ready to weld
    bool isWelding;             // Currently welding
    bool hasFault;              // Fault present

    uint32_t stateTime;         // Time in current state (ms)
    uint8_t arcRetries;         // Arc start attempts

    std::string statusMessage;

    WeldingStatus()
        : state(WeldingState::IDLE),
          fault(WeldingFault::NONE),
          isReady(false),
          isWelding(false),
          hasFault(false),
          stateTime(0),
          arcRetries(0) {}
};

// ============================================================================
// I/O Mapping
// ============================================================================

/**
 * Digital outputs to welding equipment
 */
struct WeldingOutputs {
    bool gasValve;              // Gas solenoid valve
    bool wireFeed;              // Wire feed enable
    bool arcEnable;             // Arc/power enable
    bool torchTrigger;          // Torch trigger signal
    bool robotReady;            // Robot ready signal to power source

    WeldingOutputs()
        : gasValve(false), wireFeed(false), arcEnable(false),
          torchTrigger(false), robotReady(false) {}
};

/**
 * Digital inputs from welding equipment
 */
struct WeldingInputs {
    bool arcDetect;             // Arc established
    bool gasFlowOk;             // Gas flow switch
    bool wireOk;                // Wire sensor
    bool powerSourceReady;      // Power source ready
    bool powerSourceFault;      // Power source fault
    bool currentFlowing;        // Current detected
    bool wireStick;             // Wire stuck

    WeldingInputs()
        : arcDetect(false), gasFlowOk(false), wireOk(false),
          powerSourceReady(false), powerSourceFault(false),
          currentFlowing(false), wireStick(false) {}
};

} // namespace welding
} // namespace robotics
```

---

## Step 2: Create Welding State Machine

### 2.1 Create WeldingStateMachine.hpp

**File:** `src/cpp/include/welding/WeldingStateMachine.hpp`

```cpp
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
    void handleError(WeldingEvent event);
    void handleEmergencyStop(WeldingEvent event);

    void updateOutputs();
    void setFault(WeldingFault fault);
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
        setFault(WeldingFault::POWER_SUPPLY_FAULT);
    }

    if (inputs.wireStick && state_ == WeldingState::WELDING) {
        setFault(WeldingFault::STICK_WIRE);
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
    if (state_ == WeldingState::ERROR) {
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
        case WeldingState::ERROR:
            handleError(event);
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
                    setFault(WeldingFault::ARC_FAIL);
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
                setFault(WeldingFault::ARC_LOST);
            }
            // Check for gas loss
            if (!inputs_.gasFlowOk && stateTimer_ > 500) {
                setFault(WeldingFault::NO_GAS);
            }
            break;

        default:
            break;
    }
}

inline void WeldingStateMachine::handleIdle(WeldingEvent event) {
    if (event == WeldingEvent::START_WELD) {
        if (!checkSafetyInterlocks()) {
            setFault(WeldingFault::SAFETY_FAULT);
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
        setFault(WeldingFault::ARC_LOST);
    }
}

inline void WeldingStateMachine::handleCraterFill(WeldingEvent event) {
    // Timer handled in update()
}

inline void WeldingStateMachine::handleArcEnd(WeldingEvent event) {
    // Timer handled in update()
}

inline void WeldingStateMachine::handleBurnBack(WeldingEvent event) {
    // Timer handled in update()
}

inline void WeldingStateMachine::handlePostFlow(WeldingEvent event) {
    // Timer handled in update()
}

inline void WeldingStateMachine::handleError(WeldingEvent event) {
    if (event == WeldingEvent::RESET) {
        clearFault();
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
        case WeldingState::ERROR:
        case WeldingState::EMERGENCY_STOP:
            outputs_ = WeldingOutputs();  // All off
            break;
    }
}

inline void WeldingStateMachine::exitState(WeldingState oldState) {
    // Clean up state if needed
}

inline void WeldingStateMachine::updateOutputs() {
    if (outputChangeCallback_) {
        outputChangeCallback_(outputs_);
    }
}

inline void WeldingStateMachine::setFault(WeldingFault fault) {
    fault_ = fault;
    transitionTo(WeldingState::ERROR);

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
```

---

## Step 3: Create Welding Controller

### 3.1 Create WeldingController.hpp

**File:** `src/cpp/include/welding/WeldingController.hpp`

```cpp
#pragma once

#include "WeldingStateMachine.hpp"
#include "trajectory/TrajectoryExecutor.hpp"
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
            stateMachine_.update(elapsed);
            lastUpdate = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

inline void WeldingController::onStateChange(
    WeldingState oldState, WeldingState newState) {

    // Coordinate with motion system
    if (newState == WeldingState::WELDING && motionStartCallback_) {
        motionStartCallback_();  // Start robot motion
    }

    if ((newState == WeldingState::CRATER_FILL ||
         newState == WeldingState::ERROR ||
         newState == WeldingState::EMERGENCY_STOP) && motionStopCallback_) {
        motionStopCallback_();  // Stop robot motion
    }
}

inline void WeldingController::onFault(WeldingFault fault) {
    // Stop motion on fault
    if (motionStopCallback_) {
        motionStopCallback_();
    }
}

} // namespace welding
} // namespace robotics
```

---

## Step 4: Create I/O Interface

### 4.1 Create WeldingIO.hpp

**File:** `src/cpp/include/welding/WeldingIO.hpp`

```cpp
#pragma once

#include "WeldingTypes.hpp"
#include <functional>
#include <cstdint>

namespace robotics {
namespace welding {

/**
 * Interface for welding I/O hardware
 */
class IWeldingIO {
public:
    virtual ~IWeldingIO() = default;

    // ========================================================================
    // Outputs
    // ========================================================================

    virtual void setGasValve(bool on) = 0;
    virtual void setWireFeed(bool on) = 0;
    virtual void setArcEnable(bool on) = 0;
    virtual void setTorchTrigger(bool on) = 0;
    virtual void setRobotReady(bool ready) = 0;

    virtual void setAllOutputs(const WeldingOutputs& outputs) = 0;

    // ========================================================================
    // Inputs
    // ========================================================================

    virtual WeldingInputs readInputs() = 0;

    virtual bool getArcDetect() = 0;
    virtual bool getGasFlowOk() = 0;
    virtual bool getWireOk() = 0;
    virtual bool getPowerSourceReady() = 0;
    virtual bool getPowerSourceFault() = 0;

    // ========================================================================
    // Analog
    // ========================================================================

    virtual float readCurrent() = 0;
    virtual float readVoltage() = 0;
    virtual float readWireSpeed() = 0;

    virtual void setCurrentCommand(float current) = 0;
    virtual void setVoltageCommand(float voltage) = 0;
    virtual void setWireSpeedCommand(float speed) = 0;
};

/**
 * Simulated welding I/O for testing
 */
class SimulatedWeldingIO : public IWeldingIO {
public:
    SimulatedWeldingIO();

    // Outputs
    void setGasValve(bool on) override;
    void setWireFeed(bool on) override;
    void setArcEnable(bool on) override;
    void setTorchTrigger(bool on) override;
    void setRobotReady(bool ready) override;
    void setAllOutputs(const WeldingOutputs& outputs) override;

    // Inputs
    WeldingInputs readInputs() override;
    bool getArcDetect() override;
    bool getGasFlowOk() override;
    bool getWireOk() override;
    bool getPowerSourceReady() override;
    bool getPowerSourceFault() override;

    // Analog
    float readCurrent() override;
    float readVoltage() override;
    float readWireSpeed() override;
    void setCurrentCommand(float current) override;
    void setVoltageCommand(float voltage) override;
    void setWireSpeedCommand(float speed) override;

    // Simulation control
    void setSimulatedInputs(const WeldingInputs& inputs) { simInputs_ = inputs; }
    void setSimulatedFeedback(float current, float voltage, float wireSpeed);

private:
    WeldingOutputs outputs_;
    WeldingInputs simInputs_;

    float commandedCurrent_;
    float commandedVoltage_;
    float commandedWireSpeed_;

    float simulatedCurrent_;
    float simulatedVoltage_;
    float simulatedWireSpeed_;
};

// Implementation
inline SimulatedWeldingIO::SimulatedWeldingIO()
    : commandedCurrent_(0), commandedVoltage_(0), commandedWireSpeed_(0),
      simulatedCurrent_(0), simulatedVoltage_(0), simulatedWireSpeed_(0) {

    simInputs_.powerSourceReady = true;
    simInputs_.gasFlowOk = true;
    simInputs_.wireOk = true;
}

inline void SimulatedWeldingIO::setGasValve(bool on) { outputs_.gasValve = on; }
inline void SimulatedWeldingIO::setWireFeed(bool on) { outputs_.wireFeed = on; }
inline void SimulatedWeldingIO::setArcEnable(bool on) { outputs_.arcEnable = on; }
inline void SimulatedWeldingIO::setTorchTrigger(bool on) { outputs_.torchTrigger = on; }
inline void SimulatedWeldingIO::setRobotReady(bool ready) { outputs_.robotReady = ready; }

inline void SimulatedWeldingIO::setAllOutputs(const WeldingOutputs& outputs) {
    outputs_ = outputs;
}

inline WeldingInputs SimulatedWeldingIO::readInputs() {
    // Simulate arc detection when arc is enabled and wire is feeding
    if (outputs_.arcEnable && outputs_.wireFeed && outputs_.torchTrigger) {
        simInputs_.arcDetect = true;
        simInputs_.currentFlowing = true;
    } else {
        simInputs_.arcDetect = false;
        simInputs_.currentFlowing = false;
    }

    return simInputs_;
}

inline bool SimulatedWeldingIO::getArcDetect() { return simInputs_.arcDetect; }
inline bool SimulatedWeldingIO::getGasFlowOk() { return simInputs_.gasFlowOk; }
inline bool SimulatedWeldingIO::getWireOk() { return simInputs_.wireOk; }
inline bool SimulatedWeldingIO::getPowerSourceReady() { return simInputs_.powerSourceReady; }
inline bool SimulatedWeldingIO::getPowerSourceFault() { return simInputs_.powerSourceFault; }

inline float SimulatedWeldingIO::readCurrent() {
    if (outputs_.arcEnable && outputs_.torchTrigger) {
        return commandedCurrent_ * 0.95f;  // Simulate 95% of commanded
    }
    return 0;
}

inline float SimulatedWeldingIO::readVoltage() {
    if (outputs_.arcEnable && outputs_.torchTrigger) {
        return commandedVoltage_ * 0.98f;
    }
    return 0;
}

inline float SimulatedWeldingIO::readWireSpeed() {
    if (outputs_.wireFeed) {
        return commandedWireSpeed_;
    }
    return 0;
}

inline void SimulatedWeldingIO::setCurrentCommand(float current) {
    commandedCurrent_ = current;
}

inline void SimulatedWeldingIO::setVoltageCommand(float voltage) {
    commandedVoltage_ = voltage;
}

inline void SimulatedWeldingIO::setWireSpeedCommand(float speed) {
    commandedWireSpeed_ = speed;
}

inline void SimulatedWeldingIO::setSimulatedFeedback(
    float current, float voltage, float wireSpeed) {
    simulatedCurrent_ = current;
    simulatedVoltage_ = voltage;
    simulatedWireSpeed_ = wireSpeed;
}

} // namespace welding
} // namespace robotics
```

---

## Step 5: Create Unit Tests

### 5.1 Create test_welding.cpp

**File:** `src/cpp/tests/test_welding.cpp`

```cpp
#include <gtest/gtest.h>
#include "welding/WeldingTypes.hpp"
#include "welding/WeldingStateMachine.hpp"
#include "welding/WeldingController.hpp"
#include "welding/WeldingIO.hpp"

using namespace robotics::welding;

// ============================================================================
// Welding Types Tests
// ============================================================================

TEST(WeldingTypesTest, DefaultTimings) {
    WeldingTimings timings;

    EXPECT_EQ(timings.preFlowTime, 300);
    EXPECT_EQ(timings.postFlowTime, 500);
    EXPECT_EQ(timings.arcRetryCount, 3);
}

TEST(WeldingTypesTest, DefaultParams) {
    WeldingParams params;

    EXPECT_GT(params.current, 0);
    EXPECT_GT(params.voltage, 0);
    EXPECT_GT(params.wireSpeed, 0);
}

TEST(WeldingTypesTest, DefaultJob) {
    WeldingJob job;

    EXPECT_EQ(job.process, WeldingProcess::MIG_MAG);
    EXPECT_EQ(job.transferMode, TransferMode::SHORT_ARC);
    EXPECT_TRUE(job.synergicMode);
}

// ============================================================================
// State Machine Tests
// ============================================================================

class WeldingStateMachineTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Set ready inputs
        inputs_.powerSourceReady = true;
        inputs_.gasFlowOk = true;
        inputs_.wireOk = true;

        sm_.updateInputs(inputs_);
    }

    WeldingStateMachine sm_;
    WeldingInputs inputs_;
};

TEST_F(WeldingStateMachineTest, InitialStateIsIdle) {
    EXPECT_EQ(sm_.getState(), WeldingState::IDLE);
}

TEST_F(WeldingStateMachineTest, StartWeld_TransitionsToPreFlow) {
    sm_.processEvent(WeldingEvent::START_WELD);

    EXPECT_EQ(sm_.getState(), WeldingState::PRE_FLOW);
}

TEST_F(WeldingStateMachineTest, PreFlow_GasValveOn) {
    sm_.processEvent(WeldingEvent::START_WELD);

    auto outputs = sm_.getOutputs();
    EXPECT_TRUE(outputs.gasValve);
}

TEST_F(WeldingStateMachineTest, PreFlow_TimerTransitionsToArcStart) {
    WeldingJob job;
    job.timings.preFlowTime = 100;
    sm_.setJob(job);

    sm_.processEvent(WeldingEvent::START_WELD);
    EXPECT_EQ(sm_.getState(), WeldingState::PRE_FLOW);

    // Simulate time passing
    sm_.update(50);
    EXPECT_EQ(sm_.getState(), WeldingState::PRE_FLOW);

    sm_.update(60);  // Total 110ms > 100ms
    EXPECT_EQ(sm_.getState(), WeldingState::ARC_START);
}

TEST_F(WeldingStateMachineTest, ArcStart_WireFeedAndArcEnabled) {
    WeldingJob job;
    job.timings.preFlowTime = 0;
    sm_.setJob(job);

    sm_.processEvent(WeldingEvent::START_WELD);
    sm_.update(1);

    auto outputs = sm_.getOutputs();
    EXPECT_TRUE(outputs.gasValve);
    EXPECT_TRUE(outputs.wireFeed);
    EXPECT_TRUE(outputs.arcEnable);
    EXPECT_TRUE(outputs.torchTrigger);
}

TEST_F(WeldingStateMachineTest, ArcDetected_TransitionsToWelding) {
    WeldingJob job;
    job.timings.preFlowTime = 0;
    sm_.setJob(job);

    sm_.processEvent(WeldingEvent::START_WELD);
    sm_.update(1);

    // Simulate arc established
    inputs_.arcDetect = true;
    sm_.updateInputs(inputs_);

    WeldingFeedback feedback;
    feedback.arcPresent = true;
    feedback.actualCurrent = 180;
    feedback.actualVoltage = 22;
    sm_.updateFeedback(feedback);

    sm_.update(1);

    EXPECT_EQ(sm_.getState(), WeldingState::WELDING);
}

TEST_F(WeldingStateMachineTest, StopWeld_TransitionsToCraterFill) {
    WeldingJob job;
    job.timings.preFlowTime = 0;
    sm_.setJob(job);

    sm_.processEvent(WeldingEvent::START_WELD);
    sm_.update(1);

    // Go to welding state
    inputs_.arcDetect = true;
    sm_.updateInputs(inputs_);
    sm_.update(1);

    EXPECT_EQ(sm_.getState(), WeldingState::WELDING);

    // Stop weld
    sm_.processEvent(WeldingEvent::STOP_WELD);
    EXPECT_EQ(sm_.getState(), WeldingState::CRATER_FILL);
}

TEST_F(WeldingStateMachineTest, FullWeldSequence) {
    WeldingJob job;
    job.timings.preFlowTime = 10;
    job.timings.craterFillTime = 10;
    job.timings.burnBackTime = 10;
    job.timings.postFlowTime = 10;
    sm_.setJob(job);

    // Start
    sm_.processEvent(WeldingEvent::START_WELD);
    EXPECT_EQ(sm_.getState(), WeldingState::PRE_FLOW);

    // Pre-flow complete
    sm_.update(20);
    EXPECT_EQ(sm_.getState(), WeldingState::ARC_START);

    // Arc established
    inputs_.arcDetect = true;
    sm_.updateInputs(inputs_);
    sm_.update(1);
    EXPECT_EQ(sm_.getState(), WeldingState::WELDING);

    // Stop weld
    sm_.processEvent(WeldingEvent::STOP_WELD);
    EXPECT_EQ(sm_.getState(), WeldingState::CRATER_FILL);

    // Crater fill complete
    sm_.update(20);
    EXPECT_EQ(sm_.getState(), WeldingState::ARC_END);

    // Arc end -> burn back
    sm_.update(250);
    EXPECT_EQ(sm_.getState(), WeldingState::BURN_BACK);

    // Burn back complete
    sm_.update(20);
    EXPECT_EQ(sm_.getState(), WeldingState::POST_FLOW);

    // Post flow complete
    sm_.update(20);
    EXPECT_EQ(sm_.getState(), WeldingState::IDLE);
}

TEST_F(WeldingStateMachineTest, EmergencyStop_FromAnyState) {
    sm_.processEvent(WeldingEvent::START_WELD);
    sm_.update(1);

    sm_.processEvent(WeldingEvent::EMERGENCY_STOP);

    EXPECT_EQ(sm_.getState(), WeldingState::EMERGENCY_STOP);

    // All outputs should be off
    auto outputs = sm_.getOutputs();
    EXPECT_FALSE(outputs.gasValve);
    EXPECT_FALSE(outputs.wireFeed);
    EXPECT_FALSE(outputs.arcEnable);
}

TEST_F(WeldingStateMachineTest, Abort_StopsImmediately) {
    sm_.processEvent(WeldingEvent::START_WELD);
    sm_.update(100);

    sm_.processEvent(WeldingEvent::ABORT);

    EXPECT_EQ(sm_.getState(), WeldingState::IDLE);
}

TEST_F(WeldingStateMachineTest, ArcFail_AfterRetries) {
    WeldingJob job;
    job.timings.preFlowTime = 0;
    job.timings.arcStartTime = 50;
    job.timings.arcRetryCount = 2;
    sm_.setJob(job);

    sm_.processEvent(WeldingEvent::START_WELD);
    sm_.update(1);

    // Arc never establishes
    sm_.update(60);  // First attempt
    sm_.update(60);  // Second attempt
    sm_.update(60);  // Third attempt (exceeds retries)

    EXPECT_EQ(sm_.getState(), WeldingState::ERROR);
    EXPECT_EQ(sm_.getFault(), WeldingFault::ARC_FAIL);
}

TEST_F(WeldingStateMachineTest, Reset_ClearsFault) {
    // Put into error state
    sm_.processEvent(WeldingEvent::FAULT);

    EXPECT_EQ(sm_.getState(), WeldingState::ERROR);

    sm_.processEvent(WeldingEvent::RESET);

    EXPECT_EQ(sm_.getState(), WeldingState::IDLE);
    EXPECT_FALSE(sm_.hasFault());
}

// ============================================================================
// Welding Controller Tests
// ============================================================================

TEST(WeldingControllerTest, Initialize) {
    WeldingController controller;

    bool result = controller.initialize();

    EXPECT_TRUE(result);
}

TEST(WeldingControllerTest, StartWeld) {
    WeldingController controller;
    controller.initialize();

    WeldingJob job;
    bool result = controller.startWeld(job);

    EXPECT_TRUE(result);
    EXPECT_NE(controller.getState(), WeldingState::IDLE);
}

TEST(WeldingControllerTest, StopWeld) {
    WeldingController controller;
    controller.initialize();

    WeldingJob job;
    controller.startWeld(job);

    // Let it progress
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    controller.stopWeld();

    // Should be stopping
    auto state = controller.getState();
    EXPECT_TRUE(state == WeldingState::CRATER_FILL ||
                state == WeldingState::POST_FLOW ||
                state == WeldingState::IDLE);
}

TEST(WeldingControllerTest, EmergencyStop) {
    WeldingController controller;
    controller.initialize();

    WeldingJob job;
    controller.startWeld(job);

    controller.emergencyStop();

    EXPECT_EQ(controller.getState(), WeldingState::EMERGENCY_STOP);
}

// ============================================================================
// Simulated I/O Tests
// ============================================================================

TEST(SimulatedIOTest, DefaultReady) {
    SimulatedWeldingIO io;

    auto inputs = io.readInputs();

    EXPECT_TRUE(inputs.powerSourceReady);
    EXPECT_TRUE(inputs.gasFlowOk);
    EXPECT_TRUE(inputs.wireOk);
}

TEST(SimulatedIOTest, SetOutputs) {
    SimulatedWeldingIO io;

    WeldingOutputs outputs;
    outputs.gasValve = true;
    outputs.arcEnable = true;

    io.setAllOutputs(outputs);

    // Outputs should be set
    // (we'd need a getter to verify, but the internal state is set)
}

TEST(SimulatedIOTest, ArcSimulation) {
    SimulatedWeldingIO io;

    io.setCurrentCommand(180);
    io.setVoltageCommand(22);

    // Before arc enabled
    EXPECT_EQ(io.readCurrent(), 0);

    // Enable arc
    WeldingOutputs outputs;
    outputs.arcEnable = true;
    outputs.torchTrigger = true;
    io.setAllOutputs(outputs);

    // Should read simulated current
    EXPECT_GT(io.readCurrent(), 0);
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
```

---

## Step 6: Create IPC Message Types

### 6.1 Create WeldingPayloads.hpp

**File:** `src/cpp/include/ipc/WeldingPayloads.hpp`

```cpp
#pragma once

#include <nlohmann/json.hpp>
#include <string>

namespace robotics {
namespace ipc {

// ============================================================================
// Welding Job
// ============================================================================

struct WeldingJobData {
    std::string name;
    int process;
    int transferMode;

    // Parameters
    float current;
    float voltage;
    float wireSpeed;
    float travelSpeed;

    // Timings
    uint32_t preFlowTime;
    uint32_t postFlowTime;
    uint32_t craterFillTime;

    bool synergicMode;
    int synergicProgram;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(WeldingJobData,
        name, process, transferMode,
        current, voltage, wireSpeed, travelSpeed,
        preFlowTime, postFlowTime, craterFillTime,
        synergicMode, synergicProgram)
};

// ============================================================================
// Welding Control
// ============================================================================

struct WeldingControlRequest {
    std::string action;  // "START", "STOP", "ABORT", "RESET"
    WeldingJobData job;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(WeldingControlRequest, action, job)
};

struct WeldingControlResponse {
    bool success;
    std::string state;
    std::string error;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(WeldingControlResponse, success, state, error)
};

// ============================================================================
// Welding Status
// ============================================================================

struct WeldingStatusResponse {
    std::string state;
    std::string fault;
    bool isWelding;
    bool hasFault;
    bool isReady;

    // Feedback
    float actualCurrent;
    float actualVoltage;
    float actualWireSpeed;
    bool arcPresent;
    bool gasFlowOk;

    uint32_t weldTime;
    float weldDistance;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(WeldingStatusResponse,
        state, fault, isWelding, hasFault, isReady,
        actualCurrent, actualVoltage, actualWireSpeed,
        arcPresent, gasFlowOk, weldTime, weldDistance)
};

// ============================================================================
// Parameter Adjustment
// ============================================================================

struct WeldingAdjustRequest {
    std::string parameter;  // "current", "voltage", "wireSpeed", "travelSpeed"
    float value;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(WeldingAdjustRequest, parameter, value)
};

struct WeldingAdjustResponse {
    bool success;
    float newValue;
    std::string error;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(WeldingAdjustResponse, success, newValue, error)
};

} // namespace ipc
} // namespace robotics
```

---

## Step 7: Create C# Welding Client

### 7.1 Create WeldingPayloads.cs

**File:** `src/csharp/RobotController.Core/IPC/WeldingPayloads.cs`

```csharp
using System.Text.Json.Serialization;

namespace RobotController.Core.IPC;

// ============================================================================
// Enums
// ============================================================================

public enum WeldingState
{
    Idle = 0,
    PreFlow = 1,
    ArcStart = 2,
    Welding = 3,
    CraterFill = 4,
    ArcEnd = 5,
    BurnBack = 6,
    PostFlow = 7,
    Error = 8,
    EmergencyStop = 9
}

public enum WeldingFault
{
    None = 0,
    NoGas = 1,
    NoWire = 2,
    ArcFail = 3,
    ArcLost = 4,
    OverCurrent = 5,
    OverVoltage = 6,
    StickWire = 7,
    PowerSupplyFault = 8,
    CommunicationFault = 9,
    SafetyFault = 10,
    ThermalFault = 11
}

public enum WeldingProcess
{
    MigMag = 0,
    Tig = 1,
    Spot = 2,
    Plasma = 3,
    Laser = 4
}

// ============================================================================
// Welding Job
// ============================================================================

public record WeldingJobData
{
    [JsonPropertyName("name")]
    public string Name { get; init; } = "Default";

    [JsonPropertyName("process")]
    public int Process { get; init; }

    [JsonPropertyName("transferMode")]
    public int TransferMode { get; init; }

    [JsonPropertyName("current")]
    public float Current { get; init; } = 180;

    [JsonPropertyName("voltage")]
    public float Voltage { get; init; } = 22;

    [JsonPropertyName("wireSpeed")]
    public float WireSpeed { get; init; } = 8;

    [JsonPropertyName("travelSpeed")]
    public float TravelSpeed { get; init; } = 10;

    [JsonPropertyName("preFlowTime")]
    public uint PreFlowTime { get; init; } = 300;

    [JsonPropertyName("postFlowTime")]
    public uint PostFlowTime { get; init; } = 500;

    [JsonPropertyName("craterFillTime")]
    public uint CraterFillTime { get; init; } = 500;

    [JsonPropertyName("synergicMode")]
    public bool SynergicMode { get; init; } = true;

    [JsonPropertyName("synergicProgram")]
    public int SynergicProgram { get; init; } = 1;
}

// ============================================================================
// Welding Control
// ============================================================================

public record WeldingControlRequest
{
    [JsonPropertyName("action")]
    public string Action { get; init; } = "";

    [JsonPropertyName("job")]
    public WeldingJobData? Job { get; init; }
}

public record WeldingControlResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; init; }

    [JsonPropertyName("state")]
    public string State { get; init; } = "";

    [JsonPropertyName("error")]
    public string Error { get; init; } = "";
}

// ============================================================================
// Welding Status
// ============================================================================

public record WeldingStatusResponse
{
    [JsonPropertyName("state")]
    public string State { get; init; } = "Idle";

    [JsonPropertyName("fault")]
    public string Fault { get; init; } = "None";

    [JsonPropertyName("isWelding")]
    public bool IsWelding { get; init; }

    [JsonPropertyName("hasFault")]
    public bool HasFault { get; init; }

    [JsonPropertyName("isReady")]
    public bool IsReady { get; init; }

    [JsonPropertyName("actualCurrent")]
    public float ActualCurrent { get; init; }

    [JsonPropertyName("actualVoltage")]
    public float ActualVoltage { get; init; }

    [JsonPropertyName("actualWireSpeed")]
    public float ActualWireSpeed { get; init; }

    [JsonPropertyName("arcPresent")]
    public bool ArcPresent { get; init; }

    [JsonPropertyName("gasFlowOk")]
    public bool GasFlowOk { get; init; }

    [JsonPropertyName("weldTime")]
    public uint WeldTime { get; init; }

    [JsonPropertyName("weldDistance")]
    public float WeldDistance { get; init; }
}

// ============================================================================
// Parameter Adjustment
// ============================================================================

public record WeldingAdjustRequest
{
    [JsonPropertyName("parameter")]
    public string Parameter { get; init; } = "";

    [JsonPropertyName("value")]
    public float Value { get; init; }
}

public record WeldingAdjustResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; init; }

    [JsonPropertyName("newValue")]
    public float NewValue { get; init; }

    [JsonPropertyName("error")]
    public string Error { get; init; } = "";
}
```

### 7.2 Create IWeldingClientService.cs

**File:** `src/csharp/RobotController.Core/Services/IWeldingClientService.cs`

```csharp
using RobotController.Core.IPC;

namespace RobotController.Core.Services;

/// <summary>
/// Client service for welding control
/// </summary>
public interface IWeldingClientService
{
    // Control
    Task<WeldingControlResponse> StartWeldAsync(WeldingJobData job, CancellationToken ct = default);
    Task<WeldingControlResponse> StopWeldAsync(CancellationToken ct = default);
    Task<WeldingControlResponse> AbortWeldAsync(CancellationToken ct = default);
    Task<WeldingControlResponse> ResetAsync(CancellationToken ct = default);

    // Status
    Task<WeldingStatusResponse> GetStatusAsync(CancellationToken ct = default);
    WeldingStatusResponse CachedStatus { get; }

    // Parameters
    Task<WeldingAdjustResponse> AdjustCurrentAsync(float current, CancellationToken ct = default);
    Task<WeldingAdjustResponse> AdjustVoltageAsync(float voltage, CancellationToken ct = default);
    Task<WeldingAdjustResponse> AdjustWireSpeedAsync(float speed, CancellationToken ct = default);
    Task<WeldingAdjustResponse> AdjustTravelSpeedAsync(float speed, CancellationToken ct = default);

    // Events
    event EventHandler<WeldingStatusResponse>? StatusUpdated;
    event EventHandler<WeldingFault>? FaultOccurred;
    event EventHandler<WeldingState>? StateChanged;
}
```

---

## Step 8: Update CMakeLists.txt

**Add welding module:**
```cmake
# Include welding
target_include_directories(RobotCore
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/welding
)

# Welding tests
add_executable(test_welding tests/test_welding.cpp)
target_link_libraries(test_welding
    PRIVATE
    GTest::gtest
    GTest::gtest_main
)
target_include_directories(test_welding PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME WeldingTests COMMAND test_welding)
```

---

## Step 9: Validation

### 9.1 Build and Test

```powershell
cd E:\DEV_CONTEXT_PROJECTs\Robot_controller
cmake --build build --config Debug
.\build\Debug\test_welding.exe
```

**Expected:**
```
[==========] Running 20 tests from 4 test suites.
...
[  PASSED  ] 20 tests.
```

---

## Completion Checklist

- [ ] WeldingTypes.hpp created (states, params, feedback)
- [ ] WeldingStateMachine.hpp created with full sequence
- [ ] WeldingController.hpp created
- [ ] WeldingIO.hpp created with simulation
- [ ] test_welding.cpp created with 20+ tests
- [ ] WeldingPayloads (C++ & C#) created
- [ ] IWeldingClientService.cs created
- [ ] CMakeLists.txt updated
- [ ] All tests pass

---

## Git Commit

```powershell
git add -A
git commit -m "IMPL_P3_01: Add welding sequencer

- Create WeldingTypes with process enums and parameters
- Implement WeldingStateMachine for weld sequence control
- Add full welding sequence: pre-flow, arc start, welding,
  crater fill, arc end, burn-back, post-flow
- Create WeldingController for high-level control
- Add SimulatedWeldingIO for testing
- Create IPC message types for welding
- Add C# WeldingClientService
- Create 20 unit tests for welding

Co-Authored-By: Claude <noreply@anthropic.com>"
```
