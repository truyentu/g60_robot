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
    FAULT = 8,          // Fault state
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
    FAULT_EVENT = 6,    // Fault detected
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
