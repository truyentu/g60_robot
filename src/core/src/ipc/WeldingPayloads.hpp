#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <cstdint>

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

    WeldingJobData()
        : name("Default"),
          process(0),
          transferMode(0),
          current(180.0f),
          voltage(22.0f),
          wireSpeed(8.0f),
          travelSpeed(10.0f),
          preFlowTime(300),
          postFlowTime(500),
          craterFillTime(500),
          synergicMode(true),
          synergicProgram(1) {}

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

    WeldingControlRequest() : action("") {}

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(WeldingControlRequest, action, job)
};

struct WeldingControlResponse {
    bool success;
    std::string state;
    std::string error;

    WeldingControlResponse()
        : success(false), state(""), error("") {}

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

    WeldingStatusResponse()
        : state("IDLE"),
          fault("NONE"),
          isWelding(false),
          hasFault(false),
          isReady(false),
          actualCurrent(0),
          actualVoltage(0),
          actualWireSpeed(0),
          arcPresent(false),
          gasFlowOk(false),
          weldTime(0),
          weldDistance(0) {}

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

    WeldingAdjustRequest() : parameter(""), value(0) {}

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(WeldingAdjustRequest, parameter, value)
};

struct WeldingAdjustResponse {
    bool success;
    float newValue;
    std::string error;

    WeldingAdjustResponse()
        : success(false), newValue(0), error("") {}

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(WeldingAdjustResponse, success, newValue, error)
};

} // namespace ipc
} // namespace robotics
