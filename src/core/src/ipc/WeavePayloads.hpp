#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace robotics {
namespace ipc {

// ============================================================================
// Weave Parameters
// ============================================================================

struct WeaveParamsData {
    int patternType;
    double amplitude;
    double wavelength;
    double frequency;
    double dwellLeft;
    double dwellRight;
    double dwellCenter;
    double phaseOffset;
    double speedAtEdge;
    double speedAtCenter;
    bool useFrequency;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(WeaveParamsData,
        patternType, amplitude, wavelength, frequency,
        dwellLeft, dwellRight, dwellCenter, phaseOffset,
        speedAtEdge, speedAtCenter, useFrequency)
};

// ============================================================================
// Weave Control
// ============================================================================

struct WeaveControlRequest {
    std::string action;  // "START", "STOP", "SET_PARAMS", "ADJUST"
    WeaveParamsData params;
    std::string adjustParam;
    double adjustValue;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(WeaveControlRequest,
        action, params, adjustParam, adjustValue)
};

struct WeaveControlResponse {
    bool success;
    std::string error;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(WeaveControlResponse, success, error)
};

// ============================================================================
// Weave Status
// ============================================================================

struct WeaveStatusResponse {
    bool enabled;
    bool active;
    int patternType;
    double currentPhase;
    double currentAmplitude;
    double lateralOffset;
    int cycleCount;
    double totalDistance;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(WeaveStatusResponse,
        enabled, active, patternType, currentPhase,
        currentAmplitude, lateralOffset, cycleCount, totalDistance)
};

// ============================================================================
// Weave Preview
// ============================================================================

struct WeavePreviewRequest {
    WeaveParamsData params;
    int numCycles;
    int pointsPerCycle;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(WeavePreviewRequest,
        params, numCycles, pointsPerCycle)
};

struct WeavePreviewResponse {
    std::vector<double> xPoints;
    std::vector<double> yPoints;
    double pathLengthRatio;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(WeavePreviewResponse,
        xPoints, yPoints, pathLengthRatio)
};

} // namespace ipc
} // namespace robotics
