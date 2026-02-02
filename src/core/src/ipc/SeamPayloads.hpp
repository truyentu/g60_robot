#pragma once

#include <string>
#include <vector>
#include <array>
#include <cstdint>
#include <msgpack.hpp>

namespace robot_controller::ipc {

// ============================================================================
// Seam Detection Messages
// ============================================================================

namespace SeamMessages {

// --- Request Types ---

struct StartSeamTrackingRequest {
    std::string jointType;  // "vgroove", "lap", "fillet", "auto"
    float roiXMin{-50.0f};
    float roiXMax{50.0f};
    float roiZMin{0.0f};
    float roiZMax{100.0f};

    MSGPACK_DEFINE(jointType, roiXMin, roiXMax, roiZMin, roiZMax)
};

struct SetTrackingConfigRequest {
    bool enableMedianFilter{true};
    int medianFilterSize{5};
    bool enableOutlierRemoval{true};
    float outlierThreshold{3.0f};
    int ransacIterations{100};
    float ransacThreshold{0.5f};
    float systemLatency{0.030f};
    float travelSpeed{10.0f};

    MSGPACK_DEFINE(enableMedianFilter, medianFilterSize, enableOutlierRemoval,
                   outlierThreshold, ransacIterations, ransacThreshold,
                   systemLatency, travelSpeed)
};

struct SetNominalPathRequest {
    std::vector<float> pointsX;
    std::vector<float> pointsY;
    std::vector<float> pointsZ;

    MSGPACK_DEFINE(pointsX, pointsY, pointsZ)
};

// --- Response Types ---

struct SeamFeatureResponse {
    bool success{false};
    std::string jointType;
    float rootX{0.0f};
    float rootZ{0.0f};
    float gapWidth{0.0f};
    float leftAngle{0.0f};
    float rightAngle{0.0f};
    float depth{0.0f};
    float confidence{0.0f};
    uint64_t timestamp{0};
    std::string errorMessage;

    MSGPACK_DEFINE(success, jointType, rootX, rootZ, gapWidth,
                   leftAngle, rightAngle, depth, confidence,
                   timestamp, errorMessage)
};

struct SeamPointResponse {
    bool success{false};
    float worldX{0.0f};
    float worldY{0.0f};
    float worldZ{0.0f};
    std::array<float, 3> normal;
    std::array<float, 3> tangent;
    float deviation{0.0f};
    float heightOffset{0.0f};
    uint64_t timestamp{0};
    std::string errorMessage;

    MSGPACK_DEFINE(success, worldX, worldY, worldZ, normal, tangent,
                   deviation, heightOffset, timestamp, errorMessage)
};

struct TrackingStateResponse {
    bool success{false};
    bool trackingActive{false};
    bool trackingLost{false};
    float predictedX{0.0f};
    float predictedZ{0.0f};
    float lateralCorrection{0.0f};
    float heightCorrection{0.0f};
    float trackingQuality{0.0f};
    uint64_t framesProcessed{0};
    uint64_t validDetections{0};
    float avgLatencyMs{0.0f};
    std::string errorMessage;

    MSGPACK_DEFINE(success, trackingActive, trackingLost, predictedX, predictedZ,
                   lateralCorrection, heightCorrection, trackingQuality,
                   framesProcessed, validDetections, avgLatencyMs, errorMessage)
};

struct SeamPathResponse {
    bool success{false};
    std::vector<float> pointsX;
    std::vector<float> pointsY;
    std::vector<float> pointsZ;
    std::vector<float> deviations;
    float totalLength{0.0f};
    float avgGapWidth{0.0f};
    std::string jointType;
    std::string errorMessage;

    MSGPACK_DEFINE(success, pointsX, pointsY, pointsZ, deviations,
                   totalLength, avgGapWidth, jointType, errorMessage)
};

} // namespace SeamMessages

} // namespace robot_controller::ipc
