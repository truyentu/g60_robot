#pragma once

#include <string>
#include <vector>
#include <array>
#include <cstdint>
#include <msgpack.hpp>

namespace robot_controller::ipc {

// ============================================================================
// Vision Sensor Messages
// ============================================================================

namespace SensorMessages {

// --- Request Types ---

struct ConnectLaserRequest {
    std::string deviceId;

    MSGPACK_DEFINE(deviceId)
};

struct ConfigureLaserRequest {
    uint32_t profileWidth{2048};
    float exposureTime{1000.0f};
    float gain{1.0f};
    std::string triggerMode;  // "freerun", "software", "encoder", "hardware"
    uint32_t encoderDivider{1};
    float intensityLow{10.0f};
    float intensityHigh{250.0f};

    MSGPACK_DEFINE(profileWidth, exposureTime, gain, triggerMode,
                   encoderDivider, intensityLow, intensityHigh)
};

struct StartScanRequest {
    uint32_t profileCount{0};  // 0 = continuous
    uint32_t timeoutMs{30000};

    MSGPACK_DEFINE(profileCount, timeoutMs)
};

struct HandEyeCalibRequest {
    std::vector<std::array<double, 6>> robotJoints;
    std::vector<std::array<double, 16>> targetPoses;

    MSGPACK_DEFINE(robotJoints, targetPoses)
};

struct GetProfileRequest {
    uint32_t timeoutMs{1000};

    MSGPACK_DEFINE(timeoutMs)
};

struct DownsampleRequest {
    std::vector<float> pointsX;
    std::vector<float> pointsY;
    std::vector<float> pointsZ;
    float voxelSize{1.0f};

    MSGPACK_DEFINE(pointsX, pointsY, pointsZ, voxelSize)
};

struct SaveCalibrationRequest {
    std::string filepath;

    MSGPACK_DEFINE(filepath)
};

struct LoadCalibrationRequest {
    std::string filepath;

    MSGPACK_DEFINE(filepath)
};

// --- Response Types ---

struct SensorInfoResponse {
    bool success{false};
    std::string deviceId;
    std::string serialNumber;
    std::string modelName;
    std::string status;
    float frameRate{0.0f};
    uint64_t framesReceived{0};
    uint64_t framesDropped{0};
    std::string errorMessage;

    MSGPACK_DEFINE(success, deviceId, serialNumber, modelName, status,
                   frameRate, framesReceived, framesDropped, errorMessage)
};

struct ProfileDataResponse {
    bool success{false};
    std::vector<float> pointsX;
    std::vector<float> pointsZ;
    std::vector<float> intensity;
    std::vector<bool> valid;
    uint64_t timestamp{0};
    uint64_t frameId{0};
    std::array<double, 6> robotJoints;
    std::string errorMessage;

    MSGPACK_DEFINE(success, pointsX, pointsZ, intensity, valid,
                   timestamp, frameId, robotJoints, errorMessage)
};

struct PointCloudResponse {
    bool success{false};
    std::vector<float> pointsX;
    std::vector<float> pointsY;
    std::vector<float> pointsZ;
    std::vector<float> intensity;
    uint32_t pointCount{0};
    std::array<float, 3> minBound;
    std::array<float, 3> maxBound;
    std::string frameId;
    std::string errorMessage;

    MSGPACK_DEFINE(success, pointsX, pointsY, pointsZ, intensity,
                   pointCount, minBound, maxBound, frameId, errorMessage)
};

struct CalibrationResponse {
    bool success{false};
    std::array<double, 16> sensorToFlange;
    std::array<double, 3> translation;
    std::array<double, 4> rotationQuat;
    double reprojectionError{0.0};
    std::string errorMessage;

    MSGPACK_DEFINE(success, sensorToFlange, translation, rotationQuat,
                   reprojectionError, errorMessage)
};

struct EnumerateResponse {
    bool success{false};
    std::vector<std::string> deviceIds;
    std::vector<std::string> serialNumbers;
    std::vector<std::string> modelNames;
    std::vector<std::string> manufacturers;
    std::vector<std::string> statuses;
    std::string errorMessage;

    MSGPACK_DEFINE(success, deviceIds, serialNumbers, modelNames,
                   manufacturers, statuses, errorMessage)
};

} // namespace SensorMessages

} // namespace robot_controller::ipc
