#pragma once

#include "../frame/FrameTypes.hpp"
#include <string>
#include <vector>
#include <array>
#include <nlohmann/json.hpp>

namespace robot_controller {
namespace ipc {

/**
 * Frame data for IPC
 */
struct FramePayload {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double rx = 0.0;
    double ry = 0.0;
    double rz = 0.0;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(FramePayload, x, y, z, rx, ry, rz)

    static FramePayload fromFrame(const frame::Frame& f) {
        return {f.x, f.y, f.z, f.rx, f.ry, f.rz};
    }

    frame::Frame toFrame() const {
        return {x, y, z, rx, ry, rz};
    }
};

/**
 * Base frame data for IPC
 */
struct BaseFramePayload {
    std::string id;
    std::string name;
    std::string description;
    FramePayload frame;
    bool isActive = false;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(BaseFramePayload, id, name, description, frame, isActive)
};

// ============================================================================
// Request/Response Payloads
// ============================================================================

/**
 * Response for GET_BASE_LIST
 */
struct GetBaseListResponse {
    std::vector<BaseFramePayload> bases;
    std::string activeBaseId;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(GetBaseListResponse, bases, activeBaseId)
};

/**
 * Request for GET_BASE
 */
struct GetBaseRequest {
    std::string baseId;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(GetBaseRequest, baseId)
};

/**
 * Response for GET_BASE
 */
struct GetBaseResponse {
    bool success = false;
    BaseFramePayload base;
    std::string error;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(GetBaseResponse, success, base, error)
};

/**
 * Request for CREATE_BASE
 */
struct CreateBaseRequest {
    BaseFramePayload base;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(CreateBaseRequest, base)
};

/**
 * Response for CREATE_BASE
 */
struct CreateBaseResponse {
    bool success = false;
    std::string error;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(CreateBaseResponse, success, error)
};

/**
 * Request for UPDATE_BASE
 */
struct UpdateBaseRequest {
    std::string baseId;
    BaseFramePayload base;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(UpdateBaseRequest, baseId, base)
};

/**
 * Response for UPDATE_BASE
 */
struct UpdateBaseResponse {
    bool success = false;
    std::string error;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(UpdateBaseResponse, success, error)
};

/**
 * Request for DELETE_BASE
 */
struct DeleteBaseRequest {
    std::string baseId;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(DeleteBaseRequest, baseId)
};

/**
 * Response for DELETE_BASE
 */
struct DeleteBaseResponse {
    bool success = false;
    std::string error;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(DeleteBaseResponse, success, error)
};

/**
 * Request for SELECT_BASE
 */
struct SelectBaseRequest {
    std::string baseId;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(SelectBaseRequest, baseId)
};

/**
 * Response for SELECT_BASE
 */
struct SelectBaseResponse {
    bool success = false;
    std::string error;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(SelectBaseResponse, success, error)
};

/**
 * Response for GET_ACTIVE_BASE
 */
struct GetActiveBaseResponse {
    bool success = false;
    BaseFramePayload base;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(GetActiveBaseResponse, success, base)
};

// ============================================================================
// Calibration Payloads
// ============================================================================

/**
 * Request for START_BASE_CALIBRATION
 */
struct StartBaseCalibrationRequest {
    std::string method;  // "THREE_POINT", "FOUR_POINT", "DIRECT_INPUT"

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(StartBaseCalibrationRequest, method)
};

/**
 * Response for START_BASE_CALIBRATION
 */
struct StartBaseCalibrationResponse {
    bool success = false;
    int pointsRequired = 0;
    std::string error;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(StartBaseCalibrationResponse, success, pointsRequired, error)
};

/**
 * Request for RECORD_BASE_POINT
 */
struct RecordBasePointRequest {
    int pointIndex = 0;
    std::vector<double> jointAngles;
    std::vector<double> tcpPosition;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(RecordBasePointRequest, pointIndex, jointAngles, tcpPosition)
};

/**
 * Response for RECORD_BASE_POINT
 */
struct RecordBasePointResponse {
    bool success = false;
    int pointIndex = 0;
    std::string pointName;
    int totalPoints = 0;
    int recordedPoints = 0;
    std::string error;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(RecordBasePointResponse,
        success, pointIndex, pointName, totalPoints, recordedPoints, error)
};

/**
 * Response for FINISH_BASE_CALIBRATION
 */
struct FinishBaseCalibrationResponse {
    bool success = false;
    FramePayload calculatedFrame;
    std::string error;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(FinishBaseCalibrationResponse, success, calculatedFrame, error)
};

/**
 * Response for GET_BASE_CALIBRATION_STATUS
 */
struct BaseCalibrationStatusResponse {
    std::string state;      // "IDLE", "IN_PROGRESS", "COMPLETED", "ERROR"
    std::string method;     // "THREE_POINT", "FOUR_POINT"
    int pointsRequired = 0;
    int pointsRecorded = 0;
    std::string currentPointName;
    std::string errorMessage;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(BaseCalibrationStatusResponse,
        state, method, pointsRequired, pointsRecorded, currentPointName, errorMessage)
};

/**
 * Event for BASE_CHANGED
 */
struct BaseChangedEvent {
    std::string baseId;
    std::string baseName;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(BaseChangedEvent, baseId, baseName)
};

} // namespace ipc
} // namespace robot_controller
