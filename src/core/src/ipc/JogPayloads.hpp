#pragma once

#include <array>
#include <string>
#include <nlohmann/json.hpp>

namespace robot_controller {
namespace ipc {

enum class JogMode : int {
    JOINT = 0,
    CARTESIAN = 1
};

enum class JogType : int {
    CONTINUOUS = 0,
    INCREMENTAL = 1
};

// Coordinate frame for Cartesian jog
enum class JogFrame : int {
    WORLD = 0,
    BASE = 1,
    TOOL = 2,
    USER1 = 3,
    USER2 = 4
};

struct JogStartRequest {
    bool enableDeadman = true;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(JogStartRequest, enableDeadman)
};

struct JogStartResponse {
    bool success;
    std::string error;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(JogStartResponse, success, error)
};

struct JogMoveRequest {
    int mode;           // JogMode: 0=Joint, 1=Cartesian
    int axis;           // Joint: 0-5, Cartesian: 0=X,1=Y,2=Z,3=Rx,4=Ry,5=Rz
    int direction;      // -1 or +1
    double speedPercent; // 1-100%
    int frame{0};       // JogFrame: 0=World, 1=Base, 2=Tool

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(JogMoveRequest, mode, axis, direction, speedPercent, frame)
};

struct JogStepRequest {
    int mode;
    int axis;
    int direction;
    double increment;   // Degrees for joint, mm for Cartesian
    double speedPercent;
    int frame{0};       // JogFrame: 0=World, 1=Base, 2=Tool

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(JogStepRequest, mode, axis, direction, increment, speedPercent, frame)
};

struct JogMoveResponse {
    bool success;
    std::string error;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(JogMoveResponse, success, error)
};

struct JogStatePayload {
    bool enabled;
    bool isMoving;
    int currentMode;
    int currentAxis;
    int currentDirection;
    double currentSpeed;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(JogStatePayload, enabled, isMoving,
        currentMode, currentAxis, currentDirection, currentSpeed)
};

std::string jogModeToString(JogMode mode);
std::string jogTypeToString(JogType type);
std::string jogFrameToString(JogFrame frame);

} // namespace ipc
} // namespace robot_controller
