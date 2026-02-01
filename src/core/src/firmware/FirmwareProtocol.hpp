/**
 * @file FirmwareProtocol.hpp
 * @brief Protocol definitions for firmware communication with grblHAL
 */

#pragma once

#include <cstdint>
#include <string>
#include <array>
#include <cmath>

namespace robot_controller {
namespace firmware {

// ============================================================================
// Protocol Constants
// ============================================================================

constexpr int NUM_AXES = 6;
constexpr uint32_t PROTOCOL_VERSION = 0x0100;  // v1.0

// Baud rate for serial communication
constexpr int DEFAULT_BAUD_RATE = 115200;
constexpr int HIGH_SPEED_BAUD_RATE = 921600;

// Timeouts (ms)
constexpr int RESPONSE_TIMEOUT_MS = 1000;
constexpr int HOMING_TIMEOUT_MS = 60000;
constexpr int MOTION_TIMEOUT_MS = 5000;

// Buffer sizes
constexpr int TX_BUFFER_SIZE = 256;
constexpr int RX_BUFFER_SIZE = 4096;
constexpr int COMMAND_QUEUE_SIZE = 32;

// ============================================================================
// Command Types
// ============================================================================

/**
 * Command codes sent to firmware
 */
enum class CommandCode : uint8_t {
    // System commands
    PING = 0x01,
    RESET = 0x02,
    GET_VERSION = 0x03,
    GET_STATUS = 0x04,
    SET_CONFIG = 0x05,

    // Motion commands
    MOVE_JOINT = 0x10,
    MOVE_LINEAR = 0x11,
    MOVE_RAPID = 0x12,
    JOG_START = 0x13,
    JOG_STOP = 0x14,
    STOP_MOTION = 0x15,
    FEED_HOLD = 0x16,
    CYCLE_START = 0x17,

    // Homing commands
    HOME_ALL = 0x20,
    HOME_AXIS = 0x21,
    SET_HOME = 0x22,
    GET_HOME_STATUS = 0x23,

    // Position commands
    GET_POSITION = 0x30,
    SET_POSITION = 0x31,
    GET_VELOCITY = 0x32,
    GET_FOLLOWING_ERROR = 0x33,

    // I/O commands
    GET_INPUTS = 0x40,
    SET_OUTPUTS = 0x41,
    GET_ANALOG = 0x42,

    // Safety commands
    GET_LIMITS = 0x50,
    CLEAR_ALARM = 0x51,
    GET_ALARM = 0x52,

    // G-code passthrough
    GCODE = 0xF0
};

/**
 * Response codes from firmware
 */
enum class ResponseCode : uint8_t {
    OK = 0x00,
    ERROR = 0x01,
    BUSY = 0x02,
    INVALID_COMMAND = 0x03,
    INVALID_PARAM = 0x04,
    BUFFER_FULL = 0x05,
    TIMEOUT = 0x06,
    ALARM = 0x07,
    NOT_HOMED = 0x08,

    // Status responses
    STATUS = 0x10,
    POSITION = 0x11,
    VERSION = 0x12,
    LIMITS = 0x13,

    // Async notifications
    MOTION_COMPLETE = 0x20,
    HOMING_COMPLETE = 0x21,
    LIMIT_TRIGGERED = 0x22,
    ALARM_TRIGGERED = 0x23
};

// ============================================================================
// Firmware State
// ============================================================================

/**
 * Firmware machine state (matches grblHAL)
 */
enum class MachineState : uint8_t {
    IDLE = 0,
    RUN = 1,
    HOLD = 2,
    JOG = 3,
    HOMING = 4,
    ALARM = 5,
    CHECK = 6,
    DOOR = 7,
    SLEEP = 8
};

/**
 * Alarm codes (grblHAL compatible)
 */
enum class AlarmCode : uint8_t {
    NONE = 0,
    HARD_LIMIT = 1,
    SOFT_LIMIT = 2,
    ABORT_CYCLE = 3,
    PROBE_FAIL = 4,
    PROBE_INITIAL = 5,
    HOMING_FAIL_RESET = 6,
    HOMING_FAIL_DOOR = 7,
    HOMING_FAIL_PULLOFF = 8,
    HOMING_FAIL_APPROACH = 9,
    SPINDLE_CONTROL = 10,
    MOTOR_FAULT = 11,
    ESTOP = 12
};

// ============================================================================
// Data Structures
// ============================================================================

#pragma pack(push, 1)

/**
 * Position data (6 axes, in steps or degrees*1000)
 */
struct PositionData {
    int32_t axis[NUM_AXES];

    PositionData() {
        for (int i = 0; i < NUM_AXES; ++i) axis[i] = 0;
    }
};

/**
 * Velocity data (6 axes)
 */
struct VelocityData {
    int32_t axis[NUM_AXES];

    VelocityData() {
        for (int i = 0; i < NUM_AXES; ++i) axis[i] = 0;
    }
};

/**
 * Limit switch status (bit flags)
 */
struct LimitStatus {
    uint8_t minLimits;
    uint8_t maxLimits;
    uint8_t homeSwitches;

    LimitStatus() : minLimits(0), maxLimits(0), homeSwitches(0) {}

    bool isMinLimit(int axis) const { return (minLimits >> axis) & 1; }
    bool isMaxLimit(int axis) const { return (maxLimits >> axis) & 1; }
    bool isHome(int axis) const { return (homeSwitches >> axis) & 1; }
};

/**
 * I/O status
 */
struct IOStatus {
    uint16_t digitalInputs;
    uint16_t digitalOutputs;
    uint16_t analogInputs[4];

    IOStatus() : digitalInputs(0), digitalOutputs(0), analogInputs{0} {}
};

/**
 * Complete machine status
 */
struct MachineStatus {
    MachineState state;
    AlarmCode alarm;
    PositionData position;
    PositionData targetPosition;
    VelocityData velocity;
    LimitStatus limits;
    IOStatus io;
    uint8_t homingStatus;
    uint8_t motionBuffer;
    uint16_t feedOverride;
    uint16_t rapidOverride;

    MachineStatus()
        : state(MachineState::IDLE),
          alarm(AlarmCode::NONE),
          homingStatus(0),
          motionBuffer(0),
          feedOverride(100),
          rapidOverride(100) {}

    bool isHomed(int axis) const { return (homingStatus >> axis) & 1; }
    bool isAllHomed() const { return homingStatus == 0x3F; }
};

/**
 * Command header
 */
struct CommandHeader {
    uint8_t startByte;
    uint8_t command;
    uint16_t payloadSize;
    uint16_t sequenceNum;
};

/**
 * Response header
 */
struct ResponseHeader {
    uint8_t startByte;
    uint8_t response;
    uint16_t payloadSize;
    uint16_t sequenceNum;
    uint8_t checksum;
};

/**
 * Move command payload
 */
struct MoveCommand {
    PositionData target;
    uint32_t feedRate;
    uint8_t moveType;
    uint8_t flags;
};

/**
 * Jog command payload
 */
struct JogCommand {
    int8_t direction[NUM_AXES];
    uint32_t speed;
    uint8_t continuous;
    uint32_t distance;
};

#pragma pack(pop)

// ============================================================================
// G-code Commands
// ============================================================================

namespace GCode {
    // Motion
    constexpr const char* RAPID_MOVE = "G0";
    constexpr const char* LINEAR_MOVE = "G1";
    constexpr const char* CW_ARC = "G2";
    constexpr const char* CCW_ARC = "G3";
    constexpr const char* DWELL = "G4";

    // Coordinate system
    constexpr const char* ABSOLUTE_MODE = "G90";
    constexpr const char* RELATIVE_MODE = "G91";
    constexpr const char* SET_ORIGIN = "G92";

    // Units
    constexpr const char* INCHES = "G20";
    constexpr const char* MILLIMETERS = "G21";

    // Homing
    constexpr const char* HOME_ALL = "G28";
    constexpr const char* HOME_AXIS = "G28.1";

    // Probe
    constexpr const char* PROBE = "G38.2";

    // Real-time commands (no newline needed)
    constexpr char SOFT_RESET = 0x18;
    constexpr char STATUS_QUERY = '?';
    constexpr char CYCLE_START = '~';
    constexpr char FEED_HOLD = '!';
    constexpr char SAFETY_DOOR = '\x84';
    constexpr char JOG_CANCEL = '\x85';

    // Feed overrides
    constexpr char FEED_100 = '\x90';
    constexpr char FEED_INCREASE_10 = '\x91';
    constexpr char FEED_DECREASE_10 = '\x92';
    constexpr char FEED_INCREASE_1 = '\x93';
    constexpr char FEED_DECREASE_1 = '\x94';

    // Rapid overrides
    constexpr char RAPID_100 = '\x95';
    constexpr char RAPID_50 = '\x96';
    constexpr char RAPID_25 = '\x97';
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Calculate XOR checksum of data
 */
inline uint8_t calculateChecksum(const uint8_t* data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; ++i) {
        checksum ^= data[i];
    }
    return checksum;
}

/**
 * Format G-code move command
 */
inline std::string formatMoveGCode(
    const std::array<double, NUM_AXES>& position,
    double feedRate,
    bool isRapid = false) {

    std::string cmd = isRapid ? "G0" : "G1";

    const char axisLetters[] = {'X', 'Y', 'Z', 'A', 'B', 'C'};

    for (int i = 0; i < NUM_AXES; ++i) {
        char buf[32];
        snprintf(buf, sizeof(buf), " %c%.4f", axisLetters[i], position[i]);
        cmd += buf;
    }

    if (!isRapid && feedRate > 0) {
        char buf[32];
        snprintf(buf, sizeof(buf), " F%.1f", feedRate);
        cmd += buf;
    }

    return cmd;
}

/**
 * Parse grblHAL status response
 * Format: <Idle|MPos:0.000,0.000,0.000|WPos:0.000,0.000,0.000|...>
 */
inline bool parseStatusResponse(const std::string& response, MachineStatus& status) {
    if (response.empty() || response[0] != '<') {
        return false;
    }

    // Find state
    size_t pipePos = response.find('|');
    if (pipePos == std::string::npos) return false;

    std::string stateStr = response.substr(1, pipePos - 1);

    if (stateStr == "Idle") status.state = MachineState::IDLE;
    else if (stateStr == "Run") status.state = MachineState::RUN;
    else if (stateStr == "Hold") status.state = MachineState::HOLD;
    else if (stateStr == "Jog") status.state = MachineState::JOG;
    else if (stateStr == "Home") status.state = MachineState::HOMING;
    else if (stateStr.find("Alarm") != std::string::npos) status.state = MachineState::ALARM;
    else if (stateStr == "Check") status.state = MachineState::CHECK;
    else if (stateStr == "Door") status.state = MachineState::DOOR;
    else if (stateStr == "Sleep") status.state = MachineState::SLEEP;

    // Parse MPos (machine position)
    size_t mposPos = response.find("MPos:");
    if (mposPos != std::string::npos) {
        size_t start = mposPos + 5;
        size_t end = response.find('|', start);
        if (end == std::string::npos) end = response.find('>', start);
        std::string posStr = response.substr(start, end - start);

        // Parse comma-separated values
        int axis = 0;
        size_t pos = 0;
        while (pos < posStr.length() && axis < NUM_AXES) {
            size_t comma = posStr.find(',', pos);
            if (comma == std::string::npos) comma = posStr.length();

            try {
                double value = std::stod(posStr.substr(pos, comma - pos));
                status.position.axis[axis] = static_cast<int32_t>(value * 1000);
            } catch (...) {
                // Parse error, skip
            }

            pos = comma + 1;
            axis++;
        }
    }

    // Parse Bf (buffer status)
    size_t bfPos = response.find("Bf:");
    if (bfPos != std::string::npos) {
        size_t start = bfPos + 3;
        size_t comma = response.find(',', start);
        if (comma != std::string::npos) {
            try {
                status.motionBuffer = static_cast<uint8_t>(std::stoi(response.substr(start, comma - start)));
            } catch (...) {}
        }
    }

    return true;
}

/**
 * Convert radians to degrees
 */
inline double radToDeg(double rad) {
    return rad * 180.0 / 3.14159265358979323846;
}

/**
 * Convert degrees to radians
 */
inline double degToRad(double deg) {
    return deg * 3.14159265358979323846 / 180.0;
}

} // namespace firmware
} // namespace robot_controller
