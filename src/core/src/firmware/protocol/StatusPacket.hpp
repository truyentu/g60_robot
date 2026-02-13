/**
 * @file StatusPacket.hpp
 * @brief V2 Binary Protocol — Status response from STM32
 *
 * Packed struct sent at 100Hz from STM32 → PC via UDP.
 */

#pragma once

#include "PacketTypes.hpp"
#include "PVTPoint.hpp"
#include <cstdint>
#include <cstring>
#include <string>

namespace robot_controller {
namespace firmware {
namespace protocol {

#pragma pack(push, 1)

/**
 * Complete status packet sent from STM32 at 100Hz.
 * Contains all real-time state needed by PC.
 */
struct StatusPacket {
    uint8_t  seq;                               // Sequence counter (wraps 0-255)
    uint8_t  state;                             // SystemState enum
    int32_t  actual_pos[PROTO_NUM_AXES];        // Actual position (encoder steps)
    int32_t  cmd_pos[PROTO_NUM_AXES];           // Commanded position (steps)
    int16_t  velocity[PROTO_NUM_AXES];          // Current velocity (steps/ms)
    uint8_t  drive_ready;                       // bit0-5: drive ready flags
    uint8_t  drive_alarm;                       // bit0-5: drive alarm flags
    uint16_t digital_inputs;                    // Digital input states
    uint16_t digital_outputs;                   // Digital output states
    uint8_t  pvt_buffer_lvl;                    // PVT buffer fill level (0-255)
    uint8_t  home_status;                       // bit0-5: axis homed flags
    uint32_t timestamp_us;                      // STM32 microsecond timestamp

    // Helpers
    SystemState getSystemState() const {
        return static_cast<SystemState>(state);
    }

    bool isDriveReady(int axis) const {
        return (drive_ready >> axis) & 1;
    }

    bool isDriveAlarm(int axis) const {
        return (drive_alarm >> axis) & 1;
    }

    bool isAxisHomed(int axis) const {
        return (home_status >> axis) & 1;
    }

    bool isAllHomed() const {
        return (home_status & 0x3F) == 0x3F;
    }

    bool isAllDrivesReady() const {
        return (drive_ready & 0x3F) == 0x3F;
    }

    bool hasAnyAlarm() const {
        return (drive_alarm & 0x3F) != 0;
    }

    bool isDigitalInput(int index) const {
        return (digital_inputs >> index) & 1;
    }

    bool isDigitalOutput(int index) const {
        return (digital_outputs >> index) & 1;
    }

    void clear() {
        std::memset(this, 0, sizeof(StatusPacket));
    }
};
static_assert(sizeof(StatusPacket) == 74, "StatusPacket must be 74 bytes");

/**
 * Alarm notification from STM32.
 */
struct AlarmPacket {
    uint8_t  axis;         // Which axis (0-5, or 0xFF for system alarm)
    uint8_t  alarm_code;   // AlarmCode enum
    uint32_t timestamp_us;
};
static_assert(sizeof(AlarmPacket) == 6, "AlarmPacket must be 6 bytes");

/**
 * Homing complete notification.
 */
struct HomeCompletePacket {
    uint8_t axis;          // Which axis completed (0-5)
    uint8_t success;       // 0: failed, 1: success
    int32_t home_position; // Final home position (steps)
};
static_assert(sizeof(HomeCompletePacket) == 6, "HomeCompletePacket must be 6 bytes");

/**
 * Limit switch hit notification.
 */
struct LimitHitPacket {
    uint8_t axis;         // Which axis (0-5)
    uint8_t limit_type;   // 0: min, 1: max, 2: home switch
    int32_t position;     // Position when limit was hit (steps)
};
static_assert(sizeof(LimitHitPacket) == 6, "LimitHitPacket must be 6 bytes");

/**
 * ACK/NACK response to a command.
 */
struct AckPacket {
    uint8_t cmd_seq;       // Sequence number of the command being acknowledged
    uint8_t cmd_type;      // CommandType of the command
    uint8_t error_code;    // 0 = OK, non-zero = error
};
static_assert(sizeof(AckPacket) == 3, "AckPacket must be 3 bytes");

/**
 * Version response.
 */
struct VersionPacket {
    uint8_t  major;
    uint8_t  minor;
    uint8_t  patch;
    uint32_t build;         // Build number or git commit hash
    char     board[16];     // Board name (null-terminated, e.g., "STM32H743")
};
static_assert(sizeof(VersionPacket) == 23, "VersionPacket must be 23 bytes");

#pragma pack(pop)

} // namespace protocol
} // namespace firmware
} // namespace robot_controller
