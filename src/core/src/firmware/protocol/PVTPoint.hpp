/**
 * @file PVTPoint.hpp
 * @brief V2 Binary Protocol — Motion command data structures
 *
 * Packed structs shared between PC and STM32.
 * All sizes are verified by static_assert.
 */

#pragma once

#include <cstdint>
#include <array>

namespace robot_controller {
namespace firmware {
namespace protocol {

constexpr int PROTO_NUM_AXES = 6;

#pragma pack(push, 1)

// ============================================================================
// Motion Commands
// ============================================================================

/**
 * PVT (Position-Velocity-Time) point for trajectory streaming.
 * PC sends a stream of these; STM32 interpolates between them.
 */
struct PVTPoint {
    int32_t  position[PROTO_NUM_AXES];   // Target position (encoder steps)
    int16_t  velocity[PROTO_NUM_AXES];   // Target velocity (steps/ms)
    uint16_t duration_us;                // Segment duration (microseconds)
    uint8_t  flags;                      // bit0: last_segment, bit1: blend, bit2: sync_io
    uint8_t  io_mask;                    // I/O bits to change (0 = don't change)
    uint8_t  io_value;                   // I/O values to set

    // Flag helpers
    bool isLastSegment() const { return flags & 0x01; }
    bool isBlend() const { return flags & 0x02; }
    bool hasSyncIO() const { return flags & 0x04; }

    void setLastSegment(bool v) { if (v) flags |= 0x01; else flags &= ~0x01; }
    void setBlend(bool v) { if (v) flags |= 0x02; else flags &= ~0x02; }
    void setSyncIO(bool v) { if (v) flags |= 0x04; else flags &= ~0x04; }
};
static_assert(sizeof(PVTPoint) == 41, "PVTPoint must be 41 bytes");

/**
 * Move to absolute position command.
 * Used for point-to-point moves (jog step, go-home, etc.)
 */
struct MoveAbsoluteCmd {
    int32_t  position[PROTO_NUM_AXES];   // Target position (steps)
    uint16_t max_speed;                  // Max speed (steps/ms)
    uint16_t accel;                      // Acceleration (steps/ms^2)
};
static_assert(sizeof(MoveAbsoluteCmd) == 28, "MoveAbsoluteCmd must be 28 bytes");

/**
 * Stop motion command payload.
 */
struct StopMotionCmd {
    uint8_t mode;   // 0: decelerate to stop, 1: immediate stop
};
static_assert(sizeof(StopMotionCmd) == 1, "StopMotionCmd must be 1 byte");

// ============================================================================
// Jog Commands
// ============================================================================

/**
 * Start continuous jog on a single axis.
 */
struct JogStartCmd {
    uint8_t  axis;       // 0-5 (J1-J6)
    int8_t   direction;  // +1 or -1
    uint16_t speed;      // Speed (steps/ms)
};
static_assert(sizeof(JogStartCmd) == 4, "JogStartCmd must be 4 bytes");

// JogStop has no payload (empty command)

// ============================================================================
// Homing Commands
// ============================================================================

/**
 * Start homing sequence.
 */
struct HomeStartCmd {
    uint8_t axis_mask;   // bit0-5: which axes to home (0x3F = all)
    uint8_t sequence;    // 0: sequential (J1→J6), 1: custom order
    uint8_t method;      // 0: limit_switch, 1: index_pulse, 2: current_position
};
static_assert(sizeof(HomeStartCmd) == 3, "HomeStartCmd must be 3 bytes");

/**
 * Set homing parameters for a single axis.
 */
struct HomeSetParamsCmd {
    uint8_t  axis;           // 0-5
    int8_t   direction;      // Search direction: +1 or -1
    uint16_t fast_speed;     // Fast approach speed (steps/ms)
    uint16_t slow_speed;     // Slow approach speed (steps/ms)
    int32_t  offset;         // Post-home offset (steps)
    uint16_t backoff_dist;   // Backoff distance after switch hit (steps)
    uint8_t  use_index;      // 0: no index pulse, 1: use index pulse
};
static_assert(sizeof(HomeSetParamsCmd) == 13, "HomeSetParamsCmd must be 13 bytes");

/**
 * Stop homing.
 */
struct HomeStopCmd {
    uint8_t axis_mask;   // bit0-5: which axes to stop (0x3F = all)
};
static_assert(sizeof(HomeStopCmd) == 1, "HomeStopCmd must be 1 byte");

// ============================================================================
// Drive Control Commands
// ============================================================================

/**
 * Enable/Disable drives or reset alarms.
 * Same structure for CMD_ENABLE_DRIVES, CMD_DISABLE_DRIVES, CMD_RESET_ALARM.
 */
struct DriveControlCmd {
    uint8_t axis_mask;   // bit0-5: which drives (0x3F = all)
};
static_assert(sizeof(DriveControlCmd) == 1, "DriveControlCmd must be 1 byte");

// ============================================================================
// Configuration Commands
// ============================================================================

/**
 * Set axis parameters (steps/rev, gear ratio, limits).
 */
struct AxisParamsCmd {
    uint8_t  axis;              // 0-5
    uint32_t steps_per_rev;     // Encoder steps per motor revolution
    uint16_t gear_ratio_num;    // Gear ratio numerator
    uint16_t gear_ratio_den;    // Gear ratio denominator
    int32_t  min_limit;         // Software min limit (steps)
    int32_t  max_limit;         // Software max limit (steps)
};
static_assert(sizeof(AxisParamsCmd) == 17, "AxisParamsCmd must be 17 bytes");

// ============================================================================
// I/O Commands
// ============================================================================

/**
 * Set a single digital output.
 */
struct SetOutputCmd {
    uint8_t index;   // Output index (0-15)
    uint8_t value;   // 0: OFF, 1: ON
};
static_assert(sizeof(SetOutputCmd) == 2, "SetOutputCmd must be 2 bytes");

/**
 * Set multiple digital outputs at once.
 */
struct SetOutputsBatchCmd {
    uint16_t mask;    // Which outputs to change (bit flags)
    uint16_t values;  // Output values (bit flags)
};
static_assert(sizeof(SetOutputsBatchCmd) == 4, "SetOutputsBatchCmd must be 4 bytes");

#pragma pack(pop)

} // namespace protocol
} // namespace firmware
} // namespace robot_controller
