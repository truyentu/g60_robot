/**
 * @file protocol_commands.h
 * @brief Command payload structs — C-compatible packed structs
 *
 * Mirrors: src/core/src/firmware/protocol/PVTPoint.hpp
 * Same field names, types, sizes, and byte layout.
 * Binary compatibility verified by _Static_assert.
 */

#ifndef PROTOCOL_COMMANDS_H
#define PROTOCOL_COMMANDS_H

#include "protocol_defs.h"
#include <stdint.h>

#pragma pack(push, 1)

/* ========================================================================= */
/*  Motion Commands                                                          */
/* ========================================================================= */

/**
 * PVT (Position-Velocity-Time) point for trajectory streaming.
 * PC sends a stream of these; STM32 interpolates between them.
 */
typedef struct {
    int32_t  position[PROTO_NUM_AXES];   /* Target position (encoder steps)    */
    int16_t  velocity[PROTO_NUM_AXES];   /* Target velocity (steps/ms)         */
    uint16_t duration_us;                /* Segment duration (microseconds)    */
    uint8_t  flags;                      /* bit0: last, bit1: blend, bit2: io  */
    uint8_t  io_mask;                    /* I/O bits to change (0 = no change) */
    uint8_t  io_value;                   /* I/O values to set                  */
} PVTPoint;
_Static_assert(sizeof(PVTPoint) == 41, "PVTPoint must be 41 bytes");

/* PVTPoint flag helpers (macros, since C has no methods) */
#define PVT_IS_LAST_SEGMENT(p)   ((p)->flags & 0x01)
#define PVT_IS_BLEND(p)          ((p)->flags & 0x02)
#define PVT_HAS_SYNC_IO(p)      ((p)->flags & 0x04)

#define PVT_SET_LAST_SEGMENT(p, v) \
    do { if (v) (p)->flags |= 0x01; else (p)->flags &= ~0x01; } while(0)
#define PVT_SET_BLEND(p, v) \
    do { if (v) (p)->flags |= 0x02; else (p)->flags &= ~0x02; } while(0)
#define PVT_SET_SYNC_IO(p, v) \
    do { if (v) (p)->flags |= 0x04; else (p)->flags &= ~0x04; } while(0)

/**
 * Move to absolute position command.
 */
typedef struct {
    int32_t  position[PROTO_NUM_AXES];   /* Target position (steps)  */
    uint16_t max_speed;                  /* Max speed (steps/ms)     */
    uint16_t accel;                      /* Acceleration (steps/ms2) */
} MoveAbsoluteCmd;
_Static_assert(sizeof(MoveAbsoluteCmd) == 28, "MoveAbsoluteCmd must be 28 bytes");

/**
 * Stop motion command.
 */
typedef struct {
    uint8_t mode;   /* 0: decelerate, 1: immediate stop */
} StopMotionCmd;
_Static_assert(sizeof(StopMotionCmd) == 1, "StopMotionCmd must be 1 byte");

/* ========================================================================= */
/*  Jog Commands                                                             */
/* ========================================================================= */

/**
 * Start continuous jog on a single axis.
 */
typedef struct {
    uint8_t  axis;       /* 0-5 (J1-J6)   */
    int8_t   direction;  /* +1 or -1       */
    uint16_t speed;      /* Speed (steps/ms) */
} JogStartCmd;
_Static_assert(sizeof(JogStartCmd) == 4, "JogStartCmd must be 4 bytes");

/* ========================================================================= */
/*  Homing Commands                                                          */
/* ========================================================================= */

/**
 * Start homing sequence.
 */
typedef struct {
    uint8_t axis_mask;   /* bit0-5: which axes (0x3F = all) */
    uint8_t sequence;    /* 0: sequential, 1: custom        */
    uint8_t method;      /* 0: limit_switch, 1: index, 2: current */
} HomeStartCmd;
_Static_assert(sizeof(HomeStartCmd) == 3, "HomeStartCmd must be 3 bytes");

/**
 * Set homing parameters for a single axis.
 */
typedef struct {
    uint8_t  axis;           /* 0-5                          */
    int8_t   direction;      /* Search direction: +1 or -1   */
    uint16_t fast_speed;     /* Fast approach (steps/ms)     */
    uint16_t slow_speed;     /* Slow approach (steps/ms)     */
    int32_t  offset;         /* Post-home offset (steps)     */
    uint16_t backoff_dist;   /* Backoff distance (steps)     */
    uint8_t  use_index;      /* 0: no, 1: use index pulse    */
} HomeSetParamsCmd;
_Static_assert(sizeof(HomeSetParamsCmd) == 13, "HomeSetParamsCmd must be 13 bytes");

/**
 * Stop homing.
 */
typedef struct {
    uint8_t axis_mask;   /* bit0-5: which axes (0x3F = all) */
} HomeStopCmd;
_Static_assert(sizeof(HomeStopCmd) == 1, "HomeStopCmd must be 1 byte");

/* ========================================================================= */
/*  Drive Control Commands                                                   */
/* ========================================================================= */

/**
 * Enable/Disable drives or reset alarms.
 */
typedef struct {
    uint8_t axis_mask;   /* bit0-5: which drives (0x3F = all) */
} DriveControlCmd;
_Static_assert(sizeof(DriveControlCmd) == 1, "DriveControlCmd must be 1 byte");

/* ========================================================================= */
/*  Configuration Commands                                                   */
/* ========================================================================= */

/**
 * Set axis parameters.
 */
typedef struct {
    uint8_t  axis;              /* 0-5                       */
    uint32_t steps_per_rev;     /* Encoder steps per rev     */
    uint16_t gear_ratio_num;    /* Gear ratio numerator      */
    uint16_t gear_ratio_den;    /* Gear ratio denominator    */
    int32_t  min_limit;         /* Software min limit (steps) */
    int32_t  max_limit;         /* Software max limit (steps) */
} AxisParamsCmd;
_Static_assert(sizeof(AxisParamsCmd) == 17, "AxisParamsCmd must be 17 bytes");

/* ========================================================================= */
/*  I/O Commands                                                             */
/* ========================================================================= */

/**
 * Set a single digital output.
 */
typedef struct {
    uint8_t index;   /* Output index (0-15) */
    uint8_t value;   /* 0: OFF, 1: ON       */
} SetOutputCmd;
_Static_assert(sizeof(SetOutputCmd) == 2, "SetOutputCmd must be 2 bytes");

/**
 * Set multiple digital outputs at once.
 */
typedef struct {
    uint16_t mask;    /* Which outputs to change */
    uint16_t values;  /* Output values           */
} SetOutputsBatchCmd;
_Static_assert(sizeof(SetOutputsBatchCmd) == 4, "SetOutputsBatchCmd must be 4 bytes");

#pragma pack(pop)

#endif /* PROTOCOL_COMMANDS_H */
