/**
 * @file protocol_status.h
 * @brief Response payload structs — C-compatible packed structs
 *
 * Mirrors: src/core/src/firmware/protocol/StatusPacket.hpp
 * Same field names, types, sizes, and byte layout.
 */

#ifndef PROTOCOL_STATUS_H
#define PROTOCOL_STATUS_H

#include "protocol_defs.h"
#include <stdint.h>
#include <string.h>

#pragma pack(push, 1)

/* ========================================================================= */
/*  Status Packet (STM32 -> PC, sent at 100Hz)                               */
/* ========================================================================= */

typedef struct {
    uint8_t  seq;                               /* Sequence counter (0-255)      */
    uint8_t  state;                             /* SystemState enum              */
    int32_t  actual_pos[PROTO_NUM_AXES];        /* Actual position (enc steps)   */
    int32_t  cmd_pos[PROTO_NUM_AXES];           /* Commanded position (steps)    */
    int16_t  velocity[PROTO_NUM_AXES];          /* Current velocity (steps/ms)   */
    uint8_t  drive_ready;                       /* bit0-5: drive ready flags     */
    uint8_t  drive_alarm;                       /* bit0-5: drive alarm flags     */
    uint16_t digital_inputs;                    /* Digital input states           */
    uint16_t digital_outputs;                   /* Digital output states          */
    uint8_t  pvt_buffer_lvl;                    /* PVT buffer fill (0-255)       */
    uint8_t  home_status;                       /* bit0-5: axis homed flags      */
    uint32_t timestamp_us;                      /* Microsecond timestamp         */
} StatusPacket;
_Static_assert(sizeof(StatusPacket) == 74, "StatusPacket must be 74 bytes");

/* StatusPacket helpers (macros) */
#define STATUS_IS_DRIVE_READY(sp, axis)   (((sp)->drive_ready >> (axis)) & 1)
#define STATUS_IS_DRIVE_ALARM(sp, axis)   (((sp)->drive_alarm >> (axis)) & 1)
#define STATUS_IS_AXIS_HOMED(sp, axis)    (((sp)->home_status >> (axis)) & 1)
#define STATUS_IS_ALL_HOMED(sp)           (((sp)->home_status & 0x3F) == 0x3F)
#define STATUS_IS_ALL_READY(sp)           (((sp)->drive_ready & 0x3F) == 0x3F)
#define STATUS_HAS_ANY_ALARM(sp)          (((sp)->drive_alarm & 0x3F) != 0)
#define STATUS_IS_DIN(sp, idx)            (((sp)->digital_inputs >> (idx)) & 1)
#define STATUS_IS_DOUT(sp, idx)           (((sp)->digital_outputs >> (idx)) & 1)

static inline void StatusPacket_Clear(StatusPacket *sp) {
    memset(sp, 0, sizeof(StatusPacket));
}

/* ========================================================================= */
/*  Alarm Packet                                                             */
/* ========================================================================= */

typedef struct {
    uint8_t  axis;         /* Which axis (0-5, or 0xFF for system) */
    uint8_t  alarm_code;   /* AlarmCode enum                       */
    uint32_t timestamp_us;
} AlarmPacket;
_Static_assert(sizeof(AlarmPacket) == 6, "AlarmPacket must be 6 bytes");

/* ========================================================================= */
/*  Home Complete Packet                                                     */
/* ========================================================================= */

typedef struct {
    uint8_t axis;          /* Which axis completed (0-5)   */
    uint8_t success;       /* 0: failed, 1: success        */
    int32_t home_position; /* Final home position (steps)  */
} HomeCompletePacket;
_Static_assert(sizeof(HomeCompletePacket) == 6, "HomeCompletePacket must be 6 bytes");

/* ========================================================================= */
/*  Limit Hit Packet                                                         */
/* ========================================================================= */

typedef struct {
    uint8_t axis;         /* Which axis (0-5)                     */
    uint8_t limit_type;   /* 0: min, 1: max, 2: home switch      */
    int32_t position;     /* Position when limit was hit (steps)  */
} LimitHitPacket;
_Static_assert(sizeof(LimitHitPacket) == 6, "LimitHitPacket must be 6 bytes");

/* ========================================================================= */
/*  ACK/NACK Packet                                                          */
/* ========================================================================= */

typedef struct {
    uint8_t cmd_seq;       /* Sequence of acknowledged command */
    uint8_t cmd_type;      /* CommandType of the command       */
    uint8_t error_code;    /* 0 = OK, non-zero = error         */
} AckPacket;
_Static_assert(sizeof(AckPacket) == 3, "AckPacket must be 3 bytes");

/* ========================================================================= */
/*  Version Packet                                                           */
/* ========================================================================= */

typedef struct {
    uint8_t  major;
    uint8_t  minor;
    uint8_t  patch;
    uint32_t build;         /* Build number or git hash */
    char     board[16];     /* Board name (null-terminated) */
} VersionPacket;
_Static_assert(sizeof(VersionPacket) == 23, "VersionPacket must be 23 bytes");

/* ========================================================================= */
/*  Buffer Status Packet (for RSP_BUFFER_LOW)                                */
/* ========================================================================= */

typedef struct {
    uint16_t level;      /* Current buffer fill level (number of points)  */
    uint16_t capacity;   /* Total buffer capacity                         */
} BufferStatusPacket;
_Static_assert(sizeof(BufferStatusPacket) == 4, "BufferStatusPacket must be 4 bytes");

#pragma pack(pop)

#endif /* PROTOCOL_STATUS_H */
