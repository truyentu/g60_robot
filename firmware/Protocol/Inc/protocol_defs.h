/**
 * @file protocol_defs.h
 * @brief Protocol definitions — C-compatible enums and constants
 *
 * Mirrors: src/core/src/firmware/protocol/PacketTypes.hpp
 * MUST stay in sync with the C++ PC-side definitions.
 * Same enum values guarantee binary compatibility.
 */

#ifndef PROTOCOL_DEFS_H
#define PROTOCOL_DEFS_H

#include <stdint.h>

/* Number of robot axes */
#define PROTO_NUM_AXES      6

/* ========================================================================= */
/*  Command Types (PC -> STM32)                                              */
/*  Mirrors: PacketTypes.hpp :: CommandType                                   */
/* ========================================================================= */

enum {
    /* Motion */
    CMD_PVT_POINT       = 0x10,
    CMD_PVT_BATCH       = 0x11,
    CMD_MOVE_ABSOLUTE   = 0x12,
    CMD_STOP_MOTION     = 0x13,

    /* Jog */
    CMD_JOG_START       = 0x20,
    CMD_JOG_STOP        = 0x21,

    /* Homing */
    CMD_HOME_START      = 0x30,
    CMD_HOME_SET_PARAMS = 0x31,
    CMD_HOME_STOP       = 0x32,

    /* Drive Control */
    CMD_ENABLE_DRIVES   = 0x40,
    CMD_DISABLE_DRIVES  = 0x41,
    CMD_RESET_ALARM     = 0x42,

    /* Configuration */
    CMD_SET_AXIS_PARAMS = 0x50,
    CMD_SET_ACCEL       = 0x51,

    /* I/O */
    CMD_SET_OUTPUT       = 0x60,
    CMD_SET_OUTPUTS_BATCH = 0x61,

    /* System */
    CMD_HEARTBEAT       = 0xF0,
    CMD_GET_VERSION     = 0xF1,
    CMD_E_STOP          = 0xFF,
};

/* ========================================================================= */
/*  Response Types (STM32 -> PC)                                             */
/*  Mirrors: PacketTypes.hpp :: ResponseType                                 */
/* ========================================================================= */

enum {
    RSP_STATUS          = 0x80,
    RSP_ALARM           = 0x81,
    RSP_HOME_COMPLETE   = 0x82,
    RSP_LIMIT_HIT       = 0x83,
    RSP_MOTION_COMPLETE = 0x84,
    RSP_BUFFER_LOW      = 0x85,
    RSP_ACK             = 0x8A,
    RSP_NACK            = 0x8B,
    RSP_VERSION         = 0x8C,
    RSP_ERROR           = 0x8F,
};

/* ========================================================================= */
/*  System States                                                            */
/*  Mirrors: PacketTypes.hpp :: SystemState                                  */
/* ========================================================================= */

enum {
    SYSTEM_STATE_INIT       = 0x00,
    SYSTEM_STATE_IDLE       = 0x01,
    SYSTEM_STATE_MOVING     = 0x02,
    SYSTEM_STATE_JOGGING    = 0x03,
    SYSTEM_STATE_HOMING     = 0x04,
    SYSTEM_STATE_HOLD       = 0x05,
    SYSTEM_STATE_ALARM      = 0x06,
    SYSTEM_STATE_ESTOP      = 0x07,
    SYSTEM_STATE_DISABLED   = 0x08,
    SYSTEM_STATE_ERROR      = 0x0F,
};

/* ========================================================================= */
/*  Alarm Codes                                                              */
/*  Mirrors: PacketTypes.hpp :: AlarmCode                                    */
/* ========================================================================= */

enum {
    ALARM_NONE              = 0x00,
    ALARM_HARD_LIMIT        = 0x01,
    ALARM_SOFT_LIMIT        = 0x02,
    ALARM_FOLLOWING_ERROR   = 0x03,
    ALARM_OVERCURRENT       = 0x04,
    ALARM_ENCODER_FAULT     = 0x05,
    ALARM_MOTOR_FAULT       = 0x06,
    ALARM_COMM_TIMEOUT      = 0x07,
    ALARM_WATCHDOG          = 0x08,
    ALARM_ESTOP             = 0x09,
    ALARM_HOMING_FAIL       = 0x0A,
};

#endif /* PROTOCOL_DEFS_H */
