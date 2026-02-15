/**
 * @file cmd_handler.h
 * @brief Command dispatcher — routes non-system commands to subsystems
 *
 * Registered with NetTask via NetTask_RegisterHandler().
 * Maintains axis configuration set by CMD_SET_AXIS_PARAMS / CMD_SET_ACCEL.
 */

#ifndef CMD_HANDLER_H
#define CMD_HANDLER_H

#include "protocol_defs.h"
#include <stdint.h>

/* ========================================================================= */
/*  Axis Configuration                                                       */
/* ========================================================================= */

typedef struct {
    uint32_t steps_per_rev;       /* Encoder steps per revolution     */
    uint16_t gear_ratio_num;      /* Gear ratio numerator             */
    uint16_t gear_ratio_den;      /* Gear ratio denominator           */
    int32_t  min_limit;           /* Software min limit (steps)       */
    int32_t  max_limit;           /* Software max limit (steps)       */
    uint16_t max_accel;           /* Max acceleration (steps/ms²)    */
    uint16_t max_velocity;        /* Max velocity (steps/ms)          */
} AxisConfig;

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

/**
 * Initialize command handler with default axis configs.
 */
void CmdHandler_Init(void);

/**
 * Dispatch a command to the appropriate subsystem.
 * Signature matches CommandHandler typedef in net_task.h.
 */
void CmdHandler_Dispatch(uint8_t cmd_type, const uint8_t *payload, uint16_t len);

/**
 * Get axis configuration (read-only). Returns NULL if axis >= PROTO_NUM_AXES.
 */
const AxisConfig *CmdHandler_GetAxisConfig(uint8_t axis);

/**
 * Get global acceleration setting.
 */
uint16_t CmdHandler_GetGlobalAccel(void);

#endif /* CMD_HANDLER_H */
