/**
 * @file cmd_handler.c
 * @brief Command dispatcher — routes commands from NetTask to subsystems
 *
 * Handles:
 *   - CMD_SET_AXIS_PARAMS / CMD_SET_ACCEL → store config locally
 *   - CMD_SET_OUTPUT / CMD_SET_OUTPUTS_BATCH → GPIO digital output
 *   - CMD_E_STOP / CMD_STOP_MOTION → state transitions
 *   - CMD_ENABLE/DISABLE_DRIVES, CMD_RESET_ALARM → DriveControl (Phase 3)
 *   - CMD_HOME_* → Homing module (Phase 6)
 *   - CMD_MOVE_ABSOLUTE → S-curve planner (Phase 7)
 */

#include "cmd_handler.h"
#include "board_gpio.h"
#include "motion_task.h"
#include "drive_control.h"
#include "homing.h"
#include "protocol_commands.h"
#include "protocol_defs.h"

#include <string.h>

/* ========================================================================= */
/*  State                                                                    */
/* ========================================================================= */

static AxisConfig s_axis_config[PROTO_NUM_AXES];
static uint16_t   s_global_accel = 100;  /* Default global accel (steps/ms²) */

/* ========================================================================= */
/*  Init                                                                     */
/* ========================================================================= */

void CmdHandler_Init(void)
{
    for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
        s_axis_config[i].steps_per_rev   = 10000;   /* 2500 PPR * 4x */
        s_axis_config[i].gear_ratio_num  = 1;
        s_axis_config[i].gear_ratio_den  = 1;
        s_axis_config[i].min_limit       = -2000000;
        s_axis_config[i].max_limit       =  2000000;
        s_axis_config[i].max_accel       = 100;
        s_axis_config[i].max_velocity    = 500;
    }
    s_global_accel = 100;
}

/* ========================================================================= */
/*  Dispatch                                                                 */
/* ========================================================================= */

void CmdHandler_Dispatch(uint8_t cmd_type, const uint8_t *payload, uint16_t len)
{
    switch (cmd_type) {

    /* ---- E-Stop ---- */
    case CMD_E_STOP:
        Motion_SetState(SYSTEM_STATE_ESTOP);
        break;

    /* ---- Stop Motion ---- */
    case CMD_STOP_MOTION:
        Motion_SetState(SYSTEM_STATE_IDLE);
        break;

    /* ---- Drive Control ---- */
    case CMD_ENABLE_DRIVES:
        if (len >= sizeof(DriveControlCmd)) {
            const DriveControlCmd *cmd = (const DriveControlCmd *)payload;
            DriveControl_Enable(cmd->axis_mask);
        }
        break;

    case CMD_DISABLE_DRIVES:
        if (len >= sizeof(DriveControlCmd)) {
            const DriveControlCmd *cmd = (const DriveControlCmd *)payload;
            DriveControl_Disable(cmd->axis_mask);
        }
        break;

    case CMD_RESET_ALARM:
        if (len >= sizeof(DriveControlCmd)) {
            const DriveControlCmd *cmd = (const DriveControlCmd *)payload;
            DriveControl_ResetAlarm(cmd->axis_mask);
        }
        break;

    /* ---- Homing ---- */
    case CMD_HOME_START:
        if (len >= sizeof(HomeStartCmd)) {
            const HomeStartCmd *cmd = (const HomeStartCmd *)payload;
            Homing_Start(cmd->axis_mask, cmd->method);
        }
        break;

    case CMD_HOME_SET_PARAMS:
        if (len >= sizeof(HomeSetParamsCmd)) {
            const HomeSetParamsCmd *cmd = (const HomeSetParamsCmd *)payload;
            Homing_SetParams(cmd->axis, payload);
        }
        break;

    case CMD_HOME_STOP:
        if (len >= sizeof(HomeStopCmd)) {
            const HomeStopCmd *cmd = (const HomeStopCmd *)payload;
            Homing_Stop(cmd->axis_mask);
        }
        break;

    /* ---- MoveAbsolute ---- */
    case CMD_MOVE_ABSOLUTE:
        if (len >= sizeof(MoveAbsoluteCmd)) {
            const MoveAbsoluteCmd *cmd = (const MoveAbsoluteCmd *)payload;
            Motion_RequestMoveAbsolute(cmd);
        }
        break;

    /* ---- Configuration ---- */
    case CMD_SET_AXIS_PARAMS:
        if (len >= sizeof(AxisParamsCmd)) {
            const AxisParamsCmd *cmd = (const AxisParamsCmd *)payload;
            if (cmd->axis < PROTO_NUM_AXES) {
                s_axis_config[cmd->axis].steps_per_rev  = cmd->steps_per_rev;
                s_axis_config[cmd->axis].gear_ratio_num = cmd->gear_ratio_num;
                s_axis_config[cmd->axis].gear_ratio_den = cmd->gear_ratio_den;
                s_axis_config[cmd->axis].min_limit      = cmd->min_limit;
                s_axis_config[cmd->axis].max_limit      = cmd->max_limit;
            }
        }
        break;

    case CMD_SET_ACCEL: {
        /* Global acceleration: payload is uint16_t */
        if (len >= 2) {
            uint16_t accel;
            memcpy(&accel, payload, sizeof(accel));
            s_global_accel = accel;
            for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
                s_axis_config[i].max_accel = accel;
            }
        }
        break;
    }

    /* ---- I/O ---- */
    case CMD_SET_OUTPUT:
        if (len >= sizeof(SetOutputCmd)) {
            const SetOutputCmd *cmd = (const SetOutputCmd *)payload;
            GPIO_WriteDigitalOutput(cmd->index, cmd->value);
        }
        break;

    case CMD_SET_OUTPUTS_BATCH:
        if (len >= sizeof(SetOutputsBatchCmd)) {
            const SetOutputsBatchCmd *cmd = (const SetOutputsBatchCmd *)payload;
            GPIO_WriteDigitalOutputs(cmd->mask, cmd->values);
        }
        break;

    default:
        break;
    }
}

/* ========================================================================= */
/*  Getters                                                                  */
/* ========================================================================= */

const AxisConfig *CmdHandler_GetAxisConfig(uint8_t axis)
{
    if (axis >= PROTO_NUM_AXES) return NULL;
    return &s_axis_config[axis];
}

uint16_t CmdHandler_GetGlobalAccel(void)
{
    return s_global_accel;
}
