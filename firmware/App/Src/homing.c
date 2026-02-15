/**
 * @file homing.c
 * @brief Homing state machine implementation
 *
 * Per-axis state machine:
 *   IDLE → FAST_APPROACH → DECEL_STOP → BACKOFF → SLOW_APPROACH → SET_ZERO → COMPLETE
 *
 * Each state generates step pulses by updating s_positions in MotionTask.
 * Home switch debounce: HOME_DEBOUNCE_MS ticks at 250Hz.
 *
 * SIMULATION_MODE: Uses position tracking only (no real timers).
 * Real Mode: Uses StepTimer for pulse generation.
 */

#include "homing.h"
#include "board_config.h"
#include "board_gpio.h"
#include "board_timers.h"
#include "drive_control.h"
#include "motion_task.h"
#include "protocol_commands.h"
#include "protocol_defs.h"

#include <string.h>
#include <stdlib.h>

/* ========================================================================= */
/*  Per-Axis Homing Context                                                  */
/* ========================================================================= */

typedef struct {
    HomingState state;
    HomingParams params;
    int32_t  start_pos;        /* Position at start of current phase */
    int32_t  target_pos;       /* Target position for backoff       */
    uint16_t debounce_cnt;     /* Debounce counter for switch       */
    uint16_t decel_cnt;        /* Deceleration timer (ticks)        */
    uint8_t  result;           /* 0=fail, 1=success                 */
} HomingContext;

/* Debounce threshold: HOME_DEBOUNCE_MS / 4ms per tick */
#define DEBOUNCE_TICKS  ((HOME_DEBOUNCE_MS + MOTION_LOOP_PERIOD_MS - 1) / MOTION_LOOP_PERIOD_MS)

/* Deceleration stop ticks: ~50ms */
#define DECEL_STOP_TICKS  (50 / MOTION_LOOP_PERIOD_MS)

static HomingContext s_ctx[PROTO_NUM_AXES];
static uint8_t s_active_mask = 0;

/* ========================================================================= */
/*  Init                                                                     */
/* ========================================================================= */

void Homing_Init(void)
{
    memset(s_ctx, 0, sizeof(s_ctx));
    s_active_mask = 0;

    /* Set default parameters */
    for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
        s_ctx[i].params.direction   = -1;  /* Search toward negative */
        s_ctx[i].params.fast_speed  = HOME_DEFAULT_FAST_SPEED / 1000;  /* Convert steps/ms */
        s_ctx[i].params.slow_speed  = HOME_DEFAULT_SLOW_SPEED / 1000;
        s_ctx[i].params.offset      = 0;
        s_ctx[i].params.backoff_dist = HOME_DEFAULT_BACKOFF;
        s_ctx[i].params.use_index   = 0;
    }
}

/* ========================================================================= */
/*  Set Parameters                                                           */
/* ========================================================================= */

void Homing_SetParams(uint8_t axis, const void *params)
{
    if (axis >= PROTO_NUM_AXES) return;
    const HomeSetParamsCmd *cmd = (const HomeSetParamsCmd *)params;

    s_ctx[axis].params.direction   = cmd->direction;
    s_ctx[axis].params.fast_speed  = cmd->fast_speed;
    s_ctx[axis].params.slow_speed  = cmd->slow_speed;
    s_ctx[axis].params.offset      = cmd->offset;
    s_ctx[axis].params.backoff_dist = cmd->backoff_dist;
    s_ctx[axis].params.use_index   = cmd->use_index;
}

/* ========================================================================= */
/*  Start / Stop                                                             */
/* ========================================================================= */

void Homing_Start(uint8_t axis_mask, uint8_t method)
{
    (void)method;  /* Currently only limit_switch method */

    for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
        if (!(axis_mask & (1U << i))) continue;
        if (s_ctx[i].state != HOME_STATE_IDLE &&
            s_ctx[i].state != HOME_STATE_COMPLETE &&
            s_ctx[i].state != HOME_STATE_FAILED) {
            continue;  /* Already homing */
        }

        int32_t pos[PROTO_NUM_AXES];
        Motion_GetPositions(pos);

        s_ctx[i].state       = HOME_STATE_FAST_APPROACH;
        s_ctx[i].start_pos   = pos[i];
        s_ctx[i].debounce_cnt = 0;
        s_ctx[i].decel_cnt   = 0;
        s_ctx[i].result      = 0;

        s_active_mask |= (1U << i);

#ifndef SIMULATION_MODE
        StepTimer_SetDirection(i, s_ctx[i].params.direction);
        StepTimer_SetFrequency(i, (uint32_t)s_ctx[i].params.fast_speed * 1000);
#endif
    }

    if (s_active_mask) {
        Motion_SetState(SYSTEM_STATE_HOMING);
    }
}

void Homing_Stop(uint8_t axis_mask)
{
    for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
        if (!(axis_mask & (1U << i))) continue;

#ifndef SIMULATION_MODE
        StepTimer_SetFrequency(i, 0);
#endif
        s_ctx[i].state = HOME_STATE_IDLE;
        s_active_mask &= ~(1U << i);
    }

    if (s_active_mask == 0) {
        Motion_SetState(SYSTEM_STATE_IDLE);
    }
}

/* ========================================================================= */
/*  Update (250Hz from MotionTask)                                           */
/* ========================================================================= */

void Homing_Update(void)
{
    int32_t cur_pos[PROTO_NUM_AXES];
    Motion_GetPositions(cur_pos);

    for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
        if (!(s_active_mask & (1U << i))) continue;

        HomingContext *ctx = &s_ctx[i];
        uint8_t switch_raw = GPIO_HomeGetRaw(i);

        switch (ctx->state) {

        case HOME_STATE_FAST_APPROACH: {
            /* Move toward home switch at fast speed */
            int32_t delta = (int32_t)ctx->params.direction *
                            (int32_t)ctx->params.fast_speed *
                            MOTION_LOOP_PERIOD_MS;
            Motion_AddAxisDelta(i, delta);

            /* Check switch with debounce */
            if (switch_raw) {
                ctx->debounce_cnt++;
                if (ctx->debounce_cnt >= DEBOUNCE_TICKS) {
                    /* Switch confirmed — stop and decelerate */
#ifndef SIMULATION_MODE
                    StepTimer_SetFrequency(i, 0);
#endif
                    ctx->state = HOME_STATE_DECEL_STOP;
                    ctx->decel_cnt = 0;
                    ctx->debounce_cnt = 0;
                }
            } else {
                ctx->debounce_cnt = 0;
            }
            break;
        }

        case HOME_STATE_DECEL_STOP: {
            /* Wait for deceleration */
            ctx->decel_cnt++;
            if (ctx->decel_cnt >= DECEL_STOP_TICKS) {
                /* Start backoff in reverse direction */
                int32_t pos[PROTO_NUM_AXES];
                Motion_GetPositions(pos);
                ctx->start_pos = pos[i];
                ctx->target_pos = pos[i] -
                                  (int32_t)ctx->params.direction *
                                  (int32_t)ctx->params.backoff_dist;
                ctx->state = HOME_STATE_BACKOFF;

#ifndef SIMULATION_MODE
                StepTimer_SetDirection(i, -ctx->params.direction);
                StepTimer_SetFrequency(i, (uint32_t)ctx->params.fast_speed * 1000);
#endif
            }
            break;
        }

        case HOME_STATE_BACKOFF: {
            /* Move away from switch */
            int32_t delta = -((int32_t)ctx->params.direction *
                              (int32_t)ctx->params.fast_speed *
                              MOTION_LOOP_PERIOD_MS);
            Motion_AddAxisDelta(i, delta);

            /* Check if backoff distance reached */
            int32_t pos[PROTO_NUM_AXES];
            Motion_GetPositions(pos);
            int32_t moved = abs(pos[i] - ctx->start_pos);
            if (moved >= (int32_t)ctx->params.backoff_dist) {
#ifndef SIMULATION_MODE
                StepTimer_SetFrequency(i, 0);
                StepTimer_SetDirection(i, ctx->params.direction);
                StepTimer_SetFrequency(i, (uint32_t)ctx->params.slow_speed * 1000);
#endif
                ctx->state = HOME_STATE_SLOW_APPROACH;
                ctx->debounce_cnt = 0;
            }
            break;
        }

        case HOME_STATE_SLOW_APPROACH: {
            /* Move toward switch at slow speed */
            int32_t delta = (int32_t)ctx->params.direction *
                            (int32_t)ctx->params.slow_speed *
                            MOTION_LOOP_PERIOD_MS;
            Motion_AddAxisDelta(i, delta);

            /* Check switch with debounce */
            if (switch_raw) {
                ctx->debounce_cnt++;
                if (ctx->debounce_cnt >= DEBOUNCE_TICKS) {
#ifndef SIMULATION_MODE
                    StepTimer_SetFrequency(i, 0);
#endif
                    ctx->state = HOME_STATE_SET_ZERO;
                    ctx->debounce_cnt = 0;
                }
            } else {
                ctx->debounce_cnt = 0;
            }
            break;
        }

        case HOME_STATE_SET_ZERO:
            /* Set current position as zero + offset */
            Motion_SetAxisPosition(i, ctx->params.offset);
            ctx->result = 1;
            ctx->state = HOME_STATE_COMPLETE;

            /* Mark axis as homed */
            DriveControl_SetHomed(i, 1);
            s_active_mask &= ~(1U << i);

            /* If all axes done, return to IDLE */
            if (s_active_mask == 0) {
                Motion_SetState(SYSTEM_STATE_IDLE);
            }
            break;

        case HOME_STATE_COMPLETE:
        case HOME_STATE_FAILED:
        case HOME_STATE_IDLE:
        default:
            break;
        }
    }
}

/* ========================================================================= */
/*  Getters                                                                  */
/* ========================================================================= */

uint8_t Homing_IsActive(void)
{
    return s_active_mask != 0;
}

HomingState Homing_GetAxisState(uint8_t axis)
{
    if (axis >= PROTO_NUM_AXES) return HOME_STATE_IDLE;
    return s_ctx[axis].state;
}

uint8_t Homing_GetResult(uint8_t axis)
{
    if (axis >= PROTO_NUM_AXES) return 0;
    return s_ctx[axis].result;
}
