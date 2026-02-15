/**
 * @file scurve_profile.c
 * @brief S-Curve trajectory profile — powered by Ruckig
 *
 * Uses Ruckig (MIT) for true jerk-limited, time-optimal trajectory generation.
 * This file provides the same SCurve_Plan/Sample/Advance API but delegates
 * the actual trajectory computation to ruckig_wrapper.
 *
 * Previous implementation used quintic smoothstep (6t^5 - 15t^4 + 10t^3)
 * which did NOT produce true S-curve jerk profiles. Ruckig provides
 * mathematically correct 3rd-order trajectories with bounded jerk.
 */

#include "scurve_profile.h"
#include "ruckig_wrapper.h"
#include "board_config.h"
#include <math.h>
#include <string.h>

/* ========================================================================= */
/*  Internal state                                                           */
/* ========================================================================= */

static uint8_t s_ruckig_initialized = 0;

/* Last known velocities from Ruckig (steps/s) for status reporting */
static float s_last_vel[PROTO_NUM_AXES] = {0};

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

int SCurve_Plan(SCurveMove *move, const int32_t cur[PROTO_NUM_AXES],
                const MoveAbsoluteCmd *cmd, const AxisConfig cfgs[PROTO_NUM_AXES])
{
    memset(move, 0, sizeof(SCurveMove));

    /* Lazy init Ruckig on first call */
    if (!s_ruckig_initialized) {
        ruckig_init((float)MOTION_LOOP_PERIOD_MS / 1000.0f);
        s_ruckig_initialized = 1;
    }

    /* Check if any axis needs to move */
    uint8_t active = 0;
    for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
        if (cur[i] != cmd->position[i]) {
            active |= (1U << i);
        }
    }

    if (active == 0) {
        move->complete = 1;
        return 0;
    }

    /* Build Ruckig inputs */
    float cur_pos[RUCKIG_NUM_AXES];
    float target_pos[RUCKIG_NUM_AXES];
    float max_vel[RUCKIG_NUM_AXES];
    float max_acc[RUCKIG_NUM_AXES];
    float max_jerk[RUCKIG_NUM_AXES];

    for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
        cur_pos[i]    = (float)cur[i];
        target_pos[i] = (float)cmd->position[i];

        /* Convert from steps/ms to steps/s for Ruckig */
        float v_max = (float)cmd->max_speed * 1000.0f;
        float a_max = (float)cmd->accel * 1000.0f * 1000.0f;

        /* Per-axis limits from AxisConfig if available, else use command values */
        if (cfgs[i].max_velocity > 0) {
            v_max = fminf(v_max, (float)cfgs[i].max_velocity * 1000.0f);
        }
        if (cfgs[i].max_accel > 0) {
            a_max = fminf(a_max, (float)cfgs[i].max_accel * 1000.0f * 1000.0f);
        }

        max_vel[i]  = v_max;
        max_acc[i]  = a_max;
        max_jerk[i] = a_max * 10.0f;  /* j_max = 10 * a_max (aggressive but smooth) */
    }

    /* Plan with Ruckig — starts from rest (vel=0, acc=0) */
    RuckigResult res = ruckig_plan(cur_pos, NULL, NULL,
                                    target_pos, max_vel, max_acc, max_jerk);

    if (res == RUCKIG_ERROR) {
        move->complete = 1;
        return -1;
    }

    move->total_time = ruckig_get_duration();
    move->active_mask = active;
    move->elapsed = 0;
    move->complete = (res == RUCKIG_FINISHED) ? 1 : 0;

    /* Store start/end positions in axes for compatibility */
    for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
        move->axes[i].p_start = (float)cur[i];
        move->axes[i].p_end   = (float)cmd->position[i];
        move->axes[i].total_time = move->total_time;
    }

    return 0;
}

void SCurve_Sample(const SCurveMove *move, float t, int32_t pos[PROTO_NUM_AXES])
{
    (void)t;
    /* SCurve_Sample is not used with Ruckig — use SCurve_Advance instead.
     * If called, return last known positions from move struct. */
    for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
        if (move->active_mask & (1U << i)) {
            /* Interpolate linearly as fallback (shouldn't be called in normal flow) */
            float ratio = (move->total_time > 0) ? (t / move->total_time) : 1.0f;
            if (ratio < 0) ratio = 0;
            if (ratio > 1) ratio = 1;
            pos[i] = (int32_t)(move->axes[i].p_start +
                     ratio * (move->axes[i].p_end - move->axes[i].p_start));
        }
    }
}

int SCurve_Advance(SCurveMove *move, float dt, int32_t pos[PROTO_NUM_AXES],
                   int16_t vel[PROTO_NUM_AXES])
{
    if (move->complete) return 1;

    move->elapsed += dt;

    /* Get next trajectory point from Ruckig */
    float pos_f[RUCKIG_NUM_AXES];
    float vel_f[RUCKIG_NUM_AXES];

    RuckigResult res = ruckig_update(pos_f, vel_f, NULL);

    /* Convert outputs */
    for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
        pos[i] = (int32_t)pos_f[i];
        /* Velocity: convert steps/s → steps/ms for firmware convention */
        vel[i] = (int16_t)(vel_f[i] / 1000.0f);
        s_last_vel[i] = vel_f[i];
    }

    if (res == RUCKIG_FINISHED) {
        move->elapsed = move->total_time;
        move->complete = 1;
        /* Snap to exact target */
        for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
            pos[i] = (int32_t)move->axes[i].p_end;
            vel[i] = 0;
        }
        return 1;
    }

    if (res == RUCKIG_ERROR) {
        move->complete = 1;
        return 1;
    }

    return 0;  /* Still working */
}
