/**
 * @file motion_task.c
 * @brief Motion task — PVT buffer consumer (Hybrid Sim + Real Mode)
 *
 * 250 Hz loop (4ms): pops PVTPoints from the ring buffer.
 * SIMULATION_MODE: prints coordinates to debug UART.
 * Real Mode: calculates pulse frequency and drives hardware timers.
 */

#include "motion_task.h"
#include "motion_buffer.h"
#include "app_tasks.h"
#include "board_config.h"
#include "board_timers.h"
#include "board_gpio.h"
#include "cmd_handler.h"
#include "homing.h"
#include "scurve_profile.h"
#include "protocol_defs.h"
#include "protocol_commands.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ========================================================================= */
/*  Constants                                                                */
/* ========================================================================= */

/* MOTION_LOOP_PERIOD_MS and MOTION_LOOP_PERIOD_US defined in board_config.h */

/* IDLE message throttle: print once per second (250 ticks × 4ms = 1000ms) */
#define IDLE_PRINT_INTERVAL     250

/* LED toggle interval in loop ticks (500ms / 4ms = 125) */
#define LED_TOGGLE_TICKS        125

/* ========================================================================= */
/*  State                                                                    */
/* ========================================================================= */

static volatile uint8_t  s_state = SYSTEM_STATE_IDLE;
static volatile int32_t  s_positions[PROTO_NUM_AXES] = {0};
static volatile uint8_t  s_data_ready = 0;

/* Current segment being executed */
static PVTPoint  s_current_point;
static uint32_t  s_segment_elapsed_us = 0;
static uint8_t   s_segment_active = 0;

/* Previous positions — for delta calculation in Real Mode */
static int32_t   s_prev_positions[PROTO_NUM_AXES] = {0};

/* Previous segment end velocities — for cubic Hermite interpolation */
static int16_t   s_prev_velocities[PROTO_NUM_AXES] = {0};

/* Current velocities (computed each tick) for status reporting */
static volatile int16_t s_velocities[PROTO_NUM_AXES] = {0};

/* Motion complete flag — set when last PVT segment finishes */
static volatile uint8_t s_motion_complete_flag = 0;

/* Alarm pending — set by motion task, read by status task */
static volatile uint8_t  s_alarm_pending = 0;
static volatile uint8_t  s_alarm_axis = 0;
static volatile uint8_t  s_alarm_code = 0;

/* S-curve MoveAbsolute state */
static SCurveMove s_scurve_move;
static volatile uint8_t s_move_abs_active = 0;
static volatile uint8_t s_move_abs_requested = 0;
static MoveAbsoluteCmd  s_move_abs_cmd;

/* Jog state */
static volatile uint8_t  s_jog_axis      = 0;
static volatile int8_t   s_jog_direction  = 0;
static volatile uint16_t s_jog_speed      = 0;  /* steps/ms */
static volatile uint8_t  s_jog_requested  = 0;  /* flag: new jog command pending */
static volatile uint8_t  s_jog_stop_req   = 0;  /* flag: stop jog requested */

/* Jog debug print throttle (print every 250ms = 62 ticks × 4ms) */
#define JOG_PRINT_INTERVAL  62

/* Debug print buffer */
static char s_dbg[192];

uint8_t Motion_GetState(void)
{
    return s_state;
}

void Motion_GetPositions(int32_t positions[PROTO_NUM_AXES])
{
    taskENTER_CRITICAL();
    memcpy(positions, (const void *)s_positions, sizeof(s_positions));
    taskEXIT_CRITICAL();
}

void Motion_NotifyDataReady(void)
{
    s_data_ready = 1;
}

void Motion_SetState(uint8_t new_state)
{
    s_state = new_state;
}

void MotionTask_GetPosition(float *axis_pos_out)
{
    taskENTER_CRITICAL();
#ifdef SIMULATION_MODE
    /* Perfect Servo: actual position = commanded position */
    for (int i = 0; i < PROTO_NUM_AXES; i++) {
        axis_pos_out[i] = (float)s_positions[i];
    }
#else
    /* Real Mode: TODO — read from hardware encoder counters
     * e.g. axis_pos_out[i] = (float)__HAL_TIM_GET_COUNTER(&htimEncoder[i]);
     * For now, still use commanded position as fallback */
    for (int i = 0; i < PROTO_NUM_AXES; i++) {
        axis_pos_out[i] = (float)s_positions[i];
    }
#endif
    taskEXIT_CRITICAL();
}

/* ========================================================================= */
/*  Jog Control API (called from NetTask)                                    */
/* ========================================================================= */

void Motion_JogStart(uint8_t axis, int8_t direction, uint16_t speed)
{
    if (axis >= PROTO_NUM_AXES) return;

    s_jog_axis      = axis;
    s_jog_direction  = direction;
    s_jog_speed      = speed;
    s_jog_requested  = 1;
    s_jog_stop_req   = 0;

    snprintf(s_dbg, sizeof(s_dbg),
             "JOG REQ: axis=%u dir=%d speed=%u steps/ms\r\n",
             (unsigned)axis, (int)direction, (unsigned)speed);
    Board_Debug_Print(s_dbg);
}

void Motion_JogStop(void)
{
    s_jog_stop_req = 1;
    Board_Debug_Print("JOG STOP REQ\r\n");
}

/* ========================================================================= */
/*  Velocity + Motion Complete API                                           */
/* ========================================================================= */

void Motion_GetVelocities(int16_t vel[PROTO_NUM_AXES])
{
    taskENTER_CRITICAL();
    memcpy(vel, (const void *)s_velocities, sizeof(s_velocities));
    taskEXIT_CRITICAL();
}

uint8_t Motion_IsComplete(void)
{
    uint8_t flag = s_motion_complete_flag;
    s_motion_complete_flag = 0;  /* auto-clear on read */
    return flag;
}

uint8_t Motion_GetAlarmPending(uint8_t *axis_out, uint8_t *code_out)
{
    if (!s_alarm_pending) return 0;
    if (axis_out) *axis_out = s_alarm_axis;
    if (code_out) *code_out = s_alarm_code;
    s_alarm_pending = 0;
    return 1;
}

/* ========================================================================= */
/*  Internal: Soft Limit Check                                               */
/* ========================================================================= */

/**
 * Check if target positions are within software limits.
 * Returns 0 if OK, or sets alarm and returns -1 if any axis out of range.
 */
static int check_axis_limits(const int32_t target[PROTO_NUM_AXES])
{
    for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
        const AxisConfig *cfg = CmdHandler_GetAxisConfig(i);
        if (cfg == NULL) continue;
        if (target[i] < cfg->min_limit || target[i] > cfg->max_limit) {
            s_alarm_pending = 1;
            s_alarm_axis = i;
            s_alarm_code = ALARM_SOFT_LIMIT;
            return -1;
        }
    }
    return 0;
}

/* ========================================================================= */
/*  MoveAbsolute Request (called from CmdHandler)                            */
/* ========================================================================= */

void Motion_RequestMoveAbsolute(const MoveAbsoluteCmd *cmd)
{
    memcpy(&s_move_abs_cmd, cmd, sizeof(MoveAbsoluteCmd));
    s_move_abs_requested = 1;
}

void Motion_SetAxisPosition(uint8_t axis, int32_t position)
{
    if (axis >= PROTO_NUM_AXES) return;
    taskENTER_CRITICAL();
    s_positions[axis] = position;
    taskEXIT_CRITICAL();
}

void Motion_AddAxisDelta(uint8_t axis, int32_t delta)
{
    if (axis >= PROTO_NUM_AXES) return;
    taskENTER_CRITICAL();
    s_positions[axis] += delta;
    taskEXIT_CRITICAL();
}

/* ========================================================================= */
/*  Internal: Execute PVT segment (Simulation Mode)                          */
/* ========================================================================= */

/**
 * Start executing a new PVT segment.
 */
static void start_segment(const PVTPoint *pt)
{
    /* Check soft limits before starting */
    if (check_axis_limits(pt->position) != 0) {
        s_state = SYSTEM_STATE_ALARM;
        return;
    }

    /* Save current positions as start of this segment */
    taskENTER_CRITICAL();
    memcpy(s_prev_positions, (const void *)s_positions, sizeof(s_prev_positions));
    taskEXIT_CRITICAL();

    memcpy(&s_current_point, pt, sizeof(PVTPoint));
    s_segment_elapsed_us = 0;
    s_segment_active = 1;

#ifndef SIMULATION_MODE
    /* Real Mode: calculate pulse frequency and direction for each axis */
    for (int i = 0; i < PROTO_NUM_AXES; i++) {
        int32_t delta = pt->position[i] - s_prev_positions[i];
        if (delta != 0) {
            /* Following error check: verify pulse frequency within limits */
            uint32_t abs_delta = (uint32_t)abs(delta);
            uint32_t freq = StepTimer_CalcFreq(abs_delta, pt->duration_us);
            if (freq > DEFAULT_MAX_PULSE_FREQ) {
                /* Pulse rate too high — alarm */
                s_alarm_pending = 1;
                s_alarm_axis = (uint8_t)i;
                s_alarm_code = ALARM_FOLLOWING_ERROR;
                s_segment_active = 0;
                s_state = SYSTEM_STATE_ALARM;
                return;
            }

            /* Set direction GPIO before starting pulses (DIR_SETUP_TIME_US) */
            StepTimer_SetDirection((uint8_t)i, (delta > 0) ? 1 : -1);
            StepTimer_SetFrequency((uint8_t)i, freq);
        } else {
            StepTimer_SetFrequency((uint8_t)i, 0); /* No motion on this axis */
        }
    }
#endif
}

/**
 * Advance the current segment by one loop tick (4ms = 4000us).
 * Returns true when the segment is complete.
 */
static bool advance_segment(void)
{
    s_segment_elapsed_us += MOTION_LOOP_PERIOD_US;

    if (s_segment_elapsed_us >= s_current_point.duration_us) {
        /* Segment complete — snap to target position */
        taskENTER_CRITICAL();
        for (int i = 0; i < PROTO_NUM_AXES; i++) {
            s_positions[i] = s_current_point.position[i];
        }
        taskEXIT_CRITICAL();

        /* Handle sync I/O at segment end */
        if (PVT_HAS_SYNC_IO(&s_current_point)) {
            GPIO_WriteDigitalOutputs(s_current_point.io_mask, s_current_point.io_value);
        }

        /* Save end velocities for next segment's cubic Hermite */
        memcpy(s_prev_velocities, s_current_point.velocity, sizeof(s_prev_velocities));

        /* Compute velocity = 0 at segment end (snap) */
        taskENTER_CRITICAL();
        for (int i = 0; i < PROTO_NUM_AXES; i++) {
            s_velocities[i] = s_current_point.velocity[i];
        }
        taskEXIT_CRITICAL();

#ifndef SIMULATION_MODE
        /* Real Mode: stop all pulse timers at segment end */
        for (int i = 0; i < PROTO_NUM_AXES; i++) {
            StepTimer_SetFrequency((uint8_t)i, 0);
        }
#endif

        s_segment_active = 0;
        return true;
    }

    /* Cubic Hermite PVT interpolation within segment
     * p(t) = h00*p0 + h10*v0*T + h01*p1 + h11*v1*T
     * where t in [0,1], T = segment duration (ms) */
    if (s_current_point.duration_us > 0) {
        float t = (float)s_segment_elapsed_us / (float)s_current_point.duration_us;
        float t2 = t * t;
        float t3 = t2 * t;

        /* Hermite basis functions */
        float h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
        float h10 = t3 - 2.0f * t2 + t;
        float h01 = -2.0f * t3 + 3.0f * t2;
        float h11 = t3 - t2;

        /* T = segment duration in ms (velocity units are steps/ms) */
        float T_ms = (float)s_current_point.duration_us / 1000.0f;

        int32_t prev_pos_snap[PROTO_NUM_AXES];
        taskENTER_CRITICAL();
        memcpy(prev_pos_snap, (const void *)s_positions, sizeof(prev_pos_snap));
        taskEXIT_CRITICAL();

        taskENTER_CRITICAL();
        for (int i = 0; i < PROTO_NUM_AXES; i++) {
            float p0 = (float)s_prev_positions[i];
            float p1 = (float)s_current_point.position[i];
            float v0 = (float)s_prev_velocities[i];
            float v1 = (float)s_current_point.velocity[i];

            float pos = h00 * p0 + h10 * v0 * T_ms + h01 * p1 + h11 * v1 * T_ms;
            s_positions[i] = (int32_t)pos;

            /* Velocity = (pos - prev_pos) / dt_ms */
            s_velocities[i] = (int16_t)((s_positions[i] - prev_pos_snap[i]) / MOTION_LOOP_PERIOD_MS);
        }
        taskEXIT_CRITICAL();
    }

    return false;
}

/**
 * Print current execution state (Simulation Mode only).
 * "EXEC: Time=<ms> | Pos=[p0, p1, p2, p3, p4, p5] | BufLvl=<n>"
 */
#ifdef SIMULATION_MODE
static void print_exec_status(void)
{
    /* Convert steps to approximate position (0.001 scale factor for display) */
    float p[PROTO_NUM_AXES];
    taskENTER_CRITICAL();
    for (int i = 0; i < PROTO_NUM_AXES; i++) {
        p[i] = (float)s_positions[i] * 0.001f;
    }
    taskEXIT_CRITICAL();

    uint16_t buf_lvl = MotionBuffer_GetLevel();

    snprintf(s_dbg, sizeof(s_dbg),
             "EXEC: Time=%u ms | Pos=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] | BufLvl=%u\r\n",
             (unsigned)(s_current_point.duration_us / 1000),
             (double)p[0], (double)p[1], (double)p[2],
             (double)p[3], (double)p[4], (double)p[5],
             (unsigned)buf_lvl);
    Board_Debug_Print(s_dbg);
}
#endif /* SIMULATION_MODE */

/* ========================================================================= */
/*  MotionTask — 250Hz Loop (Hybrid Sim + Real)                              */
/* ========================================================================= */

void MotionTask(void *arg)
{
    (void)arg;

    /* Initialize the motion buffer */
    MotionBuffer_Init();

    /* Initialize homing module */
    Homing_Init();

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t idle_counter = 0;
    uint32_t led_counter = 0;

    for (;;) {
        /* 4ms period (250 Hz simulation loop) */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MOTION_LOOP_PERIOD_MS));

        /* ---- State Machine ---- */
        switch (s_state) {

        case SYSTEM_STATE_IDLE:
            /* Check if jog requested */
            if (s_jog_requested) {
                s_jog_requested = 0;
                s_state = SYSTEM_STATE_JOGGING;
                idle_counter = 0;
#ifndef SIMULATION_MODE
                StepTimer_SetDirection(s_jog_axis, s_jog_direction);
                StepTimer_SetFrequency(s_jog_axis, (uint32_t)s_jog_speed * 1000);
#endif
                Board_Debug_Print("State -> JOGGING\r\n");
                break;
            }
            /* Check if MoveAbsolute requested */
            if (s_move_abs_requested) {
                s_move_abs_requested = 0;
                int32_t cur[PROTO_NUM_AXES];
                Motion_GetPositions(cur);

                /* Check soft limits for target */
                if (check_axis_limits(s_move_abs_cmd.position) != 0) {
                    s_state = SYSTEM_STATE_ALARM;
                    Board_Debug_Print("MoveAbs: SOFT LIMIT\r\n");
                    break;
                }

                /* Build axis configs array */
                AxisConfig cfgs[PROTO_NUM_AXES];
                for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
                    const AxisConfig *c = CmdHandler_GetAxisConfig(i);
                    if (c) cfgs[i] = *c; else memset(&cfgs[i], 0, sizeof(AxisConfig));
                }

                if (SCurve_Plan(&s_scurve_move, cur, &s_move_abs_cmd, cfgs) == 0 &&
                    !s_scurve_move.complete) {
                    s_move_abs_active = 1;
                    s_state = SYSTEM_STATE_MOVING;
                    idle_counter = 0;
                    Board_Debug_Print("State -> MOVING (MoveAbs S-curve)\r\n");
                }
                break;
            }
            /* Check if PVT data arrived and we should start moving */
            if (s_data_ready && !MotionBuffer_IsEmpty()) {
                s_state = SYSTEM_STATE_MOVING;
                s_data_ready = 0;
                idle_counter = 0;
                Board_Debug_Print("State -> MOVING\r\n");
            } else {
                /* Throttled idle message: once per second */
                if (++idle_counter >= IDLE_PRINT_INTERVAL) {
                    idle_counter = 0;
                    Board_Debug_Print("IDLE: Waiting for data... (BufLvl=0)\r\n");
                }
            }
            break;

        case SYSTEM_STATE_MOVING:
            /* S-curve MoveAbsolute active? */
            if (s_move_abs_active) {
                int32_t pos[PROTO_NUM_AXES];
                int16_t vel[PROTO_NUM_AXES];
                float dt_s = (float)MOTION_LOOP_PERIOD_MS / 1000.0f;

                int done = SCurve_Advance(&s_scurve_move, dt_s, pos, vel);

                taskENTER_CRITICAL();
                for (int i = 0; i < PROTO_NUM_AXES; i++) {
                    s_positions[i] = pos[i];
                    s_velocities[i] = vel[i];
                }
                taskEXIT_CRITICAL();

                if (done) {
                    s_move_abs_active = 0;
                    s_state = SYSTEM_STATE_IDLE;
                    s_motion_complete_flag = 1;
                    memset((void *)s_velocities, 0, sizeof(s_velocities));
                    idle_counter = 0;
                    Board_Debug_Print("State -> IDLE (MoveAbs complete)\r\n");
                }
                break;
            }
            /* PVT buffer mode: If no segment active, pop the next one */
            if (!s_segment_active) {
                PVTPoint pt;
                if (MotionBuffer_Pop(&pt)) {
                    start_segment(&pt);
                } else {
                    /* Buffer empty — motion complete */
                    s_state = SYSTEM_STATE_IDLE;
                    idle_counter = 0;
                    Board_Debug_Print("State -> IDLE (buffer empty)\r\n");
                    break;
                }
            }

            /* Advance the active segment */
            if (s_segment_active) {
                bool done = advance_segment();

#ifdef SIMULATION_MODE
                /* Print execution status every tick while moving */
                print_exec_status();
#endif

                if (done) {
                    if (PVT_IS_LAST_SEGMENT(&s_current_point)) {
                        s_state = SYSTEM_STATE_IDLE;
                        s_motion_complete_flag = 1;
                        memset((void *)s_velocities, 0, sizeof(s_velocities));
                        idle_counter = 0;
                        Board_Debug_Print("State -> IDLE (last segment)\r\n");
                    }
                    /* Otherwise, next iteration will pop another segment */
                }
            }
            break;

        case SYSTEM_STATE_JOGGING: {
            /* Check for stop request */
            if (s_jog_stop_req) {
                s_jog_stop_req = 0;
#ifndef SIMULATION_MODE
                StepTimer_SetFrequency(s_jog_axis, 0);
#endif
                s_state = SYSTEM_STATE_IDLE;
                idle_counter = 0;
                Board_Debug_Print("State -> IDLE (jog stopped)\r\n");
                break;
            }

            /* Check for new jog command (axis/direction/speed change) */
            if (s_jog_requested) {
                s_jog_requested = 0;
#ifndef SIMULATION_MODE
                /* Stop previous axis, start new */
                StepTimer_SetFrequency(s_jog_axis, 0);
                StepTimer_SetDirection(s_jog_axis, s_jog_direction);
                StepTimer_SetFrequency(s_jog_axis, (uint32_t)s_jog_speed * 1000);
#endif
            }

            /* Update position: pos += direction * speed * dt
             * speed is in steps/ms, dt = MOTION_LOOP_PERIOD_MS */
            {
                int32_t new_pos;
                taskENTER_CRITICAL();
                new_pos = s_positions[s_jog_axis] +
                          (int32_t)s_jog_direction *
                          (int32_t)s_jog_speed *
                          MOTION_LOOP_PERIOD_MS;
                taskEXIT_CRITICAL();

                /* Soft limit check for jog */
                const AxisConfig *cfg = CmdHandler_GetAxisConfig(s_jog_axis);
                if (cfg && (new_pos < cfg->min_limit || new_pos > cfg->max_limit)) {
                    /* Stop jog at limit */
#ifndef SIMULATION_MODE
                    StepTimer_SetFrequency(s_jog_axis, 0);
#endif
                    s_state = SYSTEM_STATE_IDLE;
                    idle_counter = 0;
                    Board_Debug_Print("JOG stopped at soft limit\r\n");
                    break;
                }

                taskENTER_CRITICAL();
                s_positions[s_jog_axis] = new_pos;
                s_velocities[s_jog_axis] = (int16_t)(s_jog_direction * (int16_t)s_jog_speed);
                taskEXIT_CRITICAL();
            }

#ifdef SIMULATION_MODE
            {
                static uint32_t jog_print_cnt = 0;
                if (++jog_print_cnt >= JOG_PRINT_INTERVAL) {
                    jog_print_cnt = 0;
                    float p = (float)s_positions[s_jog_axis] * 0.001f;
                    snprintf(s_dbg, sizeof(s_dbg),
                             "JOG: axis=%u dir=%d spd=%u | Pos=%.3f\r\n",
                             (unsigned)s_jog_axis, (int)s_jog_direction,
                             (unsigned)s_jog_speed, (double)p);
                    Board_Debug_Print(s_dbg);
                }
            }
#endif
            break;
        }

        case SYSTEM_STATE_ESTOP:
            /* E-stop: flush buffer, stop jog, freeze */
            if (s_segment_active) {
                s_segment_active = 0;
            }
            s_jog_stop_req = 0;
            s_jog_requested = 0;
#ifndef SIMULATION_MODE
            StepTimer_StopAll();
#endif
            MotionBuffer_Flush();
            break;

        case SYSTEM_STATE_HOLD:
            /* Hold: pause segment but don't flush */
            break;

        case SYSTEM_STATE_HOMING:
            /* Run homing state machine at 250Hz */
            Homing_Update();
            break;

        case SYSTEM_STATE_ALARM:
            /* Alarm: stop all motion, wait for reset */
#ifndef SIMULATION_MODE
            StepTimer_StopAll();
#endif
            if (s_segment_active) {
                s_segment_active = 0;
            }
            break;

        default:
            break;
        }

        /* Heartbeat: toggle LED every 500ms */
        if (++led_counter >= LED_TOGGLE_TICKS) {
            led_counter = 0;
            Board_LED_Toggle();
        }
    }
}
