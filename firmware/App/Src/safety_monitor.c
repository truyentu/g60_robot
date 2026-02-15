/**
 * @file safety_monitor.c
 * @brief Safety monitoring implementation
 *
 * Reads hardware GPIO:
 *   - Mode selector (2-bit: T1/T2/AUT)
 *   - Deadman switch (pressed = OK for T1/T2)
 *   - Safety fence interlock (closed = OK for AUT)
 *
 * Speed limits:
 *   - T1 mode: max 250 steps/ms (approximately 250 mm/s)
 *   - T2/AUT: no additional speed limit from safety
 */

#include "safety_monitor.h"
#include "board_gpio.h"

/* T1 mode speed limit (steps/ms) */
#define T1_MAX_SPEED  250

/* ========================================================================= */
/*  State                                                                    */
/* ========================================================================= */

static OperatingMode s_mode = OP_MODE_T1;
static uint8_t s_deadman_pressed = 0;
static uint8_t s_fence_closed = 0;
static uint8_t s_estop_detected = 0;

/* ========================================================================= */
/*  Init                                                                     */
/* ========================================================================= */

void SafetyMonitor_Init(void)
{
    s_mode = OP_MODE_T1;
    s_deadman_pressed = 0;
    s_fence_closed = 0;
    s_estop_detected = 0;
}

/* ========================================================================= */
/*  Update (100Hz from StatusTask)                                           */
/* ========================================================================= */

void SafetyMonitor_Update(void)
{
    /* Read operating mode from hardware selector */
    uint8_t mode_val = GPIO_ReadOperatingMode();
    if (mode_val <= 2) {
        s_mode = (OperatingMode)mode_val;
    }

    /* Read safety signals */
    s_deadman_pressed = GPIO_DeadmanIsPressed();
    s_fence_closed = GPIO_SafetyFenceIsClosed();

    /* Check E-stop */
    s_estop_detected = GPIO_EstopIsActive();
}

/* ========================================================================= */
/*  Getters                                                                  */
/* ========================================================================= */

OperatingMode SafetyMonitor_GetMode(void)
{
    return s_mode;
}

uint16_t SafetyMonitor_ClampSpeed(uint16_t speed)
{
    if (s_mode == OP_MODE_T1 && speed > T1_MAX_SPEED) {
        return T1_MAX_SPEED;
    }
    return speed;
}

uint8_t SafetyMonitor_MotionPermitted(void)
{
    /* E-stop always blocks */
    if (s_estop_detected) return 0;

    switch (s_mode) {
    case OP_MODE_T1:
    case OP_MODE_T2:
        /* Teach modes require deadman switch */
        return s_deadman_pressed ? 1 : 0;

    case OP_MODE_AUT:
        /* Automatic mode requires safety fence */
        return s_fence_closed ? 1 : 0;

    default:
        return 0;
    }
}

uint8_t SafetyMonitor_IsEstop(void)
{
    return s_estop_detected;
}
