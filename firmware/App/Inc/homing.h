/**
 * @file homing.h
 * @brief Homing state machine — per-axis home search with debounce
 *
 * State machine per axis:
 *   IDLE → FAST_APPROACH → DECEL_STOP → BACKOFF → SLOW_APPROACH → SET_ZERO → COMPLETE
 *
 * Called from MotionTask at 250Hz when in SYSTEM_STATE_HOMING.
 */

#ifndef HOMING_H
#define HOMING_H

#include "protocol_defs.h"
#include "protocol_commands.h"
#include <stdint.h>

/* ========================================================================= */
/*  Homing States                                                            */
/* ========================================================================= */

typedef enum {
    HOME_STATE_IDLE = 0,
    HOME_STATE_FAST_APPROACH,
    HOME_STATE_DECEL_STOP,
    HOME_STATE_BACKOFF,
    HOME_STATE_SLOW_APPROACH,
    HOME_STATE_SET_ZERO,
    HOME_STATE_COMPLETE,
    HOME_STATE_FAILED
} HomingState;

/* ========================================================================= */
/*  Homing Parameters (per axis)                                             */
/* ========================================================================= */

typedef struct {
    int8_t   direction;       /* Search direction: +1 or -1        */
    uint16_t fast_speed;      /* Fast approach speed (steps/ms)    */
    uint16_t slow_speed;      /* Slow approach speed (steps/ms)    */
    int32_t  offset;          /* Post-home offset (steps)          */
    uint16_t backoff_dist;    /* Backoff distance (steps)          */
    uint8_t  use_index;       /* 0: no, 1: use index pulse         */
} HomingParams;

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

/**
 * Initialize homing module with default parameters.
 */
void Homing_Init(void);

/**
 * Set homing parameters for a single axis.
 */
void Homing_SetParams(uint8_t axis, const void *params);

/**
 * Start homing for axes specified by mask. method: 0=limit_switch.
 */
void Homing_Start(uint8_t axis_mask, uint8_t method);

/**
 * Stop (abort) homing for axes specified by mask.
 */
void Homing_Stop(uint8_t axis_mask);

/**
 * Update homing state machine — call from MotionTask at 250Hz.
 */
void Homing_Update(void);

/**
 * Returns non-zero if any axis is currently homing.
 */
uint8_t Homing_IsActive(void);

/**
 * Get homing state for a specific axis.
 */
HomingState Homing_GetAxisState(uint8_t axis);

/**
 * Get result for completed axis: 0=fail, 1=success.
 */
uint8_t Homing_GetResult(uint8_t axis);

#endif /* HOMING_H */
