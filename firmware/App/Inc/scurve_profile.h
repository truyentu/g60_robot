/**
 * @file scurve_profile.h
 * @brief S-Curve 7-segment jerk-limited trajectory profile generator
 *
 * 7 segments: jerk+ → const accel → jerk- → cruise → jerk- → const decel → jerk+
 * Multi-axis sync: slowest axis sets total_time, others scale proportionally.
 */

#ifndef SCURVE_PROFILE_H
#define SCURVE_PROFILE_H

#include "protocol_defs.h"
#include "protocol_commands.h"
#include "cmd_handler.h"
#include <stdint.h>

/* ========================================================================= */
/*  Types                                                                    */
/* ========================================================================= */

/**
 * S-curve segments for a single axis.
 */
typedef struct {
    float T[7];          /* Duration of each of the 7 segments (seconds) */
    float total_time;    /* Total profile time (seconds)                 */
    float j_max;         /* Maximum jerk (steps/s³)                      */
    float a_max;         /* Maximum acceleration (steps/s²)              */
    float v_max;         /* Maximum velocity (steps/s)                   */
    float p_start;       /* Start position (steps)                       */
    float p_end;         /* End position (steps)                         */
    int8_t direction;    /* +1 or -1                                     */
} SCurveSegments;

/**
 * Multi-axis coordinated S-curve move.
 */
typedef struct {
    SCurveSegments axes[PROTO_NUM_AXES];
    float total_time;       /* Synchronized total time (seconds)  */
    float elapsed;          /* Current elapsed time (seconds)     */
    uint8_t active_mask;    /* Which axes are moving              */
    uint8_t complete;       /* 1 when move is complete            */
} SCurveMove;

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

/**
 * Plan an S-curve move from current positions to target.
 * Returns 0 on success, non-zero on error.
 *
 * @param move     Output: planned move profile
 * @param cur      Current positions (steps) for all axes
 * @param cmd      MoveAbsolute command (target + max_speed + accel)
 * @param cfgs     Axis configurations for limit checking
 */
int SCurve_Plan(SCurveMove *move, const int32_t cur[PROTO_NUM_AXES],
                const MoveAbsoluteCmd *cmd, const AxisConfig cfgs[PROTO_NUM_AXES]);

/**
 * Sample position at time t for all axes.
 *
 * @param move     Planned move
 * @param t        Time in seconds
 * @param pos      Output: positions for all axes
 */
void SCurve_Sample(const SCurveMove *move, float t, int32_t pos[PROTO_NUM_AXES]);

/**
 * Advance the move by dt seconds and get new positions/velocities.
 * Returns 1 when move is complete, 0 otherwise.
 *
 * @param move     Move state (elapsed is updated)
 * @param dt       Time step (seconds)
 * @param pos      Output: current positions (steps)
 * @param vel      Output: current velocities (steps/ms)
 */
int SCurve_Advance(SCurveMove *move, float dt, int32_t pos[PROTO_NUM_AXES],
                   int16_t vel[PROTO_NUM_AXES]);

#endif /* SCURVE_PROFILE_H */
