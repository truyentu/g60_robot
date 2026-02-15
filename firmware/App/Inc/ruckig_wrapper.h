/**
 * @file ruckig_wrapper.h
 * @brief C-callable wrapper for Ruckig jerk-limited trajectory generator
 *
 * Wraps Ruckig<PROTO_NUM_AXES> C++17 library into a C interface
 * so that motion_task.c (C) can call it.
 *
 * Usage:
 *   1. ruckig_init(cycle_time_s)    — call once at startup
 *   2. ruckig_plan(...)             — plan a new move
 *   3. ruckig_update(...)           — call every cycle (4ms) to get next state
 *   4. ruckig_is_done()             — check if trajectory finished
 *   5. ruckig_abort()               — cancel current trajectory
 */

#ifndef RUCKIG_WRAPPER_H
#define RUCKIG_WRAPPER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef RUCKIG_NUM_AXES
#define RUCKIG_NUM_AXES 6
#endif

/** Result codes matching Ruckig::Result */
typedef enum {
    RUCKIG_WORKING   = 0,   /**< Trajectory still in progress */
    RUCKIG_FINISHED  = 1,   /**< Trajectory reached target    */
    RUCKIG_ERROR     = -1,  /**< Planning or update error     */
} RuckigResult;

/**
 * Initialize the Ruckig trajectory generator.
 * Must be called once before any other ruckig_* function.
 *
 * @param cycle_time_s  Control loop period in seconds (e.g. 0.004 for 4ms)
 */
void ruckig_init(float cycle_time_s);

/**
 * Plan a new point-to-point move.
 *
 * @param cur_pos       Current position for each axis (steps)
 * @param cur_vel       Current velocity for each axis (steps/s), NULL = zeros
 * @param cur_acc       Current acceleration for each axis (steps/s²), NULL = zeros
 * @param target_pos    Target position for each axis (steps)
 * @param max_vel       Maximum velocity per axis (steps/s)
 * @param max_acc       Maximum acceleration per axis (steps/s²)
 * @param max_jerk      Maximum jerk per axis (steps/s³)
 * @return RUCKIG_WORKING on success, RUCKIG_ERROR on invalid input
 */
RuckigResult ruckig_plan(
    const float cur_pos[RUCKIG_NUM_AXES],
    const float cur_vel[RUCKIG_NUM_AXES],
    const float cur_acc[RUCKIG_NUM_AXES],
    const float target_pos[RUCKIG_NUM_AXES],
    const float max_vel[RUCKIG_NUM_AXES],
    const float max_acc[RUCKIG_NUM_AXES],
    const float max_jerk[RUCKIG_NUM_AXES]
);

/**
 * Advance the trajectory by one cycle.
 * Call this every cycle_time_s seconds (e.g. every 4ms).
 *
 * @param pos_out   Output: new position for each axis (steps)
 * @param vel_out   Output: new velocity for each axis (steps/s), NULL to skip
 * @param acc_out   Output: new acceleration for each axis (steps/s²), NULL to skip
 * @return RUCKIG_WORKING if still moving, RUCKIG_FINISHED if done, RUCKIG_ERROR on failure
 */
RuckigResult ruckig_update(
    float pos_out[RUCKIG_NUM_AXES],
    float vel_out[RUCKIG_NUM_AXES],
    float acc_out[RUCKIG_NUM_AXES]
);

/**
 * Abort the current trajectory immediately.
 * After calling this, ruckig_is_done() returns 1.
 */
void ruckig_abort(void);

/**
 * Check if the current trajectory is finished.
 * @return 1 if done or no trajectory active, 0 if still moving
 */
int ruckig_is_done(void);

/**
 * Get the total duration of the planned trajectory.
 * @return Duration in seconds, or 0 if no trajectory active
 */
float ruckig_get_duration(void);

#ifdef __cplusplus
}
#endif

#endif /* RUCKIG_WRAPPER_H */
