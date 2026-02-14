/**
 * @file motion_task.h
 * @brief Motion task — step pulse generation (placeholder)
 */

#ifndef MOTION_TASK_H
#define MOTION_TASK_H

#include "protocol_defs.h"

/**
 * Get current system state.
 */
uint8_t Motion_GetState(void);

/**
 * Get current commanded positions (encoder steps) for all axes.
 */
void Motion_GetPositions(int32_t positions[PROTO_NUM_AXES]);

#endif /* MOTION_TASK_H */
