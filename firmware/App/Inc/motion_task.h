/**
 * @file motion_task.h
 * @brief Motion task — PVT interpolation and step pulse generation
 */

#ifndef MOTION_TASK_H
#define MOTION_TASK_H

#include "protocol_defs.h"
#include "protocol_commands.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * Get current system state.
 */
uint8_t Motion_GetState(void);

/**
 * Get current commanded positions (encoder steps, int32) for all axes.
 * Thread-safe via critical section.
 */
void Motion_GetPositions(int32_t positions[PROTO_NUM_AXES]);

/**
 * Get current axis positions as float array.
 * Thread-safe via critical section.
 *
 * In SIMULATION_MODE: returns commanded position (Perfect Servo).
 * In Real Mode: will read from hardware encoder counters.
 *
 * @param axis_pos_out  Output array of PROTO_NUM_AXES floats (encoder steps as float)
 */
void MotionTask_GetPosition(float *axis_pos_out);

/**
 * Notify motion task that new PVT data is available in the buffer.
 * Called by NetTask after pushing points.
 */
void Motion_NotifyDataReady(void);

/**
 * Set the system state (used by E-stop or external control).
 */
void Motion_SetState(uint8_t new_state);

/**
 * Start continuous jog on a single axis.
 * Called by NetTask when CMD_JOG_START received.
 *
 * @param axis       Axis index (0-5)
 * @param direction  +1 or -1
 * @param speed      Speed in steps/ms
 */
void Motion_JogStart(uint8_t axis, int8_t direction, uint16_t speed);

/**
 * Stop jogging. Returns to IDLE state.
 */
void Motion_JogStop(void);

/**
 * Get current velocities for all axes (steps/ms).
 * Thread-safe via critical section.
 */
void Motion_GetVelocities(int16_t vel[PROTO_NUM_AXES]);

/**
 * Check if motion completed (last PVT segment finished).
 * Returns 1 once and auto-clears.
 */
uint8_t Motion_IsComplete(void);

/**
 * Check if an alarm was raised by motion task.
 * Returns 1 if pending, fills axis and code. Auto-clears.
 */
uint8_t Motion_GetAlarmPending(uint8_t *axis_out, uint8_t *code_out);

/**
 * Request a MoveAbsolute with S-curve profile.
 * Called by CmdHandler. Processed on next MotionTask tick.
 */
void Motion_RequestMoveAbsolute(const MoveAbsoluteCmd *cmd);

/**
 * Directly set a single axis position (used by Homing to update commanded pos).
 * Thread-safe via critical section.
 */
void Motion_SetAxisPosition(uint8_t axis, int32_t position);

/**
 * Add delta to a single axis position (used by Homing during approach).
 * Thread-safe via critical section.
 */
void Motion_AddAxisDelta(uint8_t axis, int32_t delta);

#endif /* MOTION_TASK_H */
