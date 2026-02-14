/**
 * @file app_tasks.h
 * @brief FreeRTOS task function prototypes
 */

#ifndef APP_TASKS_H
#define APP_TASKS_H

/**
 * Network task — UDP command reception and dispatch.
 * Priority: TASK_PRIORITY_NET (4)
 * Rate: event-driven (blocks on UDP recv)
 */
void NetTask(void *arg);

/**
 * Motion task — real-time step pulse generation.
 * Priority: TASK_PRIORITY_MOTION (7, highest app)
 * Rate: 1 kHz (1ms period)
 */
void MotionTask(void *arg);

/**
 * Status task — sends StatusPacket to PC via UDP.
 * Priority: TASK_PRIORITY_STATUS (2)
 * Rate: 100 Hz (10ms period)
 */
void StatusTask(void *arg);

#endif /* APP_TASKS_H */
