/**
 * @file motion_buffer.h
 * @brief PVT Ring Buffer — thread-safe circular buffer for motion points
 *
 * NetTask pushes PVTPoints from UDP packets.
 * MotionTask pops them at 1kHz for interpolation.
 *
 * Thread safety: FreeRTOS critical sections (safe for ISR-priority tasks).
 */

#ifndef MOTION_BUFFER_H
#define MOTION_BUFFER_H

#include "protocol_commands.h"
#include <stdint.h>
#include <stdbool.h>

/* Buffer capacity — power of 2 for efficient modulo */
#define MOTION_BUFFER_CAPACITY  512

/* Buffer low-water mark — triggers RSP_BUFFER_LOW to PC */
#define MOTION_BUFFER_LOW_MARK  64

/**
 * Initialize the motion buffer. Call once at startup.
 */
void MotionBuffer_Init(void);

/**
 * Push a PVTPoint into the buffer (called by NetTask).
 * Thread-safe via critical section.
 *
 * @param pt  Pointer to PVTPoint to copy into buffer
 * @return true if pushed, false if buffer full
 */
bool MotionBuffer_Push(const PVTPoint *pt);

/**
 * Pop a PVTPoint from the buffer (called by MotionTask).
 * Thread-safe via critical section.
 *
 * @param pt  Output: copied PVTPoint
 * @return true if popped, false if buffer empty
 */
bool MotionBuffer_Pop(PVTPoint *pt);

/**
 * Peek at the next PVTPoint without removing it.
 *
 * @param pt  Output: copied PVTPoint
 * @return true if data available, false if empty
 */
bool MotionBuffer_Peek(PVTPoint *pt);

/**
 * Get number of points currently in the buffer.
 */
uint16_t MotionBuffer_GetLevel(void);

/**
 * Get remaining free space in the buffer.
 */
uint16_t MotionBuffer_GetFree(void);

/**
 * Check if buffer level is below the low-water mark.
 */
bool MotionBuffer_IsLow(void);

/**
 * Check if buffer is empty.
 */
bool MotionBuffer_IsEmpty(void);

/**
 * Check if buffer is full.
 */
bool MotionBuffer_IsFull(void);

/**
 * Flush (clear) all points in the buffer.
 * Used on E-stop or motion abort.
 */
void MotionBuffer_Flush(void);

#endif /* MOTION_BUFFER_H */
