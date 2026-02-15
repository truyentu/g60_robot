/**
 * @file motion_buffer.c
 * @brief PVT Ring Buffer — thread-safe circular buffer implementation
 *
 * Lock-free for single-producer single-consumer (NetTask → MotionTask).
 * Critical sections used for multi-word reads/writes to ensure atomicity.
 */

#include "motion_buffer.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

/* ========================================================================= */
/*  Ring Buffer Storage                                                      */
/* ========================================================================= */

static PVTPoint  s_buffer[MOTION_BUFFER_CAPACITY];
static volatile uint16_t s_head  = 0;  /* Write index (NetTask) */
static volatile uint16_t s_tail  = 0;  /* Read index (MotionTask) */
static volatile uint16_t s_count = 0;  /* Number of items in buffer */

/* Mask for power-of-2 modulo */
#define BUFFER_MASK  (MOTION_BUFFER_CAPACITY - 1)

_Static_assert((MOTION_BUFFER_CAPACITY & BUFFER_MASK) == 0,
               "MOTION_BUFFER_CAPACITY must be a power of 2");

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

void MotionBuffer_Init(void)
{
    taskENTER_CRITICAL();
    s_head  = 0;
    s_tail  = 0;
    s_count = 0;
    taskEXIT_CRITICAL();
}

bool MotionBuffer_Push(const PVTPoint *pt)
{
    bool ok = false;

    taskENTER_CRITICAL();
    if (s_count < MOTION_BUFFER_CAPACITY) {
        memcpy(&s_buffer[s_head], pt, sizeof(PVTPoint));
        s_head = (s_head + 1) & BUFFER_MASK;
        s_count++;
        ok = true;
    }
    taskEXIT_CRITICAL();

    return ok;
}

bool MotionBuffer_Pop(PVTPoint *pt)
{
    bool ok = false;

    taskENTER_CRITICAL();
    if (s_count > 0) {
        memcpy(pt, &s_buffer[s_tail], sizeof(PVTPoint));
        s_tail = (s_tail + 1) & BUFFER_MASK;
        s_count--;
        ok = true;
    }
    taskEXIT_CRITICAL();

    return ok;
}

bool MotionBuffer_Peek(PVTPoint *pt)
{
    bool ok = false;

    taskENTER_CRITICAL();
    if (s_count > 0) {
        memcpy(pt, &s_buffer[s_tail], sizeof(PVTPoint));
        ok = true;
    }
    taskEXIT_CRITICAL();

    return ok;
}

uint16_t MotionBuffer_GetLevel(void)
{
    uint16_t level;
    taskENTER_CRITICAL();
    level = s_count;
    taskEXIT_CRITICAL();
    return level;
}

uint16_t MotionBuffer_GetFree(void)
{
    uint16_t free_slots;
    taskENTER_CRITICAL();
    free_slots = MOTION_BUFFER_CAPACITY - s_count;
    taskEXIT_CRITICAL();
    return free_slots;
}

bool MotionBuffer_IsLow(void)
{
    return MotionBuffer_GetLevel() < MOTION_BUFFER_LOW_MARK;
}

bool MotionBuffer_IsEmpty(void)
{
    return MotionBuffer_GetLevel() == 0;
}

bool MotionBuffer_IsFull(void)
{
    return MotionBuffer_GetLevel() >= MOTION_BUFFER_CAPACITY;
}

void MotionBuffer_Flush(void)
{
    taskENTER_CRITICAL();
    s_head  = 0;
    s_tail  = 0;
    s_count = 0;
    taskEXIT_CRITICAL();
}
