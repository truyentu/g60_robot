/**
 * @file motion_task.c
 * @brief Motion task — real-time step pulse generation (placeholder)
 *
 * Phase 1: 1ms periodic loop, placeholder for step generation.
 * Phase 2: PVT interpolation, jog mode, homing.
 */

#include "motion_task.h"
#include "app_tasks.h"
#include "board_config.h"
#include "protocol_defs.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

#include <string.h>

/* ========================================================================= */
/*  State                                                                    */
/* ========================================================================= */

static volatile uint8_t  s_state = SYSTEM_STATE_IDLE;
static volatile int32_t  s_positions[PROTO_NUM_AXES] = {0};

uint8_t Motion_GetState(void)
{
    return s_state;
}

void Motion_GetPositions(int32_t positions[PROTO_NUM_AXES])
{
    /* TODO: Use critical section or atomic read for multi-word data */
    taskENTER_CRITICAL();
    memcpy(positions, (const void *)s_positions, sizeof(s_positions));
    taskEXIT_CRITICAL();
}

/* ========================================================================= */
/*  MotionTask — 1kHz Loop                                                   */
/* ========================================================================= */

void MotionTask(void *arg)
{
    (void)arg;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        /* 1ms period (1 kHz servo loop rate) */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));

        /*
         * TODO (Phase 2): Implement actual motion control here:
         *
         * 1. Check system state (IDLE, JOGGING, MOVING, HOMING, etc.)
         * 2. If JOGGING: update position based on jog direction + speed
         * 3. If MOVING: interpolate next PVT point, compute step pulses
         * 4. If HOMING: run homing state machine
         * 5. Output step pulses via hardware timer
         * 6. Read encoder feedback
         * 7. Check following error
         * 8. Update s_positions[]
         */

        /* Heartbeat: toggle LED every 500ms to show task is alive */
        static uint32_t led_counter = 0;
        if (++led_counter >= 500) {
            led_counter = 0;
            Board_LED_Toggle();
        }
    }
}
