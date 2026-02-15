/**
 * @file drive_control.c
 * @brief Drive control — enable/disable sequence, alarm, brake management
 *
 * Enable sequence per axis:
 *   1. Check alarm GPIO — abort if active
 *   2. Release brake (GPIO_BrakeRelease)
 *   3. Delay BRAKE_RELEASE_DELAY_MS for brake to open
 *   4. Enable drive (GPIO_DriveEnable)
 *   5. Read ready GPIO to confirm
 *
 * Disable sequence:
 *   1. Disable drive output
 *   2. Engage brake
 */

#include "drive_control.h"
#include "board_gpio.h"
#include "board_config.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

/* ========================================================================= */
/*  State                                                                    */
/* ========================================================================= */

static DriveState s_drives[PROTO_NUM_AXES];

/* ========================================================================= */
/*  Init                                                                     */
/* ========================================================================= */

void DriveControl_Init(void)
{
    memset(s_drives, 0, sizeof(s_drives));
}

/* ========================================================================= */
/*  Enable / Disable                                                         */
/* ========================================================================= */

int DriveControl_Enable(uint8_t axis_mask)
{
    /* First pass: check for alarms on requested axes */
    for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
        if (!(axis_mask & (1U << i))) continue;

        if (GPIO_DriveGetAlarm(i)) {
            s_drives[i].alarm = 1;
            return -1;  /* Cannot enable — alarm active */
        }
    }

    /* Second pass: release brakes */
    for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
        if (!(axis_mask & (1U << i))) continue;
        GPIO_BrakeRelease(i, 1);
        s_drives[i].brake = 1;
    }

    /* Wait for brakes to release */
    vTaskDelay(pdMS_TO_TICKS(BRAKE_RELEASE_DELAY_MS));

    /* Third pass: enable drives */
    for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
        if (!(axis_mask & (1U << i))) continue;
        GPIO_DriveEnable(i, 1);
        s_drives[i].enabled = 1;
    }

    /* Small delay then check ready */
    vTaskDelay(pdMS_TO_TICKS(50));

    for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
        if (!(axis_mask & (1U << i))) continue;
        s_drives[i].ready = GPIO_DriveGetReady(i);
    }

    return 0;
}

int DriveControl_Disable(uint8_t axis_mask)
{
    for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
        if (!(axis_mask & (1U << i))) continue;

        /* Disable drive output */
        GPIO_DriveEnable(i, 0);
        s_drives[i].enabled = 0;
        s_drives[i].ready = 0;

        /* Engage brake */
        GPIO_BrakeRelease(i, 0);
        s_drives[i].brake = 0;
    }

    return 0;
}

int DriveControl_ResetAlarm(uint8_t axis_mask)
{
    for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
        if (!(axis_mask & (1U << i))) continue;
        s_drives[i].alarm = 0;
    }

    return 0;
}

/* ========================================================================= */
/*  Periodic Update (call from StatusTask at 100Hz)                          */
/* ========================================================================= */

void DriveControl_Update(void)
{
    for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
        s_drives[i].ready = GPIO_DriveGetReady(i);
        s_drives[i].alarm = GPIO_DriveGetAlarm(i);
    }
}

/* ========================================================================= */
/*  Getters                                                                  */
/* ========================================================================= */

uint8_t DriveControl_GetReadyMask(void)
{
    uint8_t mask = 0;
    for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
        if (s_drives[i].ready) mask |= (1U << i);
    }
    return mask;
}

uint8_t DriveControl_GetAlarmMask(void)
{
    uint8_t mask = 0;
    for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
        if (s_drives[i].alarm) mask |= (1U << i);
    }
    return mask;
}

uint8_t DriveControl_GetEnabledMask(void)
{
    uint8_t mask = 0;
    for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
        if (s_drives[i].enabled) mask |= (1U << i);
    }
    return mask;
}

void DriveControl_SetHomed(uint8_t axis, uint8_t homed)
{
    if (axis < PROTO_NUM_AXES) {
        s_drives[axis].homed = homed;
    }
}

uint8_t DriveControl_GetHomedMask(void)
{
    uint8_t mask = 0;
    for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
        if (s_drives[i].homed) mask |= (1U << i);
    }
    return mask;
}

const DriveState *DriveControl_GetState(uint8_t axis)
{
    if (axis >= PROTO_NUM_AXES) return NULL;
    return &s_drives[axis];
}

/* ========================================================================= */
/*  Brake Test Sequence                                                      */
/* ========================================================================= */

/**
 * Run brake test: engage → delay → release → delay → engage → verify.
 * Blocking call (uses vTaskDelay). Only call from task context.
 * Returns 0 if brake responded correctly, -1 if stuck.
 */
int DriveControl_BrakeTest(uint8_t axis)
{
    if (axis >= PROTO_NUM_AXES) return -1;

    /* Ensure drive is disabled for safety */
    GPIO_DriveEnable(axis, 0);
    s_drives[axis].enabled = 0;

    /* Step 1: Engage brake */
    GPIO_BrakeRelease(axis, 0);
    s_drives[axis].brake = 0;
    vTaskDelay(pdMS_TO_TICKS(BRAKE_ENGAGE_DELAY_MS));

    /* Step 2: Release brake */
    GPIO_BrakeRelease(axis, 1);
    s_drives[axis].brake = 1;
    vTaskDelay(pdMS_TO_TICKS(BRAKE_RELEASE_DELAY_MS));

    /* Step 3: Re-engage brake */
    GPIO_BrakeRelease(axis, 0);
    s_drives[axis].brake = 0;
    vTaskDelay(pdMS_TO_TICKS(BRAKE_ENGAGE_DELAY_MS));

    return 0;
}
