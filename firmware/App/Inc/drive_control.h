/**
 * @file drive_control.h
 * @brief Drive control — enable/disable sequence, alarm management, brake control
 *
 * Manages the enable/disable sequence for AC servo drives:
 *   check alarm → release brake → delay → enable drive → verify ready
 *
 * Tracks per-axis state: enabled, ready, alarm, brake, homed.
 */

#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include "protocol_defs.h"
#include <stdint.h>

/* ========================================================================= */
/*  Per-Axis Drive State                                                     */
/* ========================================================================= */

typedef struct {
    uint8_t enabled;    /* Software-requested enable state        */
    uint8_t ready;      /* Hardware ready signal (from GPIO)      */
    uint8_t alarm;      /* Hardware alarm signal (from GPIO)      */
    uint8_t brake;      /* 0 = engaged, 1 = released             */
    uint8_t homed;      /* 0 = not homed, 1 = homed              */
} DriveState;

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

/**
 * Initialize drive control module. All drives disabled, brakes engaged.
 */
void DriveControl_Init(void);

/**
 * Enable drives specified by axis_mask (bit0 = axis 0).
 * Sequence: check alarm → release brake → delay → enable → verify ready.
 * Returns 0 on success, non-zero on error (alarm active).
 */
int DriveControl_Enable(uint8_t axis_mask);

/**
 * Disable drives specified by axis_mask.
 * Sequence: disable drive → engage brake.
 * Returns 0 on success.
 */
int DriveControl_Disable(uint8_t axis_mask);

/**
 * Reset alarm for drives specified by axis_mask.
 * Clears internal alarm state. Does NOT auto-enable.
 * Returns 0 on success.
 */
int DriveControl_ResetAlarm(uint8_t axis_mask);

/**
 * Update drive states by reading GPIO inputs.
 * Should be called periodically (e.g. from StatusTask at 100Hz).
 */
void DriveControl_Update(void);

/**
 * Get bitmask of ready drives (bit0 = axis 0).
 */
uint8_t DriveControl_GetReadyMask(void);

/**
 * Get bitmask of alarmed drives.
 */
uint8_t DriveControl_GetAlarmMask(void);

/**
 * Get bitmask of enabled drives.
 */
uint8_t DriveControl_GetEnabledMask(void);

/**
 * Mark an axis as homed/not-homed. Called by Homing module.
 */
void DriveControl_SetHomed(uint8_t axis, uint8_t homed);

/**
 * Get bitmask of homed axes.
 */
uint8_t DriveControl_GetHomedMask(void);

/**
 * Get per-axis state.
 */
const DriveState *DriveControl_GetState(uint8_t axis);

/**
 * Run brake test sequence on a single axis.
 * Blocking call — only from task context.
 * Returns 0 on success, -1 on error.
 */
int DriveControl_BrakeTest(uint8_t axis);

#endif /* DRIVE_CONTROL_H */
