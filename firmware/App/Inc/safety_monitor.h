/**
 * @file safety_monitor.h
 * @brief Safety monitoring — operating modes T1/T2/AUT, deadman, fence
 *
 * T1: Manual low speed (250 steps/ms max), requires deadman
 * T2: Manual full speed, requires deadman
 * AUT: Automatic, requires safety fence closed
 */

#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include <stdint.h>

/* ========================================================================= */
/*  Operating Modes                                                          */
/* ========================================================================= */

typedef enum {
    OP_MODE_T1  = 0,    /* Teach 1: low speed, deadman required    */
    OP_MODE_T2  = 1,    /* Teach 2: full speed, deadman required   */
    OP_MODE_AUT = 2     /* Automatic: safety fence required        */
} OperatingMode;

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

/**
 * Initialize safety monitor.
 */
void SafetyMonitor_Init(void);

/**
 * Update safety state — call at 100Hz from StatusTask.
 * Reads mode selector, deadman, fence GPIO pins.
 */
void SafetyMonitor_Update(void);

/**
 * Get current operating mode.
 */
OperatingMode SafetyMonitor_GetMode(void);

/**
 * Clamp speed based on current operating mode.
 * In T1 mode, clamps to 250 steps/ms (~250mm/s).
 *
 * @param speed  Requested speed (steps/ms)
 * @return       Clamped speed
 */
uint16_t SafetyMonitor_ClampSpeed(uint16_t speed);

/**
 * Check if motion is permitted in current mode.
 * - T1/T2: deadman must be pressed
 * - AUT: safety fence must be closed
 *
 * @return  1 if motion permitted, 0 if blocked
 */
uint8_t SafetyMonitor_MotionPermitted(void);

/**
 * Check if an E-stop condition was detected by safety monitor.
 */
uint8_t SafetyMonitor_IsEstop(void);

#endif /* SAFETY_MONITOR_H */
