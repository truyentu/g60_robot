/**
 * @file board_gpio.h
 * @brief GPIO driver for servo drive signals, digital I/O, and safety inputs
 *
 * Provides array-indexed access to per-axis drive signals (enable, alarm,
 * ready, brake) and general-purpose digital I/O (16 DIN + 16 DOUT).
 */

#ifndef BOARD_GPIO_H
#define BOARD_GPIO_H

#include "board_config.h"
#include "protocol_defs.h"  /* PROTO_NUM_AXES */

/* ========================================================================= */
/*  Initialization                                                          */
/* ========================================================================= */

/**
 * Initialize all GPIO pins for drive signals, home switches, DIO, safety.
 * Called from Board_Init() before RTOS scheduler starts.
 */
void Board_InitGPIO(void);

/* ========================================================================= */
/*  Servo Drive Signals (per-axis)                                          */
/* ========================================================================= */

/** Set servo drive enable output. 1 = enabled, 0 = disabled. */
void GPIO_DriveEnable(uint8_t axis, uint8_t enable);

/** Read servo drive READY input. Returns 1 if drive is ready. */
uint8_t GPIO_DriveGetReady(uint8_t axis);

/** Read servo drive ALARM input. Returns 1 if alarm is active. */
uint8_t GPIO_DriveGetAlarm(uint8_t axis);

/** Control brake solenoid. 1 = release (brake off), 0 = engage (brake on). */
void GPIO_BrakeRelease(uint8_t axis, uint8_t release);

/** Get bitmask of ready drives (bit0 = axis 0, ..., bit5 = axis 5). */
uint8_t GPIO_DriveGetReadyMask(void);

/** Get bitmask of alarmed drives. */
uint8_t GPIO_DriveGetAlarmMask(void);

/** Get bitmask of enabled drives. */
uint8_t GPIO_DriveGetEnabledMask(void);

/* ========================================================================= */
/*  Home Switches                                                           */
/* ========================================================================= */

/** Read raw home switch state (0 or 1) for an axis. */
uint8_t GPIO_HomeGetRaw(uint8_t axis);

/* ========================================================================= */
/*  E-Stop                                                                  */
/* ========================================================================= */

/** Returns 1 if E-stop is active (button pressed). */
uint8_t GPIO_EstopIsActive(void);

/** Initialize E-stop EXTI interrupt (NVIC priority 0). */
void Board_InitEstopEXTI(void);

/* ========================================================================= */
/*  Digital I/O (16 DIN + 16 DOUT)                                          */
/* ========================================================================= */

/** Read all 16 digital inputs as a bitmask. */
uint16_t GPIO_ReadDigitalInputs(void);

/** Read current digital output state as a bitmask. */
uint16_t GPIO_ReadDigitalOutputs(void);

/** Write a single digital output (index 0-15). */
void GPIO_WriteDigitalOutput(uint8_t index, uint8_t value);

/** Write multiple digital outputs using mask (only bits in mask are changed). */
void GPIO_WriteDigitalOutputs(uint16_t mask, uint16_t values);

/* ========================================================================= */
/*  Safety Signals                                                          */
/* ========================================================================= */

/** Returns 1 if deadman switch is pressed. */
uint8_t GPIO_DeadmanIsPressed(void);

/** Returns 1 if safety fence interlock is closed. */
uint8_t GPIO_SafetyFenceIsClosed(void);

/** Read operating mode selector. Returns 0=T1, 1=T2, 2=AUT. */
uint8_t GPIO_ReadOperatingMode(void);

#endif /* BOARD_GPIO_H */
