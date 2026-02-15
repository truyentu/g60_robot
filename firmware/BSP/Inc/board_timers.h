/**
 * @file board_timers.h
 * @brief Hardware timer mapping for step pulse generation (6 axes)
 *
 * Each axis uses one TIM channel in Output Compare Toggle mode.
 * Timer clock = APB2 Timer Clock = 240 MHz (at 480MHz SYSCLK).
 *
 * Frequency control: ARR = (TimerClock / (2 * desired_pulse_freq)) - 1
 * Pulse width is half-period (toggle mode).
 *
 * Only used when SIMULATION_MODE is NOT defined.
 */

#ifndef BOARD_TIMERS_H
#define BOARD_TIMERS_H

#include "board_config.h"

/* ========================================================================= */
/*  Timer Clock (APB2 timer clock after PLL)                                 */
/* ========================================================================= */

#define TIMER_CLOCK_HZ              240000000UL  /* 240 MHz (APB2 x2) */

/* ========================================================================= */
/*  Axis 1 (J1) — TIM1 Channel 1                                            */
/* ========================================================================= */

#define MOTOR1_TIM                  TIM1
#define MOTOR1_TIM_CHANNEL          TIM_CHANNEL_1
#define MOTOR1_TIM_CLK_ENABLE()     __HAL_RCC_TIM1_CLK_ENABLE()
#define MOTOR1_TIM_IRQn             TIM1_CC_IRQn
#define MOTOR1_TIM_AF               GPIO_AF1_TIM1
#define MOTOR1_PULSE_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()

/* ========================================================================= */
/*  Axis 2 (J2) — TIM1 Channel 2                                            */
/* ========================================================================= */

#define MOTOR2_TIM                  TIM1
#define MOTOR2_TIM_CHANNEL          TIM_CHANNEL_2
#define MOTOR2_TIM_CLK_ENABLE()     __HAL_RCC_TIM1_CLK_ENABLE()
#define MOTOR2_TIM_AF               GPIO_AF1_TIM1
#define MOTOR2_PULSE_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()

/* ========================================================================= */
/*  Axis 3 (J3) — TIM3 Channel 1                                            */
/* ========================================================================= */

#define MOTOR3_TIM                  TIM3
#define MOTOR3_TIM_CHANNEL          TIM_CHANNEL_1
#define MOTOR3_TIM_CLK_ENABLE()     __HAL_RCC_TIM3_CLK_ENABLE()
#define MOTOR3_TIM_IRQn             TIM3_IRQn
#define MOTOR3_TIM_AF               GPIO_AF2_TIM3
#define MOTOR3_PULSE_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()

/* ========================================================================= */
/*  Axis 4 (J4) — TIM3 Channel 2                                            */
/* ========================================================================= */

#define MOTOR4_TIM                  TIM3
#define MOTOR4_TIM_CHANNEL          TIM_CHANNEL_2
#define MOTOR4_TIM_CLK_ENABLE()     __HAL_RCC_TIM3_CLK_ENABLE()
#define MOTOR4_TIM_AF               GPIO_AF2_TIM3
#define MOTOR4_PULSE_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()

/* ========================================================================= */
/*  Axis 5 (J5) — TIM1 Channel 3 (PE9 AF1)                                  */
/* ========================================================================= */

#define MOTOR5_TIM                  TIM1
#define MOTOR5_TIM_CHANNEL          TIM_CHANNEL_3
#define MOTOR5_TIM_CLK_ENABLE()     __HAL_RCC_TIM1_CLK_ENABLE()
#define MOTOR5_TIM_AF               GPIO_AF1_TIM1
#define MOTOR5_PULSE_CLK_ENABLE()   __HAL_RCC_GPIOE_CLK_ENABLE()

/* ========================================================================= */
/*  Axis 6 (J6) — TIM1 Channel 4 (PE11 — needs remap, or use TIM4)          */
/*  Using TIM4 Channel 1 instead for clean separation                        */
/* ========================================================================= */

#define MOTOR6_TIM                  TIM4
#define MOTOR6_TIM_CHANNEL          TIM_CHANNEL_1
#define MOTOR6_TIM_CLK_ENABLE()     __HAL_RCC_TIM4_CLK_ENABLE()
#define MOTOR6_TIM_IRQn             TIM4_IRQn
#define MOTOR6_TIM_AF               GPIO_AF2_TIM4
#define MOTOR6_PULSE_CLK_ENABLE()   __HAL_RCC_GPIOE_CLK_ENABLE()

/* ========================================================================= */
/*  Frequency Calculation Helpers                                            */
/* ========================================================================= */

/**
 * Calculate ARR value for a given pulse frequency.
 * Toggle mode: output toggles every ARR+1 counts → freq = clk / (2*(ARR+1))
 * ARR = (clk / (2 * freq)) - 1
 *
 * @param freq_hz  Desired pulse frequency in Hz (must be > 0)
 * @return ARR value (clamped to 16-bit max)
 */
static inline uint32_t StepTimer_FreqToARR(uint32_t freq_hz)
{
    if (freq_hz == 0) return 0xFFFF;
    uint32_t arr = (TIMER_CLOCK_HZ / (2 * freq_hz)) - 1;
    if (arr > 0xFFFF) arr = 0xFFFF;
    return arr;
}

/**
 * Calculate pulse frequency from step count and time.
 *
 * @param delta_steps  Number of steps to output (absolute value)
 * @param period_us    Time period in microseconds
 * @return Pulse frequency in Hz
 */
static inline uint32_t StepTimer_CalcFreq(uint32_t delta_steps, uint32_t period_us)
{
    if (period_us == 0) return 0;
    /* freq = steps / (period_us / 1e6) = steps * 1e6 / period_us */
    return (uint32_t)((uint64_t)delta_steps * 1000000ULL / period_us);
}

/* ========================================================================= */
/*  Public API (only available in Real Mode)                                 */
/* ========================================================================= */

#ifndef SIMULATION_MODE

/**
 * Initialize all motor timers and direction GPIOs.
 * Called from Board_Init() when not in simulation mode.
 */
void Board_InitTimers(void);

/**
 * Set pulse frequency for a given axis.
 * @param axis     Axis index (0-5)
 * @param freq_hz  Pulse frequency in Hz (0 = stop)
 */
void StepTimer_SetFrequency(uint8_t axis, uint32_t freq_hz);

/**
 * Set direction GPIO for a given axis.
 * @param axis       Axis index (0-5)
 * @param direction  +1 = forward, -1 = reverse
 */
void StepTimer_SetDirection(uint8_t axis, int8_t direction);

/**
 * Stop all pulse outputs immediately (E-stop).
 */
void StepTimer_StopAll(void);

#endif /* !SIMULATION_MODE */

#endif /* BOARD_TIMERS_H */
