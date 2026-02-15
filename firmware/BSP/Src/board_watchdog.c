/**
 * @file board_watchdog.c
 * @brief Independent Watchdog (IWDG) driver
 *
 * Uses IWDG with LSI clock (~32 kHz).
 * Prescaler = 64 → counter clock = 500 Hz
 * Reload = 500 → timeout ≈ 1 second
 *
 * Must be kicked (refreshed) periodically or MCU resets.
 */

#include "board_watchdog.h"
#include "board_config.h"
#include "stm32h7xx_hal.h"

static IWDG_HandleTypeDef s_hiwdg;

void Board_InitWatchdog(void)
{
    s_hiwdg.Instance       = IWDG1;
    s_hiwdg.Init.Prescaler = IWDG_PRESCALER_64;   /* 32kHz / 64 = 500 Hz */
    s_hiwdg.Init.Reload    = 500;                  /* 500 / 500Hz = 1s    */
    s_hiwdg.Init.Window    = IWDG_WINDOW_DISABLE;

    HAL_IWDG_Init(&s_hiwdg);
}

void Board_KickWatchdog(void)
{
    HAL_IWDG_Refresh(&s_hiwdg);
}
