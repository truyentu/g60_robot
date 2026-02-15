/**
 * @file board_watchdog.h
 * @brief Independent Watchdog (IWDG) initialization and kick
 */

#ifndef BOARD_WATCHDOG_H
#define BOARD_WATCHDOG_H

/**
 * Initialize IWDG with ~1 second timeout.
 * LSI clock (~32 kHz) / prescaler 64 = 500 Hz, reload = 500 → ~1s.
 */
void Board_InitWatchdog(void);

/**
 * Kick (refresh) the watchdog. Must be called within timeout period.
 * Typically called from StatusTask at 100Hz.
 */
void Board_KickWatchdog(void);

#endif /* BOARD_WATCHDOG_H */
