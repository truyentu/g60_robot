/**
 * @file main.h
 * @brief Main header — includes and function prototypes
 */

#ifndef MAIN_H
#define MAIN_H

#include "stm32h7xx_hal.h"
#include "board_config.h"

/* Exported functions */
void Error_Handler(void);
void SystemClock_Config(void);

/* BSP functions (board_init.c) */
void Board_Init(void);
void Board_LED_On(void);
void Board_LED_Off(void);
void Board_LED_Toggle(void);
void Board_Debug_Print(const char *msg);
UART_HandleTypeDef *Board_GetDebugUart(void);

#endif /* MAIN_H */
