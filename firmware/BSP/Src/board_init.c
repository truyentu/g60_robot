/**
 * @file board_init.c
 * @brief Board-level initialization (LED, debug UART, timers)
 */

#include "board_config.h"
#include "board_gpio.h"
#include "board_watchdog.h"
#include <string.h>

#ifndef SIMULATION_MODE
extern void Board_InitTimers(void);
#endif

/* Forward declaration */
void Board_Debug_Print(const char *msg);

static UART_HandleTypeDef huart_debug;

/**
 * Initialize board-level peripherals.
 * Called from main() before RTOS scheduler starts.
 */
void Board_Init(void)
{
    /* Status LED */
    LED_STATUS_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = LED_STATUS_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_STATUS_PORT, &gpio);

    HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_RESET);

    /* GPIO: drive signals, DIO, safety inputs */
    Board_InitGPIO();

    /* Debug UART */
    DEBUG_UART_CLK_ENABLE();
    DEBUG_UART_TX_CLK_ENABLE();

    gpio.Pin       = DEBUG_UART_TX_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = DEBUG_UART_AF;
    HAL_GPIO_Init(DEBUG_UART_TX_PORT, &gpio);

    gpio.Pin = DEBUG_UART_RX_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(DEBUG_UART_RX_PORT, &gpio);

    huart_debug.Instance          = DEBUG_UART_INSTANCE;
    huart_debug.Init.BaudRate     = DEBUG_UART_BAUDRATE;
    huart_debug.Init.WordLength   = UART_WORDLENGTH_8B;
    huart_debug.Init.StopBits     = UART_STOPBITS_1;
    huart_debug.Init.Parity       = UART_PARITY_NONE;
    huart_debug.Init.Mode         = UART_MODE_TX_RX;
    huart_debug.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart_debug.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart_debug);

    /* Hardware timers for step pulse generation */
#ifdef SIMULATION_MODE
    Board_Debug_Print("Board Init: SIMULATION MODE ACTIVE\r\n");
#else
    Board_InitTimers();
    Board_Debug_Print("Board Init: REAL MODE — Timers initialized\r\n");
#endif

    /* E-Stop EXTI interrupt */
    Board_InitEstopEXTI();

    /* Independent Watchdog (~1s timeout) */
    Board_InitWatchdog();
}

void Board_LED_On(void)
{
    HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_SET);
}

void Board_LED_Off(void)
{
    HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_RESET);
}

void Board_LED_Toggle(void)
{
    HAL_GPIO_TogglePin(LED_STATUS_PORT, LED_STATUS_PIN);
}

/**
 * Send debug string via UART (blocking, for early init only).
 * In RTOS context, use a debug task with queue instead.
 */
void Board_Debug_Print(const char *msg)
{
    HAL_UART_Transmit(&huart_debug, (uint8_t *)msg, strlen(msg), 100);
}

/**
 * Get debug UART handle (for syscalls _write redirect).
 */
UART_HandleTypeDef *Board_GetDebugUart(void)
{
    return &huart_debug;
}
