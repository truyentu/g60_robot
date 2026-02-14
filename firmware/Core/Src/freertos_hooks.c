/**
 * @file freertos_hooks.c
 * @brief FreeRTOS application hooks — stack overflow, malloc fail, idle
 */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

void vApplicationMallocFailedHook(void)
{
    /* FreeRTOS heap exhausted — fatal error */
    Board_Debug_Print("[FATAL] FreeRTOS malloc failed!\r\n");
    Error_Handler();
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    Board_Debug_Print("[FATAL] Stack overflow in task: ");
    Board_Debug_Print(pcTaskName);
    Board_Debug_Print("\r\n");
    Error_Handler();
}

void vApplicationIdleHook(void)
{
    /* Low-power mode could be entered here */
}
