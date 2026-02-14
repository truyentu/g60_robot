/**
 * @file stm32h7xx_it.c
 * @brief Interrupt handlers
 *
 * Fault handlers -> Error_Handler()
 * SysTick -> FreeRTOS
 * ETH -> HAL Ethernet driver
 */

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

/* External handles (if needed) */
extern ETH_HandleTypeDef heth;

/* ========================================================================= */
/*  Cortex-M7 Processor Fault Handlers                                       */
/* ========================================================================= */

void NMI_Handler(void)
{
    while (1) {}
}

void HardFault_Handler(void)
{
    while (1) {}
}

void MemManage_Handler(void)
{
    while (1) {}
}

void BusFault_Handler(void)
{
    while (1) {}
}

void UsageFault_Handler(void)
{
    while (1) {}
}

void DebugMon_Handler(void)
{
    /* Nothing */
}

/* ========================================================================= */
/*  FreeRTOS Tick (SysTick)                                                  */
/* ========================================================================= */

void SysTick_Handler(void)
{
    HAL_IncTick();

#if (INCLUDE_xTaskGetSchedulerState == 1)
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
#endif
}

/* ========================================================================= */
/*  Ethernet IRQ                                                             */
/* ========================================================================= */

void ETH_IRQHandler(void)
{
    HAL_ETH_IRQHandler(&heth);
}

/* ========================================================================= */
/*  Timer IRQs (placeholder for step pulse generation)                       */
/* ========================================================================= */

/* TODO: Add timer ISR handlers when motion control is implemented.
 * Example:
 * void TIM1_UP_IRQHandler(void) { HAL_TIM_IRQHandler(&htim1); }
 * void TIM2_IRQHandler(void)    { HAL_TIM_IRQHandler(&htim2); }
 */

/* ========================================================================= */
/*  E-Stop EXTI (placeholder)                                                */
/* ========================================================================= */

/* TODO: Add when E-stop hardware is wired.
 * void EXTI0_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(ESTOP_PIN); }
 */
