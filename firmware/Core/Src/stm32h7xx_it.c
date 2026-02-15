/**
 * @file stm32h7xx_it.c
 * @brief Interrupt handlers
 *
 * Fault handlers -> Error_Handler()
 * SysTick -> FreeRTOS
 * ETH -> HAL Ethernet driver
 */

#include "main.h"
#include "board_config.h"
#include "board_gpio.h"
#include "motion_task.h"
#include "protocol_defs.h"
#include "FreeRTOS.h"
#include "task.h"

#ifndef SIMULATION_MODE
#include "board_timers.h"
#endif

/* FreeRTOS SysTick handler — declared in portable/GCC/ARM_CM7/port.c */
extern void xPortSysTickHandler(void);

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
/*  E-Stop EXTI                                                              */
/* ========================================================================= */

/**
 * E-Stop EXTI interrupt handler.
 * NVIC priority 0 — above FreeRTOS configMAX_SYSCALL_INTERRUPT_PRIORITY.
 * Must NOT call any FreeRTOS API functions (FromISR or otherwise).
 * Direct GPIO manipulation only.
 */
void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(ESTOP_PIN);
}

/**
 * HAL GPIO EXTI callback — called from HAL_GPIO_EXTI_IRQHandler.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ESTOP_PIN) {
        /* Stop all motion immediately */
#ifndef SIMULATION_MODE
        StepTimer_StopAll();
#endif
        /* Set E-stop state (atomic write, no RTOS API) */
        Motion_SetState(SYSTEM_STATE_ESTOP);

        /* Disable all drives and engage all brakes — direct GPIO, no RTOS */
        for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
            GPIO_DriveEnable(i, 0);
            GPIO_BrakeRelease(i, 0);
        }
    }
}
