/**
 * @file FreeRTOSConfig.h
 * @brief FreeRTOS kernel configuration for STM32H743
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* ========================================================================= */
/*  Core Configuration                                                       */
/* ========================================================================= */

#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  1
#define configUSE_IDLE_HOOK                     1
#define configUSE_TICK_HOOK                     0
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0

/* CPU clock — STM32H743 runs at up to 480 MHz */
/* TODO: Adjust if using different PLL settings */
#define configCPU_CLOCK_HZ                      (480000000UL)
#define configTICK_RATE_HZ                      ((TickType_t)1000)  /* 1ms tick */
#define configMAX_PRIORITIES                    8                   /* 0-7 */
#define configMINIMAL_STACK_SIZE                ((uint16_t)256)     /* Words */
#define configMAX_TASK_NAME_LEN                 16
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configQUEUE_REGISTRY_SIZE               8

/* ========================================================================= */
/*  Memory Allocation                                                        */
/* ========================================================================= */

#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configTOTAL_HEAP_SIZE                   ((size_t)(64 * 1024))  /* 64KB */
#define configAPPLICATION_ALLOCATED_HEAP        0

/* ========================================================================= */
/*  Synchronization                                                          */
/* ========================================================================= */

#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           1

/* ========================================================================= */
/*  Software Timers                                                          */
/* ========================================================================= */

#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               2
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            256

/* ========================================================================= */
/*  Hook Functions                                                           */
/* ========================================================================= */

#define configCHECK_FOR_STACK_OVERFLOW          2   /* Method 2: pattern check */
#define configUSE_MALLOC_FAILED_HOOK            1

/* ========================================================================= */
/*  Run-time Stats (disabled for now)                                        */
/* ========================================================================= */

#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TRACE_FACILITY                0
#define configUSE_STATS_FORMATTING_FUNCTIONS    0

/* ========================================================================= */
/*  Co-routines (not used)                                                   */
/* ========================================================================= */

#define configUSE_CO_ROUTINES                   0

/* ========================================================================= */
/*  Cortex-M7 Interrupt Priority Configuration                               */
/* ========================================================================= */
/*
 * STM32H7 uses 4 priority bits (0-15, lower number = higher priority).
 * FreeRTOS needs to know the lowest and highest interrupt priorities
 * that can call FreeRTOS API functions.
 *
 * Priority 0-4:  Cannot call FreeRTOS API (reserved for time-critical ISRs)
 * Priority 5-15: Can safely call FreeRTOS API (FromISR variants)
 *
 * This means hardware timer ISR for step pulse generation (priority 1-2)
 * and E-stop EXTI (priority 0) run above FreeRTOS and are never delayed.
 */

#define configPRIO_BITS                         4   /* STM32H7: 4 bits = 16 levels */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5

#define configKERNEL_INTERRUPT_PRIORITY         (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* ========================================================================= */
/*  API Includes                                                             */
/* ========================================================================= */

#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTimerPendFunctionCall          1
#define INCLUDE_xQueueGetMutexHolder            1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_eTaskGetState                   1

/* ========================================================================= */
/*  SysTick Handler Mapping                                                  */
/* ========================================================================= */
/* Map FreeRTOS handlers to CMSIS standard names */

#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
/* SysTick_Handler is called manually in stm32h7xx_it.c */

#endif /* FREERTOS_CONFIG_H */
