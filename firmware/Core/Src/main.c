/**
 * @file main.c
 * @brief Main entry point — clock, MPU, cache, FreeRTOS task creation
 */

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ethernetif.h"
#include "app_tasks.h"
#include "cmd_handler.h"
#include "net_task.h"
#include "drive_control.h"
#include "safety_monitor.h"

#include <string.h>

/* ========================================================================= */
/*  Private Functions                                                        */
/* ========================================================================= */

static void MPU_Config(void);
static void CPU_CACHE_Enable(void);

/* ========================================================================= */
/*  SystemClock_Config                                                       */
/* ========================================================================= */
/**
 * Configure system clocks: HSE -> PLL -> 480 MHz SYSCLK.
 *
 * TODO: Adjust PLL parameters for your crystal frequency.
 * Current config assumes 25 MHz HSE crystal.
 *
 * SYSCLK = HSE / M * N / P = 25 / 5 * 192 / 2 = 480 MHz
 * AHB    = 240 MHz (HPRE /2)
 * APB1   = 120 MHz (PPRE1 /2)
 * APB2   = 120 MHz (PPRE2 /2)
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    /* Supply configuration: LDO */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /* Voltage scaling: VOS1 (highest performance) */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    /* HSE Oscillator */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM       = 5;    /* HSE/5 = 5 MHz    */
    osc.PLL.PLLN       = 192;  /* 5*192 = 960 MHz   */
    osc.PLL.PLLP       = 2;    /* 960/2 = 480 MHz   */
    osc.PLL.PLLQ       = 4;    /* 960/4 = 240 MHz   */
    osc.PLL.PLLR       = 2;
    osc.PLL.PLLRGE     = RCC_PLL1VCIRANGE_2;   /* 4-8 MHz input */
    osc.PLL.PLLVCOSEL  = RCC_PLL1VCOWIDE;      /* Wide VCO range */
    osc.PLL.PLLFRACN   = 0;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        Error_Handler();
    }

    /* Clock tree */
    clk.ClockType      = RCC_CLOCKTYPE_HCLK   | RCC_CLOCKTYPE_SYSCLK |
                          RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2  |
                          RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.SYSCLKDivider  = RCC_SYSCLK_DIV1;     /* 480 MHz */
    clk.AHBCLKDivider  = RCC_HCLK_DIV2;       /* 240 MHz */
    clk.APB1CLKDivider = RCC_APB1_DIV2;       /* 120 MHz */
    clk.APB2CLKDivider = RCC_APB2_DIV2;       /* 120 MHz */
    clk.APB3CLKDivider = RCC_APB3_DIV2;       /* 120 MHz */
    clk.APB4CLKDivider = RCC_APB4_DIV2;       /* 120 MHz */

    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

/* ========================================================================= */
/*  MPU Configuration                                                        */
/* ========================================================================= */
/**
 * Configure MPU regions for proper cache behavior.
 *
 * Critical for STM32H7:
 * - D2 SRAM (Ethernet DMA) must be non-cacheable or write-through
 * - AXI SRAM can be write-back for performance
 */
static void MPU_Config(void)
{
    HAL_MPU_Disable();

    MPU_Region_InitTypeDef mpu = {0};

    /* Region 0: D2 SRAM1 (0x30000000, 256KB) — Device/Non-cacheable for ETH DMA */
    mpu.Enable           = MPU_REGION_ENABLE;
    mpu.Number           = MPU_REGION_NUMBER0;
    mpu.BaseAddress      = 0x30000000;
    mpu.Size             = MPU_REGION_SIZE_256KB;
    mpu.SubRegionDisable = 0;
    mpu.TypeExtField     = MPU_TEX_LEVEL1;
    mpu.AccessPermission = MPU_REGION_FULL_ACCESS;
    mpu.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
    mpu.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
    mpu.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
    mpu.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&mpu);

    /* Region 1: AXI SRAM (0x24000000, 512KB) — Write-back, read/write allocate */
    mpu.Number           = MPU_REGION_NUMBER1;
    mpu.BaseAddress      = 0x24000000;
    mpu.Size             = MPU_REGION_SIZE_512KB;
    mpu.TypeExtField     = MPU_TEX_LEVEL1;
    mpu.IsCacheable      = MPU_ACCESS_CACHEABLE;
    mpu.IsBufferable     = MPU_ACCESS_BUFFERABLE;
    HAL_MPU_ConfigRegion(&mpu);

    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/* ========================================================================= */
/*  CPU Cache                                                                */
/* ========================================================================= */

static void CPU_CACHE_Enable(void)
{
    SCB_EnableICache();
    SCB_EnableDCache();
}

/* ========================================================================= */
/*  Main                                                                     */
/* ========================================================================= */

int main(void)
{
    /* 1. MPU must be configured before cache enable */
    MPU_Config();

    /* 2. Enable CPU caches */
    CPU_CACHE_Enable();

    /* 3. HAL initialization (SysTick, etc.) */
    HAL_Init();

    /* 4. System clock configuration */
    SystemClock_Config();
    SystemCoreClockUpdate();

    /* 5. Board-level init (LED, debug UART) */
    Board_Init();
    Board_Debug_Print("\r\n[FW] Robot Controller Firmware starting...\r\n");

    /* 6. LwIP stack initialization */
    MX_LWIP_Init();

    /* 7. Command handler initialization */
    CmdHandler_Init();
    DriveControl_Init();
    SafetyMonitor_Init();
    NetTask_RegisterHandler(CmdHandler_Dispatch);

    /* 8. Create FreeRTOS tasks */
    xTaskCreate(NetTask,    "NetTask",    TASK_STACK_NET,    NULL, TASK_PRIORITY_NET,    NULL);
    xTaskCreate(MotionTask, "MotionTask", TASK_STACK_MOTION, NULL, TASK_PRIORITY_MOTION, NULL);
    xTaskCreate(StatusTask, "StatusTask", TASK_STACK_STATUS, NULL, TASK_PRIORITY_STATUS, NULL);

    Board_Debug_Print("[FW] Tasks created. Starting scheduler...\r\n");

    /* 9. Start FreeRTOS scheduler — does not return */
    vTaskStartScheduler();

    /* Should never reach here */
    Error_Handler();
    return 0;
}

/* ========================================================================= */
/*  Error Handler                                                            */
/* ========================================================================= */

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        Board_LED_Toggle();
        for (volatile uint32_t i = 0; i < 1000000; i++) {}
    }
}
