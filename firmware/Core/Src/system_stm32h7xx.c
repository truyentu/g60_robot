/**
 * @file system_stm32h7xx.c
 * @brief CMSIS System initialization — vector table, FPU enable
 *
 * This is a minimal version. The full file comes from STM32CubeH7
 * (Drivers/CMSIS/Device/ST/STM32H7xx/Source/Templates/system_stm32h7xx.c).
 *
 * When you populate Drivers/, you can replace this with the official version.
 */

#include "stm32h7xx.h"

/* Vector table offset — default: start of Flash */
#if !defined(VECT_TAB_SRAM)
  #define VECT_TAB_BASE   0x08000000UL
  #define VECT_TAB_OFFSET 0x00000000UL
#else
  #define VECT_TAB_BASE   0x24000000UL
  #define VECT_TAB_OFFSET 0x00000000UL
#endif

uint32_t SystemCoreClock = 64000000UL;  /* Updated by SystemClock_Config() */

void SystemInit(void)
{
    /* FPU settings: enable CP10 and CP11 full access */
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << (10 * 2)) | (3UL << (11 * 2)));
#endif

    /* Set vector table location */
    SCB->VTOR = VECT_TAB_BASE | VECT_TAB_OFFSET;
}
