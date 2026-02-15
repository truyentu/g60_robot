/**
 * @file system_stm32h7xx.c
 * @brief CMSIS System initialization — vector table, FPU enable, clock variables
 *
 * Provides SystemInit(), SystemCoreClock, SystemD2Clock, D1CorePrescTable
 * required by the HAL and CMSIS framework.
 */

#include "stm32h7xx.h"
#include "stm32h7xx_hal_conf.h"
#include <math.h>

/* Vector table offset — default: start of Flash */
#if !defined(VECT_TAB_SRAM)
  #define VECT_TAB_BASE   0x08000000UL
  #define VECT_TAB_OFFSET 0x00000000UL
#else
  #define VECT_TAB_BASE   0x24000000UL
  #define VECT_TAB_OFFSET 0x00000000UL
#endif

/* System clock variables — updated by SystemClock_Config() in main.c */
uint32_t SystemCoreClock = 64000000UL;  /* Default HSI = 64 MHz */
uint32_t SystemD2Clock   = 64000000UL;  /* D2 domain clock */

/* D1 Core prescaler table — maps CDCPRE register value to divider
 * Used by HAL_RCC to compute HCLK from SYSCLK.
 * Index: RCC_D1CFGR.CDCPRE[3:0] value, Value: prescaler divider */
const uint8_t D1CorePrescTable[16] = {
    0, 0, 0, 0,   /* 0-3:   /1 (no division) */
    1, 2, 3, 4,   /* 4-7:   /2, /4, /8, /16  */
    6, 7, 8, 9,   /* 8-11:  /64, /128, /256, /512 */
    0, 0, 0, 0    /* 12-15: reserved */
};

void SystemInit(void)
{
    /* FPU settings: enable CP10 and CP11 full access */
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << (10 * 2)) | (3UL << (11 * 2)));
#endif

    /* Set vector table location */
    SCB->VTOR = VECT_TAB_BASE | VECT_TAB_OFFSET;
}

void SystemCoreClockUpdate(void)
{
    uint32_t pllp, pllsource, pllm, pllfracen, hsivalue, tmp;
    float_t fracn1, pllvco;

    /* Get SYSCLK source */
    switch (RCC->CFGR & RCC_CFGR_SWS)
    {
    case RCC_CFGR_SWS_HSI:
        SystemCoreClock = (uint32_t)(HSI_VALUE >> ((RCC->CR & RCC_CR_HSIDIV) >> RCC_CR_HSIDIV_Pos));
        break;

    case RCC_CFGR_SWS_CSI:
        SystemCoreClock = CSI_VALUE;
        break;

    case RCC_CFGR_SWS_HSE:
        SystemCoreClock = HSE_VALUE;
        break;

    case RCC_CFGR_SWS_PLL1:
        /* PLL1 used as system clock source */
        pllsource = (RCC->PLLCKSELR & RCC_PLLCKSELR_PLLSRC);
        pllm = ((RCC->PLLCKSELR & RCC_PLLCKSELR_DIVM1) >> RCC_PLLCKSELR_DIVM1_Pos);
        pllfracen = ((RCC->PLLCFGR & RCC_PLLCFGR_PLL1FRACEN) >> RCC_PLLCFGR_PLL1FRACEN_Pos);
        fracn1 = (float_t)(uint32_t)(pllfracen * ((RCC->PLL1FRACR & RCC_PLL1FRACR_FRACN1) >> RCC_PLL1FRACR_FRACN1_Pos));

        switch (pllsource)
        {
        case RCC_PLLCKSELR_PLLSRC_HSI:
            hsivalue = (uint32_t)(HSI_VALUE >> ((RCC->CR & RCC_CR_HSIDIV) >> RCC_CR_HSIDIV_Pos));
            pllvco = ((float_t)hsivalue / (float_t)pllm) *
                     ((float_t)(uint32_t)(RCC->PLL1DIVR & RCC_PLL1DIVR_N1) + (fracn1 / 0x2000U) + 1U);
            break;

        case RCC_PLLCKSELR_PLLSRC_CSI:
            pllvco = ((float_t)CSI_VALUE / (float_t)pllm) *
                     ((float_t)(uint32_t)(RCC->PLL1DIVR & RCC_PLL1DIVR_N1) + (fracn1 / 0x2000U) + 1U);
            break;

        case RCC_PLLCKSELR_PLLSRC_HSE:
            pllvco = ((float_t)HSE_VALUE / (float_t)pllm) *
                     ((float_t)(uint32_t)(RCC->PLL1DIVR & RCC_PLL1DIVR_N1) + (fracn1 / 0x2000U) + 1U);
            break;

        default:
            pllvco = ((float_t)CSI_VALUE / (float_t)pllm) *
                     ((float_t)(uint32_t)(RCC->PLL1DIVR & RCC_PLL1DIVR_N1) + (fracn1 / 0x2000U) + 1U);
            break;
        }

        pllp = (((RCC->PLL1DIVR & RCC_PLL1DIVR_P1) >> RCC_PLL1DIVR_P1_Pos) + 1U);
        SystemCoreClock = (uint32_t)(float_t)(pllvco / (float_t)pllp);
        break;

    default:
        SystemCoreClock = CSI_VALUE;
        break;
    }

    /* Compute HCLK frequency: apply D1CPRE prescaler */
    tmp = D1CorePrescTable[(RCC->D1CFGR & RCC_D1CFGR_D1CPRE) >> RCC_D1CFGR_D1CPRE_Pos];
    SystemCoreClock >>= tmp;

    /* Update D2 domain clock */
    SystemD2Clock = SystemCoreClock >>
        D1CorePrescTable[(RCC->D1CFGR & RCC_D1CFGR_HPRE) >> RCC_D1CFGR_HPRE_Pos];
}
