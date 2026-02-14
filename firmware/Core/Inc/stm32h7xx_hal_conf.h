/**
 * @file stm32h7xx_hal_conf.h
 * @brief HAL module selection and oscillator values
 *
 * Enable only the HAL modules used by this project.
 * Add more modules as features are implemented.
 */

#ifndef STM32H7XX_HAL_CONF_H
#define STM32H7XX_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*  HAL Module Selection                                                     */
/* ========================================================================= */

#define HAL_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_ETH_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_HSEM_MODULE_ENABLED
#define HAL_MDMA_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

/* Modules to enable later:
 * #define HAL_ADC_MODULE_ENABLED
 * #define HAL_DAC_MODULE_ENABLED
 * #define HAL_I2C_MODULE_ENABLED
 * #define HAL_IWDG_MODULE_ENABLED
 * #define HAL_SPI_MODULE_ENABLED
 * #define HAL_WWDG_MODULE_ENABLED
 */

/* ========================================================================= */
/*  Oscillator Values                                                        */
/* ========================================================================= */

/* External High Speed oscillator (HSE) */
#if !defined(HSE_VALUE)
  #define HSE_VALUE             25000000UL  /* TODO: Adjust for your crystal (Hz) */
#endif

#if !defined(HSE_STARTUP_TIMEOUT)
  #define HSE_STARTUP_TIMEOUT   100UL       /* ms */
#endif

/* Internal High Speed oscillator (HSI) */
#if !defined(HSI_VALUE)
  #define HSI_VALUE             64000000UL  /* 64 MHz */
#endif

/* External Low Speed oscillator (LSE) */
#if !defined(LSE_VALUE)
  #define LSE_VALUE             32768UL     /* 32.768 kHz */
#endif

#if !defined(LSE_STARTUP_TIMEOUT)
  #define LSE_STARTUP_TIMEOUT   5000UL      /* ms */
#endif

/* Internal Low Speed oscillator (LSI) */
#if !defined(LSI_VALUE)
  #define LSI_VALUE             32000UL     /* 32 kHz (approximate) */
#endif

/* CSI calibration */
#if !defined(CSI_VALUE)
  #define CSI_VALUE             4000000UL   /* 4 MHz */
#endif

/* External clock source for I2S */
#if !defined(EXTERNAL_CLOCK_VALUE)
  #define EXTERNAL_CLOCK_VALUE  12288000UL
#endif

/* ========================================================================= */
/*  System Configuration                                                     */
/* ========================================================================= */

#define VDD_VALUE                       3300UL  /* mV */
#define TICK_INT_PRIORITY               15UL    /* Lowest priority for SysTick */
#define USE_RTOS                        1
#define PREFETCH_ENABLE                 0       /* Not available on Cortex-M7 */
#define ART_ACCELERATOR_ENABLE          0

/* ========================================================================= */
/*  HAL Tick Source — TIM6 (SysTick reserved for FreeRTOS)                   */
/* ========================================================================= */

#define HAL_TICK_FREQ_1KHZ              1000UL

/* Use TIM6 as HAL timebase instead of SysTick.
 * SysTick is used by FreeRTOS for its scheduler tick.
 * Requires stm32h7xx_hal_timebase_tim.c or manual config in hal_msp.c */

/* ========================================================================= */
/*  Ethernet Configuration                                                   */
/* ========================================================================= */

#define ETH_TX_DESC_CNT                 4
#define ETH_RX_DESC_CNT                 4
#define ETH_MAC_ADDR0                   0x00
#define ETH_MAC_ADDR1                   0x80
#define ETH_MAC_ADDR2                   0xE1
#define ETH_MAC_ADDR3                   0x00
#define ETH_MAC_ADDR4                   0x00
#define ETH_MAC_ADDR5                   0x01    /* TODO: Set unique MAC */

/* ========================================================================= */
/*  Include HAL Headers                                                      */
/* ========================================================================= */

#ifdef HAL_RCC_MODULE_ENABLED
  #include "stm32h7xx_hal_rcc.h"
#endif
#ifdef HAL_GPIO_MODULE_ENABLED
  #include "stm32h7xx_hal_gpio.h"
#endif
#ifdef HAL_DMA_MODULE_ENABLED
  #include "stm32h7xx_hal_dma.h"
#endif
#ifdef HAL_MDMA_MODULE_ENABLED
  #include "stm32h7xx_hal_mdma.h"
#endif
#ifdef HAL_ETH_MODULE_ENABLED
  #include "stm32h7xx_hal_eth.h"
#endif
#ifdef HAL_EXTI_MODULE_ENABLED
  #include "stm32h7xx_hal_exti.h"
#endif
#ifdef HAL_CORTEX_MODULE_ENABLED
  #include "stm32h7xx_hal_cortex.h"
#endif
#ifdef HAL_FLASH_MODULE_ENABLED
  #include "stm32h7xx_hal_flash.h"
#endif
#ifdef HAL_HSEM_MODULE_ENABLED
  #include "stm32h7xx_hal_hsem.h"
#endif
#ifdef HAL_PWR_MODULE_ENABLED
  #include "stm32h7xx_hal_pwr.h"
#endif
#ifdef HAL_TIM_MODULE_ENABLED
  #include "stm32h7xx_hal_tim.h"
#endif
#ifdef HAL_UART_MODULE_ENABLED
  #include "stm32h7xx_hal_uart.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* STM32H7XX_HAL_CONF_H */
