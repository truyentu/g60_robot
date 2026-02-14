/**
 * @file stm32h7xx_hal_msp.c
 * @brief HAL MSP initialization — peripheral GPIO and clock setup
 *
 * Called automatically by HAL_xxx_Init() functions.
 * Reads pin definitions from board_config.h.
 */

#include "main.h"

/* ========================================================================= */
/*  HAL MspInit (global)                                                     */
/* ========================================================================= */

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
}

/* ========================================================================= */
/*  Ethernet MSP                                                             */
/* ========================================================================= */

void HAL_ETH_MspInit(ETH_HandleTypeDef *heth)
{
    GPIO_InitTypeDef gpio = {0};

    /* Enable clocks */
    __HAL_RCC_ETH1MAC_CLK_ENABLE();
    __HAL_RCC_ETH1TX_CLK_ENABLE();
    __HAL_RCC_ETH1RX_CLK_ENABLE();

    /* Enable GPIO clocks for all RMII pins */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /* RMII pins — all Alternate Function, High Speed */
    /* REF_CLK */
    gpio.Pin       = ETH_RMII_REF_CLK_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(ETH_RMII_REF_CLK_PORT, &gpio);

    /* MDIO */
    gpio.Pin = ETH_RMII_MDIO_PIN;
    HAL_GPIO_Init(ETH_RMII_MDIO_PORT, &gpio);

    /* MDC */
    gpio.Pin = ETH_RMII_MDC_PIN;
    HAL_GPIO_Init(ETH_RMII_MDC_PORT, &gpio);

    /* CRS_DV */
    gpio.Pin = ETH_RMII_CRS_DV_PIN;
    HAL_GPIO_Init(ETH_RMII_CRS_DV_PORT, &gpio);

    /* RXD0 */
    gpio.Pin = ETH_RMII_RXD0_PIN;
    HAL_GPIO_Init(ETH_RMII_RXD0_PORT, &gpio);

    /* RXD1 */
    gpio.Pin = ETH_RMII_RXD1_PIN;
    HAL_GPIO_Init(ETH_RMII_RXD1_PORT, &gpio);

    /* TX_EN */
    gpio.Pin = ETH_RMII_TX_EN_PIN;
    HAL_GPIO_Init(ETH_RMII_TX_EN_PORT, &gpio);

    /* TXD0 */
    gpio.Pin = ETH_RMII_TXD0_PIN;
    HAL_GPIO_Init(ETH_RMII_TXD0_PORT, &gpio);

    /* TXD1 */
    gpio.Pin = ETH_RMII_TXD1_PIN;
    HAL_GPIO_Init(ETH_RMII_TXD1_PORT, &gpio);

    /* Ethernet IRQ */
    HAL_NVIC_SetPriority(ETH_IRQn, 7, 0);  /* Below FreeRTOS syscall threshold */
    HAL_NVIC_EnableIRQ(ETH_IRQn);
}

void HAL_ETH_MspDeInit(ETH_HandleTypeDef *heth)
{
    __HAL_RCC_ETH1MAC_CLK_DISABLE();
    __HAL_RCC_ETH1TX_CLK_DISABLE();
    __HAL_RCC_ETH1RX_CLK_DISABLE();

    HAL_GPIO_DeInit(ETH_RMII_REF_CLK_PORT, ETH_RMII_REF_CLK_PIN);
    HAL_GPIO_DeInit(ETH_RMII_MDIO_PORT,    ETH_RMII_MDIO_PIN);
    HAL_GPIO_DeInit(ETH_RMII_MDC_PORT,     ETH_RMII_MDC_PIN);
    HAL_GPIO_DeInit(ETH_RMII_CRS_DV_PORT,  ETH_RMII_CRS_DV_PIN);
    HAL_GPIO_DeInit(ETH_RMII_RXD0_PORT,    ETH_RMII_RXD0_PIN);
    HAL_GPIO_DeInit(ETH_RMII_RXD1_PORT,    ETH_RMII_RXD1_PIN);
    HAL_GPIO_DeInit(ETH_RMII_TX_EN_PORT,   ETH_RMII_TX_EN_PIN);
    HAL_GPIO_DeInit(ETH_RMII_TXD0_PORT,    ETH_RMII_TXD0_PIN);
    HAL_GPIO_DeInit(ETH_RMII_TXD1_PORT,    ETH_RMII_TXD1_PIN);

    HAL_NVIC_DisableIRQ(ETH_IRQn);
}

/* ========================================================================= */
/*  Timer MSP (placeholder for step pulse timers)                            */
/* ========================================================================= */

/* TODO: Implement when motion control timers are configured.
 * void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim) { ... }
 */
