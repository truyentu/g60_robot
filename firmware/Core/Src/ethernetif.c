/**
 * @file ethernetif.c
 * @brief LwIP Ethernet interface — STM32H7 HAL ETH driver (HAL v1.11.3)
 *
 * Based on:
 * - r-gal/CNC (MIT) — DMA descriptor placement, TxConfig, HAL callback model
 * - STM32CubeH7 LwIP examples (BSD-3-Clause) — ethernetif pattern
 *
 * Key architecture:
 * - DMA descriptors + Rx buffers placed in D2 SRAM (SRAM1, 0x30000000)
 *   via linker section attributes — CRITICAL for STM32H7 Ethernet DMA
 * - D-Cache invalidation on Rx, clean on Tx for cache coherency
 * - ISR-driven Rx: ETH_IRQHandler → HAL callback → semaphore → ethernetif_input()
 * - Blocking Tx: HAL_ETH_Transmit() in FreeRTOS task context
 * - LAN8742 PHY (standard BMCR/BMSR registers)
 *
 * HAL v1.11.3 Rx model:
 * - Uses HAL_ETH_ReadData() with callback-based buffer management
 * - HAL_ETH_RxAllocateCallback() provides fresh Rx buffers to DMA
 * - HAL_ETH_RxLinkCallback() chains received buffers into pbuf
 */

#include "ethernetif.h"
#include "board_config.h"
#include "stm32h7xx_hal.h"

#include "lwip/opt.h"
#include "lwip/tcpip.h"
#include "lwip/netif.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "netif/etharp.h"
#include "lwip/pbuf.h"
#include "lwip/stats.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <string.h>

/* ========================================================================= */
/*  Ethernet Handle (global — used by stm32h7xx_it.c)                       */
/* ========================================================================= */

ETH_HandleTypeDef heth;

/* ========================================================================= */
/*  DMA Descriptors & Buffers — MUST be in D2 SRAM (0x30000000)              */
/*  ETH DMA cannot access AXI SRAM or DTCM directly!                        */
/* ========================================================================= */

#if defined(__GNUC__)
  ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT]
      __attribute__((section(".RxDecripSection")));
  ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT]
      __attribute__((section(".TxDecripSection")));
  uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]
      __attribute__((section(".RxArraySection")));
#endif

/* Tx packet configuration (checksum offload, CRC padding) */
static ETH_TxPacketConfigTypeDef s_tx_config;

/* Netif stored for use in callbacks */
static struct netif gnetif;
static struct netif *s_netif_ref = NULL;

/* FreeRTOS binary semaphore — signaled from ETH Rx ISR */
static SemaphoreHandle_t s_rx_semaphore = NULL;
static StaticSemaphore_t s_rx_sem_buf;

/* Rx input task handle */
static TaskHandle_t s_rx_task_handle = NULL;

/* Rx buffer tracking — index into Rx_Buff[] for HAL_ETH_RxAllocateCallback */
static uint32_t s_rx_buf_idx = 0;

/* Rx pbuf chain assembly — used by HAL_ETH_RxLinkCallback */
static struct pbuf *s_rx_pbuf_head = NULL;
static struct pbuf *s_rx_pbuf_tail = NULL;

/* ========================================================================= */
/*  PHY Constants (LAN8742 — compatible with DP83848, KSZ8081)               */
/* ========================================================================= */

#define PHY_BCR                     0x00U   /* Basic Control Register        */
#define PHY_BSR                     0x01U   /* Basic Status Register         */
#define PHY_PHYSCSR                 0x1FU   /* PHY Special Control/Status    */

#define PHY_BCR_SOFT_RESET          0x8000U
#define PHY_BCR_AUTONEGO_EN         0x1000U
#define PHY_BCR_RESTART_AUTONEGO    0x0200U

#define PHY_BSR_AUTONEGO_DONE       0x0020U
#define PHY_BSR_LINK_STATUS         0x0004U

#define PHY_PHYSCSR_SPEED_MASK      0x001CU
#define PHY_PHYSCSR_100BTX_FD       0x0018U
#define PHY_PHYSCSR_100BTX_HD       0x0008U
#define PHY_PHYSCSR_10BT_FD         0x0014U
#define PHY_PHYSCSR_10BT_HD         0x0004U

#define PHY_RESET_DELAY_MS          100U
#define PHY_AUTONEGO_TIMEOUT_MS     3000U
#define PHY_LINK_CHECK_INTERVAL_MS  1000U

/* ========================================================================= */
/*  Forward declarations                                                     */
/* ========================================================================= */

static void low_level_init(struct netif *netif);
static err_t low_level_output(struct netif *netif, struct pbuf *p);
static void ethernetif_rx_task(void *arg);

/* ========================================================================= */
/*  HAL v1.11.3 Rx Callbacks                                                 */
/* ========================================================================= */

/**
 * HAL calls this when it needs a fresh Rx buffer for a DMA descriptor.
 * We provide the next pre-allocated buffer from Rx_Buff[] in D2 SRAM.
 */
void HAL_ETH_RxAllocateCallback(uint8_t **buff)
{
    *buff = Rx_Buff[s_rx_buf_idx];
    s_rx_buf_idx = (s_rx_buf_idx + 1) % ETH_RX_DESC_CNT;
}

/**
 * HAL calls this for each buffer in a received frame.
 * We invalidate D-Cache, allocate a pbuf, copy data, and chain it.
 *
 * @param pStart  Pointer to head of pbuf chain (set on first call)
 * @param pEnd    Pointer to tail of pbuf chain (updated each call)
 * @param buff    Raw DMA buffer containing received data
 * @param Length  Length of data in this buffer
 */
void HAL_ETH_RxLinkCallback(void **pStart, void **pEnd,
                             uint8_t *buff, uint16_t Length)
{
    struct pbuf *p;

    /* D-Cache invalidate: DMA wrote to D2 SRAM, ensure CPU reads fresh data */
    SCB_InvalidateDCache_by_Addr((uint32_t *)buff, (int32_t)Length);

    p = pbuf_alloc(PBUF_RAW, Length, PBUF_POOL);
    if (p != NULL) {
        memcpy(p->payload, buff, Length);

        if (*pStart == NULL) {
            *pStart = p;
        }

        if (*pEnd != NULL) {
            /* Chain: previous tail -> this pbuf */
            pbuf_cat((struct pbuf *)*pEnd, p);
        }

        *pEnd = p;
        LINK_STATS_INC(link.recv);
    } else {
        LINK_STATS_INC(link.memerr);
        LINK_STATS_INC(link.drop);
    }
}

/* ========================================================================= */
/*  Low-level Init                                                           */
/* ========================================================================= */

static void low_level_init(struct netif *netif)
{
    /* Set MAC address from board_config.h */
    netif->hwaddr[0] = ETH_MAC_ADDR0;
    netif->hwaddr[1] = ETH_MAC_ADDR1;
    netif->hwaddr[2] = ETH_MAC_ADDR2;
    netif->hwaddr[3] = ETH_MAC_ADDR3;
    netif->hwaddr[4] = ETH_MAC_ADDR4;
    netif->hwaddr[5] = ETH_MAC_ADDR5;
    netif->hwaddr_len = ETH_HWADDR_LEN;
    netif->mtu = 1500;
    netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

    /* Clear DMA descriptors before HAL init */
    memset(DMATxDscrTab, 0, sizeof(DMATxDscrTab));
    memset(DMARxDscrTab, 0, sizeof(DMARxDscrTab));

    /* ETH HAL init */
    heth.Instance            = ETH;
    heth.Init.MACAddr        = netif->hwaddr;
    heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
    heth.Init.TxDesc         = DMATxDscrTab;
    heth.Init.RxDesc         = DMARxDscrTab;
    heth.Init.RxBuffLen      = ETH_MAX_PACKET_SIZE;

    if (HAL_ETH_Init(&heth) != HAL_OK) {
        return;
    }

    /* Configure Tx packet template (checksum offload) */
    memset(&s_tx_config, 0, sizeof(ETH_TxPacketConfigTypeDef));
    s_tx_config.Attributes   = ETH_TX_PACKETS_FEATURES_CSUM
                             | ETH_TX_PACKETS_FEATURES_CRCPAD;
    s_tx_config.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
    s_tx_config.CRCPadCtrl   = ETH_CRC_PAD_INSERT;

    /* Create Rx semaphore */
    s_rx_semaphore = xSemaphoreCreateBinaryStatic(&s_rx_sem_buf);

    /* Create Rx input task */
    xTaskCreate(ethernetif_rx_task, "EthRx", 512, netif,
                configMAX_PRIORITIES - 2, &s_rx_task_handle);

    /* PHY init: hardware reset if reset pin is available */
#if defined(ETH_PHY_RESET_PORT) && defined(ETH_PHY_RESET_PIN)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitTypeDef gpio = {0};
        gpio.Pin   = ETH_PHY_RESET_PIN;
        gpio.Mode  = GPIO_MODE_OUTPUT_PP;
        gpio.Pull  = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(ETH_PHY_RESET_PORT, &gpio);

        HAL_GPIO_WritePin(ETH_PHY_RESET_PORT, ETH_PHY_RESET_PIN, GPIO_PIN_RESET);
        HAL_Delay(PHY_RESET_DELAY_MS);
        HAL_GPIO_WritePin(ETH_PHY_RESET_PORT, ETH_PHY_RESET_PIN, GPIO_PIN_SET);
        HAL_Delay(PHY_RESET_DELAY_MS);
    }
#endif

    /* PHY software reset */
    HAL_ETH_WritePHYRegister(&heth, ETH_PHY_ADDRESS, PHY_BCR,
                             PHY_BCR_SOFT_RESET);
    HAL_Delay(PHY_RESET_DELAY_MS);

    /* Enable auto-negotiation */
    HAL_ETH_WritePHYRegister(&heth, ETH_PHY_ADDRESS, PHY_BCR,
                             PHY_BCR_AUTONEGO_EN | PHY_BCR_RESTART_AUTONEGO);

    /* Wait for auto-negotiation to complete */
    {
        uint32_t reg_val = 0;
        uint32_t tickstart = HAL_GetTick();

        while ((reg_val & PHY_BSR_AUTONEGO_DONE) == 0) {
            HAL_ETH_ReadPHYRegister(&heth, ETH_PHY_ADDRESS, PHY_BSR, &reg_val);

            if ((HAL_GetTick() - tickstart) > PHY_AUTONEGO_TIMEOUT_MS) {
                break;
            }
        }
    }

    /* Read link status and configure MAC speed/duplex */
    {
        uint32_t physcsr = 0;
        HAL_ETH_ReadPHYRegister(&heth, ETH_PHY_ADDRESS, PHY_PHYSCSR, &physcsr);

        ETH_MACConfigTypeDef mac_config;
        HAL_ETH_GetMACConfig(&heth, &mac_config);

        uint32_t speed_mode = physcsr & PHY_PHYSCSR_SPEED_MASK;
        if (speed_mode == PHY_PHYSCSR_100BTX_FD) {
            mac_config.DuplexMode = ETH_FULLDUPLEX_MODE;
            mac_config.Speed      = ETH_SPEED_100M;
        } else if (speed_mode == PHY_PHYSCSR_100BTX_HD) {
            mac_config.DuplexMode = ETH_HALFDUPLEX_MODE;
            mac_config.Speed      = ETH_SPEED_100M;
        } else if (speed_mode == PHY_PHYSCSR_10BT_FD) {
            mac_config.DuplexMode = ETH_FULLDUPLEX_MODE;
            mac_config.Speed      = ETH_SPEED_10M;
        } else {
            mac_config.DuplexMode = ETH_HALFDUPLEX_MODE;
            mac_config.Speed      = ETH_SPEED_10M;
        }

        HAL_ETH_SetMACConfig(&heth, &mac_config);
    }

    /* Check link status — if up, start ETH */
    {
        uint32_t bsr = 0;
        HAL_ETH_ReadPHYRegister(&heth, ETH_PHY_ADDRESS, PHY_BSR, &bsr);

        if (bsr & PHY_BSR_LINK_STATUS) {
            netif->flags |= NETIF_FLAG_LINK_UP;
            HAL_ETH_Start_IT(&heth);
        }
    }
}

/* ========================================================================= */
/*  Low-level Output — Send one Ethernet frame                               */
/* ========================================================================= */

/**
 * Transmit a pbuf chain as one Ethernet frame.
 *
 * Walks the pbuf chain, builds an ETH_BufferTypeDef linked list,
 * then calls HAL_ETH_Transmit() (blocking, suitable for FreeRTOS task context).
 *
 * Cache clean is needed because Tx data lives in cacheable AXI SRAM
 * but ETH DMA reads from physical memory.
 */
static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
    (void)netif;

    uint32_t frame_len = 0;
    ETH_BufferTypeDef tx_buf[ETH_TX_DESC_CNT];
    uint32_t buf_idx = 0;

    memset(tx_buf, 0, sizeof(tx_buf));

    /* Walk pbuf chain, build scatter-gather buffer list */
    for (struct pbuf *q = p; q != NULL; q = q->next) {
        if (buf_idx >= ETH_TX_DESC_CNT) {
            return ERR_MEM;
        }

        tx_buf[buf_idx].buffer = q->payload;
        tx_buf[buf_idx].len    = q->len;

        frame_len += q->len;

        if (q->next != NULL) {
            tx_buf[buf_idx].next = &tx_buf[buf_idx + 1];
        } else {
            tx_buf[buf_idx].next = NULL;
        }

        buf_idx++;
    }

    /* Configure Tx packet */
    s_tx_config.Length   = frame_len;
    s_tx_config.TxBuffer = tx_buf;
    s_tx_config.pData    = p;

    /* D-Cache clean: ensure DMA sees latest data from CPU cache */
    for (struct pbuf *q = p; q != NULL; q = q->next) {
        SCB_CleanDCache_by_Addr((uint32_t *)q->payload,
                                (int32_t)q->len);
    }

    /* Blocking transmit (waits for DMA completion) */
    pbuf_ref(p);  /* Hold reference during DMA */

    HAL_StatusTypeDef status = HAL_ETH_Transmit(&heth, &s_tx_config, 100);

    pbuf_free(p);  /* Release our reference */

    if (status != HAL_OK) {
        return ERR_IF;
    }

    LINK_STATS_INC(link.xmit);
    return ERR_OK;
}

/* ========================================================================= */
/*  Netif Init Callback                                                      */
/* ========================================================================= */

err_t ethernetif_init(struct netif *netif)
{
    netif->name[0] = 'e';
    netif->name[1] = '0';

#if LWIP_NETIF_HOSTNAME
    netif->hostname = "stm32h7-robot";
#endif

    netif->output     = etharp_output;
    netif->linkoutput = low_level_output;

    s_netif_ref = netif;

    low_level_init(netif);

    return ERR_OK;
}

/* ========================================================================= */
/*  Rx Input Processing                                                      */
/* ========================================================================= */

/**
 * Process received Ethernet frames — called from Rx task.
 *
 * Uses HAL v1.11.3 HAL_ETH_ReadData() which internally calls
 * HAL_ETH_RxAllocateCallback() and HAL_ETH_RxLinkCallback()
 * to build pbuf chains from DMA buffers.
 */
void ethernetif_input(struct netif *netif)
{
    void *app_buff = NULL;

    while (HAL_ETH_ReadData(&heth, &app_buff) == HAL_OK) {
        struct pbuf *p = (struct pbuf *)app_buff;
        if (p != NULL) {
            if (netif->input(p, netif) != ERR_OK) {
                pbuf_free(p);
            }
        }
    }
}

/**
 * FreeRTOS task that waits for ETH Rx interrupt and processes frames.
 *
 * This decouples frame processing from ISR context, allowing LwIP
 * to use FreeRTOS synchronization safely.
 */
static void ethernetif_rx_task(void *arg)
{
    struct netif *netif = (struct netif *)arg;

    for (;;) {
        /* Block until ETH Rx ISR signals a frame is available */
        if (xSemaphoreTake(s_rx_semaphore, pdMS_TO_TICKS(PHY_LINK_CHECK_INTERVAL_MS))
            == pdTRUE) {
            ethernetif_input(netif);
        }
    }
}

/* ========================================================================= */
/*  HAL ETH Callbacks (called from ISR context via ETH_IRQHandler)           */
/* ========================================================================= */

/**
 * Frame received callback — called from HAL_ETH_IRQHandler().
 * Gives semaphore to wake the Rx task.
 */
void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth_ref)
{
    (void)heth_ref;

    if (s_rx_semaphore != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(s_rx_semaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * ETH error callback — attempt to recover from DMA errors.
 */
void HAL_ETH_ErrorCallback(ETH_HandleTypeDef *heth_ref)
{
    if ((HAL_ETH_GetError(heth_ref) & HAL_ETH_ERROR_DMA) != 0) {
        if ((HAL_ETH_GetDMAError(heth_ref) & ETH_DMA_RX_BUFFER_UNAVAILABLE_FLAG) != 0) {
            /* Rx buffer unavailable — signal Rx task to process pending frames */
        }
    }

    /* Signal Rx task to check for any pending frames */
    if (s_rx_semaphore != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(s_rx_semaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* ========================================================================= */
/*  Link Check — called periodically                                         */
/* ========================================================================= */

/**
 * Check Ethernet link status via PHY BSR register.
 * Updates netif link state and starts/stops ETH accordingly.
 *
 * Should be called periodically (e.g., every 1s from a timer or task).
 */
void ethernet_link_check(struct netif *netif)
{
    uint32_t bsr = 0;

    if (HAL_ETH_ReadPHYRegister(&heth, ETH_PHY_ADDRESS, PHY_BSR, &bsr) != HAL_OK) {
        return;
    }

    if (bsr & PHY_BSR_LINK_STATUS) {
        /* Link is up */
        if (!netif_is_link_up(netif)) {
            /* Link just came up — re-read speed/duplex from PHY */
            uint32_t physcsr = 0;
            HAL_ETH_ReadPHYRegister(&heth, ETH_PHY_ADDRESS, PHY_PHYSCSR, &physcsr);

            ETH_MACConfigTypeDef mac_config;
            HAL_ETH_GetMACConfig(&heth, &mac_config);

            uint32_t speed_mode = physcsr & PHY_PHYSCSR_SPEED_MASK;
            if (speed_mode == PHY_PHYSCSR_100BTX_FD) {
                mac_config.DuplexMode = ETH_FULLDUPLEX_MODE;
                mac_config.Speed      = ETH_SPEED_100M;
            } else if (speed_mode == PHY_PHYSCSR_100BTX_HD) {
                mac_config.DuplexMode = ETH_HALFDUPLEX_MODE;
                mac_config.Speed      = ETH_SPEED_100M;
            } else if (speed_mode == PHY_PHYSCSR_10BT_FD) {
                mac_config.DuplexMode = ETH_FULLDUPLEX_MODE;
                mac_config.Speed      = ETH_SPEED_10M;
            } else {
                mac_config.DuplexMode = ETH_HALFDUPLEX_MODE;
                mac_config.Speed      = ETH_SPEED_10M;
            }

            HAL_ETH_SetMACConfig(&heth, &mac_config);
            HAL_ETH_Start_IT(&heth);
            netif_set_link_up(netif);
            netif_set_up(netif);
        }
    } else {
        /* Link is down */
        if (netif_is_link_up(netif)) {
            HAL_ETH_Stop_IT(&heth);
            netif_set_link_down(netif);
        }
    }
}

void ethernet_link_periodic(void)
{
    ethernet_link_check(&gnetif);
}

/* ========================================================================= */
/*  LwIP Stack Init                                                          */
/* ========================================================================= */

void MX_LWIP_Init(void)
{
    ip_addr_t ipaddr, netmask, gw;

    tcpip_init(NULL, NULL);

    IP4_ADDR(&ipaddr,  NET_IP_ADDR0, NET_IP_ADDR1, NET_IP_ADDR2, NET_IP_ADDR3);
    IP4_ADDR(&netmask, NET_NETMASK0, NET_NETMASK1, NET_NETMASK2, NET_NETMASK3);
    IP4_ADDR(&gw,      NET_GW_ADDR0, NET_GW_ADDR1, NET_GW_ADDR2, NET_GW_ADDR3);

    netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);
    netif_set_default(&gnetif);

    if (netif_is_link_up(&gnetif)) {
        netif_set_up(&gnetif);
    }
}
