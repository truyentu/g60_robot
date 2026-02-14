/**
 * @file ethernetif.c
 * @brief LwIP Ethernet interface — STM32H7 HAL ETH driver
 *
 * Skeleton implementation. Full functionality requires:
 * - Drivers/STM32H7xx_HAL_Driver populated
 * - PHY driver (LAN8742 or your specific PHY)
 *
 * DMA descriptors and buffers are placed in D2 SRAM (SRAM1)
 * via linker section attributes — critical for STM32H7!
 */

#include "ethernetif.h"
#include "board_config.h"
#include "stm32h7xx_hal.h"

#include "lwip/opt.h"
#include "lwip/tcpip.h"
#include "lwip/netif.h"
#include "lwip/ip_addr.h"
#include "netif/etharp.h"
#include "lwip/udp.h"

#include <string.h>

/* ========================================================================= */
/*  Ethernet Handle (global — used by stm32h7xx_it.c)                       */
/* ========================================================================= */

ETH_HandleTypeDef heth;

/* ========================================================================= */
/*  DMA Descriptors & Buffers — MUST be in D2 SRAM (0x30000000)              */
/* ========================================================================= */

#if defined(__GNUC__)
  /* GCC section attributes for linker script placement */
  ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection")));
  ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));
  uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection")));
#endif

/* Network interface */
static struct netif gnetif;

/* ========================================================================= */
/*  PHY Configuration                                                        */
/* ========================================================================= */
/*
 * === PHY CONFIG (LAN8742) ===
 *
 * If your board uses a different PHY (DP83848, KSZ8081, etc.),
 * change the register addresses and init sequence below.
 *
 * LAN8742 key registers:
 *   BCR (0x00): Basic Control Register
 *   BSR (0x01): Basic Status Register
 *   PHYI1 (0x02): PHY Identifier 1
 *   PHYI2 (0x03): PHY Identifier 2
 *   PHYSCSR (0x1F): PHY Special Control/Status Register
 */

#define LAN8742_BCR         0x00
#define LAN8742_BSR         0x01
#define LAN8742_PHYSCSR     0x1F

#define LAN8742_BCR_AUTONEGO_EN     0x1000
#define LAN8742_BSR_AUTONEGO_DONE   0x0020
#define LAN8742_BSR_LINK_STATUS     0x0004

/* ========================================================================= */
/*  Low-level Init                                                           */
/* ========================================================================= */

static void low_level_init(struct netif *netif)
{
    /* Set MAC address */
    netif->hwaddr[0] = ETH_MAC_ADDR0;
    netif->hwaddr[1] = ETH_MAC_ADDR1;
    netif->hwaddr[2] = ETH_MAC_ADDR2;
    netif->hwaddr[3] = ETH_MAC_ADDR3;
    netif->hwaddr[4] = ETH_MAC_ADDR4;
    netif->hwaddr[5] = ETH_MAC_ADDR5;
    netif->hwaddr_len = ETH_HWADDR_LEN;
    netif->mtu = 1500;
    netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;

    /* ETH HAL init */
    heth.Instance            = ETH;
    heth.Init.MACAddr        = netif->hwaddr;
    heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
    heth.Init.TxDesc         = DMATxDscrTab;
    heth.Init.RxDesc         = DMARxDscrTab;
    heth.Init.RxBuffLen      = ETH_MAX_PACKET_SIZE;

    if (HAL_ETH_Init(&heth) != HAL_OK) {
        /* Init error — Error_Handler() or retry */
        return;
    }

    /* Assign Rx buffers to DMA descriptors */
    for (int i = 0; i < ETH_RX_DESC_CNT; i++) {
        HAL_ETH_DescAssignMemory(&heth, i, Rx_Buff[i], NULL);
    }

    /* Start Ethernet */
    HAL_ETH_Start_IT(&heth);
}

/* ========================================================================= */
/*  Netif Init Callback                                                      */
/* ========================================================================= */

err_t ethernetif_init(struct netif *netif)
{
    netif->name[0] = 'e';
    netif->name[1] = '0';
    netif->output     = etharp_output;
    netif->linkoutput  = NULL;  /* TODO: implement low_level_output() */

    low_level_init(netif);
    return ERR_OK;
}

/* ========================================================================= */
/*  Input Processing                                                         */
/* ========================================================================= */

void ethernetif_input(struct netif *netif)
{
    /* TODO: Implement frame reception from HAL ETH DMA
     * 1. HAL_ETH_ReadData(&heth, (void**)&buffer)
     * 2. Allocate pbuf, copy data
     * 3. Pass to netif->input(p, netif)
     */
    (void)netif;
}

/* ========================================================================= */
/*  Link Check                                                               */
/* ========================================================================= */

void ethernet_link_check(struct netif *netif)
{
    /* TODO: Read PHY BSR register, update netif link status
     * uint32_t regval;
     * HAL_ETH_ReadPHYRegister(&heth, ETH_PHY_ADDRESS, LAN8742_BSR, &regval);
     * if (regval & LAN8742_BSR_LINK_STATUS) netif_set_link_up(netif);
     * else netif_set_link_down(netif);
     */
    (void)netif;
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
    netif_set_up(&gnetif);
}
