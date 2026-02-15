/**
 * @file ethernetif.h
 * @brief LwIP Ethernet interface header
 */

#ifndef ETHERNETIF_H
#define ETHERNETIF_H

#include "lwip/netif.h"

/**
 * Initialize LwIP stack with static IP from board_config.h.
 * Creates TCPIP thread and registers Ethernet netif.
 */
void MX_LWIP_Init(void);

/**
 * LwIP netif init callback — sets up the Ethernet interface.
 */
err_t ethernetif_init(struct netif *netif);

/**
 * Process received Ethernet frames (called from ETH IRQ or polling).
 */
void ethernetif_input(struct netif *netif);

/**
 * Check Ethernet link status and update netif accordingly.
 */
void ethernet_link_check(struct netif *netif);

/**
 * Periodic link check using internal gnetif.
 * Call this from a task (e.g., StatusTask) every ~1s.
 */
void ethernet_link_periodic(void);

#endif /* ETHERNETIF_H */
