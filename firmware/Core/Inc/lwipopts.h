/**
 * @file lwipopts.h
 * @brief LwIP configuration for STM32H743 + FreeRTOS (UDP only)
 *
 * Optimized for robot controller: UDP command/status, no TCP overhead.
 */

#ifndef LWIPOPTS_H
#define LWIPOPTS_H

/* ========================================================================= */
/*  OS Integration (FreeRTOS)                                                */
/* ========================================================================= */

#define NO_SYS                          0       /* Using OS (FreeRTOS) */
#define LWIP_NETCONN                    1       /* Netconn API enabled */
#define LWIP_SOCKET                     0       /* Socket API disabled (use netconn) */

/* TCPIP thread (LwIP internal) */
#define TCPIP_THREAD_STACKSIZE          1024    /* Words */
#define TCPIP_THREAD_PRIO               4       /* Same as NetTask */
#define TCPIP_MBOX_SIZE                 16
#define DEFAULT_THREAD_STACKSIZE        512

/* ========================================================================= */
/*  Protocol Selection                                                       */
/* ========================================================================= */

#define LWIP_UDP                        1       /* UDP enabled */
#define LWIP_TCP                        0       /* TCP disabled — UDP only */
#define LWIP_ICMP                       1       /* Ping support (debugging) */
#define LWIP_RAW                        0
#define LWIP_IGMP                       0

/* ========================================================================= */
/*  Memory Configuration                                                     */
/* ========================================================================= */

/* Heap memory */
#define MEM_SIZE                        (16 * 1024)     /* 16KB LwIP heap */
#define MEM_ALIGNMENT                   4

/* Memory pools */
#define MEMP_NUM_PBUF                   16
#define MEMP_NUM_UDP_PCB                4
#define MEMP_NUM_NETCONN                4
#define MEMP_NUM_NETBUF                 8
#define MEMP_NUM_TCPIP_MSG_INPKT        16
#define MEMP_NUM_TCPIP_MSG_API          16
#define MEMP_NUM_SYS_TIMEOUT            10

/* Packet buffers */
#define PBUF_POOL_SIZE                  16
#define PBUF_POOL_BUFSIZE               1536    /* Ethernet MTU + headers */

/* ========================================================================= */
/*  IP Configuration (Static)                                                */
/* ========================================================================= */

#define LWIP_DHCP                       0       /* No DHCP — static IP */
#define LWIP_AUTOIP                     0

/* Static IP set via board_config.h macros in ethernetif.c */

/* ========================================================================= */
/*  Netconn / Mbox Configuration                                             */
/* ========================================================================= */

#define DEFAULT_UDP_RECVMBOX_SIZE       16
#define DEFAULT_ACCEPTMBOX_SIZE         4

/* ========================================================================= */
/*  Checksum (Hardware Offload on STM32H7)                                   */
/* ========================================================================= */
/* STM32H7 Ethernet MAC supports hardware checksum generation/verification   */

#define CHECKSUM_GEN_IP                 0       /* Hardware generates */
#define CHECKSUM_GEN_UDP                0
#define CHECKSUM_GEN_ICMP               0
#define CHECKSUM_CHECK_IP               0       /* Hardware verifies */
#define CHECKSUM_CHECK_UDP              0
#define CHECKSUM_CHECK_ICMP             0

/* ========================================================================= */
/*  ARP Configuration                                                        */
/* ========================================================================= */

#define LWIP_ARP                        1
#define ARP_TABLE_SIZE                  10
#define ARP_QUEUEING                    1
#define ETHARP_SUPPORT_STATIC_ENTRIES   1

/* ========================================================================= */
/*  Misc                                                                     */
/* ========================================================================= */

#define LWIP_NETIF_LINK_CALLBACK        1
#define LWIP_NETIF_STATUS_CALLBACK      1
#define ETH_PAD_SIZE                    0       /* No padding */
#define LWIP_STATS                      0       /* Disable stats (save RAM) */
#define LWIP_PROVIDE_ERRNO              1

/* ========================================================================= */
/*  Debug (disabled by default, enable as needed)                            */
/* ========================================================================= */

/* #define LWIP_DEBUG                   1 */
/* #define UDP_DEBUG                    LWIP_DBG_ON */
/* #define ETHARP_DEBUG                 LWIP_DBG_ON */
/* #define NETIF_DEBUG                  LWIP_DBG_ON */

#endif /* LWIPOPTS_H */
