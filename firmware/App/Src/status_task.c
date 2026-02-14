/**
 * @file status_task.c
 * @brief Status task — periodic StatusPacket sender via UDP
 *
 * Sends a 74-byte StatusPacket to the PC every 10ms (100Hz).
 * Uses a separate UDP port (NET_UDP_STATUS_PORT = 5001).
 */

#include "status_task.h"
#include "app_tasks.h"
#include "board_config.h"
#include "motion_task.h"
#include "protocol_packet.h"
#include "protocol_status.h"
#include "protocol_defs.h"

#include "FreeRTOS.h"
#include "task.h"

#include "lwip/api.h"
#include "lwip/netbuf.h"
#include "lwip/ip_addr.h"

#include "stm32h7xx_hal.h"

#include <string.h>

/* ========================================================================= */
/*  State                                                                    */
/* ========================================================================= */

static StatusPacket s_status;
static uint8_t      s_seq = 0;

/* PC address — set when first command received, or hardcoded */
static ip_addr_t    s_pc_addr;
static uint16_t     s_pc_port = NET_UDP_STATUS_PORT;
static uint8_t      s_pc_known = 0;

const StatusPacket *Status_GetCurrent(void)
{
    return &s_status;
}

/**
 * Set the PC address (called by NetTask when first packet arrives).
 */
void Status_SetPCAddress(const ip_addr_t *addr, uint16_t port)
{
    ip_addr_copy(s_pc_addr, *addr);
    s_pc_port  = port;
    s_pc_known = 1;
}

/* ========================================================================= */
/*  StatusTask — 100Hz Loop                                                  */
/* ========================================================================= */

void StatusTask(void *arg)
{
    (void)arg;

    struct netconn *conn;
    struct netbuf  *buf;

    /* Create UDP connection for sending */
    conn = netconn_new(NETCONN_UDP);
    if (conn == NULL) {
        vTaskDelete(NULL);
        return;
    }

    /* Transmit buffer for serialized packet */
    uint8_t tx_buf[PROTO_HEADER_SIZE + sizeof(StatusPacket) + PROTO_CRC_SIZE];

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        /* 10ms period (100 Hz) */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));

        /* Build StatusPacket */
        StatusPacket_Clear(&s_status);
        s_status.seq          = s_seq++;
        s_status.state        = Motion_GetState();
        s_status.timestamp_us = HAL_GetTick() * 1000;  /* Approximate us */

        /* Get current positions from motion task */
        Motion_GetPositions(s_status.cmd_pos);
        /* actual_pos = cmd_pos for now (no encoder feedback yet) */
        memcpy(s_status.actual_pos, s_status.cmd_pos, sizeof(s_status.actual_pos));

        /* Drive status — placeholder (all ready, no alarms) */
        s_status.drive_ready  = 0x3F;
        s_status.drive_alarm  = 0x00;
        s_status.pvt_buffer_lvl = 0;

        /* Only send if PC address is known */
        if (!s_pc_known) {
            continue;
        }

        /* Serialize into binary protocol packet */
        size_t pkt_len = protocol_serialize(tx_buf, sizeof(tx_buf),
                                             s_status.seq,
                                             RSP_STATUS,
                                             (const uint8_t *)&s_status,
                                             sizeof(StatusPacket));
        if (pkt_len == 0) {
            continue;
        }

        /* Send via UDP */
        buf = netbuf_new();
        if (buf == NULL) {
            continue;
        }

        void *data = netbuf_alloc(buf, (u16_t)pkt_len);
        if (data != NULL) {
            memcpy(data, tx_buf, pkt_len);
            netconn_sendto(conn, buf, &s_pc_addr, s_pc_port);
        }

        netbuf_delete(buf);
    }
}
