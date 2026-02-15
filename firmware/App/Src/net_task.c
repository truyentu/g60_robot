/**
 * @file net_task.c
 * @brief Network task — UDP command reception, validation, and dispatch
 *
 * Uses LwIP Netconn API (thread-safe for FreeRTOS).
 *
 * Binds to UDP port NET_UDP_CMD_PORT (board_config.h).
 * Receives packets, validates sync word + CRC, dispatches commands.
 *
 * System commands handled internally:
 *   - CMD_HEARTBEAT  → replies with StatusPacket (state=IDLE)
 *   - CMD_GET_VERSION → replies with VersionPacket
 *   - CMD_E_STOP      → sets E-stop state
 *
 * All other commands are forwarded to the registered CommandHandler.
 */

#include "net_task.h"
#include "app_tasks.h"
#include "board_config.h"
#include "motion_task.h"
#include "motion_buffer.h"
#include "drive_control.h"
#include "status_task.h"
#include "protocol_packet.h"
#include "protocol_commands.h"
#include "protocol_status.h"
#include "protocol_defs.h"

#include "FreeRTOS.h"
#include "task.h"

#include "lwip/api.h"
#include "lwip/netbuf.h"
#include "lwip/ip_addr.h"

#include "stm32h7xx_hal.h"
#include "main.h"

#include <string.h>

/* ========================================================================= */
/*  State                                                                    */
/* ========================================================================= */

/* Command handler callback for non-system commands */
static CommandHandler s_handler = NULL;

/* Connection handle (module-level for reply sending) */
static struct netconn *s_conn = NULL;

/* Sender address of last received packet */
static ip_addr_t s_sender_addr;
static uint16_t  s_sender_port = 0;
static volatile uint8_t s_sender_known = 0;

/* Statistics */
static volatile uint32_t s_packets_received = 0;
static volatile uint32_t s_packets_invalid  = 0;
static volatile uint32_t s_last_heartbeat_tick = 0;

/* Transmit buffer for replies */
static uint8_t s_tx_buf[PROTO_MAX_PACKET];

/* Sequence counter for outgoing replies */
static uint8_t s_reply_seq = 0;

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

void NetTask_RegisterHandler(CommandHandler handler)
{
    s_handler = handler;
}

bool NetTask_GetPCAddress(ip_addr_t *addr_out, uint16_t *port_out)
{
    if (!s_sender_known) {
        return false;
    }
    if (addr_out) {
        ip_addr_copy(*addr_out, s_sender_addr);
    }
    if (port_out) {
        *port_out = s_sender_port;
    }
    return true;
}

uint32_t NetTask_GetLastHeartbeatTick(void)
{
    return s_last_heartbeat_tick;
}

uint32_t NetTask_GetPacketsReceived(void)
{
    return s_packets_received;
}

uint32_t NetTask_GetPacketsInvalid(void)
{
    return s_packets_invalid;
}

/* ========================================================================= */
/*  Internal: Send reply packet to sender                                    */
/* ========================================================================= */

/**
 * Send a reply packet back to the last sender.
 *
 * @param rsp_type  Response type (RSP_STATUS, RSP_ACK, etc.)
 * @param payload   Payload data (NULL for empty)
 * @param payload_len Payload length
 */
static void NetTask_SendReply(uint8_t rsp_type,
                              const uint8_t *payload, uint16_t payload_len)
{
    if (!s_conn || !s_sender_known) {
        return;
    }

    size_t pkt_len = protocol_serialize(s_tx_buf, sizeof(s_tx_buf),
                                         s_reply_seq++, rsp_type,
                                         payload, payload_len);
    if (pkt_len == 0) {
        return;
    }

    struct netbuf *buf = netbuf_new();
    if (buf == NULL) {
        return;
    }

    void *data = netbuf_alloc(buf, (u16_t)pkt_len);
    if (data != NULL) {
        memcpy(data, s_tx_buf, pkt_len);
        netconn_sendto(s_conn, buf, &s_sender_addr, s_sender_port);
    }

    netbuf_delete(buf);
}

/**
 * Send ACK for a received command.
 */
static void NetTask_SendAck(uint8_t cmd_seq, uint8_t cmd_type, uint8_t error_code)
{
    AckPacket ack;
    ack.cmd_seq    = cmd_seq;
    ack.cmd_type   = cmd_type;
    ack.error_code = error_code;

    NetTask_SendReply(RSP_ACK, (const uint8_t *)&ack, sizeof(ack));
}

/* ========================================================================= */
/*  Internal: Command Handlers                                               */
/* ========================================================================= */

/**
 * Handle CMD_HEARTBEAT: reply with a StatusPacket showing current state.
 */
static void handle_heartbeat(const PacketHeader *hdr)
{
    s_last_heartbeat_tick = HAL_GetTick();

    /* Build a StatusPacket reply */
    StatusPacket sp;
    StatusPacket_Clear(&sp);
    sp.seq          = hdr->seq;
    sp.state        = Motion_GetState();
    sp.timestamp_us = HAL_GetTick() * 1000;
    sp.drive_ready  = DriveControl_GetReadyMask();
    sp.drive_alarm  = DriveControl_GetAlarmMask();

    /* Fill current positions */
    Motion_GetPositions(sp.cmd_pos);
    memcpy(sp.actual_pos, sp.cmd_pos, sizeof(sp.actual_pos));

    NetTask_SendReply(RSP_STATUS, (const uint8_t *)&sp, sizeof(sp));
}

/**
 * Handle CMD_GET_VERSION: reply with firmware version info.
 */
static void handle_get_version(const PacketHeader *hdr)
{
    VersionPacket vp;
    memset(&vp, 0, sizeof(vp));
    vp.major = 1;
    vp.minor = 0;
    vp.patch = 0;
    vp.build = 1;
    strncpy(vp.board, "STM32H743-G60", sizeof(vp.board) - 1);

    NetTask_SendReply(RSP_VERSION, (const uint8_t *)&vp, sizeof(vp));
}

/**
 * Dispatch a validated command.
 */
static void dispatch_command(const PacketHeader *hdr,
                             const uint8_t *payload, uint16_t payload_len)
{
    switch (hdr->type) {
    /* ---- System commands (handled here) ---- */
    case CMD_HEARTBEAT:
        handle_heartbeat(hdr);
        return;

    case CMD_GET_VERSION:
        handle_get_version(hdr);
        return;

    case CMD_E_STOP:
        /* ACK immediately, then let motion task handle the stop */
        NetTask_SendAck(hdr->seq, hdr->type, 0);
        if (s_handler) {
            s_handler(hdr->type, payload, payload_len);
        }
        return;

    /* ---- Motion / Jog / Homing / Drive / I/O commands ---- */

    /* PVT Point: parse and push into motion buffer */
    case CMD_PVT_POINT:
        if (payload_len >= sizeof(PVTPoint)) {
            const PVTPoint *pt = (const PVTPoint *)payload;
            if (MotionBuffer_Push(pt)) {
                NetTask_SendAck(hdr->seq, hdr->type, 0);
                /* Notify MotionTask that data is available */
                Motion_NotifyDataReady();
                /* Send buffer-low warning if needed */
                if (MotionBuffer_IsLow()) {
                    BufferStatusPacket bsp;
                    bsp.level    = MotionBuffer_GetLevel();
                    bsp.capacity = MOTION_BUFFER_CAPACITY;
                    NetTask_SendReply(RSP_BUFFER_LOW,
                                     (const uint8_t *)&bsp, sizeof(bsp));
                }
            } else {
                /* Buffer full — NACK with error 0x02 */
                Board_Debug_Print("PVT Buffer Overflow!\r\n");
                NetTask_SendAck(hdr->seq, hdr->type, 0x02);
            }
        } else {
            NetTask_SendAck(hdr->seq, hdr->type, 0x03); /* bad payload size */
        }
        return;

    /* PVT Batch: parse multiple PVTPoints from one packet */
    case CMD_PVT_BATCH: {
        uint16_t num_points = payload_len / sizeof(PVTPoint);
        uint16_t pushed = 0;
        const PVTPoint *pts = (const PVTPoint *)payload;

        for (uint16_t i = 0; i < num_points; i++) {
            if (!MotionBuffer_Push(&pts[i])) {
                Board_Debug_Print("PVT Buffer Overflow!\r\n");
                break;
            }
            pushed++;
        }

        if (pushed > 0) {
            Motion_NotifyDataReady();
        }

        if (pushed == num_points) {
            NetTask_SendAck(hdr->seq, hdr->type, 0);
        } else {
            /* Partial push — NACK with error 0x02 */
            NetTask_SendAck(hdr->seq, hdr->type, 0x02);
        }

        if (MotionBuffer_IsLow()) {
            BufferStatusPacket bsp;
            bsp.level    = MotionBuffer_GetLevel();
            bsp.capacity = MOTION_BUFFER_CAPACITY;
            NetTask_SendReply(RSP_BUFFER_LOW,
                             (const uint8_t *)&bsp, sizeof(bsp));
        }
        return;
    }

    case CMD_STOP_MOTION:
        /* Flush buffer on stop */
        MotionBuffer_Flush();
        NetTask_SendAck(hdr->seq, hdr->type, 0);
        if (s_handler) {
            s_handler(hdr->type, payload, payload_len);
        }
        return;

    case CMD_JOG_START:
        if (payload_len >= sizeof(JogStartCmd)) {
            const JogStartCmd *jog = (const JogStartCmd *)payload;
            Motion_JogStart(jog->axis, jog->direction, jog->speed);
            NetTask_SendAck(hdr->seq, hdr->type, 0);
        } else {
            NetTask_SendAck(hdr->seq, hdr->type, 0x03);
        }
        return;

    case CMD_JOG_STOP:
        Motion_JogStop();
        NetTask_SendAck(hdr->seq, hdr->type, 0);
        return;

    case CMD_MOVE_ABSOLUTE:
    case CMD_HOME_START:
    case CMD_HOME_SET_PARAMS:
    case CMD_HOME_STOP:
    case CMD_ENABLE_DRIVES:
    case CMD_DISABLE_DRIVES:
    case CMD_RESET_ALARM:
    case CMD_SET_AXIS_PARAMS:
    case CMD_SET_ACCEL:
    case CMD_SET_OUTPUT:
    case CMD_SET_OUTPUTS_BATCH:
        /* ACK the command */
        NetTask_SendAck(hdr->seq, hdr->type, 0);
        /* Forward to registered handler */
        if (s_handler) {
            s_handler(hdr->type, payload, payload_len);
        }
        return;

    default:
        /* Unknown command — NACK */
        NetTask_SendAck(hdr->seq, hdr->type, 0x01);
        return;
    }
}

/* ========================================================================= */
/*  NetTask — FreeRTOS Task Entry Point                                      */
/* ========================================================================= */

void NetTask(void *arg)
{
    (void)arg;

    struct netbuf *buf;
    err_t          err;

    /* Create UDP connection */
    s_conn = netconn_new(NETCONN_UDP);
    if (s_conn == NULL) {
        vTaskDelete(NULL);
        return;
    }

    /* Bind to command port (from board_config.h) */
    err = netconn_bind(s_conn, IP_ADDR_ANY, NET_UDP_CMD_PORT);
    if (err != ERR_OK) {
        netconn_delete(s_conn);
        s_conn = NULL;
        vTaskDelete(NULL);
        return;
    }

    /* ---- Main receive loop ---- */
    for (;;) {
        /* Block until a UDP packet arrives */
        err = netconn_recv(s_conn, &buf);
        if (err != ERR_OK) {
            continue;
        }

        /* Record sender address (for replies and StatusTask) */
        ip_addr_t *remote_addr = netbuf_fromaddr(buf);
        uint16_t   remote_port = netbuf_fromport(buf);

        ip_addr_copy(s_sender_addr, *remote_addr);
        s_sender_port = remote_port;

        if (!s_sender_known) {
            s_sender_known = 1;
            /* Notify StatusTask of the PC address so it can stream status */
            Status_SetPCAddress(&s_sender_addr, NET_UDP_STATUS_PORT);
        }

        /* Get data pointer and length from netbuf */
        void  *data;
        u16_t  data_len;
        netbuf_data(buf, &data, &data_len);

        /* Quick sanity: minimum packet size */
        if (data_len < PROTO_MIN_PACKET) {
            s_packets_invalid++;
            netbuf_delete(buf);
            continue;
        }

        /* Verify sync word before full parse (fast rejection) */
        uint16_t sync;
        memcpy(&sync, data, sizeof(sync));
        if (sync != PROTO_SYNC_WORD) {
            s_packets_invalid++;
            netbuf_delete(buf);
            continue;
        }

        /* Parse and validate packet (sync + CRC + length) */
        PacketHeader header;
        const uint8_t *payload;
        uint16_t payload_len;

        if (!protocol_parse((const uint8_t *)data, data_len,
                            &header, &payload, &payload_len)) {
            s_packets_invalid++;
            netbuf_delete(buf);
            continue;
        }

        /* Valid packet */
        s_packets_received++;

        /* Dispatch command */
        dispatch_command(&header, payload, payload_len);

        /* Free the netbuf */
        netbuf_delete(buf);
    }
}
