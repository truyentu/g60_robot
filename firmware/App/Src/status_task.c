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
#include "board_watchdog.h"
#include "board_gpio.h"
#include "motion_task.h"
#include "motion_buffer.h"
#include "drive_control.h"
#include "net_task.h"
#include "safety_monitor.h"
#include "ethernetif.h"
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

    /* Transmit buffer for serialized packet (large enough for any response) */
    uint8_t tx_buf[PROTO_HEADER_SIZE + sizeof(StatusPacket) + PROTO_CRC_SIZE];
    uint8_t rsp_seq = 0;

    /* Link check counter: 100 iterations × 10ms = 1s */
    uint8_t link_check_cnt = 0;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        /* 10ms period (100 Hz) */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));

        /* Kick the hardware watchdog */
        Board_KickWatchdog();

        /* Periodic Ethernet link check (~1s interval) */
        if (++link_check_cnt >= 100) {
            link_check_cnt = 0;
            ethernet_link_periodic();
        }

        /* Update drive GPIO inputs (ready/alarm) */
        DriveControl_Update();

        /* Update safety monitor */
        SafetyMonitor_Update();

        /* Communication watchdog — check heartbeat timeout */
        {
            uint32_t now = HAL_GetTick();
            uint32_t last_hb = NetTask_GetLastHeartbeatTick();
            if (s_pc_known && last_hb > 0 &&
                (now - last_hb) > COMM_TIMEOUT_MS) {
                /* Communication lost — flush motion buffer, set alarm */
                if (Motion_GetState() == SYSTEM_STATE_MOVING ||
                    Motion_GetState() == SYSTEM_STATE_JOGGING) {
                    Motion_SetState(SYSTEM_STATE_ALARM);
                }
            }
        }

        /* Build StatusPacket */
        StatusPacket_Clear(&s_status);
        s_status.seq          = s_seq++;
        s_status.state        = Motion_GetState();
        s_status.timestamp_us = HAL_GetTick() * 1000;  /* Approximate us */

        /* Get current positions from motion task */
        Motion_GetPositions(s_status.cmd_pos);

        /* Get actual positions (float → int32 for StatusPacket)
         * SIMULATION_MODE: Perfect Servo — actual = commanded
         * Real Mode: will read from hardware encoders */
        {
            float actual_f[PROTO_NUM_AXES];
            MotionTask_GetPosition(actual_f);
            for (int i = 0; i < PROTO_NUM_AXES; i++) {
                s_status.actual_pos[i] = (int32_t)actual_f[i];
            }
        }

        /* Drive status from DriveControl */
        s_status.drive_ready    = DriveControl_GetReadyMask();
        s_status.drive_alarm    = DriveControl_GetAlarmMask();
        s_status.home_status    = DriveControl_GetHomedMask();
        s_status.digital_inputs = GPIO_ReadDigitalInputs();
        s_status.digital_outputs = GPIO_ReadDigitalOutputs();
        s_status.pvt_buffer_lvl = (uint8_t)((MotionBuffer_GetLevel() * 255U) / MOTION_BUFFER_CAPACITY);

        /* Velocity from motion task */
        Motion_GetVelocities(s_status.velocity);

        /* Only send if PC address is known */
        if (!s_pc_known) {
            continue;
        }

        /* Check for motion complete event → send RSP_MOTION_COMPLETE */
        if (Motion_IsComplete()) {
            uint8_t mc_buf[PROTO_HEADER_SIZE + 1 + PROTO_CRC_SIZE];
            uint8_t mc_payload = 0x01;  /* success */
            size_t mc_len = protocol_serialize(mc_buf, sizeof(mc_buf),
                                                rsp_seq++, RSP_MOTION_COMPLETE,
                                                &mc_payload, 1);
            if (mc_len > 0) {
                struct netbuf *mc_nb = netbuf_new();
                if (mc_nb) {
                    void *mc_data = netbuf_alloc(mc_nb, (u16_t)mc_len);
                    if (mc_data) {
                        memcpy(mc_data, mc_buf, mc_len);
                        netconn_sendto(conn, mc_nb, &s_pc_addr, s_pc_port);
                    }
                    netbuf_delete(mc_nb);
                }
            }
        }

        /* Check for alarm event → send RSP_ALARM */
        {
            uint8_t alarm_axis, alarm_code;
            if (Motion_GetAlarmPending(&alarm_axis, &alarm_code)) {
                AlarmPacket ap;
                ap.axis = alarm_axis;
                ap.alarm_code = alarm_code;
                ap.timestamp_us = HAL_GetTick() * 1000;

                uint8_t al_buf[PROTO_HEADER_SIZE + sizeof(AlarmPacket) + PROTO_CRC_SIZE];
                size_t al_len = protocol_serialize(al_buf, sizeof(al_buf),
                                                    rsp_seq++, RSP_ALARM,
                                                    (const uint8_t *)&ap, sizeof(ap));
                if (al_len > 0) {
                    struct netbuf *al_nb = netbuf_new();
                    if (al_nb) {
                        void *al_data = netbuf_alloc(al_nb, (u16_t)al_len);
                        if (al_data) {
                            memcpy(al_data, al_buf, al_len);
                            netconn_sendto(conn, al_nb, &s_pc_addr, s_pc_port);
                        }
                        netbuf_delete(al_nb);
                    }
                }
            }
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
