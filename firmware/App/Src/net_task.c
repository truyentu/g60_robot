/**
 * @file net_task.c
 * @brief Network task — UDP command reception and dispatch
 *
 * Binds to UDP port NET_UDP_CMD_PORT (5000).
 * Receives packets, validates CRC, dispatches to command handler.
 */

#include "net_task.h"
#include "app_tasks.h"
#include "board_config.h"
#include "protocol_packet.h"
#include "protocol_defs.h"

#include "FreeRTOS.h"
#include "task.h"

#include "lwip/api.h"
#include "lwip/netbuf.h"

#include <string.h>

/* Command handler callback */
static CommandHandler s_handler = NULL;

void NetTask_RegisterHandler(CommandHandler handler)
{
    s_handler = handler;
}

/* ========================================================================= */
/*  NetTask — Main Loop                                                      */
/* ========================================================================= */

void NetTask(void *arg)
{
    (void)arg;

    struct netconn *conn;
    struct netbuf  *buf;
    err_t           err;

    /* Create UDP connection */
    conn = netconn_new(NETCONN_UDP);
    if (conn == NULL) {
        /* Fatal: cannot create netconn */
        vTaskDelete(NULL);
        return;
    }

    /* Bind to command port */
    err = netconn_bind(conn, IP_ADDR_ANY, NET_UDP_CMD_PORT);
    if (err != ERR_OK) {
        netconn_delete(conn);
        vTaskDelete(NULL);
        return;
    }

    /* Main receive loop */
    for (;;) {
        /* Block until a UDP packet arrives */
        err = netconn_recv(conn, &buf);
        if (err != ERR_OK) {
            continue;
        }

        /* Get data pointer and length from netbuf */
        void    *data;
        u16_t    data_len;
        netbuf_data(buf, &data, &data_len);

        /* Parse and validate packet */
        PacketHeader header;
        const uint8_t *payload;
        uint16_t payload_len;

        if (protocol_parse((const uint8_t *)data, data_len,
                           &header, &payload, &payload_len)) {
            /* Valid packet — dispatch to handler */
            if (s_handler) {
                s_handler(header.type, payload, payload_len);
            }
        }
        /* else: invalid packet, silently discard (UDP — no retransmit) */

        /* Free the netbuf */
        netbuf_delete(buf);
    }
}
