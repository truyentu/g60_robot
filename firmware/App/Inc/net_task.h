/**
 * @file net_task.h
 * @brief Network task — UDP command reception and dispatch
 */

#ifndef NET_TASK_H
#define NET_TASK_H

#include "protocol_defs.h"
#include "protocol_commands.h"
#include "protocol_status.h"
#include "protocol_packet.h"

#include "lwip/ip_addr.h"

#include <stdint.h>
#include <stdbool.h>

/**
 * Command callback type — called by NetTask when a valid command is parsed.
 * Implemented by motion_task or other modules.
 */
typedef void (*CommandHandler)(uint8_t cmd_type, const uint8_t *payload, uint16_t payload_len);

/**
 * Register a command handler for non-system commands (jog, motion, etc.).
 */
void NetTask_RegisterHandler(CommandHandler handler);

/**
 * Get the last known PC sender address (set when first packet arrives).
 * Returns false if no packet has been received yet.
 */
bool NetTask_GetPCAddress(ip_addr_t *addr_out, uint16_t *port_out);

/**
 * Get heartbeat statistics.
 */
uint32_t NetTask_GetLastHeartbeatTick(void);
uint32_t NetTask_GetPacketsReceived(void);
uint32_t NetTask_GetPacketsInvalid(void);

#endif /* NET_TASK_H */
