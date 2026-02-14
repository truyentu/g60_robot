/**
 * @file net_task.h
 * @brief Network task — UDP command reception
 */

#ifndef NET_TASK_H
#define NET_TASK_H

#include "protocol_defs.h"
#include "protocol_commands.h"

/**
 * Command callback type — called by NetTask when a valid command is parsed.
 * Implemented by motion_task or other modules.
 */
typedef void (*CommandHandler)(uint8_t cmd_type, const uint8_t *payload, uint16_t payload_len);

/**
 * Register a command handler (called during init).
 */
void NetTask_RegisterHandler(CommandHandler handler);

#endif /* NET_TASK_H */
