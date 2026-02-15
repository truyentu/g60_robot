/**
 * @file status_task.h
 * @brief Status task — periodic StatusPacket sender
 */

#ifndef STATUS_TASK_H
#define STATUS_TASK_H

#include "protocol_status.h"
#include "lwip/ip_addr.h"

/**
 * Get pointer to the current status packet (for debug/inspection).
 */
const StatusPacket *Status_GetCurrent(void);

/**
 * Set the PC address for status/event packets.
 * Called by NetTask when first command packet arrives.
 */
void Status_SetPCAddress(const ip_addr_t *addr, uint16_t port);

#endif /* STATUS_TASK_H */
