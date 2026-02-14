/**
 * @file status_task.h
 * @brief Status task — periodic StatusPacket sender
 */

#ifndef STATUS_TASK_H
#define STATUS_TASK_H

#include "protocol_status.h"

/**
 * Get pointer to the current status packet (for debug/inspection).
 */
const StatusPacket *Status_GetCurrent(void);

#endif /* STATUS_TASK_H */
