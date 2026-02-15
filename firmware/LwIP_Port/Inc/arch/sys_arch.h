/**
 * @file sys_arch.h
 * @brief LwIP OS abstraction layer — FreeRTOS declarations
 *
 * Defines mutex, semaphore, mailbox, and thread types
 * using FreeRTOS primitives.
 */

#ifndef SYS_ARCH_H
#define SYS_ARCH_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* ========================================================================= */
/*  Type Definitions                                                         */
/* ========================================================================= */

/* Semaphore: binary or counting */
typedef SemaphoreHandle_t   sys_sem_t;

/* Mutex: recursive mutex */
typedef SemaphoreHandle_t   sys_mutex_t;

/* Mailbox: FreeRTOS queue */
typedef QueueHandle_t       sys_mbox_t;

/* Thread: FreeRTOS task handle */
typedef TaskHandle_t        sys_thread_t;

/* Protection: interrupt mask level */
typedef UBaseType_t         sys_prot_t;

/* ========================================================================= */
/*  Validity Checks                                                          */
/* ========================================================================= */

#define sys_sem_valid(sem)          ((*(sem)) != NULL)
#define sys_sem_set_invalid(sem)    ((*(sem)) = NULL)

#define sys_mutex_valid(mutex)      ((*(mutex)) != NULL)
#define sys_mutex_set_invalid(mutex)((*(mutex)) = NULL)

#define sys_mbox_valid(mbox)        ((*(mbox)) != NULL)
#define sys_mbox_set_invalid(mbox)  ((*(mbox)) = NULL)

/* ========================================================================= */
/*  Critical Section Macros                                                  */
/* ========================================================================= */

#define SYS_ARCH_DECL_PROTECT(lev)    sys_prot_t lev
#define SYS_ARCH_PROTECT(lev)         (lev) = taskENTER_CRITICAL_FROM_ISR()
#define SYS_ARCH_UNPROTECT(lev)       taskEXIT_CRITICAL_FROM_ISR(lev)

#endif /* SYS_ARCH_H */
