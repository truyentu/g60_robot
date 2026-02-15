/**
 * @file sys_arch.c
 * @brief LwIP OS abstraction layer — FreeRTOS implementation
 *
 * Implements LwIP's sys_arch interface using FreeRTOS primitives:
 *   - Semaphores (sys_sem_*)
 *   - Mutexes (sys_mutex_*)
 *   - Mailboxes (sys_mbox_* via FreeRTOS queues)
 *   - Thread creation (sys_thread_new)
 *   - Timing (sys_now, sys_msleep)
 *   - Init (sys_init)
 */

#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/err.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include <string.h>

/* ========================================================================= */
/*  Initialization                                                           */
/* ========================================================================= */

void sys_init(void)
{
    /* Nothing to do — FreeRTOS is already initialized before LwIP */
}

/* ========================================================================= */
/*  Time                                                                     */
/* ========================================================================= */

u32_t sys_now(void)
{
    return (u32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

/* ========================================================================= */
/*  Semaphores                                                               */
/* ========================================================================= */

err_t sys_sem_new(sys_sem_t *sem, u8_t count)
{
    *sem = xSemaphoreCreateCounting(0xFFFF, count);
    if (*sem == NULL) {
        return ERR_MEM;
    }
    return ERR_OK;
}

void sys_sem_free(sys_sem_t *sem)
{
    if (sys_sem_valid(sem)) {
        vSemaphoreDelete(*sem);
        sys_sem_set_invalid(sem);
    }
}

void sys_sem_signal(sys_sem_t *sem)
{
    xSemaphoreGive(*sem);
}

u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
    TickType_t start = xTaskGetTickCount();
    TickType_t ticks;

    if (timeout == 0) {
        ticks = portMAX_DELAY;
    } else {
        ticks = pdMS_TO_TICKS(timeout);
    }

    if (xSemaphoreTake(*sem, ticks) == pdTRUE) {
        u32_t elapsed = (u32_t)((xTaskGetTickCount() - start) * portTICK_PERIOD_MS);
        return elapsed;
    }

    return SYS_ARCH_TIMEOUT;
}

/* ========================================================================= */
/*  Mutexes                                                                  */
/* ========================================================================= */

err_t sys_mutex_new(sys_mutex_t *mutex)
{
    *mutex = xSemaphoreCreateRecursiveMutex();
    if (*mutex == NULL) {
        return ERR_MEM;
    }
    return ERR_OK;
}

void sys_mutex_free(sys_mutex_t *mutex)
{
    if (sys_mutex_valid(mutex)) {
        vSemaphoreDelete(*mutex);
        sys_mutex_set_invalid(mutex);
    }
}

void sys_mutex_lock(sys_mutex_t *mutex)
{
    xSemaphoreTakeRecursive(*mutex, portMAX_DELAY);
}

void sys_mutex_unlock(sys_mutex_t *mutex)
{
    xSemaphoreGiveRecursive(*mutex);
}

/* ========================================================================= */
/*  Mailboxes (FreeRTOS Queues)                                              */
/* ========================================================================= */

err_t sys_mbox_new(sys_mbox_t *mbox, int size)
{
    if (size <= 0) {
        size = 16;
    }

    *mbox = xQueueCreate((UBaseType_t)size, sizeof(void *));
    if (*mbox == NULL) {
        return ERR_MEM;
    }
    return ERR_OK;
}

void sys_mbox_free(sys_mbox_t *mbox)
{
    if (sys_mbox_valid(mbox)) {
        /* Drain any remaining messages */
        void *msg;
        while (xQueueReceive(*mbox, &msg, 0) == pdTRUE) {
            /* discard */
        }
        vQueueDelete(*mbox);
        sys_mbox_set_invalid(mbox);
    }
}

void sys_mbox_post(sys_mbox_t *mbox, void *msg)
{
    /* Block forever until space available */
    xQueueSendToBack(*mbox, &msg, portMAX_DELAY);
}

err_t sys_mbox_trypost(sys_mbox_t *mbox, void *msg)
{
    if (xQueueSendToBack(*mbox, &msg, 0) == pdTRUE) {
        return ERR_OK;
    }
    return ERR_MEM;
}

err_t sys_mbox_trypost_fromisr(sys_mbox_t *mbox, void *msg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (xQueueSendToBackFromISR(*mbox, &msg, &xHigherPriorityTaskWoken) == pdTRUE) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        return ERR_OK;
    }
    return ERR_MEM;
}

u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox, void **msg, u32_t timeout)
{
    TickType_t start = xTaskGetTickCount();
    TickType_t ticks;
    void *temp;

    if (msg == NULL) {
        msg = &temp;
    }

    if (timeout == 0) {
        ticks = portMAX_DELAY;
    } else {
        ticks = pdMS_TO_TICKS(timeout);
    }

    if (xQueueReceive(*mbox, msg, ticks) == pdTRUE) {
        u32_t elapsed = (u32_t)((xTaskGetTickCount() - start) * portTICK_PERIOD_MS);
        return elapsed;
    }

    *msg = NULL;
    return SYS_ARCH_TIMEOUT;
}

u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg)
{
    void *temp;
    if (msg == NULL) {
        msg = &temp;
    }

    if (xQueueReceive(*mbox, msg, 0) == pdTRUE) {
        return 0;
    }

    *msg = NULL;
    return SYS_MBOX_EMPTY;
}

/* ========================================================================= */
/*  Thread Creation                                                          */
/* ========================================================================= */

sys_thread_t sys_thread_new(const char *name, lwip_thread_fn thread,
                            void *arg, int stacksize, int prio)
{
    TaskHandle_t handle = NULL;

    xTaskCreate(thread, name, (uint16_t)stacksize, arg,
                (UBaseType_t)prio, &handle);

    return handle;
}

/* ========================================================================= */
/*  Critical Section (non-ISR context)                                       */
/* ========================================================================= */

/* Note: SYS_ARCH_PROTECT/UNPROTECT are defined as macros in sys_arch.h
 * using taskENTER_CRITICAL_FROM_ISR / taskEXIT_CRITICAL_FROM_ISR.
 *
 * For non-ISR use in LwIP core, the standard approach is:
 *   taskENTER_CRITICAL() / taskEXIT_CRITICAL()
 *
 * The FROM_ISR variants are safe to use in both contexts on Cortex-M
 * but return the interrupt mask level for proper nesting. */
