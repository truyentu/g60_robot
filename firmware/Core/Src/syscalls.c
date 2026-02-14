/**
 * @file syscalls.c
 * @brief Newlib stubs for embedded — _write redirects to debug UART
 */

#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>
#include <string.h>

#include "stm32h7xx_hal.h"

/* External: debug UART handle from board_init.c */
extern UART_HandleTypeDef *Board_GetDebugUart(void);

/* Heap pointer for _sbrk */
extern uint32_t _end;      /* Defined in linker script */
static uint8_t *heap_end = NULL;

int _write(int file, char *ptr, int len)
{
    (void)file;
    UART_HandleTypeDef *huart = Board_GetDebugUart();
    if (huart) {
        HAL_UART_Transmit(huart, (uint8_t *)ptr, (uint16_t)len, 100);
    }
    return len;
}

caddr_t _sbrk(int incr)
{
    if (heap_end == NULL) {
        heap_end = (uint8_t *)&_end;
    }

    uint8_t *prev_heap_end = heap_end;
    heap_end += incr;
    return (caddr_t)prev_heap_end;
}

int _close(int file) { (void)file; return -1; }
int _fstat(int file, struct stat *st) { (void)file; st->st_mode = S_IFCHR; return 0; }
int _isatty(int file) { (void)file; return 1; }
int _lseek(int file, int ptr, int dir) { (void)file; (void)ptr; (void)dir; return 0; }
int _read(int file, char *ptr, int len) { (void)file; (void)ptr; (void)len; return 0; }
int _kill(int pid, int sig) { (void)pid; (void)sig; errno = EINVAL; return -1; }
int _getpid(void) { return 1; }
