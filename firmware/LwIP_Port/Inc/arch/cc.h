/**
 * @file cc.h
 * @brief LwIP compiler/platform abstraction for ARM GCC + FreeRTOS
 *
 * Required by LwIP core. Defines types, byte order, alignment, and
 * diagnostic output macros.
 */

#ifndef CC_H
#define CC_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

/* ========================================================================= */
/*  Typedefs (if not already provided by LwIP arch.h)                        */
/* ========================================================================= */

/* LwIP 2.2 uses its own typedefs when LWIP_NO_STDINT_H is not defined.
 * We rely on LwIP's default stdint-based types. */

/* ========================================================================= */
/*  Byte Order                                                               */
/* ========================================================================= */

/* ARM Cortex-M7 is little-endian */
#ifndef BYTE_ORDER
#define BYTE_ORDER  LITTLE_ENDIAN
#endif

/* ========================================================================= */
/*  Structure Packing                                                        */
/* ========================================================================= */

/* GCC packing macros for LwIP protocol headers */
#define PACK_STRUCT_BEGIN
#define PACK_STRUCT_STRUCT  __attribute__((packed))
#define PACK_STRUCT_END
#define PACK_STRUCT_FIELD(x) x

/* ========================================================================= */
/*  Alignment                                                                */
/* ========================================================================= */

/* Alignment macro for memory pools */
#ifndef LWIP_MEM_ALIGN_SIZE
#define LWIP_MEM_ALIGN_SIZE(size)   (((size) + MEM_ALIGNMENT - 1U) & ~(MEM_ALIGNMENT - 1U))
#endif

/* ========================================================================= */
/*  Diagnostic Output                                                        */
/* ========================================================================= */

/* Platform-specific diagnostic output (debug UART via printf) */
#define LWIP_PLATFORM_DIAG(x)   do { printf x; } while(0)
#define LWIP_PLATFORM_ASSERT(x) do { printf("LwIP Assert: %s\n", x); while(1); } while(0)

/* ========================================================================= */
/*  Critical Section (provided by sys_arch via FreeRTOS)                     */
/* ========================================================================= */

/* SYS_ARCH_PROTECT / UNPROTECT defined in sys_arch.h when NO_SYS == 0 */

/* ========================================================================= */
/*  Random Number Generator                                                  */
/* ========================================================================= */

/* Used by LwIP for initial TCP sequence numbers, etc. */
#define LWIP_RAND()   ((uint32_t)rand())

#endif /* CC_H */
