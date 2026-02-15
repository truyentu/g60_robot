/**
 * @file board_flash.h
 * @brief Flash parameter storage — save/load axis configs to internal Flash
 *
 * Uses Sector 7 (0x081E0000, last 128KB of Bank 1).
 * CRC32 validation on load.
 */

#ifndef BOARD_FLASH_H
#define BOARD_FLASH_H

#include "cmd_handler.h"
#include "protocol_defs.h"
#include <stdint.h>

/* ========================================================================= */
/*  Flash Parameter Block                                                    */
/* ========================================================================= */

#define FLASH_PARAMS_MAGIC    0xBEEF1234U
#define FLASH_PARAMS_VERSION  1U

#pragma pack(push, 1)

typedef struct {
    uint32_t magic;                         /* Must be FLASH_PARAMS_MAGIC    */
    uint32_t version;                       /* FLASH_PARAMS_VERSION          */
    AxisConfig axes[PROTO_NUM_AXES];        /* Per-axis configuration        */
    uint16_t global_accel;                  /* Global acceleration           */
    uint8_t  operating_mode;                /* Default operating mode        */
    uint8_t  reserved[3];                   /* Alignment padding             */
    uint32_t crc;                           /* CRC32 over all preceding data */
} FlashParams;

#pragma pack(pop)

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

/**
 * Load parameters from flash.
 * Returns 0 on success (valid magic + CRC), -1 on error (blank/corrupt).
 */
int Flash_LoadParams(FlashParams *out);

/**
 * Save parameters to flash (erases sector first).
 * Returns 0 on success, -1 on error.
 */
int Flash_SaveParams(const FlashParams *data);

/**
 * Erase the parameter flash sector.
 */
void Flash_EraseParams(void);

#endif /* BOARD_FLASH_H */
