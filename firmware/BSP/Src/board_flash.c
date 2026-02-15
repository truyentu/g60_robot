/**
 * @file board_flash.c
 * @brief Flash parameter storage implementation
 *
 * Uses STM32H7 internal Flash Sector 7 (Bank 1, last 128KB).
 * Address: 0x081E0000
 *
 * Flash programming on STM32H7:
 * - Must erase full sector before writing
 * - Write in 256-bit (32-byte) flash words
 * - Uses HAL_FLASH API
 */

#include "board_flash.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* Flash sector 7 address (Bank 1) */
#define FLASH_PARAMS_ADDR     0x081E0000UL
#define FLASH_PARAMS_SECTOR   FLASH_SECTOR_7

/* ========================================================================= */
/*  CRC32 (software, same polynomial as STM32 hardware CRC)                  */
/* ========================================================================= */

static uint32_t crc32_compute(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc >>= 1;
        }
    }
    return ~crc;
}

/* ========================================================================= */
/*  Load Parameters                                                          */
/* ========================================================================= */

int Flash_LoadParams(FlashParams *out)
{
    const FlashParams *stored = (const FlashParams *)FLASH_PARAMS_ADDR;

    /* Check magic */
    if (stored->magic != FLASH_PARAMS_MAGIC) {
        return -1;
    }

    /* Check version */
    if (stored->version != FLASH_PARAMS_VERSION) {
        return -1;
    }

    /* Compute CRC over data (excluding crc field itself) */
    uint32_t data_size = sizeof(FlashParams) - sizeof(uint32_t);
    uint32_t crc = crc32_compute((const uint8_t *)stored, data_size);

    if (crc != stored->crc) {
        return -1;
    }

    memcpy(out, stored, sizeof(FlashParams));
    return 0;
}

/* ========================================================================= */
/*  Save Parameters                                                          */
/* ========================================================================= */

int Flash_SaveParams(const FlashParams *data)
{
    FlashParams block;
    memcpy(&block, data, sizeof(FlashParams));

    /* Set magic and version */
    block.magic = FLASH_PARAMS_MAGIC;
    block.version = FLASH_PARAMS_VERSION;

    /* Compute CRC */
    uint32_t data_size = sizeof(FlashParams) - sizeof(uint32_t);
    block.crc = crc32_compute((const uint8_t *)&block, data_size);

    /* Unlock flash */
    HAL_FLASH_Unlock();

    /* Erase sector 7 */
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = FLASH_BANK_1;
    erase.Sector = FLASH_PARAMS_SECTOR;
    erase.NbSectors = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;  /* 2.7V - 3.6V */

    uint32_t sector_error;
    if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return -1;
    }

    /* Write in 256-bit (32-byte) flash words */
    uint32_t addr = FLASH_PARAMS_ADDR;
    const uint8_t *src = (const uint8_t *)&block;
    uint32_t remaining = sizeof(FlashParams);

    /* Pad to 32-byte alignment */
    uint8_t padded[((sizeof(FlashParams) + 31) / 32) * 32];
    memset(padded, 0xFF, sizeof(padded));
    memcpy(padded, src, sizeof(FlashParams));

    for (uint32_t i = 0; i < sizeof(padded); i += 32) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                              addr + i, (uint32_t)(uintptr_t)&padded[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return -1;
        }
    }

    HAL_FLASH_Lock();

    /* Verify */
    FlashParams verify;
    if (Flash_LoadParams(&verify) != 0) {
        return -1;
    }

    return 0;
}

/* ========================================================================= */
/*  Erase                                                                    */
/* ========================================================================= */

void Flash_EraseParams(void)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = FLASH_BANK_1;
    erase.Sector = FLASH_PARAMS_SECTOR;
    erase.NbSectors = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    uint32_t sector_error;
    HAL_FLASHEx_Erase(&erase, &sector_error);

    HAL_FLASH_Lock();
}
