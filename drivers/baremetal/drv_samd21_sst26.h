#ifndef __DRV_SAMD21_SST26_H_
#define __DRV_SAMD21_SST26_H_

#include <stdbool.h>
#include <stdint.h>

#include "board.h"

#define SST26_BLOCK_SIZE    65536
#define SST26_SECTOR_SIZE   4096
#define SST26_SECTOR_COUNT  512
#define SST26_PAGE_SIZE     256
#define SST26_MAX_ADDRESS   (SST26_SECTOR_COUNT * SST26_SECTOR_SIZE)

#define SST26_ID_MFC        0xBF
#define SST26_ID_TYPE       0x26
#define SST26_ID_DEVICE     0x41

typedef enum {
    SST26_CMD_FLASH_RESET_ENABLE = 0x66,    /* Reset enable command. */
    SST26_CMD_FLASH_RESET        = 0x99,    /* Command to reset the flash. */
    SST26_CMD_ENABLE_QUAD_IO     = 0x38,    /* Command to Enable QUAD IO */
    SST26_CMD_RESET_QUAD_IO      = 0xFF,    /* Command to Reset QUAD IO */
    SST26_CMD_JEDEC_ID_READ      = 0x9F,    /* Command to read JEDEC-ID of the flash device. */
    SST26_CMD_QUAD_JEDEC_ID_READ = 0xAF,    /* QUAD Command to read JEDEC-ID of the flash device. */
    SST26_CMD_READ               = 0x03,    /* Command to perform Read */
    SST26_CMD_HIGH_SPEED_READ    = 0x0B,    /* Command to perform High Speed Read */
    SST26_CMD_WRITE_ENABLE       = 0x06,    /* Write enable command. */
    SST26_CMD_WRITE_DISABLE      = 0x04,    /* Write disable command */
    SST26_CMD_PAGE_PROGRAM       = 0x02,    /* Page Program command. */
    SST26_CMD_READ_STATUS_REG    = 0x05,    /* Command to read the Flash status register. */
    SST26_CMD_SECTOR_ERASE       = 0x20,    /* Command to perform sector erase */
    SST26_CMD_BULK_ERASE_64K     = 0xD8,    /* Command to perform Bulk erase */
    SST26_CMD_CHIP_ERASE         = 0xC7,    /* Command to perform Chip erase */
    SST26_CMD_UNPROTECT_GLOBAL   = 0x98     /* Command to unlock the flash device. */
} SST26_CMD;

typedef struct {
    struct samd21_spi_t *spi;
} sst26_obj_t;

int sst26_init (void);
void sst26_reset (void);
void sst26_unlock_flash (void);

void sst26_write_enable (void);
void sst26_write_disable (void);
uint8_t sst26_read_status (void);

void sst26_erase_sector (uint32_t address);
void sst26_erase_64k_block (uint32_t address);
void sst26_erase_full_chip (void);

uint16_t sst26_read_id (uint8_t *id, uint8_t length);

uint8_t sst26_read_byte (uint32_t address);
uint16_t sst26_read_bytes (uint32_t address, uint8_t *buffer, uint16_t length);

void sst26_write_byte (uint32_t address, uint8_t data);
uint16_t sst26_write_page (uint32_t address, uint8_t *buffer, uint16_t length);
uint16_t sst26_write_bytes (uint32_t address, uint8_t *buffer, uint16_t length);

#endif
