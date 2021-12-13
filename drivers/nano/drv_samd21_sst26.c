#include "board.h"

#if defined(BSP_USING_SPI) && defined(BSP_USING_SERCOM1_SPI)

#define FLASH_CS_PORT       PORT_GROUP_0
#define FLASH_CS_PIN        (1<<PIN_PA17)

#define SST26_CSN_RESET()   { PORT_GroupClear(FLASH_CS_PORT, FLASH_CS_PIN);}
#define SST26_CSN_SET()     { PORT_GroupSet(FLASH_CS_PORT, FLASH_CS_PIN);}

sst26_obj_t sst26 = {
    .spi = &samd21_spi1,
};

void sst26_reset (void)
{
    SST26_CSN_RESET();
    SERCOM_SPI_TransferDataSPI (sst26.spi, SST26_CMD_FLASH_RESET_ENABLE);
    SST26_CSN_SET();

    SST26_CSN_RESET();
    SERCOM_SPI_TransferDataSPI (sst26.spi, SST26_CMD_FLASH_RESET);
    SST26_CSN_SET();
}

void sst26_unlock_flash (void)
{
    SST26_CSN_RESET();
    SERCOM_SPI_TransferDataSPI (sst26.spi, SST26_CMD_UNPROTECT_GLOBAL);
    SST26_CSN_SET();
}

void sst26_write_enable (void)
{
    SST26_CSN_RESET();
    SERCOM_SPI_TransferDataSPI (sst26.spi, SST26_CMD_WRITE_ENABLE);
    SST26_CSN_SET();
}

void sst26_write_disable (void)
{
    SST26_CSN_RESET();
    SERCOM_SPI_TransferDataSPI (sst26.spi, SST26_CMD_WRITE_DISABLE);
    SST26_CSN_SET();
}

uint8_t sst26_read_status (void)
{
    uint8_t status;
    SST26_CSN_RESET();
    SERCOM_SPI_TransferDataSPI (sst26.spi, SST26_CMD_READ_STATUS_REG);
    status = SERCOM_SPI_TransferDataSPI (sst26.spi, 0xFF);
    SST26_CSN_SET();

    return status;
}

uint16_t sst26_read_id (uint8_t *id, uint8_t length)
{
    if (id == NULL)
        return 0;

    SST26_CSN_RESET();
    SERCOM_SPI_TransferDataSPI (sst26.spi, SST26_CMD_JEDEC_ID_READ);
    SERCOM_SPI_Receive (sst26.spi, id, length);
    SST26_CSN_SET();

    return length;
}

uint8_t sst26_read_byte (uint32_t address)
{
    uint8_t cmd[4];

    if (address >= SST26_MAX_ADDRESS)
        return 0;

    cmd[0] = SST26_CMD_READ;
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;

    while ((sst26_read_status() & 0x1));

    SST26_CSN_RESET();
    SERCOM_SPI_Transfer (sst26.spi, cmd, 4);
    cmd[0] = SERCOM_SPI_TransferDataSPI (sst26.spi, 0xFF);
    SST26_CSN_SET();

    return cmd[0];
}

uint16_t sst26_read_bytes (uint32_t address, uint8_t *buffer, uint16_t length)
{
    uint8_t cmd[4];

    if (buffer == NULL)
        return 0;

    if ((address + length) >= SST26_MAX_ADDRESS)
        return 0;

    cmd[0] = SST26_CMD_READ;
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;

    while ((sst26_read_status() & 0x1));

    SST26_CSN_RESET();
    SERCOM_SPI_Transfer (sst26.spi, cmd, 4);
    SERCOM_SPI_Receive (sst26.spi, buffer, length);
    SST26_CSN_SET();

    return length;
}

void sst26_erase_sector (uint32_t address)
{
    uint8_t cmd[4];

    if (address >= SST26_MAX_ADDRESS)
        return;

    cmd[0] = SST26_CMD_SECTOR_ERASE;
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;

    while ((sst26_read_status() & 0x1));
    sst26_write_enable();

    SST26_CSN_RESET();
    SERCOM_SPI_Transfer (sst26.spi, cmd, 4);
    SST26_CSN_SET();
}

void sst26_erase_64k_block (uint32_t address)
{
    uint8_t cmd[4];

    if (address >= SST26_MAX_ADDRESS)
        return;

    cmd[0] = SST26_CMD_BULK_ERASE_64K;
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;

    while ((sst26_read_status() & 0x1));
    sst26_write_enable();

    SST26_CSN_RESET();
    SERCOM_SPI_Transfer (sst26.spi, cmd, 4);
    SST26_CSN_SET();
}

void sst26_erase_full_chip (void)
{
    while ((sst26_read_status() & 0x1));
    sst26_write_enable();

    SST26_CSN_RESET();
    SERCOM_SPI_TransferDataSPI (sst26.spi, SST26_CMD_CHIP_ERASE);
    SST26_CSN_SET();
}

void sst26_write_byte (uint32_t address, uint8_t data)
{
    uint8_t cmd[5];

    if (address >= SST26_MAX_ADDRESS)
        return;

    cmd[0] = SST26_CMD_PAGE_PROGRAM;
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;
    cmd[4] = data;

    while ((sst26_read_status() & 0x1));
    sst26_write_enable();

    SST26_CSN_RESET();
    SERCOM_SPI_Transfer (sst26.spi, cmd, 5);
    SST26_CSN_SET();
}

uint16_t sst26_write_page (uint32_t address, uint8_t *buffer, uint16_t length)
{
    uint8_t cmd[4];

    if (buffer == NULL)
        return 0;

    if ((address + length) >= SST26_MAX_ADDRESS)
        return 0;

    while ((sst26_read_status() & 0x1));

    cmd[0] = SST26_CMD_PAGE_PROGRAM;
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;

    SST26_CSN_RESET();
    SERCOM_SPI_Transfer (sst26.spi, cmd, 4);
    SERCOM_SPI_Transfer (sst26.spi, buffer, length);
    SST26_CSN_SET();

    return length;
}

uint16_t sst26_write_bytes (uint32_t address, uint8_t *buffer, uint16_t length)
{
    uint8_t len = 0;
    uint32_t addr = address;
    uint8_t *ptr = buffer;

    if (buffer == NULL)
        return 0;

    if ((address + length) >= SST26_MAX_ADDRESS)
        return 0;

    sst26_write_enable();

    len = addr & (SST26_PAGE_SIZE - 1);
    if (len != 0) {
        sst26_write_page (addr, ptr, len);
        ptr += len;
        addr += len;
        length -= len;
    }

    do {
        if ((addr & (SST26_SECTOR_SIZE - 1)) == 0) {
            sst26_erase_sector (addr);
        }

        sst26_write_page (addr, ptr, SST26_PAGE_SIZE);

        ptr += SST26_PAGE_SIZE;
        addr += SST26_PAGE_SIZE;
        length -= SST26_PAGE_SIZE;
    } while (length >= SST26_PAGE_SIZE);

    if (length > 0) {
        sst26_write_page (addr, ptr, length);
        length -= length;
    }

    return length;
}

int sst26_init (void)
{
    PORT_GroupSet (FLASH_CS_PORT, FLASH_CS_PIN);
    PORT_GroupOutputEnable (FLASH_CS_PORT, FLASH_CS_PIN);

    sst26_reset();

    return RT_EOK;
}

#endif // defined(RT_USING_SPI) && defined(BSP_USING_SERCOM1_SPI)
