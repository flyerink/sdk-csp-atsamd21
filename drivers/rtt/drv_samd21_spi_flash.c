#include <rtthread.h>
#include "rtdevice.h"

#include <board.h>

#if defined(RT_USING_SPI) && defined(RT_USING_SFUD)

#define DBG_TAG "fla"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define FLASH_CS_PORT       GPIOA
#define FLASH_CS_PIN        PIN_PA17

#include "spi_flash.h"

extern rt_spi_flash_device_t rt_sfud_flash_probe (const char *spi_flash_dev_name,
        const char *spi_dev_name);

#ifdef RT_USING_DFS_ELMFAT
#include "dfs.h"
#include "dfs_fs.h"
#endif

int rt_spi_flash_init (void)
{
    extern rt_err_t rt_hw_spi_device_attach (const char *bus_name,
            const char *device_name,
            uint16_t cs_gpio_port, uint16_t cs_gpio_pin);

    rt_hw_spi_device_attach ("spi1", "spi10", FLASH_CS_PORT, FLASH_CS_PIN);

     if (RT_NULL == rt_sfud_flash_probe ("sst26", "spi10")) {
         LOG_I ("Probe flash fail\r\n");
         return -RT_ERROR;
     };

#ifdef RT_USING_DFS_ELMFAT
    if (dfs_mount ("sst26", "/", "elm", 0, 0) == RT_EOK) {
        LOG_I ("SPI Flash mounted to /\n");
    } else {
        LOG_I ("SPI Flash mount to / fail\n");
    }
#endif

    return RT_EOK;
}
INIT_ENV_EXPORT (rt_spi_flash_init);

#endif // defined(RT_USING_SPI) && defined(RT_USING_SFUD)
