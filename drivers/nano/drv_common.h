
#ifndef __DRV_COMMON_H__
#define __DRV_COMMON_H__

#include <stdbool.h>
#include <rtthread.h>
#include <rthw.h>

#include <sam.h>
#include <system_samd21.h>

#include "board.h"
#include "drv_ringbuffer.h"
#include "drv_samd21_gpio.h"
#include "drv_samd21_uart.h"
#include "drv_samd21_i2c.h"
#include "drv_samd21_spi.h"
#include "drv_samd21_sst26.h"

#define CRYSTALLESS

#define BSP_USING_I2C
#define BSP_USING_SERCOM2_I2C
#define BSP_SERCOM2_I2C_CLOCK   100000UL

#define BSP_USING_SPI
#define BSP_USING_SERCOM1_SPI
#define BSP_SERCOM1_SPI_CLOCK   1000000UL

#ifdef __cplusplus
extern "C" {
#endif

#define SAMD21_FLASH_START_ADRESS       ROM_START
#define SAMD21_FLASH_SIZE               ROM_SIZE
#define SAMD21_FLASH_END_ADDRESS        ROM_END

#define SAMD21_SRAM1_SIZE               RAM_SIZE
#define SAMD21_SRAM1_START              RAM_START
#define SAMD21_SRAM1_END                RAM_END

#if defined(__CC_ARM) || defined(__CLANG_ARM)
extern int Image$$RW_IRAM1$$ZI$$Limit;
#define HEAP_BEGIN      (&Image$$RW_IRAM1$$ZI$$Limit)
#elif __ICCARM__
#pragma section="CSTACK"
#define HEAP_BEGIN      (__segment_end("CSTACK"))
#else
extern int __bss_end;
#define HEAP_BEGIN      (&__bss_end)
#endif

#define HEAP_END        SAMD21_SRAM1_END

void rt_hw_delay_us (rt_uint32_t us);

#ifdef __cplusplus
}
#endif

#endif
