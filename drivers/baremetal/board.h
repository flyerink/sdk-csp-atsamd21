/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-10-26     RealThread   first version
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include <sam.h>
#include <system_samd21.h>

#include "common.h"
#include "drv_samd21_gpio.h"
#include "drv_samd21_uart.h"
#include "drv_samd21_spi.h"
#include "drv_samd21_i2c.h"
#include "drv_samd21_sst26.h"

#define CRYSTALLESS

#ifdef __cplusplus
extern "C"
{
#endif

/*-------------------------- CHIP CONFIG BEGIN --------------------------*/

#define CHIP_FAMILY_ATSAMD21
#define CHIP_SERIES_ATSAMD21
#define CHIP_NAME_ATSAMD21E18A

/*-------------------------- CHIP CONFIG END --------------------------*/

/*-------------------------- ROM/RAM CONFIG BEGIN --------------------------*/

#define ROM_START              ((uint32_t)0x00000000)
#define ROM_SIZE               (256 * 1024)
#define ROM_END                ((uint32_t)(ROM_START + ROM_SIZE))

#define RAM_START              (0x20000000)
#define RAM_SIZE               (32 * 1024)
#define RAM_END                (RAM_START + RAM_SIZE)

/*-------------------------- ROM/RAM CONFIG END --------------------------*/

/*-------------------------- CLOCK CONFIG BEGIN --------------------------*/

#define BSP_CLOCK_SOURCE                    ("HSI")
#define BSP_CLOCK_SOURCE_FREQ_MHZ           ((int32_t)0)
#define BSP_CLOCK_SYSTEM_FREQ_MHZ           ((int32_t)48)
#define BSP_SYSTICK_PER_SECOND              (1000)

/*-------------------------- CLOCK CONFIG END --------------------------*/

/*-------------------------- UART CONFIG BEGIN --------------------------*/

/** After configuring corresponding UART or UART DMA, you can use it.
 *
 * STEP 1, define macro define related to the serial port opening based on the serial port number
 *                 such as     #define BSP_USING_UART1
 *
 * STEP 2, according to the corresponding pin of serial port, define the related serial port information macro
 *                 such as     #define BSP_UART1_TX_PIN       "PA9"
 *                             #define BSP_UART1_RX_PIN       "PA10"
 *
 */
#define BSP_USING_UART
#define BSP_USING_SERCOM3_USART

#define BSP_USING_SPI
#define BSP_USING_SERCOM1_SPI
#define BSP_SERCOM1_SPI_CLOCK       1000000UL

#define BSP_USING_I2C
#define BSP_USING_SERCOM2_I2C
#define BSP_SERCOM2_I2C_CLOCK       100000UL

typedef void (* SysTick_User_CB_t)(void);

void SysTick_SetCB(SysTick_User_CB_t user_cb);

extern void hw_board_init (void);

extern void hw_delay_ms(const uint32_t ms);
extern void hw_delay_us (uint32_t us);
extern void wait_ms (unsigned long ms);

extern int entry (void);

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H__ */
