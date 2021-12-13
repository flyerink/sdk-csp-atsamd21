
#ifndef __DRV_COMMON_H__
#define __DRV_COMMON_H__

#include <stdbool.h>
#include <rtthread.h>
#include <rthw.h>

#include <sam.h>
#include <system_samd21.h>
#include "board.h"

#define CRYSTALLESS

#ifdef __cplusplus
extern "C" {
#endif

/* Named type for port group */
typedef uint32_t PORT_GROUP;

/* Group 0 */
#define PORT_GROUP_0 (PORT_BASE_ADDRESS + (0U * 0x80U))
/* Group 1 */
#define PORT_GROUP_1 (PORT_BASE_ADDRESS + (1U * 0x80U))
/* Group 2 */
#define PORT_GROUP_2 (PORT_BASE_ADDRESS + (2U * 0x80U))
/* Group 3 */
#define PORT_GROUP_3 (PORT_BASE_ADDRESS + (3U * 0x80U))

/* Helper macros to get port information from the pin */
#define GET_PORT_GROUP(pin)     ((PORT_GROUP)(PORT_BASE_ADDRESS + (0x80U * (((uint32_t)pin) >> 5U))))
#define GET_PIN_MASK(pin)       (((uint32_t)(0x1U)) << (((uint32_t)pin) & 0x1FU))

#define PORTA_INDEX  0
#define PORTB_INDEX  1
#define PORTC_INDEX  2
#define PORTD_INDEX  3

#define __SAMD21_PORT(port)     PORT##port##_INDEX
#define GET_PIN(PORTx, PIN)     (rt_base_t)(32 * __SAMD21_PORT(PORTx) + PIN)

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

#define PINMUX_UNUSED   0xFFFFFFFF

enum samd21_gpio_port {
    GPIOA, GPIOB
};

void rt_hw_delay_us (rt_uint32_t us);

void PORT_PinMUX_Config (uint32_t pin_mux);

#ifdef __cplusplus
}
#endif

#endif
