
#ifndef __DRV_SAMD21_GPIO_H__
#define __DRV_SAMD21_GPIO_H__

#include <stdbool.h>

#include <sam.h>
#include <system_samd21.h>

/* Named type for port group */
typedef uint32_t PORT_GROUP;

#define PINMUX_UNUSED   0xFFFFFFFF

/* Group 0 */
#define PORT_GROUP_0 (PORT_BASE_ADDRESS + (0U * 0x80U))
/* Group 1 */
#define PORT_GROUP_1 (PORT_BASE_ADDRESS + (1U * 0x80U))

/* Helper macros to get port information from the pin */
#define GET_PORT_GROUP(pin)  ((PORT_GROUP)(PORT_BASE_ADDRESS + (0x80U * (((uint32_t)pin) >> 5U))))
#define GET_PIN_MASK(pin)   (((uint32_t)(0x1U)) << (((uint32_t)pin) & 0x1FU))

typedef enum {
    PERIPHERAL_FUNCTION_A = 0x0,
    PERIPHERAL_FUNCTION_B = 0x1,
    PERIPHERAL_FUNCTION_C = 0x2,
    PERIPHERAL_FUNCTION_D = 0x3,
    PERIPHERAL_FUNCTION_E = 0x4,
    PERIPHERAL_FUNCTION_F = 0x5,
    PERIPHERAL_FUNCTION_G = 0x6,
    PERIPHERAL_FUNCTION_H = 0x7,
} PERIPHERAL_FUNCTION;

typedef enum {
    /* PA00 pin */
    PORT_PIN_PA00 = 0U,

    /* PA01 pin */
    PORT_PIN_PA01 = 1U,

    /* PA02 pin */
    PORT_PIN_PA02 = 2U,

    /* PA03 pin */
    PORT_PIN_PA03 = 3U,

    /* PA04 pin */
    PORT_PIN_PA04 = 4U,

    /* PA05 pin */
    PORT_PIN_PA05 = 5U,

    /* PA06 pin */
    PORT_PIN_PA06 = 6U,

    /* PA07 pin */
    PORT_PIN_PA07 = 7U,

    /* PA08 pin */
    PORT_PIN_PA08 = 8U,

    /* PA09 pin */
    PORT_PIN_PA09 = 9U,

    /* PA10 pin */
    PORT_PIN_PA10 = 10U,

    /* PA11 pin */
    PORT_PIN_PA11 = 11U,

    /* PA12 pin */
    PORT_PIN_PA12 = 12U,

    /* PA13 pin */
    PORT_PIN_PA13 = 13U,

    /* PA14 pin */
    PORT_PIN_PA14 = 14U,

    /* PA15 pin */
    PORT_PIN_PA15 = 15U,

    /* PA16 pin */
    PORT_PIN_PA16 = 16U,

    /* PA17 pin */
    PORT_PIN_PA17 = 17U,

    /* PA18 pin */
    PORT_PIN_PA18 = 18U,

    /* PA19 pin */
    PORT_PIN_PA19 = 19U,

    /* PA20 pin */
    PORT_PIN_PA20 = 20U,

    /* PA21 pin */
    PORT_PIN_PA21 = 21U,

    /* PA22 pin */
    PORT_PIN_PA22 = 22U,

    /* PA23 pin */
    PORT_PIN_PA23 = 23U,

    /* PA24 pin */
    PORT_PIN_PA24 = 24U,

    /* PA25 pin */
    PORT_PIN_PA25 = 25U,

    /* PA27 pin */
    PORT_PIN_PA27 = 27U,

    /* PA28 pin */
    PORT_PIN_PA28 = 28U,

    /* PA30 pin */
    PORT_PIN_PA30 = 30U,

    /* PA31 pin */
    PORT_PIN_PA31 = 31U,

    /* PB00 pin */
    PORT_PIN_PB00 = 32U,

    /* PB01 pin */
    PORT_PIN_PB01 = 33U,

    /* PB02 pin */
    PORT_PIN_PB02 = 34U,

    /* PB03 pin */
    PORT_PIN_PB03 = 35U,

    /* PB04 pin */
    PORT_PIN_PB04 = 36U,

    /* PB05 pin */
    PORT_PIN_PB05 = 37U,

    /* PB06 pin */
    PORT_PIN_PB06 = 38U,

    /* PB07 pin */
    PORT_PIN_PB07 = 39U,

    /* PB08 pin */
    PORT_PIN_PB08 = 40U,

    /* PB09 pin */
    PORT_PIN_PB09 = 41U,

    /* PB10 pin */
    PORT_PIN_PB10 = 42U,

    /* PB11 pin */
    PORT_PIN_PB11 = 43U,

    /* PB12 pin */
    PORT_PIN_PB12 = 44U,

    /* PB13 pin */
    PORT_PIN_PB13 = 45U,

    /* PB14 pin */
    PORT_PIN_PB14 = 46U,

    /* PB15 pin */
    PORT_PIN_PB15 = 47U,

    /* PB16 pin */
    PORT_PIN_PB16 = 48U,

    /* PB17 pin */
    PORT_PIN_PB17 = 49U,

    /* PB22 pin */
    PORT_PIN_PB22 = 54U,

    /* PB23 pin */
    PORT_PIN_PB23 = 55U,

    /* PB30 pin */
    PORT_PIN_PB30 = 62U,

    /* PB31 pin */
    PORT_PIN_PB31 = 63U,

    /* This element should not be used in any of the PORT APIs.
     * It will be used by other modules or application to denote that none of
     * the PORT Pin is used */
    PORT_PIN_NONE = 65535U,

} PORT_PIN;

uint32_t PORT_GroupRead (PORT_GROUP group);
void PORT_GroupWrite (PORT_GROUP group, uint32_t mask, uint32_t value);
uint32_t PORT_GroupLatchRead (PORT_GROUP group);
void PORT_GroupSet (PORT_GROUP group, uint32_t mask);
void PORT_GroupClear (PORT_GROUP group, uint32_t mask);
void PORT_GroupToggle (PORT_GROUP group, uint32_t mask);
void PORT_GroupInputEnable (PORT_GROUP group, uint32_t mask);
void PORT_GroupOutputEnable (PORT_GROUP group, uint32_t mask);
void PORT_PinPeripheralFunctionConfig (PORT_PIN pin, PERIPHERAL_FUNCTION function);
void PORT_PinGPIOConfig (PORT_PIN pin);

void PORT_PinMUX_Config (uint32_t pin_mux);

#endif
