#include "board.h"


uint32_t PORT_GroupRead (PORT_GROUP group)
{
    return (((port_group_registers_t *)group)->PORT_IN);
}


void PORT_GroupWrite (PORT_GROUP group, uint32_t mask, uint32_t value)
{
    /* Write the desired value */
    ((port_group_registers_t *)group)->PORT_OUT = (((port_group_registers_t *)group)->PORT_OUT & (~mask)) | (mask & value);
}

uint32_t PORT_GroupLatchRead (PORT_GROUP group)
{
    return (((port_group_registers_t *)group)->PORT_OUT);
}

void PORT_GroupSet (PORT_GROUP group, uint32_t mask)
{
    ((port_group_registers_t *)group)->PORT_OUTSET = mask;
}

void PORT_GroupClear (PORT_GROUP group, uint32_t mask)
{
    ((port_group_registers_t *)group)->PORT_OUTCLR = mask;
}

void PORT_GroupToggle (PORT_GROUP group, uint32_t mask)
{
    ((port_group_registers_t *)group)->PORT_OUTTGL = mask;
}

void PORT_GroupInputEnable (PORT_GROUP group, uint32_t mask)
{
    ((port_group_registers_t *)group)->PORT_DIRCLR = mask;
}

void PORT_GroupOutputEnable (PORT_GROUP group, uint32_t mask)
{
    ((port_group_registers_t *)group)->PORT_DIRSET = mask;
}

void PORT_PinPeripheralFunctionConfig (PORT_PIN pin, PERIPHERAL_FUNCTION function)
{
    uint32_t periph_func = (uint32_t) function;
    PORT_GROUP group = GET_PORT_GROUP (pin);
    uint32_t pin_num = ((uint32_t)pin) & 0x1FU;
    uint32_t pinmux_val = (uint32_t) ((port_group_registers_t *)group)->PORT_PMUX[ (pin_num >> 1)];

    /* For odd pins */
    if (0U != (pin_num & 0x01U)) {
        pinmux_val = (pinmux_val & ~0xF0U) | (periph_func << 4);
    } else {
        pinmux_val = (pinmux_val & ~0x0FU) | periph_func;
    }
    ((port_group_registers_t *)group)->PORT_PMUX[ (pin_num >> 1)] = (uint8_t)pinmux_val;

    /* Enable peripheral control of the pin */
    ((port_group_registers_t *)group)->PORT_PINCFG[pin_num] |= (uint8_t)PORT_PINCFG_PMUXEN_Msk;
}

void PORT_PinGPIOConfig (PORT_PIN pin)
{
    PORT_GROUP group = GET_PORT_GROUP (pin);
    uint32_t pin_num = ((uint32_t)pin) & 0x1FU;

    /* Disable peripheral control of the pin */
    ((port_group_registers_t *)group)->PORT_PINCFG[pin_num] &= ((uint8_t) (~PORT_PINCFG_PMUXEN_Msk));
}

void PORT_PinMUX_Config (uint32_t pin_mux)
{
    uint32_t port;
    uint32_t pin;

    if (pin_mux != PINMUX_UNUSED) {
        port = (pin_mux & 0x600000) >> 21;
        pin = pin_mux >> 16;
        PORT_REGS->GROUP[port].PORT_PINCFG[ (pin - (port * 32))] = PORT_PINCFG_PMUXEN (1);
        PORT_REGS->GROUP[port].PORT_PMUX[ (pin - (port * 32)) / 2] &= ~ (0xF << (4 * (pin & 0x01u)));
        PORT_REGS->GROUP[port].PORT_PMUX[ (pin - (port * 32)) / 2] |= (pin_mux & 0xFF) << (4 * (pin & 0x01u));
    }
}
