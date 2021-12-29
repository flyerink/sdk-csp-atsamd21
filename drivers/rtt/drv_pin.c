#include "board.h"
#include <rtthread.h>

#if defined(RT_USING_PIN)

#include "rtdevice.h"

struct pin_index {
    uint16_t index;
    uint8_t group;
    uint8_t pin;
};

static const struct pin_index pins[] = {
#ifdef PORT_PA00
    {0, 0, 0},
#endif
#ifdef PORT_PA01
    {1, 0, 1},
#endif
#ifdef PORT_PA02
    {2, 0, 2},
#endif
#ifdef PORT_PA03
    {3, 0, 3},
#endif
#ifdef PORT_PA04
    {4, 0, 4},
#endif
#ifdef PORT_PA05
    {5, 0, 5},
#endif
#ifdef PORT_PA06
    {6, 0, 6},
#endif
#ifdef PORT_PA07
    {7, 0, 7},
#endif
#ifdef PORT_PA08
    {8, 0, 8},
#endif
#ifdef PORT_PA09
    {9, 0, 9},
#endif
#ifdef PORT_PA10
    {10, 0, 10},
#endif
#ifdef PORT_PA11
    {11, 0, 11},
#endif
#ifdef PORT_PA12
    {12, 0, 12},
#endif
#ifdef PORT_PA13
    {13, 0, 13},
#endif
#ifdef PORT_PA14
    {14, 0, 14},
#endif
#ifdef PORT_PA15
    {15, 0, 15},
#endif
#ifdef PORT_PA16
    {16, 0, 16},
#endif
#ifdef PORT_PA17
    {17, 0, 17},
#endif
#ifdef PORT_PA18
    {18, 0, 18},
#endif
#ifdef PORT_PA19
    {19, 0, 19},
#endif
#ifdef PORT_PA20
    {20, 0, 20},
#endif
#ifdef PORT_PA21
    {21, 0, 21},
#endif
#ifdef PORT_PA22
    {22, 0, 22},
#endif
#ifdef PORT_PA23
    {23, 0, 23},
#endif
#ifdef PORT_PA24
    {24, 0, 24},
#endif
#ifdef PORT_PA25
    {25, 0, 25},
#endif
#ifdef PORT_PA26
    {26, 0, 26},
#endif
#ifdef PORT_PA27
    {27, 0, 27},
#endif
#ifdef PORT_PA28
    {28, 0, 28},
#endif
#ifdef PORT_PA29
    {29, 0, 29},
#endif
#ifdef PORT_PA30
    {30, 0, 30},
#endif
#ifdef PORT_PA31
    {31, 0, 31},
#endif

#ifdef PORT_PB00
    {32, 1, 0},
#endif
#ifdef PORT_PB01
    {33, 1, 1},
#endif
#ifdef PORT_PB02
    {34, 1, 2},
#endif
#ifdef PORT_PB03
    {35, 1, 3},
#endif
#ifdef PORT_PB04
    {36, 1, 4},
#endif
#ifdef PORT_PB05
    {37, 1, 5},
#endif
#ifdef PORT_PB06
    {38, 1, 6},
#endif
#ifdef PORT_PB07
    {39, 1, 7},
#endif
#ifdef PORT_PB08
    {40, 1, 8},
#endif
#ifdef PORT_PB09
    {41, 1, 9},
#endif
#ifdef PORT_PB10
    {42, 1, 10},
#endif
#ifdef PORT_PB11
    {43, 1, 11},
#endif
#ifdef PORT_PB12
    {44, 1, 12},
#endif
#ifdef PORT_PB13
    {45, 1, 13},
#endif
#ifdef PORT_PB14
    {46, 1, 14},
#endif
#ifdef PORT_PB15
    {47, 1, 15},
#endif
#ifdef PORT_PB16
    {48, 1, 16},
#endif
#ifdef PORT_PB17
    {49, 1, 17},
#endif
#ifdef PORT_PB18
    {50, 1, 18},
#endif
#ifdef PORT_PB19
    {51, 1, 19},
#endif

#ifdef PORT_PB20
    {52, 1, 20},
#endif
#ifdef PORT_PB21
    {53, 1, 21},
#endif
#ifdef PORT_PB22
    {54, 1, 22},
#endif
#ifdef PORT_PB23
    {55, 1, 23},
#endif
#ifdef PORT_PB24
    {56, 1, 24},
#endif
#ifdef PORT_PB25
    {57, 1, 25},
#endif
#ifdef PORT_PB26
    {58, 1, 26},
#endif
#ifdef PORT_PB27
    {59, 1, 27},
#endif
#ifdef PORT_PB28
    {60, 1, 28},
#endif
#ifdef PORT_PB29
    {61, 1, 29},
#endif
#ifdef PORT_PB30
    {62, 1, 30},
#endif
#ifdef PORT_PB31
    {63, 1, 31}
#endif
};

#define ITEM_NUM(items) sizeof(items) / sizeof(items[0])

static const struct pin_index *get_pin (rt_base_t pin)
{
    const struct pin_index *index;

    if (pin < ITEM_NUM (pins)) {
        index = &pins[pin];
        if (index->index == (uint16_t)-1)
            index = RT_NULL;
    } else {
        index = RT_NULL;
    }

    return index;
};

static void samd2x_pin_mode (rt_device_t dev, rt_base_t pin, rt_base_t mode)
{
    const struct pin_index *index;

    index = get_pin (pin);
    if (index == RT_NULL) {
        return;
    }

    if (mode == PIN_MODE_OUTPUT) {
        /* output setting */
        PORT_REGS->GROUP[index->group].PORT_DIRSET = (1 << index->pin);
        PORT_REGS->GROUP[index->group].PORT_PINCFG[index->pin] = PORT_PINCFG_PULLEN (0) | PORT_PINCFG_INEN (1);
    } else if (mode == PIN_MODE_INPUT) {
        /* input setting: not pull. */
        PORT_REGS->GROUP[index->group].PORT_DIRCLR = (1 << index->pin);
        PORT_REGS->GROUP[index->group].PORT_PINCFG[index->pin] = PORT_PINCFG_PULLEN (0) | PORT_PINCFG_INEN (1);
    } else if (mode == PIN_MODE_INPUT_PULLUP) {
        /* input setting: pull up. */
        PORT_REGS->GROUP[index->group].PORT_OUTSET = (1 << index->pin);
        PORT_REGS->GROUP[index->group].PORT_DIRCLR = (1 << index->pin);
        PORT_REGS->GROUP[index->group].PORT_PINCFG[index->pin] = PORT_PINCFG_PULLEN (1) | PORT_PINCFG_INEN (1);
    } else if (mode == PIN_MODE_INPUT_PULLDOWN) {
        /* input setting: pull down. */
        PORT_REGS->GROUP[index->group].PORT_OUTCLR = (1 << index->pin);
        PORT_REGS->GROUP[index->group].PORT_DIRCLR = (1 << index->pin);
        PORT_REGS->GROUP[index->group].PORT_PINCFG[index->pin] = PORT_PINCFG_PULLEN (1) | PORT_PINCFG_INEN (1);
    }
}

static void samd2x_pin_write (rt_device_t dev, rt_base_t pin, rt_base_t value)
{
    const struct pin_index *index;

    index = get_pin (pin);
    if (index == RT_NULL) {
        return;
    }

    if (value)
        PORT_REGS->GROUP[index->group].PORT_OUTSET = (1 << index->pin);
    else
        PORT_REGS->GROUP[index->group].PORT_OUTCLR = (1 << index->pin);
}

static int samd2x_pin_read (rt_device_t dev, rt_base_t pin)
{
    int value;
    const struct pin_index *index;

    value = PIN_LOW;

    index = get_pin (pin);
    if (index == RT_NULL) {
        return value;
    }

    if (PORT_REGS->GROUP[index->group].PORT_IN & (1 << index->pin))
        return PIN_HIGH;
    else
        return PIN_LOW;
}

const static struct rt_pin_ops _samd2x_pin_ops = {
    samd2x_pin_mode,
    samd2x_pin_write,
    samd2x_pin_read,
    RT_NULL,
    RT_NULL,
    RT_NULL,
};

int rt_hw_pin_init (void)
{
    return rt_device_pin_register ("pin", &_samd2x_pin_ops, RT_NULL);
}

#endif
