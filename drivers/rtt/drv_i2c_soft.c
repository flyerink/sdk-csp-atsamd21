#include <rtthread.h>
#include "board.h"

#if defined(RT_USING_I2C) && defined(RT_USING_I2C_BITOPS)

#if !defined(BSP_USING_I2C1) && !defined(BSP_USING_I2C2) && !defined(BSP_USING_I2C3) && !defined(BSP_USING_I2C4)
#error "Please define at least one BSP_USING_I2Cx in board.h"
/* this driver can be disabled at menuconfig → RT-Thread Components → Device Drivers */
#endif

#include "rtdevice.h"

/* samd21 config class */
struct samd21_soft_i2c_config {
    rt_uint8_t scl;
    rt_uint8_t sda;
    const char *bus_name;
};
/* samd21 i2c dirver class */
struct samd21_i2c {
    struct rt_i2c_bit_ops ops;
    struct rt_i2c_bus_device i2c2_bus;
};

#ifdef BSP_USING_I2C1
#define I2C1_BUS_CONFIG                                  \
    {                                                    \
        .scl = BSP_I2C1_SCL_PIN,                         \
        .sda = BSP_I2C1_SDA_PIN,                         \
        .bus_name = "i2c1",                              \
    }
#endif

#ifdef BSP_USING_I2C2
#define I2C2_BUS_CONFIG                                  \
    {                                                    \
        .scl = BSP_I2C2_SCL_PIN,                         \
        .sda = BSP_I2C2_SDA_PIN,                         \
        .bus_name = "i2c2",                              \
    }
#endif

#ifdef BSP_USING_I2C3
#define I2C3_BUS_CONFIG                                  \
    {                                                    \
        .scl = BSP_I2C3_SCL_PIN,                         \
        .sda = BSP_I2C3_SDA_PIN,                         \
        .bus_name = "i2c3",                              \
    }
#endif

#ifdef BSP_USING_I2C4
#define I2C4_BUS_CONFIG                                  \
    {                                                    \
        .scl = BSP_I2C4_SCL_PIN,                         \
        .sda = BSP_I2C4_SDA_PIN,                         \
        .bus_name = "i2c4",                              \
    }
#endif

static const struct samd21_soft_i2c_config soft_i2c_config[] = {
#ifdef BSP_USING_I2C1
    I2C1_BUS_CONFIG,
#endif
#ifdef BSP_USING_I2C2
    I2C2_BUS_CONFIG,
#endif
#ifdef BSP_USING_I2C3
    I2C3_BUS_CONFIG,
#endif
#ifdef BSP_USING_I2C4
    I2C4_BUS_CONFIG,
#endif
};

static struct samd21_i2c i2c_obj[sizeof (soft_i2c_config) / sizeof (soft_i2c_config[0])];

/**
 * This function initializes the i2c pin.
 *
 * @param samd21 i2c dirver class.
 */
static void samd21_i2c_gpio_init (struct samd21_i2c *i2c)
{
    struct samd21_soft_i2c_config *cfg = (struct samd21_soft_i2c_config *)i2c->ops.data;

    rt_pin_mode (cfg->scl, PIN_MODE_OUTPUT);
    rt_pin_mode (cfg->sda, PIN_MODE_OUTPUT);

    rt_pin_write (cfg->scl, PIN_HIGH);
    rt_pin_write (cfg->sda, PIN_HIGH);
}

/**
 * This function sets the sda pin.
 *
 * @param samd21 config class.
 * @param The sda pin state.
 */
static void samd21_set_sda (void *data, rt_int32_t state)
{
    struct samd21_soft_i2c_config *cfg = (struct samd21_soft_i2c_config *)data;
    if (state) {
        rt_pin_write (cfg->sda, PIN_HIGH);
    } else {
        rt_pin_write (cfg->sda, PIN_LOW);
    }
}

/**
 * This function sets the scl pin.
 *
 * @param samd21 config class.
 * @param The scl pin state.
 */
static void samd21_set_scl (void *data, rt_int32_t state)
{
    struct samd21_soft_i2c_config *cfg = (struct samd21_soft_i2c_config *)data;
    if (state) {
        rt_pin_write (cfg->scl, PIN_HIGH);
    } else {
        rt_pin_write (cfg->scl, PIN_LOW);
    }
}

/**
 * This function gets the sda pin state.
 *
 * @param The sda pin state.
 */
static rt_int32_t samd21_get_sda (void *data)
{
    struct samd21_soft_i2c_config *cfg = (struct samd21_soft_i2c_config *)data;
    return rt_pin_read (cfg->sda);
}

/**
 * This function gets the scl pin state.
 *
 * @param The scl pin state.
 */
static rt_int32_t samd21_get_scl (void *data)
{
    struct samd21_soft_i2c_config *cfg = (struct samd21_soft_i2c_config *)data;
    //rt_pin_mode (cfg->scl, PIN_MODE_INPUT);
    return rt_pin_read (cfg->scl);
}

/**
 * The time delay function.
 *
 * @param microseconds.
 */
static void samd21_udelay (rt_uint32_t us)
{
#if 1
    rt_uint32_t ticks;
    rt_uint32_t told, tnow, tcnt = 0;
    rt_uint32_t reload = SysTick->LOAD;

    ticks = us * reload / (1000000 / RT_TICK_PER_SECOND);
    told = SysTick->VAL;
    while (1) {
        tnow = SysTick->VAL;
        if (tnow != told) {
            if (tnow < told) {
                tcnt += told - tnow;
            } else {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks) {
                break;
            }
        }
    }
#else
    rt_uint32_t start, now, delta, reload, us_tick;

    start = SysTick->VAL;
    reload = SysTick->LOAD;
    us_tick = SystemCoreClock / 1000000UL;
    do {
        now = SysTick->VAL;
        delta = start > now ? start - now : reload + start - now;
    } while (delta < us_tick * us);
#endif
}

static const struct rt_i2c_bit_ops samd21_bit_ops_default = {
    .data     = RT_NULL,
    .set_sda  = samd21_set_sda,
    .set_scl  = samd21_set_scl,
    .get_sda  = samd21_get_sda,
    .get_scl  = samd21_get_scl,
    .udelay   = samd21_udelay,
    .delay_us = 1,
    .timeout  = 100
};


/**
 * if i2c is locked, this function will unlock it
 *
 * @param samd21 config class
 *
 * @return RT_EOK indicates successful unlock.
 */
static rt_err_t samd21_i2c_bus_unlock (const struct samd21_soft_i2c_config *cfg)
{
    rt_int32_t i = 0;

    if (PIN_LOW == rt_pin_read (cfg->sda)) {
        while (i++ <= 9) {
            rt_pin_write (cfg->scl, PIN_HIGH);
            samd21_udelay (100);
            rt_pin_write (cfg->scl, PIN_LOW);
            samd21_udelay (100);
        }
        rt_pin_write (cfg->scl, PIN_HIGH);
    }
    if (PIN_LOW == rt_pin_read (cfg->sda)) {
        return -RT_ERROR;
    }

    return RT_EOK;
}

/* I2C initialization function */
int rt_hw_i2c_init (void)
{
    rt_size_t obj_num = sizeof (i2c_obj) / sizeof (struct samd21_i2c);
    rt_err_t result;

    for (int i = 0; i < obj_num; i++) {
        i2c_obj[i].ops = samd21_bit_ops_default;
        i2c_obj[i].ops.data = (void *)&soft_i2c_config[i];
        i2c_obj[i].i2c2_bus.priv = &i2c_obj[i].ops;
        samd21_i2c_gpio_init (&i2c_obj[i]);
        result = rt_i2c_bit_add_bus (&i2c_obj[i].i2c2_bus, soft_i2c_config[i].bus_name);
        RT_ASSERT (result == RT_EOK);
        samd21_i2c_bus_unlock (&soft_i2c_config[i]);

        rt_kprintf ("software simulation %s init done, pin scl: %d, pin sda %d\n",
                    soft_i2c_config[i].bus_name,
                    soft_i2c_config[i].scl,
                    soft_i2c_config[i].sda);
    }

    return result;
}
INIT_DEVICE_EXPORT (rt_hw_i2c_init);

#endif
