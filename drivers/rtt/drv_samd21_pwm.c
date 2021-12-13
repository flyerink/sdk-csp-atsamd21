#include "board.h"
#include <rtthread.h>
#include "rtdevice.h"

#if defined(RT_USING_DEVICE) && defined(RT_USING_PWM)

#if defined(BSP_USING_TCC1_PWM) && defined(BSP_USING_SERCOM0_USART)
#error "TCC1 use PA6 & PA7 is conflict with SERCOM0 USART, only 1 enabled"
#endif

static void TCC_ClockInit (void)
{
#ifdef BSP_USING_TCC1_PWM
    /* Selection of the Generator and write Lock for TCC0 TCC1 */
    GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_ID (26) | GCLK_CLKCTRL_GEN (0x0)  | GCLK_CLKCTRL_CLKEN_Msk;
    /* Configure the APBC Bridge Clocks */
    PM_REGS->PM_APBCMASK |= PM_APBCMASK_TCC1_Msk;
#endif

#ifdef BSP_USING_TCC2_PWM
    /* Selection of the Generator and write Lock for TC3 TCC2 */
    GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_ID (27) | GCLK_CLKCTRL_GEN (0x0)  | GCLK_CLKCTRL_CLKEN_Msk;
    /* Configure the APBC Bridge Clocks */
    PM_REGS->PM_APBCMASK |= PM_APBCMASK_TCC2_Msk;
#endif
    while ( GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk ) {
        /* Wait for synchronization */
    }
}

static void TCC_PortInit (void)
{
#ifdef BSP_USING_TCC1_PWM
    PORT_REGS->GROUP[0].PORT_PINCFG[6] = 0x1;
    PORT_REGS->GROUP[0].PORT_PINCFG[7] = 0x1;

    PORT_REGS->GROUP[0].PORT_PMUX[3] = 0x44;
#endif

#ifdef BSP_USING_TCC2_PWM
    PORT_REGS->GROUP[0].PORT_PINCFG[0] = 0x1;
    PORT_REGS->GROUP[0].PORT_PINCFG[1] = 0x1;

    PORT_REGS->GROUP[0].PORT_PMUX[0] = 0x44;
#endif
}

/* Initialize TCC module */
void TCC_PWM_Init (tcc_registers_t *tcc_regs)
{
    /* Reset TCC */
    tcc_regs->TCC_CTRLA = TCC_CTRLA_SWRST_Msk;
    while ((tcc_regs->TCC_SYNCBUSY & TCC_SYNCBUSY_SWRST_Msk) != 0U); /* Wait for sync */

    /* Clock prescaler */
    tcc_regs->TCC_CTRLA = TCC_CTRLA_PRESCALER_DIV1;
    tcc_regs->TCC_WAVE = TCC_WAVE_WAVEGEN_NPWM;

    /* Configure duty cycle values */
    tcc_regs->TCC_CC[0] = 1000U;
    tcc_regs->TCC_CC[1] = 200U;
    tcc_regs->TCC_PER = 2399;

    TCC1_REGS->TCC_PATT = (uint16_t) (TCC_PATT_PGE0_Msk | TCC_PATT_PGE1_Msk |
                                      TCC_PATT_PGV0_Msk | TCC_PATT_PGV1_Msk);

    while (tcc_regs->TCC_SYNCBUSY != 0U);    /* Wait for sync */
}

/* Start the PWM generation */
void TCC_PWMStart (tcc_registers_t *tcc_regs)
{
    tcc_regs->TCC_CTRLA |= TCC_CTRLA_ENABLE_Msk;
    while (tcc_regs->TCC_SYNCBUSY & (TCC_SYNCBUSY_ENABLE_Msk)) {
        /* Wait for sync */
    }
}

/* Stop the PWM generation */
void TCC_PWMStop (tcc_registers_t *tcc_regs)
{
    tcc_regs->TCC_CTRLA &= ~TCC_CTRLA_ENABLE_Msk;
    while (tcc_regs->TCC_SYNCBUSY & (TCC_SYNCBUSY_ENABLE_Msk)) {
        /* Wait for sync */
    }
}

/* Configure PWM period */
void TCC_PWM24bitPeriodSet (tcc_registers_t *tcc_regs, uint32_t period)
{
    tcc_regs->TCC_PERB = period & 0xFFFFFF;
    while ((tcc_regs->TCC_SYNCBUSY & (TCC_SYNCBUSY_PER_Msk)) == TCC_SYNCBUSY_PER_Msk) {
        /* Wait for sync */
    }
}

/* Read TCC period */
uint32_t TCC_PWM24bitPeriodGet (tcc_registers_t *tcc_regs)
{
    while (tcc_regs->TCC_SYNCBUSY & (TCC_SYNCBUSY_PER_Msk)) {
        /* Wait for sync */
    }
    return (tcc_regs->TCC_PER & 0xFFFFFF);
}

/* Configure dead time */
void TCC_PWMDeadTimeSet (tcc_registers_t *tcc_regs, uint8_t deadtime_high, uint8_t deadtime_low)
{
    tcc_regs->TCC_WEXCTRL &= ~ (TCC_WEXCTRL_DTHS_Msk | TCC_WEXCTRL_DTLS_Msk);
    tcc_regs->TCC_WEXCTRL |= TCC_WEXCTRL_DTHS (deadtime_high) | TCC_WEXCTRL_DTLS (deadtime_low);
}

void TCC_PWMPatternSet (tcc_registers_t *tcc_regs, uint8_t pattern_enable, uint8_t pattern_output)
{
    tcc_regs->TCC_PATTB = (uint16_t) (pattern_enable | (pattern_output << 8));
    while ((tcc_regs->TCC_SYNCBUSY & (TCC_SYNCBUSY_PATT_Msk)) == TCC_SYNCBUSY_PATT_Msk) {
        /* Wait for sync */
    }
}

/* Set the counter*/
void TCC_PWM24bitCounterSet (tcc_registers_t *tcc_regs, uint32_t count_value)
{
    tcc_regs->TCC_COUNT = count_value & 0xFFFFFF;
    while (tcc_regs->TCC_SYNCBUSY & (TCC_SYNCBUSY_COUNT_Msk)) {
        /* Wait for sync */
    }
}

void TCC_PWM24bitPulseSet (tcc_registers_t *tcc_regs, uint32_t channel, uint32_t pulse)
{
    tcc_regs->TCC_CCB[channel] = pulse & 0xFFFFFF;
    while (tcc_regs->TCC_SYNCBUSY & (TCC_SYNCBUSY_PER_Msk)) {
        /* Wait for sync */
    }
}

uint32_t TCC_PWM24bitPulseGet (tcc_registers_t *tcc_regs, uint32_t channel)
{
    while (tcc_regs->TCC_SYNCBUSY & (TCC_SYNCBUSY_PER_Msk)) {
        /* Wait for sync */
    }
    return (tcc_regs->TCC_CCB[channel] & 0xFFFFFF);
}

/* Enable forced synchronous update */
void TCC_PWMForceUpdate (tcc_registers_t *tcc_regs)
{
    tcc_regs->TCC_CTRLBSET |= TCC_CTRLBCLR_CMD_UPDATE;
    while (tcc_regs->TCC_SYNCBUSY & (TCC_SYNCBUSY_CTRLB_Msk)) {
        /* Wait for sync */
    }
}

/* Enable the period interrupt - overflow or underflow interrupt */
void TCC_PWMPeriodInterruptEnable (tcc_registers_t *tcc_regs)
{
    tcc_regs->TCC_INTENSET = TCC_INTENSET_OVF_Msk;
}

/* Disable the period interrupt - overflow or underflow interrupt */
void TCC_PWMPeriodInterruptDisable (tcc_registers_t *tcc_regs)
{
    tcc_regs->TCC_INTENCLR = TCC_INTENCLR_OVF_Msk;
}

/* Read interrupt flags */
uint32_t TCC_PWMInterruptStatusGet (tcc_registers_t *tcc_regs)
{
    uint32_t interrupt_status;
    interrupt_status = tcc_regs->TCC_INTFLAG;
    /* Clear interrupt flags */
    tcc_regs->TCC_INTFLAG = interrupt_status;
    return interrupt_status;
}

struct samd21_pwm {
    struct rt_device_pwm pwm_device;
    tcc_registers_t *tcc_regs;
    rt_uint8_t channels: 4;
    rt_uint8_t enabled: 4;
    const char *name;
};

#ifdef BSP_USING_TCC1_PWM
#define PWM1_CONFIG                         \
    {                                       \
        .tcc_regs = TCC1_REGS,              \
        .channels = 2,                      \
        .enabled = 0,                       \
        .name = "pwm1",                     \
    }
#endif

#ifdef BSP_USING_TCC2_PWM
#define PWM2_CONFIG                         \
    {                                       \
        .tcc_regs = TCC2_REGS,              \
        .channels = 2,                      \
        .enabled = 0,                       \
        .name = "pwm2",                     \
    }
#endif

static struct samd21_pwm samd21_pwm_objs [] = {
#ifdef BSP_USING_TCC1_PWM
    PWM1_CONFIG,
#endif
#ifdef BSP_USING_TCC2_PWM
    PWM2_CONFIG,
#endif
};

static rt_err_t drv_pwm_control (struct rt_device_pwm *device, int cmd, void *arg)
{
    struct samd21_pwm *pwm_config = (struct samd21_pwm *)device->parent.user_data;
    struct rt_pwm_configuration *configuration = (struct rt_pwm_configuration *)arg;

    switch (cmd) {
        case PWM_CMD_ENABLE:
            if (configuration->channel < pwm_config->channels) {
                pwm_config->enabled |= (1 << configuration->channel);
                TCC_PWMPatternSet (pwm_config->tcc_regs, ~ (pwm_config->enabled) & 0x000F, ~ (pwm_config->enabled) & 0x000F);
                TCC_PWMStart (pwm_config->tcc_regs);
                return RT_EOK;
            } else
                return RT_EEMPTY;
        case PWM_CMD_DISABLE:
            if (configuration->channel < pwm_config->channels) {
                pwm_config->enabled &= ~ (1 << configuration->channel);
                TCC_PWMPatternSet (pwm_config->tcc_regs, ~ (pwm_config->enabled) & 0x000F, ~ (pwm_config->enabled) & 0x000F);
                if (pwm_config->enabled == 0)
                    TCC_PWMStop (pwm_config->tcc_regs);
                return RT_EOK;
            } else
                return RT_EEMPTY;
        case PWM_CMD_SET:
            if (configuration->channel < pwm_config->channels) {
                TCC_PWM24bitPeriodSet (pwm_config->tcc_regs, configuration->period);
                TCC_PWM24bitPulseSet (pwm_config->tcc_regs, configuration->channel, configuration->pulse);
                return RT_EOK;
            } else
                return RT_EEMPTY;

        case PWM_CMD_GET:
            if (configuration->channel < pwm_config->channels) {
                configuration->period = TCC_PWM24bitPeriodGet (pwm_config->tcc_regs);
                configuration->pulse = TCC_PWM24bitPulseGet (pwm_config->tcc_regs, configuration->channel);
                return RT_EOK;
            } else
                return RT_EEMPTY;

        default:
            return RT_EINVAL;
    }
}

static struct rt_pwm_ops samd21_tcc_ops = {
    drv_pwm_control,
};

static int rt_pwm_init (void)
{
    rt_size_t obj_num = sizeof (samd21_pwm_objs) / sizeof (struct samd21_pwm);
    int result = RT_EOK;

    /* pwm init */
    TCC_PortInit();
    TCC_ClockInit();

    for (int i = 0; i < obj_num; i++) {
        TCC_PWM_Init (samd21_pwm_objs[i].tcc_regs);

        /* register pwm device */
        if (rt_device_pwm_register (&samd21_pwm_objs[i].pwm_device, samd21_pwm_objs[i].name,
                                    &samd21_tcc_ops, &samd21_pwm_objs[i]) == RT_EOK) {
            rt_kprintf ("%s register success\n", samd21_pwm_objs[i].name);
        } else {
            rt_kprintf ("%s register failed\n", samd21_pwm_objs[i].name);
            result = -RT_ERROR;
            break;
        }
    }

    return result;
}
INIT_DEVICE_EXPORT (rt_pwm_init);

#endif // RT_USING_PWM