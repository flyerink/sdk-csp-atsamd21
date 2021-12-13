/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-7      SummerGift   first version
 */

#include "drv_common.h"
#include "board.h"

#ifdef RT_USING_FINSH
#include <finsh.h>

static void reboot (uint8_t argc, char **argv)
{
    rt_hw_cpu_reset();
}
FINSH_FUNCTION_EXPORT_ALIAS (reboot, __cmd_reboot, Reboot System);
#endif /* RT_USING_FINSH */


rt_inline void gclk_sync (void)
{
    while (GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk)
        ;
}

rt_inline void dfll_sync (void)
{
    while ((SYSCTRL_REGS->SYSCTRL_PCLKSR & SYSCTRL_PCLKSR_DFLLRDY_Msk) == 0)
        ;
}

void SystemCoreClock_Update (uint32_t clock)
{
    SystemCoreClock = clock;
}

void SystemClock_Config (void)
{
    NVMCTRL_REGS->NVMCTRL_CTRLB = NVMCTRL_CTRLB_RWS (3);

    /* Disable 32K oscillator */
    SYSCTRL_REGS->SYSCTRL_OSC32K = 0x0;

#if defined(CRYSTALLESS)
    /* Configure OSC8M as source for GCLK_GEN 2 */
    GCLK_REGS->GCLK_GENDIV = GCLK_GENDIV_ID (2); // Read GENERATOR_ID - GCLK_GEN_2
    gclk_sync();

    GCLK_REGS->GCLK_GENCTRL = GCLK_GENCTRL_ID (2) | GCLK_GENCTRL_SRC_OSC8M_Val | GCLK_GENCTRL_GENEN_Msk;
    gclk_sync();

    // Turn on DFLL with USB correction and sync to internal 8 mhz oscillator
    SYSCTRL_REGS->SYSCTRL_DFLLCTRL &= ~SYSCTRL_DFLLCTRL_ONDEMAND_Msk;
    dfll_sync();

    /*Load Calibration Value*/
    uint8_t calibCoarse = (uint8_t) (((* (uint32_t *)0x806024) >> 26 ) & 0x3f);
    calibCoarse = (((calibCoarse) == 0x3F) ? 0x1F : (calibCoarse));
    uint16_t calibFine = (uint16_t) (((* (uint32_t *)0x806028)) & 0x3ff);

    SYSCTRL_REGS->SYSCTRL_DFLLVAL = SYSCTRL_DFLLVAL_COARSE (calibCoarse) | SYSCTRL_DFLLVAL_FINE (calibFine);
    dfll_sync();

    SYSCTRL_REGS->SYSCTRL_DFLLCTRL = SYSCTRL_DFLLCTRL_MODE_Msk |
                                     SYSCTRL_DFLLCTRL_CCDIS_Msk |
                                     SYSCTRL_DFLLCTRL_USBCRM_Msk | /* USB correction */
                                     SYSCTRL_DFLLCTRL_BPLCKC_Msk;

    SYSCTRL_REGS->SYSCTRL_DFLLCTRL |= SYSCTRL_DFLLCTRL_ENABLE_Msk ;
    dfll_sync();

#else

    SYSCTRL_REGS->SYSCTRL_XOSC32K =
        SYSCTRL_XOSC32K_STARTUP (6) | SYSCTRL_XOSC32K_XTALEN_Msk | SYSCTRL_XOSC32K_EN32K_Msk;
    SYSCTRL_REGS->SYSCTRL_XOSC32K |= SYSCTRL_XOSC32K_ENABLE_Msk;
    while ((SYSCTRL_REGS->SYSCTRL_PCLKSR & SYSCTRL_PCLKSR_XOSC32KRDY_Msk) == 0)
        ;

    GCLK_REGS->GCLK_GENDIV = GCLK_GENDIV_ID (1);
    gclk_sync();

    GCLK_REGS->GCLK_GENCTRL = GCLK_GENCTRL_ID (1) | GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_GENEN_Msk;
    gclk_sync();

    GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_ID (0) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN_Msk;
    gclk_sync();

    SYSCTRL_REGS->SYSCTRL_DFLLCTRL &= ~SYSCTRL_DFLLCTRL_ONDEMAND_Msk;
    dfll_sync();

    SYSCTRL_REGS->SYSCTRL_DFLLMUL = SYSCTRL_DFLLMUL_CSTEP (31) | SYSCTRL_DFLLMUL_FSTEP (511) |
                                    SYSCTRL_DFLLMUL_MUL ((48000000 / (32 * 1024)));
    dfll_sync();

    SYSCTRL_REGS->SYSCTRL_DFLLCTRL |=
        SYSCTRL_DFLLCTRL_MODE_Msk | SYSCTRL_DFLLCTRL_WAITLOCK_Msk | SYSCTRL_DFLLCTRL_QLDIS_Msk;
    dfll_sync();

    SYSCTRL_REGS->SYSCTRL_DFLLCTRL |= SYSCTRL_DFLLCTRL_ENABLE_Msk;

    while ((SYSCTRL_REGS->SYSCTRL_PCLKSR & SYSCTRL_PCLKSR_DFLLLCKC_Msk) == 0 ||
           (SYSCTRL_REGS->SYSCTRL_PCLKSR & SYSCTRL_PCLKSR_DFLLLCKF_Msk) == 0)
        ;
    dfll_sync();

#endif

    // Configure DFLL48M as source for GCLK_GEN 0
    GCLK_REGS->GCLK_GENDIV = GCLK_GENDIV_ID (0);
    gclk_sync();

    // Add GCLK_GENCTRL_OE below to output GCLK0 on the SWCLK pin.
    GCLK_REGS->GCLK_GENCTRL =
        GCLK_GENCTRL_ID (0) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_IDC_Msk | GCLK_GENCTRL_GENEN_Msk;
    gclk_sync();

    /*Disable RC oscillator*/
    SYSCTRL_REGS->SYSCTRL_OSC8M = 0x0;

    SystemCoreClock_Update (BSP_CLOCK_SYSTEM_FREQ_MHZ * 1000000UL);
}

/* SysTick configuration */
void rt_hw_systick_init (void)
{
    SysTick_Config (SystemCoreClock / RT_TICK_PER_SECOND);

    NVIC_SetPriority (SysTick_IRQn, 0xFF);
}

void SysTick_Handler (void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}

/**
 * This function will initial SAMD21 board.
 */
void hw_board_init (char *clock_src, int32_t clock_src_freq, int32_t clock_target_freq)
{
    /* enable interrupt */
    __set_PRIMASK (0);
    /* System clock initialization */
    SystemClock_Config();
    /* disable interrupt */
    __set_PRIMASK (1);

    rt_hw_systick_init();

    /* Pin driver initialization is open by default */
#ifdef RT_USING_PIN
    extern int rt_hw_pin_init (void);
    rt_hw_pin_init();
#endif

    /* USART driver initialization is open by default */
#ifdef RT_USING_SERIAL
    extern int rt_hw_usart_init (void);
    rt_hw_usart_init();
#endif
}

void rt_hw_delay_us (rt_uint32_t us)
{
    rt_uint32_t start, now, delta, reload, us_tick;

    start = SysTick->VAL;
    reload = SysTick->LOAD;
    us_tick = SystemCoreClock / 1000000UL;

    do {
        now = SysTick->VAL;
        delta = start > now ? start - now : reload + start - now;
    } while (delta < us_tick * us);
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
