/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-10-26     RealThread   the first version
 */

#include <stdio.h>
#include <stdint.h>

#include "board.h"

inline void dfll_sync (void)
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
    while (GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk)
        ;

    GCLK_REGS->GCLK_GENCTRL = GCLK_GENCTRL_ID (2) | GCLK_GENCTRL_SRC_OSC8M_Val | GCLK_GENCTRL_GENEN_Msk;
    while (GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk)
        ;

    // Turn on DFLL with USB correction and sync to internal 8 mhz oscillator
    SYSCTRL_REGS->SYSCTRL_DFLLCTRL &= ~SYSCTRL_DFLLCTRL_ONDEMAND_Msk;
    while ((SYSCTRL_REGS->SYSCTRL_PCLKSR & SYSCTRL_PCLKSR_DFLLRDY_Msk) == 0)
        ;

    /*Load Calibration Value*/
    uint8_t calibCoarse = (uint8_t) (((* (uint32_t *)0x806024) >> 26 ) & 0x3f);
    calibCoarse = (((calibCoarse) == 0x3F) ? 0x1F : (calibCoarse));
    uint16_t calibFine = (uint16_t) (((* (uint32_t *)0x806028)) & 0x3ff);

    SYSCTRL_REGS->SYSCTRL_DFLLVAL = SYSCTRL_DFLLVAL_COARSE (calibCoarse) | SYSCTRL_DFLLVAL_FINE (calibFine);
    while ((SYSCTRL_REGS->SYSCTRL_PCLKSR & SYSCTRL_PCLKSR_DFLLRDY_Msk) == 0)
        ;

    SYSCTRL_REGS->SYSCTRL_DFLLCTRL = SYSCTRL_DFLLCTRL_MODE_Msk |
                                     SYSCTRL_DFLLCTRL_CCDIS_Msk |
                                     SYSCTRL_DFLLCTRL_USBCRM_Msk | /* USB correction */
                                     SYSCTRL_DFLLCTRL_BPLCKC_Msk;

    SYSCTRL_REGS->SYSCTRL_DFLLCTRL |= SYSCTRL_DFLLCTRL_ENABLE_Msk ;
    while ((SYSCTRL_REGS->SYSCTRL_PCLKSR & SYSCTRL_PCLKSR_DFLLRDY_Msk) == 0)
        ;

#else

    SYSCTRL_REGS->SYSCTRL_XOSC32K =
        SYSCTRL_XOSC32K_STARTUP (6) | SYSCTRL_XOSC32K_XTALEN_Msk | SYSCTRL_XOSC32K_EN32K_Msk;
    SYSCTRL_REGS->SYSCTRL_XOSC32K |= SYSCTRL_XOSC32K_ENABLE_Msk;
    while ((SYSCTRL_REGS->SYSCTRL_PCLKSR & SYSCTRL_PCLKSR_XOSC32KRDY_Msk) == 0)
        ;

    GCLK_REGS->GCLK_GENDIV = GCLK_GENDIV_ID (1);
    while (GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk)
        ;

    GCLK_REGS->GCLK_GENCTRL = GCLK_GENCTRL_ID (1) | GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_GENEN_Msk;
    while (GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk)
        ;

    GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_ID (0) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN_Msk;
    while (GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk)
        ;

    SYSCTRL_REGS->SYSCTRL_DFLLCTRL &= ~SYSCTRL_DFLLCTRL_ONDEMAND_Msk;
    while ((SYSCTRL_REGS->SYSCTRL_PCLKSR & SYSCTRL_PCLKSR_DFLLRDY_Msk) == 0)
        ;

    SYSCTRL_REGS->SYSCTRL_DFLLMUL = SYSCTRL_DFLLMUL_CSTEP (31) | SYSCTRL_DFLLMUL_FSTEP (511) |
                                    SYSCTRL_DFLLMUL_MUL ((48000000 / (32 * 1024)));
    while ((SYSCTRL_REGS->SYSCTRL_PCLKSR & SYSCTRL_PCLKSR_DFLLRDY_Msk) == 0)
        ;

    SYSCTRL_REGS->SYSCTRL_DFLLCTRL |=
        SYSCTRL_DFLLCTRL_MODE_Msk | SYSCTRL_DFLLCTRL_WAITLOCK_Msk | SYSCTRL_DFLLCTRL_QLDIS_Msk;
    while ((SYSCTRL_REGS->SYSCTRL_PCLKSR & SYSCTRL_PCLKSR_DFLLRDY_Msk) == 0)
        ;

    SYSCTRL_REGS->SYSCTRL_DFLLCTRL |= SYSCTRL_DFLLCTRL_ENABLE_Msk;

    while ((SYSCTRL_REGS->SYSCTRL_PCLKSR & SYSCTRL_PCLKSR_DFLLLCKC_Msk) == 0 ||
           (SYSCTRL_REGS->SYSCTRL_PCLKSR & SYSCTRL_PCLKSR_DFLLLCKF_Msk) == 0)
        ;
    while ((SYSCTRL_REGS->SYSCTRL_PCLKSR & SYSCTRL_PCLKSR_DFLLRDY_Msk) == 0)
        ;

#endif

    // Configure DFLL48M as source for GCLK_GEN 0
    GCLK_REGS->GCLK_GENDIV = GCLK_GENDIV_ID (0);
    while (GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk)
        ;

    // Add GCLK_GENCTRL_OE below to output GCLK0 on the SWCLK pin.
    GCLK_REGS->GCLK_GENCTRL =
        GCLK_GENCTRL_ID (0) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_IDC_Msk | GCLK_GENCTRL_GENEN_Msk;
    while (GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk)
        ;

    /*Disable RC oscillator*/
    SYSCTRL_REGS->SYSCTRL_OSC8M = 0x0;

    SystemCoreClock_Update (BSP_CLOCK_SYSTEM_FREQ_MHZ * 1000000UL);
}

/* SysTick configuration */
void hw_systick_init (void)
{
    SysTick_Config (SystemCoreClock / BSP_SYSTICK_PER_SECOND);

    NVIC_SetPriority (SysTick_IRQn, 0xFF);
    NVIC_EnableIRQ (SysTick_IRQn);
}

volatile uint32_t fac_ms = 0;      //ms延时倍乘数
SysTick_User_CB_t systick_cb = NULL;

void SysTick_SetCB (SysTick_User_CB_t user_cb)
{
    systick_cb = user_cb;
}

void SysTick_Handler (void)
{
    if (fac_ms)     fac_ms--;

    if (systick_cb != NULL)
        systick_cb();
}

extern void uart_rx_handle (void *dev, uint16_t size);

/**
 * This function will initial SAMD21 board.
 */
void hw_board_init (void)
{
    /* enable interrupt */
    __set_PRIMASK (0);
    /* System clock initialization */
    SystemClock_Config();
    /* disable interrupt */
    __set_PRIMASK (1);

    /* Enable NVIC Controller */
    __DMB();
    __enable_irq();

    hw_systick_init();

    /* USART driver initialization is open by default */
#ifdef BSP_USING_UART
    hw_usart_init();
#endif

#ifdef BSP_USING_SPI
    hw_spi_init();
#endif

#ifdef BSP_USING_I2C
    hw_i2c_init();
#endif

}

/**
 * \brief Perform delay in ms
 */
void hw_delay_ms (const uint32_t ms)
{
    fac_ms = ms;

    while (fac_ms > 0);
}

void wait_ms (unsigned long ms)
{
    hw_delay_ms (ms);
}

void hw_delay_us (uint32_t us)
{
    uint32_t start, now, delta, reload, us_tick;

    start = SysTick->VAL;
    reload = SysTick->LOAD;
    us_tick = SystemCoreClock / 1000000UL;

    do {
        now = SysTick->VAL;
        delta = start > now ? start - now : reload + start - now;
    } while (delta < us_tick * us);
}

int entry (void)
{
    extern int main (void);
    hw_board_init();

    return main();
}
