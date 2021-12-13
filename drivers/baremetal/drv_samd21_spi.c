#include "board.h"

#include <string.h>

/* SERCOM clk freq value for the baud calculation */
#define SERCOM_Frequency      (48000000UL)

/* SERCOM SPI baud value for 1000000 Hz baud rate */
#define SERCOM_SPIM_BAUD_VALUE         (23UL)

#ifdef BSP_USING_SPI

#if defined(BSP_USING_SERCOM0_SPI) || defined(BSP_USING_SERCOM1_SPI)

#if defined(BSP_USING_I2C0) && defined(BSP_USING_SERCOM0_SPI)
#error "Please define only 1 of BSP_USING_I2C0 and BSP_USING_SERCOM0_SPI"
#endif

#if defined(BSP_USING_SERCOM0_USART) && defined(BSP_USING_SERCOM0_SPI)
#error "Please define only 1 of BSP_USING_SERCOM0_USART and BSP_USING_SERCOM0_SPI"
#endif

#ifdef BSP_USING_SERCOM0_SPI
#define SPI0_MODULE       SERCOM0_REGS
#define SPI0_PAD3         PINMUX_PA07D_SERCOM0_PAD3     // PINMUX_PA07D_SERCOM0_PAD3 PINMUX_PA11C_SERCOM0_PAD3
#define SPI0_PAD2         PINMUX_PA06D_SERCOM0_PAD2     // PINMUX_PA06D_SERCOM0_PAD2 PINMUX_PA10C_SERCOM0_PAD2
#define SPI0_PAD1         PINMUX_UNUSED                 // PINMUX_PA05D_SERCOM0_PAD1 PINMUX_PA09C_SERCOM0_PAD1
#define SPI0_PAD0         PINMUX_PA04D_SERCOM0_PAD0     // PINMUX_PA04D_SERCOM0_PAD0 PINMUX_PA08C_SERCOM0_PAD0
#endif

#ifdef BSP_USING_SERCOM1_SPI
#define SPI1_MODULE       SERCOM1_REGS
#define SPI1_PAD3         PINMUX_PA19D_SERCOM3_PAD3     // PINMUX_PA19D_SERCOM3_PAD3 PINMUX_PA25C_SERCOM3_PAD3
#define SPI1_PAD2         PINMUX_PA18D_SERCOM3_PAD2     // PINMUX_PA18D_SERCOM3_PAD2 PINMUX_PA24C_SERCOM3_PAD2
#define SPI1_PAD1         PINMUX_UNUSED                 // PINMUX_PA17D_SERCOM3_PAD1 PINMUX_PA23C_SERCOM3_PAD1
#define SPI1_PAD0         PINMUX_PA16D_SERCOM3_PAD0     // PINMUX_PA16D_SERCOM3_PAD0 PINMUX_PA22C_SERCOM3_PAD0
#endif

// *****************************************************************************
// *****************************************************************************
// Section: SERCOM USART Interface Routines
// *****************************************************************************
// *****************************************************************************
static void SERCOM_MSPI_ClockInit ( void )
{
#ifdef BSP_USING_SERCOM0_SPI
    /* Selection of the Generator and write Lock for SERCOM0_CORE */
    GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_ID (GCLK_CLKCTRL_ID_SERCOM0_CORE_Val) |
                              GCLK_CLKCTRL_GEN (GCLK_CLKCTRL_GEN_GCLK0_Val)  |
                              GCLK_CLKCTRL_CLKEN_Msk;

    /* Configure the APBC Bridge Clocks */
    PM_REGS->PM_APBCMASK |= PM_APBCMASK_SERCOM (1 << 0);
#endif

#ifdef BSP_USING_SERCOM1_SPI
    /* Selection of the Generator and write Lock for SERCOM1_CORE */
    GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_ID (GCLK_CLKCTRL_ID_SERCOM1_CORE_Val) |
                              GCLK_CLKCTRL_GEN (GCLK_CLKCTRL_GEN_GCLK0_Val)  |
                              GCLK_CLKCTRL_CLKEN_Msk;

    /* Configure the APBC Bridge Clocks */
    PM_REGS->PM_APBCMASK |= PM_APBCMASK_SERCOM (1 << 1);
#endif

    while ( GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk ) {
        /* Wait for synchronization */
    }
}

static void SERCOM_MSPI_PortInit ( void )
{
#ifdef BSP_USING_SERCOM0_SPI
    /************************** GROUP 0 Initialization *************************/
    PORT_PinMUX_Config (SPI0_PAD0);
    PORT_PinMUX_Config (SPI0_PAD1);
    PORT_PinMUX_Config (SPI0_PAD2);
    PORT_PinMUX_Config (SPI0_PAD3);
#endif

#ifdef BSP_USING_SERCOM1_SPI
    /************************** GROUP 0 Initialization *************************/
    PORT_PinMUX_Config (SPI1_PAD0);
    PORT_PinMUX_Config (SPI1_PAD1);
    PORT_PinMUX_Config (SPI1_PAD2);
    PORT_PinMUX_Config (SPI1_PAD3);
#endif
}

void SERCOM_MSPI_Initialize (struct samd21_spi_t *sam_spim)
{
    sercom_spim_registers_t *spim_regs = (sercom_spim_registers_t *)sam_spim->regs;

    /* Selection of the Character Size and Receiver Enable */
    spim_regs->SERCOM_CTRLB = SERCOM_SPIM_CTRLB_CHSIZE_8_BIT | SERCOM_SPIM_CTRLB_RXEN_Msk ;

    /* Wait for synchronization */
    while ((spim_regs->SERCOM_SYNCBUSY) != 0U);

    /* Selection of the Baud Value */
    spim_regs->SERCOM_BAUD = (uint8_t)SERCOM_SPIM_BAUD_BAUD (SERCOM_SPIM_BAUD_VALUE);

    /* Configure Data Out Pin Out , Master Mode,
     * Data In and Pin Out,Data Order and Standby mode if configured
     * and Selection of the Clock Phase and Polarity and Enable the SPI Module
     */
    spim_regs->SERCOM_CTRLA = SERCOM_SPIM_CTRLA_MODE_SPI_MASTER | SERCOM_SPIM_CTRLA_DOPO_PAD1 |
                              SERCOM_SPIM_CTRLA_DIPO_PAD0 | SERCOM_SPIM_CTRLA_CPOL_IDLE_HIGH |
                              SERCOM_SPIM_CTRLA_CPHA_TRAILING_EDGE | SERCOM_SPIM_CTRLA_DORD_MSB |
                              SERCOM_SPIM_CTRLA_ENABLE_Msk ;

    /* Wait for synchronization */
    while ((spim_regs->SERCOM_SYNCBUSY) != 0U);
}

uint32_t SERCOM_SPI_TransferSetup (struct samd21_spi_t *sam_spim, SPI_TRANSFER_SETUP *setup, uint32_t spiSourceClock)
{
    sercom_spim_registers_t *spim_regs = (sercom_spim_registers_t *) (sam_spim->regs);
    uint32_t baudValue = 0U;
    uint32_t statusValue = RT_EOK;

    if (spiSourceClock == 0U)    {
        /* Fetch Master Clock Frequency directly */
        spiSourceClock = SERCOM_Frequency;
    }

    /* Disable the SPI Module */
    spim_regs->SERCOM_CTRLA &= ~ (SERCOM_SPIM_CTRLA_ENABLE_Msk);

    /* Wait for synchronization */
    while ((spim_regs->SERCOM_SYNCBUSY) != 0U);

    if (setup != NULL) {
        baudValue = (spiSourceClock / (2U * (setup->clockFrequency))) - 1U;

        if ((baudValue > 0U) && (baudValue <= 255U)) {
            /* Selection of the Clock Polarity and Clock Phase */
            spim_regs->SERCOM_CTRLA &= ~ (SERCOM_SPIM_CTRLA_CPOL_Msk | SERCOM_SPIM_CTRLA_CPHA_Msk);
            spim_regs->SERCOM_CTRLA |= (uint32_t)setup->clockPolarity | (uint32_t)setup->clockPhase;

            /* Selection of the Baud Value */
            spim_regs->SERCOM_BAUD = (uint8_t)baudValue;

            /* Selection of the Character Size */
            spim_regs->SERCOM_CTRLB &= ~SERCOM_SPIM_CTRLB_CHSIZE_Msk;
            spim_regs->SERCOM_CTRLB |= (uint32_t)setup->dataBits;

            /* Wait for synchronization */
            while ((spim_regs->SERCOM_SYNCBUSY) != 0U);

            statusValue = RT_EOK;
        }
    }

    /* Enabling the SPI Module */
    spim_regs->SERCOM_CTRLA |= SERCOM_SPIM_CTRLA_ENABLE_Msk;

    /* Wait for synchronization */
    while ((spim_regs->SERCOM_SYNCBUSY) != 0U);

    return statusValue;
}


uint8_t SERCOM_SPI_TransferDataSPI (struct samd21_spi_t *sam_spim, uint8_t data)
{
    sercom_spim_registers_t *spim_regs = (sercom_spim_registers_t *) (sam_spim->regs);

    while (! (spim_regs->SERCOM_INTFLAG & SERCOM_SPIM_INTFLAG_DRE_Msk)) {
        // Waiting Complete Reception
    }

    spim_regs->SERCOM_DATA = data; // Writing data into Data register
    while (! (spim_regs->SERCOM_INTFLAG & SERCOM_SPIM_INTFLAG_RXC_Msk)) {
        // Waiting Complete Reception
    }

    return spim_regs->SERCOM_DATA & 0xFF;  // Reading data
}

uint32_t SERCOM_SPI_Transfer (struct samd21_spi_t *sam_spim, uint8_t *txBuffer, uint16_t txLength)
{
    sercom_spim_registers_t *spim_regs = (sercom_spim_registers_t *) (sam_spim->regs);

    uint32_t statusValue = RT_EOK;
    uint32_t i;

    /* If settings are not applied (pending), we can not go on */
    if (spim_regs->SERCOM_SYNCBUSY & (SERCOM_SPIM_SYNCBUSY_SWRST_Msk | SERCOM_SPIM_SYNCBUSY_ENABLE_Msk | SERCOM_SPIM_SYNCBUSY_CTRLB_Msk)) {
        return RT_EBUSY;
    }

    /* SPI must be enabled to start synchronous transfer */
    if (! (spim_regs->SERCOM_CTRLA & SERCOM_SPIM_CTRLA_ENABLE_Msk)) {
        return RT_EEMPTY;
    }

    if (txBuffer != NULL) {
        for (i = 0; i < txLength; i++) {
            SERCOM_SPI_TransferDataSPI (sam_spim, txBuffer[i]);
        }
    }

    return statusValue;
}

uint32_t SERCOM_SPI_Receive (struct samd21_spi_t *sam_spim,
                             uint8_t *rxBuffer, uint16_t rxMaxLength)
{
    uint32_t statusValue = RT_EOK;
    uint32_t i;

    if (rxBuffer != NULL) {
        for (i = 0; i < rxMaxLength; i++) {
            rxBuffer[i] = SERCOM_SPI_TransferDataSPI (sam_spim, 0xFF);
        }
    }

    return statusValue;
}

uint32_t SERCOM_SPI_TransferReceive (struct samd21_spi_t *sam_spim,
                                     uint8_t *txBuffer, uint8_t *rxBuffer,
                                     uint16_t txLength, uint16_t rxMaxLength)
{
    uint32_t statusValue = RT_EOK;
    uint32_t i;

    if (txBuffer != NULL) {
        for (i = 0; i < txLength; i++) {
            SERCOM_SPI_TransferDataSPI (sam_spim, txBuffer[i]);
        }
    }

    if (rxBuffer != NULL) {
        for (i = 0; i < rxMaxLength; i++) {
            rxBuffer[i] = SERCOM_SPI_TransferDataSPI (sam_spim, 0xFF);
        }
    }

    return statusValue;
}

struct samd21_spi_t samd21_spi1 = {
    .regs = SERCOM1_REGS,
    .clock = 8000000,
};

void hw_spi_init (void)
{
    SERCOM_MSPI_ClockInit();
    SERCOM_MSPI_PortInit();

    SERCOM_MSPI_Initialize (&samd21_spi1);
}

#endif /* BSP_USING_SERCOM0_SPI || BSP_USING_SERCOM1_SPI */
#endif /* BSP_USING_SPI */
