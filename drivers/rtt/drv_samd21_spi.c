#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"

#include <string.h>

/* SERCOM clk freq value for the baud calculation */
#define SERCOM_Frequency      (48000000UL)

/* SERCOM SPI baud value for 1000000 Hz baud rate */
#define SERCOM_SPIM_BAUD_VALUE         (23UL)

//#define DRV_DEBUG
#define LOG_TAG     "spi"
#define DBG_LVL     DBG_INFO
#include <rtdbg.h>

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

struct samd21_spi_config {
    void *regs;
    uint32_t clock;
    char *bus_name;
};

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

void SERCOM_MSPI_Initialize (struct samd21_spi_config *spim_config)
{
    sercom_spim_registers_t *spim_regs = (sercom_spim_registers_t *)spim_config->regs;

    if (! (spim_regs->SERCOM_SYNCBUSY & SERCOM_SPIM_SYNCBUSY_SWRST_Msk)) {
        uint32_t mode = spim_regs->SERCOM_CTRLA & SERCOM_SPIM_CTRLA_MODE_Msk;
        if (spim_regs->SERCOM_CTRLA & SERCOM_SPIM_CTRLA_ENABLE_Msk) {
            spim_regs->SERCOM_CTRLA &= ~SERCOM_SPIM_CTRLA_ENABLE_Msk;
            /* Wait for synchronization */
            while ((spim_regs->SERCOM_SYNCBUSY) != 0U);
        }
        spim_regs->SERCOM_CTRLA = mode | SERCOM_SPIM_CTRLA_SWRST_Msk;
        /* Wait for synchronization */
        while ((spim_regs->SERCOM_SYNCBUSY) != 0U);
    }

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
                              SERCOM_SPIM_CTRLA_DIPO_PAD0 | SERCOM_SPIM_CTRLA_CPOL_IDLE_LOW |
                              SERCOM_SPIM_CTRLA_CPHA_LEADING_EDGE | SERCOM_SPIM_CTRLA_DORD_MSB |
                              SERCOM_SPIM_CTRLA_ENABLE_Msk ;

    /* Wait for synchronization */
    while ((spim_regs->SERCOM_SYNCBUSY) != 0U);
}

rt_err_t SERCOM_SPI_TransferSetup (struct samd21_spi_config *spim_config, struct rt_spi_configuration *cfg)
{
    sercom_spim_registers_t *spim_regs = (sercom_spim_registers_t *)spim_config->regs;
    uint32_t baudValue = 0U;
    rt_err_t statusValue = RT_EOK;

    /* Wait for synchronization */
    while ((spim_regs->SERCOM_SYNCBUSY) != 0U);

    /* Disable the SPI Module */
    spim_regs->SERCOM_CTRLA &= ~ SERCOM_SPIM_CTRLA_ENABLE_Msk;

    if (cfg != NULL) {
        if (cfg->mode & RT_SPI_SLAVE) {
            LOG_E ("%s only support master mode\n", spim_config->bus_name);
            return RT_EIO;
        }

        baudValue = (SERCOM_Frequency / (2U * (cfg->max_hz))) - 1U;

        if ((baudValue > 0U) && (baudValue <= 255U)) {
            /* Selection of the Clock Polarity and Clock Phase */
            spim_regs->SERCOM_CTRLA &= ~ (SERCOM_SPIM_CTRLA_CPOL_Msk | SERCOM_SPIM_CTRLA_CPHA_Msk);
            if (cfg->mode & RT_SPI_CPOL)
                spim_regs->SERCOM_CTRLA |= SERCOM_SPIM_CTRLA_CPOL_IDLE_HIGH;
            if (cfg->mode & RT_SPI_CPHA)
                spim_regs->SERCOM_CTRLA |= SERCOM_SPIM_CTRLA_CPHA_TRAILING_EDGE;

            /* Selection of the Baud Value */
            spim_regs->SERCOM_BAUD = (uint8_t)baudValue;

            /* Selection of the Character Size */
            spim_regs->SERCOM_CTRLB &= ~SERCOM_SPIM_CTRLB_CHSIZE_Msk;
            if (cfg->data_width == 9)   // Support 8b and 9b only
                spim_regs->SERCOM_CTRLB |= SERCOM_SPIM_CTRLB_CHSIZE_9_BIT;
            else if (cfg->data_width == 8)
                spim_regs->SERCOM_CTRLB |= SERCOM_SPIM_CTRLB_CHSIZE_8_BIT;
            else {
                LOG_E ("%s only support 8b & 9b mode\n", spim_config->bus_name);
                return RT_EIO;
            }

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

uint8_t SERCOM_SPI_TransferDataSPI (sercom_spim_registers_t *spim_regs, uint8_t data)
{
    while (! (spim_regs->SERCOM_INTFLAG & SERCOM_SPIM_INTFLAG_DRE_Msk)) {
        // Waiting Complete Reception
    }

    spim_regs->SERCOM_DATA = data; // Writing data into Data register
    while (! (spim_regs->SERCOM_INTFLAG & SERCOM_SPIM_INTFLAG_RXC_Msk)) {
        // Waiting Complete Reception
    }
    return spim_regs->SERCOM_DATA & 0xFF;  // Reading data
}

rt_err_t SERCOM_SPI_Transfer (sercom_spim_registers_t *spim_regs, uint8_t *txBuffer, uint16_t txLength)
{
    rt_err_t statusValue = RT_EOK;
    rt_uint32_t i;

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
            SERCOM_SPI_TransferDataSPI (spim_regs, txBuffer[i]);
        }
    }

    return statusValue;
}

rt_err_t SERCOM_SPI_Receive (sercom_spim_registers_t *spim_regs,
                             uint8_t *rxBuffer, uint16_t rxMaxLength)
{
    rt_err_t statusValue = RT_EOK;
    rt_uint32_t i;

    if (rxBuffer != NULL) {
        for (i = 0; i < rxMaxLength; i++) {
            rxBuffer[i] = SERCOM_SPI_TransferDataSPI (spim_regs, 0xFF);
        }
    }

    return statusValue;
}

rt_err_t SERCOM_SPI_TransferReceive (sercom_spim_registers_t *spim_regs,
                                     uint8_t *txBuffer, uint8_t *rxBuffer,
                                     uint16_t txLength, uint16_t rxMaxLength)
{
    rt_err_t statusValue = RT_EOK;
    rt_uint32_t i;

    if (txBuffer != NULL) {
        for (i = 0; i < txLength; i++) {
            SERCOM_SPI_TransferDataSPI (spim_regs, txBuffer[i]);
        }
    }

    if (rxBuffer != NULL) {
        for (i = 0; i < rxMaxLength; i++) {
            rxBuffer[i] = SERCOM_SPI_TransferDataSPI (spim_regs, 0xFF);
        }
    }

    return statusValue;
}

struct samd21_hw_spi_cs {
    uint16_t GPIO_Port;
    uint16_t GPIO_Pin;
};

#ifdef BSP_USING_SERCOM0_SPI
#ifndef SPI0_BUS_CONFIG
#define SPI0_BUS_CONFIG                             \
    {                                               \
        .regs = SERCOM0_REGS,                       \
        .clock = 2000000,                           \
        .bus_name = "spi0",                         \
    }
#endif /* SPI0_BUS_CONFIG */
#endif /* BSP_USING_SERCOM0_SPI */

#ifdef BSP_USING_SERCOM1_SPI
#ifndef SPI1_BUS_CONFIG
#define SPI1_BUS_CONFIG                             \
    {                                               \
        .regs = SERCOM1_REGS,                       \
        .clock = 8000000,                           \
        .bus_name = "spi1",                         \
    }
#endif /* SPI1_BUS_CONFIG */
#endif /* BSP_USING_SERCOM1_SPI */

static struct samd21_spi_config spi_config[] = {
#ifdef BSP_USING_SERCOM0_SPI
    SPI0_BUS_CONFIG,
#endif

#ifdef BSP_USING_SERCOM1_SPI
    SPI1_BUS_CONFIG,
#endif
};

/* samd21 spi dirver class */
struct samd21_spi {
    struct rt_spi_bus spi_bus;
    struct samd21_spi_config *config;
    struct rt_spi_configuration *cfg;
};

static struct samd21_spi spi_bus_obj[sizeof (spi_config) / sizeof (spi_config[0])];

/**
  * Attach the spi device to SPI bus, this function must be used after initialization.
  */
rt_err_t rt_hw_spi_device_attach (const char *bus_name, const char *device_name, uint16_t cs_gpio_port, uint16_t cs_gpio_pin)
{
    RT_ASSERT (bus_name != RT_NULL);
    RT_ASSERT (device_name != RT_NULL);

    rt_err_t result;
    struct rt_spi_device *spi_device;
    struct samd21_hw_spi_cs *cs_pin;

    /* initialize the cs pin && select the slave*/
    PORT_REGS->GROUP[cs_gpio_port].PORT_PINCFG[cs_gpio_pin] = 0;
    PORT_REGS->GROUP[cs_gpio_port].PORT_DIRSET = (1 << cs_gpio_pin);
    PORT_REGS->GROUP[cs_gpio_port].PORT_PINCFG[cs_gpio_pin] = PORT_PINCFG_PULLEN (0) | PORT_PINCFG_INEN (1);
    PORT_REGS->GROUP[cs_gpio_port].PORT_OUTSET = (1 << cs_gpio_pin);

    /* attach the device to spi bus*/
    spi_device = (struct rt_spi_device *)rt_malloc (sizeof (struct rt_spi_device));
    RT_ASSERT (spi_device != RT_NULL);
    cs_pin = (struct samd21_hw_spi_cs *)rt_malloc (sizeof (struct samd21_hw_spi_cs));
    RT_ASSERT (cs_pin != RT_NULL);
    cs_pin->GPIO_Port = cs_gpio_port;
    cs_pin->GPIO_Pin = cs_gpio_pin;
    result = rt_spi_bus_attach_device (spi_device, device_name, bus_name, (void *)cs_pin);

    if (result != RT_EOK) {
        LOG_E ("%s attach to %s faild, %d\n", device_name, bus_name, result);
    }

    RT_ASSERT (result == RT_EOK);

    LOG_I ("%s attach to %s done", device_name, bus_name);

    return result;
}

static rt_err_t spi_configure (struct rt_spi_device *device,
                               struct rt_spi_configuration *configuration)
{
    RT_ASSERT (device != RT_NULL);
    RT_ASSERT (configuration != RT_NULL);

    struct samd21_spi *spi_drv =  rt_container_of (device->bus, struct samd21_spi, spi_bus);
    spi_drv->cfg = configuration;

    return SERCOM_SPI_TransferSetup (spi_drv->config, configuration);
}

static rt_uint32_t spixfer (struct rt_spi_device *device, struct rt_spi_message *message)
{
    rt_size_t message_length, already_send_length;
    rt_uint16_t send_length;
    rt_uint8_t *recv_buf;
    const rt_uint8_t *send_buf;
    rt_err_t state;

    RT_ASSERT (device != RT_NULL);
    RT_ASSERT (device->bus != RT_NULL);
    RT_ASSERT (device->bus->parent.user_data != RT_NULL);
    RT_ASSERT (message != RT_NULL);

//    struct samd21_spi *spi_drv =  rt_container_of (device->bus, struct samd21_spi, spi_bus);
    struct samd21_spi_config *spi_config = device->bus->parent.user_data;
    struct samd21_hw_spi_cs *cs = device->parent.user_data;

    LOG_D ("%s transfer prepare and start", spi_drv->config->bus_name);
    LOG_D ("%s sendbuf: %X, recvbuf: %X, length: %d", spi_drv->config->bus_name,
           (uint32_t)message->send_buf,
           (uint32_t)message->recv_buf, message->length);

    message_length = message->length;
    recv_buf = message->recv_buf;
    send_buf = message->send_buf;

    ((sercom_spim_registers_t *)spi_config->regs)->SERCOM_STATUS |= (uint16_t)SERCOM_SPIM_STATUS_BUFOVF_Msk;
    ((sercom_spim_registers_t *)spi_config->regs)->SERCOM_INTFLAG |= (uint8_t)SERCOM_SPIM_INTFLAG_ERROR_Msk;

    if (message->cs_take && ! (device->config.mode & RT_SPI_NO_CS)) {
        PORT_REGS->GROUP[cs->GPIO_Port].PORT_OUTCLR = (1 << cs->GPIO_Pin);
    }

    while (message_length) {
        /* the HAL library use uint16 to save the data length */
        if (message_length > 65535) {
            send_length = 65535;
            message_length = message_length - 65535;
        } else {
            send_length = message_length;
            message_length = 0;
        }

        /* calculate the start address */
        already_send_length = message->length - send_length - message_length;
        send_buf = (rt_uint8_t *)message->send_buf + already_send_length;
        recv_buf = (rt_uint8_t *)message->recv_buf + already_send_length;

        /* start once data exchange in DMA mode */
        if (message->send_buf && message->recv_buf) {
            state = SERCOM_SPI_TransferReceive (spi_config->regs, (uint8_t *)send_buf, (uint8_t *)recv_buf, send_length, 1000);
        } else if (message->send_buf) {
            state = SERCOM_SPI_Transfer (spi_config->regs, (uint8_t *)send_buf, send_length);

            if (message->cs_release && (device->config.mode & RT_SPI_3WIRE)) {
                /* release the CS by disable SPI when using 3 wires SPI */
                //__HAL_SPI_DISABLE (spi_handle);
            }
        } else {
            memset ((uint8_t *)recv_buf, 0xff, send_length);
            /* clear the old error flag */
            ((sercom_spim_registers_t *)spi_config->regs)->SERCOM_INTFLAG |= SERCOM_SPIM_INTFLAG_RXC_Msk;
            ((sercom_spim_registers_t *)spi_config->regs)->SERCOM_INTFLAG |= SERCOM_SPIM_INTFLAG_ERROR_Msk;
            state = SERCOM_SPI_Receive (spi_config->regs, (uint8_t *)recv_buf, send_length);
        }

        if (state != RT_EOK) {
            LOG_I ("spi transfer error : %d", state);
            message->length = 0;
        }
    }

    if (message->cs_release && ! (device->config.mode & RT_SPI_NO_CS)) {
        PORT_REGS->GROUP[cs->GPIO_Port].PORT_OUTSET = (1 << cs->GPIO_Pin);
    }

    return message->length;
}

static const struct rt_spi_ops samd21_spi_ops = {
    .configure = spi_configure,
    .xfer = spixfer,
};

int rt_hw_spi_init (void)
{
    rt_err_t result = RT_EIO;

    SERCOM_MSPI_ClockInit();
    SERCOM_MSPI_PortInit();
    for (int i = 0; i < sizeof (spi_config) / sizeof (spi_config[0]); i++) {
        spi_bus_obj[i].config = &spi_config[i];
        spi_bus_obj[i].spi_bus.parent.user_data = &spi_config[i];

        SERCOM_MSPI_Initialize (&spi_config[i]);

        result = rt_spi_bus_register (&spi_bus_obj[i].spi_bus, spi_config[i].bus_name, &samd21_spi_ops);
        RT_ASSERT (result == RT_EOK);

        LOG_I ("%s bus init done", spi_config[i].bus_name);
    }

    return result;
}
INIT_DEVICE_EXPORT (rt_hw_spi_init);

#endif /* BSP_USING_SERCOM0_SPI || BSP_USING_SERCOM1_SPI */
#endif /* BSP_USING_SPI */
