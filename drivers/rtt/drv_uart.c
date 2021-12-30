#include <rtthread.h>
#include "board.h"

#ifdef RT_USING_SERIAL

#define BSP_UART_USING_TX_RBx

#if (RT_CONSOLEBUF_SIZE > 2048)
#define UART_TX_RB_SIZE RT_CONSOLEBUF_SIZE
#else
#define UART_TX_RB_SIZE 2048
#endif

#if !defined(BSP_USING_UART0) && !defined(BSP_USING_UART3)
#error "Please define at least one BSP_USING_SERCOMx_USART"
/* this driver can be disabled at menuconfig → RT-Thread Components → Device Drivers */
#endif

/* SERCOM USART baud value for 115200 Hz baud rate */
#define SERCOM_USART_INIT_BAUD_VALUE            (63019U)

#ifdef BSP_USING_UART0
#define USART0_MODULE       SERCOM0_REGS
#define USART0_TXPO         SERCOM_USART_INT_CTRLA_TXPO (0x0UL)
#define USART0_RXPO         SERCOM_USART_INT_CTRLA_RXPO (0x1UL)
#define USART0_PAD3         PINMUX_UNUSED   // PINMUX_PA07D_SERCOM0_PAD3 PINMUX_PA11C_SERCOM0_PAD3
#define USART0_PAD2         PINMUX_UNUSED   // PINMUX_PA06D_SERCOM0_PAD2 PINMUX_PA10C_SERCOM0_PAD2
#define USART0_PAD1         PINMUX_PA05D_SERCOM0_PAD1               // PINMUX_PA05D_SERCOM0_PAD1 PINMUX_PA09C_SERCOM0_PAD1
#define USART0_PAD0         PINMUX_PA04D_SERCOM0_PAD0               // PINMUX_PA04D_SERCOM0_PAD0 PINMUX_PA08C_SERCOM0_PAD0

#ifdef BSP_UART_USING_TX_RB
#ifndef RT_USING_HEAP
rt_uint8_t uart0_tx_buffer[UART_TX_RB_SIZE];
ringbuffer_t uart0_tx_rb;
#endif
#endif
#endif

#ifdef BSP_USING_UART3
#define USART3_MODULE       SERCOM3_REGS
#define USART3_TXPO         SERCOM_USART_INT_CTRLA_TXPO (0x0UL)
#define USART3_RXPO         SERCOM_USART_INT_CTRLA_RXPO (0x1UL)
#define USART3_PAD3         PINMUX_UNUSED               // PINMUX_PA19D_SERCOM3_PAD3 PINMUX_PA25C_SERCOM3_PAD3
#define USART3_PAD2         PINMUX_UNUSED               // PINMUX_PA18D_SERCOM3_PAD2 PINMUX_PA24C_SERCOM3_PAD2
#define USART3_PAD1         PINMUX_PA23C_SERCOM3_PAD1   // PINMUX_PA17D_SERCOM3_PAD1 PINMUX_PA23C_SERCOM3_PAD1
#define USART3_PAD0         PINMUX_PA22C_SERCOM3_PAD0   // PINMUX_PA16D_SERCOM3_PAD0 PINMUX_PA22C_SERCOM3_PAD0

#ifdef BSP_UART_USING_TX_RB
#ifndef RT_USING_HEAP
rt_uint8_t uart3_tx_buffer[UART_TX_RB_SIZE];
ringbuffer_t uart3_tx_rb;
#endif
#endif

#endif

// *****************************************************************************
/* USART Serial Configuration
  Summary:
    Defines the data type for the USART serial configurations.

  Description:
    This may be used to set the serial configurations for USART.

  Remarks:
    None.
*/

#define USART_PARITY_NONE   0x02
#define USART_ERROR_NONE    0x00

typedef struct {
    uint32_t baudRate;
    uint32_t parity;
    uint32_t dataWidth;
    uint32_t stopBits;
} USART_SERIAL_SETUP;

/**
 * \brief Retrieve ordinal number of the given sercom hardware instance
 */
static rt_uint8_t sercom_get_hardware_index (const void *const hw)
{
    return ((uint32_t)hw - (uint32_t)SERCOM0_REGS) >> 10;
}

// *****************************************************************************
// *****************************************************************************
// Section: SERCOM USART Interface Routines
// *****************************************************************************
// *****************************************************************************
static void SERCOM_USART_ClockInit ( void )
{
#ifdef BSP_USING_UART0
    /* Selection of the Generator and write Lock for SERCOM4_CORE */
    GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_ID (GCLK_CLKCTRL_ID_SERCOM0_CORE_Val) |
                              GCLK_CLKCTRL_GEN (GCLK_CLKCTRL_GEN_GCLK0_Val)  |
                              GCLK_CLKCTRL_CLKEN_Msk;

    /* Configure the APBC Bridge Clocks */
    PM_REGS->PM_APBCMASK |= PM_APBCMASK_SERCOM (1 << 0);
#endif

#ifdef BSP_USING_UART3
    /* Selection of the Generator and write Lock for SERCOM3_CORE */
    GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_ID (GCLK_CLKCTRL_ID_SERCOM3_CORE_Val) |
                              GCLK_CLKCTRL_GEN (GCLK_CLKCTRL_GEN_GCLK0_Val)  |
                              GCLK_CLKCTRL_CLKEN_Msk;

    /* Configure the APBC Bridge Clocks */
    PM_REGS->PM_APBCMASK |= PM_APBCMASK_SERCOM (1 << 3);
#endif

    while ( GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk ) {
        /* Wait for synchronization */
    }
}

static void SERCOM_USART_PortInit ( void )
{
    /* Configure the port pins for SERCOM_USART */
#ifdef BSP_USING_UART0
    PORT_PinMUX_Config (USART0_PAD0);
    PORT_PinMUX_Config (USART0_PAD1);
    PORT_PinMUX_Config (USART0_PAD2);
    PORT_PinMUX_Config (USART0_PAD3);
#endif

#ifdef BSP_USING_UART3
    PORT_PinMUX_Config (USART3_PAD0);
    PORT_PinMUX_Config (USART3_PAD1);
    PORT_PinMUX_Config (USART3_PAD2);
    PORT_PinMUX_Config (USART3_PAD3);
#endif
}

static void SERCOM_USART_NvicInit ( uint32_t IRQn )
{
    NVIC_SetPriority (IRQn, 3);
    NVIC_EnableIRQ (IRQn);
}

void static SERCOM_USART_ErrorClear ( sercom_registers_t *usart_regs )
{
    rt_uint8_t  u8dummyData = 0;

    /* Clear error flag */
    usart_regs->USART_INT.SERCOM_INTFLAG = SERCOM_USART_INT_INTFLAG_ERROR_Msk;

    /* Clear all errors */
    usart_regs->USART_INT.SERCOM_STATUS = SERCOM_USART_INT_STATUS_PERR_Msk | SERCOM_USART_INT_STATUS_FERR_Msk | SERCOM_USART_INT_STATUS_BUFOVF_Msk;

    /* Flush existing error bytes from the RX FIFO */
    while ((usart_regs->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_RXC_Msk) == SERCOM_USART_INT_INTFLAG_RXC_Msk) {
        u8dummyData = usart_regs->USART_INT.SERCOM_DATA;
    }

    /* Ignore the warning */
    (void)u8dummyData;
}

void SERCOM_USART_Initialize ( sercom_registers_t *usart_regs )
{
    /*
    * Configures USART Clock Mode
    * Configures TXPO and RXPO
    * Configures Data Order
    * Configures Standby Mode
    * Configures Sampling rate
    * Configures IBON
    */
#ifdef BSP_USING_UART0
    if (usart_regs == USART0_MODULE) {
        /* UART0 use 2 & 3 */
        usart_regs->USART_INT.SERCOM_CTRLA = SERCOM_USART_INT_CTRLA_MODE_USART_INT_CLK |
                                             USART0_RXPO | USART0_TXPO |
                                             SERCOM_USART_INT_CTRLA_DORD_Msk | SERCOM_USART_INT_CTRLA_IBON_Msk |
                                             SERCOM_USART_INT_CTRLA_FORM (0x0UL) | SERCOM_USART_INT_CTRLA_SAMPR (0UL);
    }
#endif
#ifdef BSP_USING_UART3
    if (usart_regs == USART3_MODULE) {
        usart_regs->USART_INT.SERCOM_CTRLA = SERCOM_USART_INT_CTRLA_MODE_USART_INT_CLK |
                                             USART3_RXPO | USART3_TXPO |
                                             SERCOM_USART_INT_CTRLA_DORD_Msk | SERCOM_USART_INT_CTRLA_IBON_Msk |
                                             SERCOM_USART_INT_CTRLA_FORM (0x0UL) | SERCOM_USART_INT_CTRLA_SAMPR (0UL);
    }
#endif

    /* Configure Baud Rate */
    usart_regs->USART_INT.SERCOM_BAUD = SERCOM_USART_INT_BAUD_BAUD (SERCOM_USART_INIT_BAUD_VALUE);

    /*
     * Configures RXEN
     * Configures TXEN
     * Configures CHSIZE
     * Configures Parity
     * Configures Stop bits
     */
    usart_regs->USART_INT.SERCOM_CTRLB = SERCOM_USART_INT_CTRLB_CHSIZE_8_BIT | SERCOM_USART_INT_CTRLB_SBMODE_1_BIT |
                                         SERCOM_USART_INT_CTRLB_RXEN_Msk | SERCOM_USART_INT_CTRLB_TXEN_Msk;

    /* Wait for sync */
    while (usart_regs->USART_INT.SERCOM_SYNCBUSY);

    /* Enable the UART after the configurations */
    usart_regs->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_ENABLE_Msk;

    /* Wait for sync */
    while (usart_regs->USART_INT.SERCOM_SYNCBUSY);

    /* Disable Error interrupt */
    usart_regs->USART_INT.SERCOM_INTENCLR = SERCOM_USART_INT_INTENSET_ERROR_Msk;

    /* Disable Receive Complete interrupt */
    usart_regs->USART_INT.SERCOM_INTENCLR = SERCOM_USART_INT_INTENSET_RXC_Msk;

    /* Disable Data Register Empty interrupt */
    usart_regs->USART_INT.SERCOM_INTENCLR = SERCOM_USART_INT_INTENSET_DRE_Msk;
}

uint32_t SERCOM_USART_FrequencyGet ( sercom_registers_t *usart_regs )
{
    rt_uint8_t index = sercom_get_hardware_index (usart_regs);
    uint32_t clock = 0;

    switch (index) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
            clock = 48000000UL;
            break;
        default:
            break;
    }

    return clock;
}

bool SERCOM_USART_SerialSetup ( sercom_registers_t *usart_regs, USART_SERIAL_SETUP *serialSetup, uint32_t clkFrequency )
{
    bool setupStatus       = false;
    uint32_t baudValue     = 0;
    uint32_t sampleRate    = 0;

    if ((serialSetup != RT_NULL) & (serialSetup->baudRate != 0)) {
        if (clkFrequency == 0) {
            clkFrequency = SERCOM_USART_FrequencyGet (usart_regs);
        }

        if (clkFrequency >= (16 * serialSetup->baudRate)) {
            baudValue = 65536 - ((uint64_t)65536 * 16 * serialSetup->baudRate) / clkFrequency;
            sampleRate = 0;
        } else if (clkFrequency >= (8 * serialSetup->baudRate)) {
            baudValue = 65536 - ((uint64_t)65536 * 8 * serialSetup->baudRate) / clkFrequency;
            sampleRate = 2;
        } else if (clkFrequency >= (3 * serialSetup->baudRate)) {
            baudValue = 65536 - ((uint64_t)65536 * 3 * serialSetup->baudRate) / clkFrequency;
            sampleRate = 4;
        }

        if (baudValue != 0) {
            /* Disable the USART before configurations */
            usart_regs->USART_INT.SERCOM_CTRLA &= ~SERCOM_USART_INT_CTRLA_ENABLE_Msk;

            /* Wait for sync */
            while (usart_regs->USART_INT.SERCOM_SYNCBUSY);

            /* Configure Baud Rate */
            usart_regs->USART_INT.SERCOM_BAUD = SERCOM_USART_INT_BAUD_BAUD (baudValue);

            /* Configure Parity Options */
            if (serialSetup->parity == USART_PARITY_NONE) {
                usart_regs->USART_INT.SERCOM_CTRLA =  (usart_regs->USART_INT.SERCOM_CTRLA & ~ (SERCOM_USART_INT_CTRLA_SAMPR_Msk |
                                                       SERCOM_USART_INT_CTRLA_FORM_Msk)) |
                                                      SERCOM_USART_INT_CTRLA_FORM (0x0) |
                                                      SERCOM_USART_INT_CTRLA_SAMPR (sampleRate);
                usart_regs->USART_INT.SERCOM_CTRLB = (usart_regs->USART_INT.SERCOM_CTRLB & ~ (SERCOM_USART_INT_CTRLB_CHSIZE_Msk |
                                                      SERCOM_USART_INT_CTRLB_SBMODE_Pos)) |
                                                     ((uint32_t) serialSetup->dataWidth |
                                                      (uint32_t) serialSetup->stopBits);
            } else {
                usart_regs->USART_INT.SERCOM_CTRLA =  (usart_regs->USART_INT.SERCOM_CTRLA & ~ (SERCOM_USART_INT_CTRLA_SAMPR_Msk |
                                                       SERCOM_USART_INT_CTRLA_FORM_Msk)) |
                                                      SERCOM_USART_INT_CTRLA_FORM (0x1) |
                                                      SERCOM_USART_INT_CTRLA_SAMPR (sampleRate);
                usart_regs->USART_INT.SERCOM_CTRLB = (usart_regs->USART_INT.SERCOM_CTRLB & ~ (SERCOM_USART_INT_CTRLB_CHSIZE_Msk |
                                                      SERCOM_USART_INT_CTRLB_SBMODE_Pos |
                                                      SERCOM_USART_INT_CTRLB_PMODE_Msk)) |
                                                     (uint32_t) serialSetup->dataWidth |
                                                     (uint32_t) serialSetup->stopBits |
                                                     (uint32_t) serialSetup->parity ;
            }

            /* Wait for sync */
            while (usart_regs->USART_INT.SERCOM_SYNCBUSY);

            /* Enable the USART after the configurations */
            usart_regs->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_ENABLE_Msk;

            /* Wait for sync */
            while (usart_regs->USART_INT.SERCOM_SYNCBUSY);

            setupStatus = true;
        }
    }

    return setupStatus;
}

uint32_t SERCOM_USART_ErrorGet ( sercom_registers_t *usart_regs )
{
    uint32_t errorStatus = USART_ERROR_NONE;

    errorStatus = (uint32_t) (usart_regs->USART_INT.SERCOM_STATUS & (SERCOM_USART_INT_STATUS_PERR_Msk |
                              SERCOM_USART_INT_STATUS_FERR_Msk |
                              SERCOM_USART_INT_STATUS_BUFOVF_Msk));

    if (errorStatus != USART_ERROR_NONE) {
        SERCOM_USART_ErrorClear (usart_regs);
    }

    return errorStatus;
}


void SERCOM_USART_TransmitterEnable ( sercom_registers_t *usart_regs )
{
    usart_regs->USART_INT.SERCOM_CTRLB |= SERCOM_USART_INT_CTRLB_TXEN_Msk;

    /* Wait for sync */
    while (usart_regs->USART_INT.SERCOM_SYNCBUSY);
}

void SERCOM_USART_TransmitterDisable ( sercom_registers_t *usart_regs )
{
    usart_regs->USART_INT.SERCOM_CTRLB &= ~SERCOM_USART_INT_CTRLB_TXEN_Msk;

    /* Wait for sync */
    while (usart_regs->USART_INT.SERCOM_SYNCBUSY);
}

void SERCOM_USART_ReceiverEnable ( sercom_registers_t *usart_regs )
{
    usart_regs->USART_INT.SERCOM_CTRLB |= SERCOM_USART_INT_CTRLB_RXEN_Msk;

    /* Wait for sync */
    while (usart_regs->USART_INT.SERCOM_SYNCBUSY);
}

void SERCOM_USART_ReceiverDisable ( sercom_registers_t *usart_regs )
{
    usart_regs->USART_INT.SERCOM_CTRLB &= ~SERCOM_USART_INT_CTRLB_RXEN_Msk;

    /* Wait for sync */
    while (usart_regs->USART_INT.SERCOM_SYNCBUSY);
}

void static SERCOM_USART_ISR_ERR_Handler ( sercom_registers_t *usart_regs )
{
    uint32_t errorStatus = USART_ERROR_NONE;

    errorStatus = (usart_regs->USART_INT.SERCOM_STATUS &
                   (SERCOM_USART_INT_STATUS_PERR_Msk |
                    SERCOM_USART_INT_STATUS_FERR_Msk |
                    SERCOM_USART_INT_STATUS_BUFOVF_Msk));

    if (errorStatus != USART_ERROR_NONE) {
        /* Clear error and receive interrupt to abort on-going transfer */
        usart_regs->USART_INT.SERCOM_INTENCLR = SERCOM_USART_INT_INTENCLR_ERROR_Msk | SERCOM_USART_INT_INTENCLR_RXC_Msk;

        /* In case of errors are not cleared by client using ErrorGet API */
        SERCOM_USART_ErrorClear (usart_regs);
    }
}

#include "rtdevice.h"

typedef struct _samd2x_uart_t {
    struct rt_serial_device *serial;
#ifdef BSP_UART_USING_TX_RB
    struct rt_ringbuffer *out_rb;
#endif
    sercom_registers_t *sercom_reg;
    IRQn_Type vector;
} SAMD2x_UART_T;

#ifdef BSP_USING_UART0
static struct rt_serial_device _serial0;

static SAMD2x_UART_T _uart0 = {
    .serial = &_serial0,
#ifdef BSP_UART_USING_TX_RB
    .out_rb = RT_NULL,
#endif
    .sercom_reg = USART0_MODULE,
    .vector = SERCOM0_IRQn,
};
#endif

#ifdef BSP_USING_UART3
static struct rt_serial_device _serial3;

static SAMD2x_UART_T _uart3 = {
    .serial = &_serial3,
#ifdef BSP_UART_USING_TX_RB
    .out_rb = RT_NULL,
#endif
    .sercom_reg = USART3_MODULE,
    .vector = SERCOM3_IRQn,
};
#endif

static rt_err_t _uart_cfg (struct rt_serial_device *serial, struct serial_configure *cfg)
{
    SAMD2x_UART_T *uart;
    USART_SERIAL_SETUP config_usart;
    sercom_registers_t *usart_regs;

    RT_ASSERT (serial != RT_NULL);
    RT_ASSERT (cfg != RT_NULL);

    config_usart.baudRate = cfg->baud_rate;
    uart = (SAMD2x_UART_T *) (serial->parent.user_data);
    usart_regs = uart->sercom_reg;

    switch (cfg->data_bits ) {
        case DATA_BITS_8:
            config_usart.dataWidth = SERCOM_USART_INT_CTRLB_CHSIZE_8_BIT;
            break;
        case DATA_BITS_5:
            config_usart.dataWidth = SERCOM_USART_INT_CTRLB_CHSIZE_5_BIT;
            break;
        case DATA_BITS_6:
            config_usart.dataWidth = SERCOM_USART_INT_CTRLB_CHSIZE_6_BIT;
            break;
        case DATA_BITS_7:
            config_usart.dataWidth = SERCOM_USART_INT_CTRLB_CHSIZE_7_BIT;
            break;
        case DATA_BITS_9:
            config_usart.dataWidth = SERCOM_USART_INT_CTRLB_CHSIZE_9_BIT;
            break;
        default:
            config_usart.dataWidth = SERCOM_USART_INT_CTRLB_CHSIZE_8_BIT;
            break;
    }

    switch (cfg->parity) {
        case PARITY_NONE:
            config_usart.parity = USART_PARITY_NONE;
            break;
        case PARITY_EVEN:
            config_usart.parity = SERCOM_USART_INT_CTRLB_PMODE_EVEN;
            break;
        case PARITY_ODD:
            config_usart.parity = SERCOM_USART_INT_CTRLB_PMODE_ODD;
            break;
        default:
            config_usart.parity = USART_PARITY_NONE;
            break;
    }

    config_usart.stopBits = SERCOM_USART_INT_CTRLB_SBMODE_1_BIT;
    if (cfg->stop_bits != SERCOM_USART_INT_CTRLB_SBMODE_1_BIT)
        config_usart.stopBits = SERCOM_USART_INT_CTRLB_SBMODE_2_BIT;

    while (SERCOM_USART_SerialSetup (usart_regs, &config_usart, SERCOM_USART_FrequencyGet (usart_regs)) != true);

    return RT_EOK;
}

static rt_err_t _uart_ctrl (struct rt_serial_device *serial, int cmd, void *arg)
{
    SAMD2x_UART_T *uart;

    RT_ASSERT (serial != RT_NULL);

    uart = (SAMD2x_UART_T *) (serial->parent.user_data);

    switch (cmd) {
        /* disable interrupt */
        case RT_DEVICE_CTRL_CLR_INT:
            /* Disable RX interrupt. */
            /* Disable the RX Complete Interrupt */
            uart->sercom_reg->USART_INT.SERCOM_INTENCLR = SERCOM_USART_INT_INTENCLR_RXC_Msk;
            uart->sercom_reg->USART_INT.SERCOM_INTENCLR = SERCOM_USART_INT_INTENSET_ERROR_Msk;
            uart->sercom_reg->USART_INT.SERCOM_CTRLB &= ~SERCOM_USART_INT_CTRLB_RXEN_Msk;
            NVIC_DisableIRQ (uart->vector);
            break;
        /* enable interrupt */
        case RT_DEVICE_CTRL_SET_INT:
            /* Enable RX interrupt. */
            /* Enable the RX Complete Interrupt */
            uart->sercom_reg->USART_INT.SERCOM_INTENSET = SERCOM_USART_INT_INTENSET_RXC_Msk;
            uart->sercom_reg->USART_INT.SERCOM_INTENSET = SERCOM_USART_INT_INTENSET_ERROR_Msk;
            uart->sercom_reg->USART_INT.SERCOM_CTRLB |= SERCOM_USART_INT_CTRLB_RXEN_Msk;
            NVIC_EnableIRQ (uart->vector);
            break;
        case RT_DEVICE_CTRL_CONFIG: {
            struct serial_configure *cfg = (struct serial_configure *)arg;
            _uart_cfg (serial, cfg);
        } break;

        default:
            return RT_ERROR;
    }

    return RT_EOK;
}

static int _uart_putc (struct rt_serial_device *serial, char c)
{
    SAMD2x_UART_T *uart;
    sercom_registers_t *usart_regs;

    RT_ASSERT (serial != RT_NULL);
    uart = (SAMD2x_UART_T *) (serial->parent.user_data);
    usart_regs = uart->sercom_reg;

#ifndef BSP_UART_USING_TX_RB
    usart_regs->USART_INT.SERCOM_DATA = c;
    while (! (usart_regs->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE_Msk));
#else
    if (rt_ringbuffer_putchar (uart->out_rb, c) == 0) {
        return 0;
    }

    /* Check if any data is pending for transmission */
    if (rt_ringbuffer_data_len (uart->out_rb) > 0) {
        /* Enable Data Register Empty interrupt */
        usart_regs->USART_INT.SERCOM_INTENSET = SERCOM_USART_INT_INTENSET_DRE_Msk;
    }
#endif

    return 1;
}

static int _uart_getc (struct rt_serial_device *serial)
{
    int ch;
    SAMD2x_UART_T *uart;
    sercom_registers_t *usart_regs;

    RT_ASSERT (serial != RT_NULL);
    uart = (SAMD2x_UART_T *) (serial->parent.user_data);
    usart_regs = uart->sercom_reg;

    /* Check if USART has new data */
    if (! (usart_regs->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_RXC_Msk)) {
        /* Return error code */
        return -1;
    }

    ch = usart_regs->USART_INT.SERCOM_DATA & 0x1FF;

    return ch;
}

static struct rt_uart_ops _uart_ops = {
    _uart_cfg,
    _uart_ctrl,
    _uart_putc,
    _uart_getc
};

static void uart_int_cb (SAMD2x_UART_T *uart_handle)
{
    sercom_registers_t *usart_regs;

    RT_ASSERT (uart_handle != RT_NULL);
    usart_regs = uart_handle->sercom_reg;

    if (usart_regs->USART_INT.SERCOM_INTENSET != 0) {
        /* Checks for receive complete empty flag */
        if ((usart_regs->USART_INT.SERCOM_INTENSET & SERCOM_USART_INT_INTENSET_RXC_Msk)
            && (usart_regs->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_RXC_Msk)) {
            rt_hw_serial_isr (uart_handle->serial, RT_SERIAL_EVENT_RX_IND);
        }

#ifdef BSP_UART_USING_TX_RB
        if ((usart_regs->USART_INT.SERCOM_INTENSET & SERCOM_USART_INT_INTENSET_DRE_Msk)
            &&  (usart_regs->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE_Msk)) {
            rt_uint8_t c;
            if (rt_ringbuffer_getchar (uart_handle->out_rb, &c) == 1) {
                usart_regs->USART_INT.SERCOM_DATA = c;
            } else {
                /* Nothing to transmit. Disable the data register empty interrupt. */
                usart_regs->USART_INT.SERCOM_INTENCLR = SERCOM_USART_INT_INTENCLR_DRE_Msk;
            }
        }
#endif

        /* Checks for error flag */
        if ((usart_regs->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_ERROR_Msk) == SERCOM_USART_INT_INTFLAG_ERROR_Msk) {
            SERCOM_USART_ISR_ERR_Handler (usart_regs);
        }
    }
}

#ifdef BSP_USING_UART0
void SERCOM0_Handler ( void )
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_int_cb (&_uart0);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#ifdef BSP_USING_UART3
void SERCOM3_Handler ( void )
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_int_cb (&_uart3);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

int rt_hw_usart_init (void)
{
    int result = RT_EOK;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    SERCOM_USART_ClockInit();
    SERCOM_USART_PortInit();

    config.bufsz = UART_TX_RB_SIZE;

#ifdef BSP_USING_UART0
    SERCOM_USART_NvicInit (_uart0.vector);
    SERCOM_USART_Initialize (_uart0.sercom_reg);

#ifdef BSP_UART_USING_TX_RB
#ifdef RT_USING_HEAP
    _uart0.out_rb = rt_ringbuffer_create (config.bufsz);
#else
    rt_ringbuffer_init (_uart0.out_rb, uart0_tx_buffer, config.bufsz);
#endif
#endif

    _serial0.config = config;
    _serial0.ops = &_uart_ops;

    if (rt_hw_serial_register (&_serial0, "uart0", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX, &_uart0) == RT_EOK) {
        rt_kprintf ("%s register success\n", "uart0");
    } else {
        result = -RT_ERROR;
    }
#endif

#ifdef BSP_USING_UART3
    SERCOM_USART_NvicInit (_uart3.vector);
    SERCOM_USART_Initialize (_uart3.sercom_reg);

#ifdef BSP_UART_USING_TX_RB
#ifdef RT_USING_HEAP
    _uart3.out_rb = rt_ringbuffer_create (config.bufsz);
#else
    rt_ringbuffer_init (_uart3.out_rb, uart3_tx_buffer, config.bufsz);
#endif
#endif

    _serial3.config = config;
    _serial3.ops = &_uart_ops;

    if (rt_hw_serial_register (&_serial3, "uart3", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX, &_uart3) == RT_EOK) {
        rt_kprintf ("%s register success\n", "uart3");
    } else {
        result = -RT_ERROR;
    }
#endif

    return result;
}

void rt_hw_console_output (const char *str)
{
    rt_size_t i, len = rt_strlen (str);

    if (rt_strcmp (RT_CONSOLE_DEVICE_NAME, "uart3") == 0) {
        /* Disable Data Register Empty interrupt */
        USART3_MODULE->USART_INT.SERCOM_INTENCLR = SERCOM_USART_INT_INTENSET_DRE_Msk;

        for (i = 0; i < len; i++) {
            USART3_MODULE->USART_INT.SERCOM_DATA = str[i];
            while (! (USART3_MODULE->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE_Msk));
        }
    }
#ifdef BSP_USING_UART0
    else if (rt_strcmp (RT_CONSOLE_DEVICE_NAME, "uart0") == 0) {
        /* Disable Data Register Empty interrupt */
        USART0_MODULE->USART_INT.SERCOM_INTENCLR = SERCOM_USART_INT_INTENSET_DRE_Msk;

        for (i = 0; i < len; i++) {
            USART0_MODULE->USART_INT.SERCOM_DATA = str[i];
            while (! (USART0_MODULE->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE_Msk));
        }
    }
#endif
}
#endif
