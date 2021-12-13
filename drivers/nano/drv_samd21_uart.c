#include <rtthread.h>
#include "board.h"

#ifdef BSP_USING_UART0
#define BSP_USING_SERCOM0_USART
#endif

#ifdef BSP_USING_UART3
#define BSP_USING_SERCOM3_USART
#endif

#if !defined(BSP_USING_SERCOM0_USART) && !defined(BSP_USING_SERCOM1_USART) && !defined(BSP_USING_SERCOM2_USART) && !defined(BSP_USING_SERCOM3_USART)
#error "Please define at least one BSP_USING_SERCOMx_USART"
/* this driver can be disabled at menuconfig → RT-Thread Components → Device Drivers */
#endif

/* SERCOM USART baud value for 115200 Hz baud rate */
#define SERCOM_USART_INIT_BAUD_VALUE            (63019U)

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

#ifdef BSP_USING_SERCOM0_USART
#define USART0_MODULE       SERCOM0_REGS
#define USART0_IRQ          SERCOM0_IRQn
#define USART0_TXPO         SERCOM_USART_INT_CTRLA_TXPO (0x1UL)
#define USART0_RXPO         SERCOM_USART_INT_CTRLA_RXPO (0x3UL)
#define USART0_PAD3         PINMUX_PA07D_SERCOM0_PAD3   // PINMUX_PA07D_SERCOM0_PAD3 PINMUX_PA11C_SERCOM0_PAD3
#define USART0_PAD2         PINMUX_PA06D_SERCOM0_PAD2   // PINMUX_PA06D_SERCOM0_PAD2 PINMUX_PA10C_SERCOM0_PAD2
#define USART0_PAD1         PINMUX_UNUSED               // PINMUX_PA05D_SERCOM0_PAD1 PINMUX_PA09C_SERCOM0_PAD1
#define USART0_PAD0         PINMUX_UNUSED               // PINMUX_PA04D_SERCOM0_PAD0 PINMUX_PA08C_SERCOM0_PAD0

#ifndef RT_USING_HEAP
uint8_t uart0_tx_buffer[RT_CONSOLEBUF_SIZE];
uint8_t uart0_rx_buffer[RT_CONSOLEBUF_SIZE];
ringbuffer_t uart0_tx_rb;
ringbuffer_t uart0_rx_rb;
#endif

#endif

#ifdef BSP_USING_SERCOM3_USART
#define USART3_MODULE       SERCOM3_REGS
#define USART3_IRQ          SERCOM3_IRQn
#define USART3_TXPO         SERCOM_USART_INT_CTRLA_TXPO (0x0UL)
#define USART3_RXPO         SERCOM_USART_INT_CTRLA_RXPO (0x1UL)
#define USART3_PAD3         PINMUX_UNUSED               // PINMUX_PA19D_SERCOM3_PAD3 PINMUX_PA25C_SERCOM3_PAD3
#define USART3_PAD2         PINMUX_UNUSED               // PINMUX_PA18D_SERCOM3_PAD2 PINMUX_PA24C_SERCOM3_PAD2
#define USART3_PAD1         PINMUX_PA23C_SERCOM3_PAD1   // PINMUX_PA17D_SERCOM3_PAD1 PINMUX_PA23C_SERCOM3_PAD1
#define USART3_PAD0         PINMUX_PA22C_SERCOM3_PAD0   // PINMUX_PA16D_SERCOM3_PAD0 PINMUX_PA22C_SERCOM3_PAD0

#ifndef RT_USING_HEAP
rt_uint8_t uart3_tx_buffer[RT_CONSOLEBUF_SIZE];
rt_uint8_t uart3_rx_buffer[RT_CONSOLEBUF_SIZE];
ringbuffer_t uart3_tx_rb;
ringbuffer_t uart3_rx_rb;
#endif

#endif

typedef struct {
    uint32_t baudRate;
    uint32_t parity;
    uint32_t dataWidth;
    uint32_t stopBits;
} USART_SERIAL_SETUP;

/**
 * \brief Retrieve ordinal number of the given sercom hardware instance
 */
static uint8_t sercom_get_hardware_index (const void *const hw)
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
#ifdef BSP_USING_SERCOM0_USART
    /* Selection of the Generator and write Lock for SERCOM4_CORE */
    GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_ID (GCLK_CLKCTRL_ID_SERCOM0_CORE_Val) |
                              GCLK_CLKCTRL_GEN (GCLK_CLKCTRL_GEN_GCLK0_Val)  |
                              GCLK_CLKCTRL_CLKEN_Msk;

    /* Configure the APBC Bridge Clocks */
    PM_REGS->PM_APBCMASK |= PM_APBCMASK_SERCOM (1 << 0);
#endif

#ifdef BSP_USING_SERCOM3_USART
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
#ifdef BSP_USING_SERCOM0_USART
    /************************** GROUP 1 Initialization *************************/
    PORT_PinMUX_Config (USART0_PAD0);
    PORT_PinMUX_Config (USART0_PAD1);
    PORT_PinMUX_Config (USART0_PAD2);
    PORT_PinMUX_Config (USART0_PAD3);
#endif

#ifdef BSP_USING_SERCOM3_USART
    /************************** GROUP 1 Initialization *************************/
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
    uint8_t  u8dummyData = 0;

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

#ifdef BSP_USING_SERCOM0_USART
    if (usart_regs == USART0_MODULE) {
        /* UART0 use 2 & 3 */
        usart_regs->USART_INT.SERCOM_CTRLA = SERCOM_USART_INT_CTRLA_MODE_USART_INT_CLK |
                                             USART0_RXPO | USART0_TXPO |
                                             SERCOM_USART_INT_CTRLA_DORD_Msk | SERCOM_USART_INT_CTRLA_IBON_Msk |
                                             SERCOM_USART_INT_CTRLA_FORM (0x0UL) | SERCOM_USART_INT_CTRLA_SAMPR (0UL);
    } else
#elif defined BSP_USING_SERCOM3_USART
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

    /* Enable Error interrupt */
    usart_regs->USART_INT.SERCOM_INTENSET = SERCOM_USART_INT_INTENSET_ERROR_Msk;

    /* Enable Receive Complete interrupt */
    usart_regs->USART_INT.SERCOM_INTENSET = SERCOM_USART_INT_INTENSET_RXC_Msk;
}

uint32_t SERCOM_USART_FrequencyGet ( sercom_registers_t *usart_regs )
{
    uint8_t index = sercom_get_hardware_index (usart_regs);
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

#ifdef BSP_USING_SERCOM0_USART
static SAMD2x_UART_T uart0 = {
    .in_rb = NULL,
    .out_rb = NULL,
    .sercom_reg = USART0_MODULE,
    .vector = USART0_IRQ,
};
#endif

#ifdef BSP_USING_SERCOM3_USART
static SAMD2x_UART_T uart3 = {
    .in_rb = NULL,
    .out_rb = NULL,
    .sercom_reg = USART3_MODULE,
    .vector = USART3_IRQ,
};
#endif

static void uart_int_cb (SAMD2x_UART_T *uart)
{
    uint8_t c;
    sercom_registers_t *usart_regs = uart->sercom_reg;

    if (usart_regs->USART_INT.SERCOM_INTENSET != 0) {
        /* Checks for receive complete empty flag */
        if ((usart_regs->USART_INT.SERCOM_INTENSET & SERCOM_USART_INT_INTENSET_RXC_Msk)
                && (usart_regs->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_RXC_Msk)) {
            if (ringbuffer_putchar (uart->in_rb, usart_regs->USART_INT.SERCOM_DATA) == 0) {
                /* Trigger the receiving completion callback */
                if (uart->rx_callback != NULL)
                    uart->rx_callback ((void *)uart, uart->context);
            }
        }

        if ((usart_regs->USART_INT.SERCOM_INTENSET & SERCOM_USART_INT_INTENSET_DRE_Msk)
                &&  (usart_regs->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE_Msk)) {
            if (ringbuffer_getchar (uart->out_rb, &c) == 1) {
                usart_regs->USART_INT.SERCOM_DATA = c;
            } else {
                /* Nothing to transmit. Disable the data register empty interrupt. */
                usart_regs->USART_INT.SERCOM_INTENCLR = SERCOM_USART_INT_INTENCLR_DRE_Msk;
            }
        }

        /* Checks for error flag */
        if ((usart_regs->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_ERROR_Msk) == SERCOM_USART_INT_INTFLAG_ERROR_Msk) {
            SERCOM_USART_ISR_ERR_Handler (usart_regs);
        }
    }
}

#ifdef BSP_USING_SERCOM0_USART
void SERCOM0_Handler ( void )
{
    uart_int_cb (&uart0);
}
#endif

#ifdef BSP_USING_SERCOM3_USART
void SERCOM3_Handler ( void )
{
    uart_int_cb (&uart3);
}
#endif

void uart_set_rx_indicate (SAMD2x_UART_T *uart, uart_rx_callback callback, void *context)
{
    if (uart == NULL || callback == NULL)
        return;

    uart->rx_callback = callback;
    uart->context = context;
}

int uart_putc (SAMD2x_UART_T *uart, char c)
{
    sercom_registers_t *usart_regs = uart->sercom_reg;

    if (ringbuffer_putchar (uart->out_rb, c) == 0) {
        return 0;
    }

    /* Check if any data is pending for transmission */
    if (ringbuffer_data_len (uart->out_rb) > 0) {
        /* Enable Data Register Empty interrupt */
        usart_regs->USART_INT.SERCOM_INTENSET = SERCOM_USART_INT_INTENSET_DRE_Msk;
    }

    return 1;
}

int uart_write (SAMD2x_UART_T *uart, uint8_t *buffer, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++) {
        while (!uart_putc (uart, buffer[i]));
    }

    return length;
}

int uart_getc (SAMD2x_UART_T *uart)
{
    uint8_t ch = 0;

    if (uart == NULL)       return -1;

    /* Check if any data is pending for transmission */
    if (ringbuffer_data_len (uart->in_rb) > 0) {
        /* Enable Data Register Empty interrupt */
        if (ringbuffer_getchar (uart->in_rb, &ch) == 1)
            return ch;
        else
            return -1;
    }

    return ch;
}

int rt_hw_usart_init (void)
{
    int result = 0;

    SERCOM_USART_ClockInit();
    SERCOM_USART_PortInit();

#ifdef BSP_USING_SERCOM0_USART
#ifdef RT_USING_HEAP
    uart0.out_rb = ringbuffer_create (RT_CONSOLEBUF_SIZE);
    uart0.in_rb = ringbuffer_create (RT_CONSOLEBUF_SIZE);
#else
    uart0.out_rb = ringbuffer_init(&uart3_tx_rb, uart3_tx_buffer, RT_CONSOLEBUF_SIZE);
    uart0.in_rb = ringbuffer_init(&uart3_rx_rb, uart3_rx_buffer, RT_CONSOLEBUF_SIZE);
#endif

    SERCOM_USART_NvicInit (uart0.vector);
    SERCOM_USART_Initialize (uart0.sercom_reg);
#endif

#ifdef BSP_USING_SERCOM3_USART
#ifdef RT_USING_HEAP
    uart3.out_rb = ringbuffer_create (RT_CONSOLEBUF_SIZE);
    uart3.in_rb = ringbuffer_create (RT_CONSOLEBUF_SIZE);
#else
    uart3.out_rb = ringbuffer_init(&uart3_tx_rb, uart3_tx_buffer, RT_CONSOLEBUF_SIZE);
    uart3.in_rb = ringbuffer_init(&uart3_rx_rb, uart3_rx_buffer, RT_CONSOLEBUF_SIZE);
#endif

    SERCOM_USART_NvicInit (uart3.vector);
    SERCOM_USART_Initialize (uart3.sercom_reg);
#endif

    return result;
}

#if defined(RT_USING_CONSOLE)

#ifdef BSP_USING_SERCOM3_USART
static SAMD2x_UART_T *console_uart = &uart3;
#endif

void print_char (char c)
{
    uart_putc (console_uart, c);
}

#ifdef RT_USING_FINSH
void rt_hw_console_output(const char *str)
{
    rt_size_t i = 0, size = 0;

    size = rt_strlen(str);
    for (i = 0; i < size; i++) {
        if (str[i] == '\n') {
            print_char('\r');
        }
        print_char((str[i]));
    }
}

int rt_hw_console_getchar(void)
{
    return uart_getc(console_uart);
}
#endif /* RT_USING_FINSH */

#endif /* RT_USING_CONSOLE */
