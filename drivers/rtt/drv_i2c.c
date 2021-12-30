#include <rtthread.h>
#include "board.h"

#if !defined(RT_USING_I2C_BITOPS) && defined(RT_USING_I2C)

#define DBG_TAG "i2c"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#if !defined(BSP_USING_I2C0) && !defined(BSP_USING_I2C1) && !defined(BSP_USING_I2C2) && !defined(BSP_USING_I2C3)
#error "Please define at least one BSP_USING_I2Cx"
/* this driver can be disabled at menuconfig → RT-Thread Components → Device Drivers */
#endif

#if defined(BSP_USING_I2C0) && defined(BSP_USING_UART0)
#error "Please define only 1 of BSP_USING_I2C0 and BSP_USING_UART0"
#endif

#if defined(BSP_USING_I2C0) && defined(BSP_USING_SPI0)
#error "Please define only 1 of BSP_USING_I2C0 and BSP_USING_SPI0"
#endif

#include "rtdevice.h"

/* SERCOM I2C baud value */
#define SERCOM_I2CM_BAUD_VALUE         (0x32U)      // 100K

#ifdef BSP_USING_I2C0
#define I2C0_MODULE       SERCOM0_REGS
#define I2C0_PAD1         PINMUX_PA05D_SERCOM0_PAD1     // PINMUX_PA05D_SERCOM0_PAD1 PINMUX_PA09C_SERCOM0_PAD1
#define I2C0_PAD0         PINMUX_PA04D_SERCOM0_PAD0     // PINMUX_PA04D_SERCOM0_PAD0 PINMUX_PA08C_SERCOM0_PAD0
#endif

#ifdef BSP_USING_I2C2
#define I2C2_MODULE       SERCOM2_REGS
#define I2C2_PAD1         PINMUX_PA09D_SERCOM2_PAD1     // PINMUX_PA09D_SERCOM2_PAD1
#define I2C2_PAD0         PINMUX_PA08D_SERCOM2_PAD0     // PINMUX_PA08D_SERCOM2_PAD0
#endif


// *****************************************************************************
/* SERCOM I2C Transfer type
  Summary:    List of transfer direction.

  Description:This enum defines the I2C transfer direction.
  Remarks:    None.
*/
enum {
    I2C_TRANSFER_WRITE = 0,
    I2C_TRANSFER_READ = 1,
};

typedef enum {
    SERCOM_I2C_ERROR_NONE, /* No error has occurred. */
    SERCOM_I2C_ERROR_NAK,  /* A bus transaction was NAK'ed */
    SERCOM_I2C_ERROR_BUS,  /* A bus error has occurred. */
} SERCOM_I2C_ERROR;

typedef enum {
    SERCOM_I2C_STATE_ERROR = -1, /* SERCOM PLib Task Error State */
    SERCOM_I2C_STATE_IDLE,       /* SERCOM PLib Task Idle State */
    SERCOM_I2C_STATE_ADDR_SEND,  /* SERCOM PLib Task Address Send State */
    SERCOM_I2C_REINITIATE_TRANSFER,
    SERCOM_I2C_STATE_TRANSFER_READ,    /* SERCOM PLib Task Read Transfer State */
    SERCOM_I2C_STATE_TRANSFER_WRITE,   /* SERCOM PLib Task Write Transfer State */
    SERCOM_I2C_STATE_TRANSFER_ADDR_HS, /* SERCOM PLib Task High Speed Slave Address Send State */
    SERCOM_I2C_STATE_TRANSFER_DONE,    /* SERCOM PLib Task Transfer Done State */
} SERCOM_I2C_STATE;

/* samd2x config class */
typedef struct samd2x_i2c_config {
    sercom_i2cm_registers_t *i2cm_regs;
    rt_base_t i2cm_irqn;    /* Must between SERCOM0_IRQn to SERCOM5_IRQn */
    uint32_t clock;         /* I2C Clock frequency in Hz */
    const char *bus_name;
} samd2x_i2c_config_t;

// *****************************************************************************
rt_inline void SERCOM_I2CM_ClockInit (void)
{
#ifdef BSP_USING_I2C0
    /* Selection of the Generator and write Lock for SERCOM2_CORE */
    GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_ID (GCLK_CLKCTRL_ID_SERCOM0_CORE_Val) |
                              GCLK_CLKCTRL_GEN (GCLK_CLKCTRL_GEN_GCLK0_Val) |
                              GCLK_CLKCTRL_CLKEN_Msk;

    /* Configure the APBC Bridge Clocks */
    PM_REGS->PM_APBCMASK |= PM_APBCMASK_SERCOM (1 << 0);
#endif

#ifdef BSP_USING_I2C2
    /* Selection of the Generator and write Lock for SERCOM2_CORE */
    GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_ID (GCLK_CLKCTRL_ID_SERCOM2_CORE_Val) |
                              GCLK_CLKCTRL_GEN (GCLK_CLKCTRL_GEN_GCLK0_Val) |
                              GCLK_CLKCTRL_CLKEN_Msk;

    /* Configure the APBC Bridge Clocks */
    PM_REGS->PM_APBCMASK |= PM_APBCMASK_SERCOM (1 << 2);
#endif

    while ( GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk ) {
        /* Wait for synchronization */
    }
}

rt_inline void SERCOM_I2CM_PortInit (void)
{
#ifdef BSP_USING_I2C0
    PORT_PinMUX_Config (I2C0_PAD0);
    PORT_PinMUX_Config (I2C0_PAD1);
#endif

#ifdef BSP_USING_I2C2
    PORT_PinMUX_Config (I2C2_PAD0);
    PORT_PinMUX_Config (I2C2_PAD1);
#endif
}

static bool SERCOM_I2C_CalculateBaudValue (uint32_t srcClkFreq, uint32_t i2cClkSpeed, uint32_t *baudVal)
{
    uint32_t baudValue;

    /* Reference clock frequency must be atleast two times the baud rate */
    if (srcClkFreq < (2 * i2cClkSpeed)) {
        return false;
    }

    if (i2cClkSpeed <= 1000000) {
        /* Standard, FM and FM+ baud calculation */
        baudValue = (uint32_t) (((((float)srcClkFreq) / i2cClkSpeed) - ((((float)srcClkFreq) * (200 / 1000000000.0)) + 10)));
    } else {
        return false;
    }
    if (i2cClkSpeed <= 400000) {
        /* For I2C clock speed upto 400 kHz, the value of BAUD<7:0> determines both SCL_L and SCL_H with SCL_L = SCL_H */
        if (baudValue > (0xFF * 2)) {
            /* Set baud rate to the minimum possible value */
            baudValue = 0xFF;
        } else if (baudValue <= 1) {
            /* Baud value cannot be 0. Set baud rate to maximum possible value */
            baudValue = 1;
        } else {
            baudValue /= 2;
        }
    } else {
        /* To maintain the ratio of SCL_L:SCL_H to 2:1, the max value of BAUD_LOW<15:8>:BAUD<7:0> can be 0xFF:0x7F. Hence BAUD_LOW + BAUD can not exceed 255+127 = 382 */
        if (baudValue >= 382) {
            /* Set baud rate to the minimum possible value while maintaining SCL_L:SCL_H to 2:1 */
            baudValue = (0xFF << 8) | (0x7F);
        } else if (baudValue <= 3) {
            /* Baud value cannot be 0. Set baud rate to maximum possible value while maintaining SCL_L:SCL_H to 2:1 */
            baudValue = (2 << 8) | 1;
        } else {
            /* For Fm+ mode, I2C SCL_L:SCL_H to 2:1 */
            baudValue = ((((baudValue * 2) / 3) << 8) | (baudValue / 3));
        }
    }

    *baudVal = baudValue;
    return true;
}

void SERCOM_I2C_Initialize (samd2x_i2c_config_t *i2c_config)
{
    sercom_i2cm_registers_t *i2cm_regs = i2c_config->i2cm_regs;
    uint32_t baudValue;
    uint32_t i2cSpeedMode = 0;

    /* Reset the module */
    i2cm_regs->SERCOM_CTRLA = SERCOM_I2CM_CTRLA_SWRST_Msk;

    /* Wait for synchronization */
    while (i2cm_regs->SERCOM_SYNCBUSY)
        ;

    /* Enable smart mode enable */
    i2cm_regs->SERCOM_CTRLB = SERCOM_I2CM_CTRLB_SMEN_Msk;

    /* Wait for synchronization */
    while (i2cm_regs->SERCOM_SYNCBUSY)
        ;

    if (SERCOM_I2C_CalculateBaudValue (BSP_CLOCK_SYSTEM_FREQ_MHZ * 1000000UL,
                                       i2c_config->clock, &baudValue) == false) {
        i2c_config->clock = 100000;
        baudValue = SERCOM_I2CM_BAUD_VALUE;
    }

    if (i2c_config->clock > 400000) {
        i2cSpeedMode = 1;
    }

    /* Baud rate - Master Baud Rate*/
    i2cm_regs->SERCOM_BAUD = baudValue;

    /* Set Operation Mode (Master), SDA Hold time, run in stand by and i2c master enable */
    i2cm_regs->SERCOM_CTRLA = SERCOM_I2CM_CTRLA_MODE_I2C_MASTER
                              | SERCOM_I2CM_CTRLA_SDAHOLD_75NS
                              | SERCOM_I2CM_CTRLA_SPEED (i2cSpeedMode)
                              | SERCOM_I2CM_CTRLA_SCLSM (0)
                              | SERCOM_I2CM_CTRLA_ENABLE_Msk;

    /* Wait for synchronization */
    while (i2cm_regs->SERCOM_SYNCBUSY)
        ;

    /* Initial Bus State: IDLE */
    i2cm_regs->SERCOM_STATUS = SERCOM_I2CM_STATUS_BUSSTATE (0x01);

    /* Wait for synchronization */
    while (i2cm_regs->SERCOM_SYNCBUSY)
        ;

    if (i2c_config->i2cm_irqn >= SERCOM0_IRQn) {
        /* Enable all Interrupts */
        i2cm_regs->SERCOM_INTENSET = SERCOM_I2CM_INTENSET_Msk;

        NVIC_SetPriority (i2c_config->i2cm_irqn, 3);
        NVIC_EnableIRQ (i2c_config->i2cm_irqn);
    }
}

bool SERCOM_I2C_TransferSetup (samd2x_i2c_config_t *i2c_config, uint32_t i2cClkFreq, uint32_t srcClkFreq)
{
    sercom_i2cm_registers_t *i2cm_regs = i2c_config->i2cm_regs;
    uint32_t baudValue;
    uint32_t i2cSpeedMode = 0;

    if (srcClkFreq == 0) {
        srcClkFreq = 48000000UL;
    }

    if (SERCOM_I2C_CalculateBaudValue (srcClkFreq, i2cClkFreq, &baudValue) == false) {
        return false;
    }

    if (i2cClkFreq > 400000) {
        i2cSpeedMode = 1;
    }

    /* Disable the I2C before changing the I2C clock speed */
    i2cm_regs->SERCOM_CTRLA &= ~SERCOM_I2CM_CTRLA_ENABLE_Msk;
    /* Wait for synchronization */
    while (i2cm_regs->SERCOM_SYNCBUSY)
        ;

    /* Baud rate - Master Baud Rate*/
    i2cm_regs->SERCOM_BAUD = baudValue;
    i2cm_regs->SERCOM_CTRLA = ((i2cm_regs->SERCOM_CTRLA & ~SERCOM_I2CM_CTRLA_SPEED_Msk) | (SERCOM_I2CM_CTRLA_SPEED (i2cSpeedMode)));

    /* Re-enable the I2C module */
    i2cm_regs->SERCOM_CTRLA |= SERCOM_I2CM_CTRLA_ENABLE_Msk;
    /* Wait for synchronization */
    while (i2cm_regs->SERCOM_SYNCBUSY)
        ;

    /* Since the I2C module was disabled, re-initialize the bus state to IDLE */
    i2cm_regs->SERCOM_STATUS = SERCOM_I2CM_STATUS_BUSSTATE (0x01);
    /* Wait for synchronization */
    while (i2cm_regs->SERCOM_SYNCBUSY)
        ;

    return true;
}

static void SERCOM_I2C_InitiateTransfer (samd2x_i2c_config_t *i2c_config, uint16_t address, bool dir)
{
    sercom_i2cm_registers_t *i2cm_regs = i2c_config->i2cm_regs;

    /* Clear all flags */
    i2cm_regs->SERCOM_INTFLAG = SERCOM_I2CM_INTFLAG_Msk;

    /* Smart mode enabled with SCLSM = 0, - ACK is set to send while receiving the data */
    i2cm_regs->SERCOM_CTRLB &= ~SERCOM_I2CM_CTRLB_ACKACT_Msk;
    /* Wait for synchronization */
    while (i2cm_regs->SERCOM_SYNCBUSY)
        ;

    /* Set I2C device address */
    i2cm_regs->SERCOM_ADDR = (address << 1) | dir;
    /* Wait for synchronization */
    while (i2cm_regs->SERCOM_SYNCBUSY)
        ;
}

static bool SERCOM_I2CM_Xfer (samd2x_i2c_config_t *i2c_config, uint16_t address,
                              uint8_t *buff, uint32_t length, uint32_t flags)
{
    sercom_i2cm_registers_t *i2cm_regs = i2c_config->i2cm_regs;
    volatile SERCOM_I2C_STATE state = SERCOM_I2C_STATE_IDLE;
    //volatile SERCOM_I2C_ERROR error = SERCOM_I2C_ERROR_NONE;
    uint32_t xfer_count = 0;
    uint32_t timeout = 0;

    if ((flags & RT_I2C_RD) == RT_I2C_RD) {
        /* <xxxx-xxxR> <read-data> <P> */
        /* Next state will be to read data */
        SERCOM_I2C_InitiateTransfer (i2c_config, address, I2C_TRANSFER_READ);
        state = SERCOM_I2C_STATE_TRANSFER_READ;
    } else {
        /* <xxxx-xxxW> <write-data> <P> */
        /* Next state will be to write data */
        SERCOM_I2C_InitiateTransfer (i2c_config, address, I2C_TRANSFER_WRITE);
        state = SERCOM_I2C_STATE_TRANSFER_WRITE;
    }

    while (1) {
        /* Check I2CM Interrupt Flags */
        if ((i2cm_regs->SERCOM_INTFLAG & SERCOM_I2CM_INTFLAG_Msk) == 0) {
            if (++timeout > 2000000)
                return false;
            else
                continue;
        }

        timeout = 0;
        /* Checks if the arbitration lost in multi-master scenario */
        if ((i2cm_regs->SERCOM_STATUS & SERCOM_I2CM_STATUS_ARBLOST_Msk) == SERCOM_I2CM_STATUS_ARBLOST_Msk) {
            state = SERCOM_I2C_STATE_ERROR;
            //error = SERCOM_I2C_ERROR_BUS;
        } else if ((i2cm_regs->SERCOM_STATUS & SERCOM_I2CM_STATUS_BUSERR_Msk) == SERCOM_I2CM_STATUS_BUSERR_Msk) {
            state = SERCOM_I2C_STATE_ERROR;
            //error = SERCOM_I2C_ERROR_BUS;
        }
        /* Checks slave acknowledge for address or data */
        else if ((i2cm_regs->SERCOM_STATUS & SERCOM_I2CM_STATUS_RXNACK_Msk) == SERCOM_I2CM_STATUS_RXNACK_Msk) {
            state = SERCOM_I2C_STATE_ERROR;
            //error = SERCOM_I2C_ERROR_NAK;
        } else {
            switch (state) {
                case SERCOM_I2C_STATE_IDLE:
                case SERCOM_I2C_STATE_TRANSFER_DONE:
                    goto checkstate;

                case SERCOM_I2C_STATE_TRANSFER_WRITE:
                    if (xfer_count == length) {
                        i2cm_regs->SERCOM_CTRLB |= SERCOM_I2CM_CTRLB_CMD (3);
                        /* Wait for synchronization */
                        while (i2cm_regs->SERCOM_SYNCBUSY)
                            ;

                        state = SERCOM_I2C_STATE_TRANSFER_DONE;
                    } else {
                        /* Write next byte */
                        i2cm_regs->SERCOM_DATA = buff[xfer_count++];
                    }
                    break;

                case SERCOM_I2C_STATE_TRANSFER_READ:
                    if (xfer_count == (length - 1)) {
                        /* Set NACK and send stop condition to the slave from master */
                        i2cm_regs->SERCOM_CTRLB |= SERCOM_I2CM_CTRLB_ACKACT_Msk | SERCOM_I2CM_CTRLB_CMD (3);
                        /* Wait for synchronization */
                        while (i2cm_regs->SERCOM_SYNCBUSY)
                            ;

                        state = SERCOM_I2C_STATE_TRANSFER_DONE;
                    }
                    /* Read the received data */
                    buff[xfer_count++] = i2cm_regs->SERCOM_DATA;
                    break;

                default:
                    break;
            }
        }

checkstate:
        /* Error Status */
        if (state == SERCOM_I2C_STATE_ERROR) {
            /* Reset the PLib objects and Interrupts */
            state = SERCOM_I2C_STATE_IDLE;

            /* Generate STOP condition */
            i2cm_regs->SERCOM_CTRLB |= SERCOM_I2CM_CTRLB_CMD (3);

            /* Wait for synchronization */
            while (i2cm_regs->SERCOM_SYNCBUSY)
                ;

            i2cm_regs->SERCOM_INTFLAG = SERCOM_I2CM_INTFLAG_Msk;

            return false;
        }
        /* Transfer Complete */
        else if (state == SERCOM_I2C_STATE_TRANSFER_DONE) {
            /* Reset the PLib objects and interrupts */
            state = SERCOM_I2C_STATE_IDLE;
            //error = SERCOM_I2C_ERROR_NONE;

            i2cm_regs->SERCOM_INTFLAG = SERCOM_I2CM_INTFLAG_Msk;

            /* Wait for the NAK and STOP bit to be transmitted out and I2C state machine to rest in IDLE state */
            while ((i2cm_regs->SERCOM_STATUS & SERCOM_I2CM_STATUS_BUSSTATE_Msk) != SERCOM_I2CM_STATUS_BUSSTATE (0x01))
                ;

            return true;
        }
    }

    return false;
}

#ifdef BSP_USING_I2C0
#define I2C0_BUS_CONFIG                                     \
{                                                           \
    .i2cm_regs = (sercom_i2cm_registers_t *)SERCOM0_REGS,   \
    .i2cm_irqn = 0,                                         \
    .clock = BSP_I2C0_CLOCK,                                \
    .bus_name = "i2c0",                                     \
}
#endif

#ifdef BSP_USING_I2C2
#define I2C2_BUS_CONFIG                                     \
{                                                           \
    .i2cm_regs = (sercom_i2cm_registers_t *)SERCOM2_REGS,   \
    .i2cm_irqn = 0,                                         \
    .clock = BSP_I2C2_CLOCK,                                \
    .bus_name = "i2c2",                                     \
}
#endif

static const samd2x_i2c_config_t samd21_i2c_config[] = {
#ifdef BSP_USING_I2C0
    I2C0_BUS_CONFIG,
#endif
#ifdef BSP_USING_I2C2
    I2C2_BUS_CONFIG,
#endif
};

static rt_size_t samd21_i2cm_xfer (struct rt_i2c_bus_device *bus,
                                   struct rt_i2c_msg msgs[],
                                   rt_uint32_t num)
{
    samd2x_i2c_config_t *i2c_config = (samd2x_i2c_config_t *)bus->priv;
    struct rt_i2c_msg *msg;
    rt_uint32_t i;

    for (i = 0; i < num; i++) {
        msg = &msgs[i];
        if (!SERCOM_I2CM_Xfer (i2c_config, msg->addr, msg->buf, msg->len, msg->flags))
            return -RT_ERROR;
    }

    return i;
}

static const struct rt_i2c_bus_device_ops samd21_i2c_ops_default = {
    .master_xfer     = samd21_i2cm_xfer,
    .slave_xfer      = RT_NULL,
    .i2c_bus_control = RT_NULL,
};

/* samd2x i2c driver class */
struct samd2x_i2c {
    struct rt_i2c_bus_device_ops ops;
    struct rt_i2c_bus_device i2c2_bus;
};

struct samd2x_i2c rt_i2c_dev[sizeof (samd21_i2c_config) / sizeof (samd2x_i2c_config_t)];

/* I2C initialization function */
int rt_hw_i2c_init (void)
{
    rt_size_t obj_num = sizeof (samd21_i2c_config) / sizeof (samd2x_i2c_config_t);
    rt_err_t result;

    SERCOM_I2CM_ClockInit();
    SERCOM_I2CM_PortInit();

    for (int i = 0; i < obj_num; i++) {
        rt_i2c_dev[i].ops = samd21_i2c_ops_default;
        rt_i2c_dev[i].i2c2_bus.priv = (void *)&samd21_i2c_config[i];
        rt_i2c_dev[i].i2c2_bus.ops = &rt_i2c_dev[i].ops;

        SERCOM_I2C_Initialize ((samd2x_i2c_config_t *)&samd21_i2c_config[i]);

        result = rt_i2c_bus_device_register (&rt_i2c_dev[i].i2c2_bus, samd21_i2c_config[i].bus_name);
        RT_ASSERT (result == RT_EOK);

        LOG_I ("SAMD21 %s init done", samd21_i2c_config[i].bus_name);
    }

    return result;
}

INIT_BOARD_EXPORT (rt_hw_i2c_init);

#endif
