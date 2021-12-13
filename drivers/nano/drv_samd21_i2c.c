/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-11-18     A19582       the first version
 */

#include "board.h"

/* SERCOM I2C baud value */
#define SERCOM_I2CM_SPEED_HZ           100000UL
#define SERCOM_I2CM_BAUD_VALUE         (0xE8U)      // 100K

#if defined (BSP_USING_I2C)

#ifdef BSP_USING_SERCOM0_I2C
#define I2C0_MODULE       SERCOM0_REGS
#define I2C0_PAD1         PINMUX_PA05D_SERCOM0_PAD1     // PINMUX_PA05D_SERCOM0_PAD1 PINMUX_PA09C_SERCOM0_PAD1
#define I2C0_PAD0         PINMUX_PA04D_SERCOM0_PAD0     // PINMUX_PA04D_SERCOM0_PAD0 PINMUX_PA08C_SERCOM0_PAD0
#endif

#ifdef BSP_USING_SERCOM2_I2C
#define I2C2_MODULE       SERCOM2_REGS
#define I2C2_PAD1         PINMUX_PA09D_SERCOM2_PAD1     // PINMUX_PA09D_SERCOM2_PAD1
#define I2C2_PAD0         PINMUX_PA08D_SERCOM2_PAD0     // PINMUX_PA08D_SERCOM2_PAD0
#endif

// *****************************************************************************
static void SERCOM_I2CM_ClockInit (void)
{
#ifdef BSP_USING_SERCOM0_I2C
    /* Selection of the Generator and write Lock for SERCOM2_CORE */
    GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_ID (GCLK_CLKCTRL_ID_SERCOM0_CORE_Val) |
                              GCLK_CLKCTRL_GEN (GCLK_CLKCTRL_GEN_GCLK0_Val) |
                              GCLK_CLKCTRL_CLKEN_Msk;

    /* Configure the APBC Bridge Clocks */
    PM_REGS->PM_APBCMASK |= PM_APBCMASK_SERCOM (1 << 0);
#endif  // BSP_USING_SERCOM0_I2C

#ifdef BSP_USING_SERCOM2_I2C
    /* Selection of the Generator and write Lock for SERCOM2_CORE */
    GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_ID (GCLK_CLKCTRL_ID_SERCOM2_CORE_Val) |
                              GCLK_CLKCTRL_GEN (GCLK_CLKCTRL_GEN_GCLK0_Val) |
                              GCLK_CLKCTRL_CLKEN_Msk;

    /* Configure the APBC Bridge Clocks */
    PM_REGS->PM_APBCMASK |= PM_APBCMASK_SERCOM (1 << 2);
#endif  // BSP_USING_SERCOM2_I2C

    while ( GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk ) {
        /* Wait for synchronization */
    }
}

static void SERCOM_I2CM_PortInit (void)
{
#ifdef BSP_USING_SERCOM0_I2C
    PORT_PinMUX_Config (I2C0_PAD0);
    PORT_PinMUX_Config (I2C0_PAD1);
#endif

#ifdef BSP_USING_SERCOM2_I2C
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

void SERCOM_I2C_Initialize (samd2x_i2c_t *i2c_config)
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

    if (i2c_config->clock > 400000) {
        i2cSpeedMode = 1;
    }

    if (SERCOM_I2C_CalculateBaudValue (BSP_CLOCK_SYSTEM_FREQ_MHZ * 1000000UL,
                                       i2c_config->clock, &baudValue) == false) {
        i2c_config->clock = SERCOM_I2CM_SPEED_HZ;
        baudValue = SERCOM_I2CM_BAUD_VALUE;
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
}

bool SERCOM_I2C_TransferSetup (samd2x_i2c_t *i2c_config, uint32_t i2cClkFreq, uint32_t srcClkFreq)
{
    sercom_i2cm_registers_t *i2cm_regs = i2c_config->i2cm_regs;
    uint32_t baudValue;
    uint32_t i2cSpeedMode = 0;

    if (srcClkFreq == 0) {
        srcClkFreq = 48000000UL;
    }

    if (i2cClkFreq > 400000) {
        i2cSpeedMode = 1;
    }

    if (SERCOM_I2C_CalculateBaudValue (srcClkFreq, i2cClkFreq, &baudValue) == false) {
        return false;
    }

    i2c_config->clock = i2cClkFreq;

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

static void SERCOM_I2C_InitiateTransfer (samd2x_i2c_t *i2c_config, uint16_t address, bool dir)
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

uint32_t SERCOM_I2CM_Write (samd2x_i2c_t *i2c_config, uint16_t address,
                            uint8_t *buff, uint32_t length)
{
    sercom_i2cm_registers_t *i2cm_regs = i2c_config->i2cm_regs;
    volatile SERCOM_I2C_STATE state = SERCOM_I2C_STATE_IDLE;

    uint32_t xfer_count = 0;
    uint32_t timeout = 0;

    /* <xxxx-xxxW> <write-data> <P> */
    /* Next state will be to write data */
    SERCOM_I2C_InitiateTransfer (i2c_config, address, I2C_TRANSFER_WRITE);
    state = SERCOM_I2C_STATE_TRANSFER_WRITE;

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
        if (i2cm_regs->SERCOM_STATUS & (SERCOM_I2CM_STATUS_ARBLOST_Msk | SERCOM_I2CM_STATUS_BUSERR_Msk | SERCOM_I2CM_STATUS_RXNACK_Msk)) {
            state = SERCOM_I2C_STATE_ERROR;
        } else {
            switch (state) {
                case SERCOM_I2C_STATE_IDLE:
                case SERCOM_I2C_STATE_TRANSFER_DONE:
                    goto checkstate_write;

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

                default:
                    break;
            }
        }

checkstate_write:
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

            return 0;
        } else if (state == SERCOM_I2C_STATE_TRANSFER_DONE) { /* Transfer Complete */
            /* Reset the PLib objects and interrupts */
            state = SERCOM_I2C_STATE_IDLE;

            i2cm_regs->SERCOM_INTFLAG = SERCOM_I2CM_INTFLAG_Msk;

            /* Wait for the NAK and STOP bit to be transmitted out and I2C state machine to rest in IDLE state */
            while ((i2cm_regs->SERCOM_STATUS & SERCOM_I2CM_STATUS_BUSSTATE_Msk) != SERCOM_I2CM_STATUS_BUSSTATE (0x01))
                ;

            return xfer_count;
        }
    }
}

uint32_t SERCOM_I2CM_Read (samd2x_i2c_t *i2c_config, uint16_t address,
                           uint8_t *buff, uint32_t length)
{
    sercom_i2cm_registers_t *i2cm_regs = i2c_config->i2cm_regs;
    volatile SERCOM_I2C_STATE state = SERCOM_I2C_STATE_IDLE;

    uint32_t xfer_count = 0;
    uint32_t timeout = 0;

    /* <xxxx-xxxR> <read-data> <P> */
    /* Next state will be to read data */
    SERCOM_I2C_InitiateTransfer (i2c_config, address, I2C_TRANSFER_READ);
    state = SERCOM_I2C_STATE_TRANSFER_READ;

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
        if (i2cm_regs->SERCOM_STATUS & (SERCOM_I2CM_STATUS_ARBLOST_Msk | SERCOM_I2CM_STATUS_BUSERR_Msk | SERCOM_I2CM_STATUS_RXNACK_Msk)) {
            state = SERCOM_I2C_STATE_ERROR;
        } else {
            switch (state) {
                case SERCOM_I2C_STATE_IDLE:
                case SERCOM_I2C_STATE_TRANSFER_DONE:
                    goto checkstate_read;

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

checkstate_read:
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

            return 0;
        } else if (state == SERCOM_I2C_STATE_TRANSFER_DONE) { /* Transfer Complete */
            /* Reset the PLib objects and interrupts */
            state = SERCOM_I2C_STATE_IDLE;

            i2cm_regs->SERCOM_INTFLAG = SERCOM_I2CM_INTFLAG_Msk;

            /* Wait for the NAK and STOP bit to be transmitted out and I2C state machine to rest in IDLE state */
            while ((i2cm_regs->SERCOM_STATUS & SERCOM_I2CM_STATUS_BUSSTATE_Msk) != SERCOM_I2CM_STATUS_BUSSTATE (0x01))
                ;

            return xfer_count;
        }
    }

}

#if defined(BSP_USING_SERCOM0_I2C)
samd2x_i2c_t samd21_i2c0 = {
    .i2cm_regs = (sercom_i2cm_registers_t *)I2C0_MODULE,
    .clock = BSP_SERCOM0_I2C_CLOCK,
};
#endif  // BSP_USING_SERCOM0_I2C

#if defined(BSP_USING_SERCOM2_I2C)
samd2x_i2c_t samd21_i2c2 = {
    .i2cm_regs = (sercom_i2cm_registers_t *)I2C2_MODULE,
    .clock = BSP_SERCOM2_I2C_CLOCK,
};
#endif  // BSP_USING_SERCOM2_I2C

void hw_i2c_init (void)
{
    SERCOM_I2CM_ClockInit();
    SERCOM_I2CM_PortInit();

#if defined(BSP_USING_SERCOM2_I2C)
    SERCOM_I2C_Initialize (&samd21_i2c2);
#endif  // BSP_USING_SERCOM2_I2C
}

#endif  // BSP_USING_I2C
