/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-11-18     A19582       the first version
 */
#ifndef _DRV_SAMD21_I2C_H_
#define _DRV_SAMD21_I2C_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include <sam.h>
#include <system_samd21.h>

#include "board.h"

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
typedef struct samd2x_i2c_t {
    sercom_i2cm_registers_t *i2cm_regs;
    uint32_t clock;                     /* I2C Clock frequency in Hz */
} samd2x_i2c_t;

//#if defined(BSP_USING_SERCOM2_I2C)
extern samd2x_i2c_t samd21_i2c2;
//#endif  // BSP_USING_SERCOM2_I2C

void hw_i2c_init (void);

void SERCOM_I2C_Initialize (samd2x_i2c_t *i2c_config);

bool SERCOM_I2C_TransferSetup (samd2x_i2c_t *i2c_config, uint32_t i2cClkFreq, uint32_t srcClkFreq);

uint32_t SERCOM_I2CM_Write (samd2x_i2c_t *i2c_config, uint16_t address,
                            uint8_t *buff, uint32_t length);
uint32_t SERCOM_I2CM_Read (samd2x_i2c_t *i2c_config, uint16_t address,
                           uint8_t *buff, uint32_t length);

#endif /* _DRV_SAMD21_I2C_H_ */
