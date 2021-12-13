#ifndef __DRV_SAMD21_SPI_
#define __DRV_SAMD21_SPI_

#include <stdbool.h>
#include <stdint.h>

#include "board.h"

// *****************************************************************************
/* SPI Clock Phase
  Summary:
    Identifies SPI Clock Phase Options
  Description:
    This enumeration identifies possible SPI Clock Phase Options.
  Remarks:
    None.  */
typedef enum {
    /* Input data is sampled on clock trailing edge and changed on
       leading edge */
    SPI_CLOCK_PHASE_TRAILING_EDGE = SERCOM_SPIM_CTRLA_CPHA_TRAILING_EDGE,
    /* Input data is sampled on clock leading edge and changed on
       trailing edge */
    SPI_CLOCK_PHASE_LEADING_EDGE = SERCOM_SPIM_CTRLA_CPHA_LEADING_EDGE,
    /* Force the compiler to reserve 32-bit space for each enum value */
    SPI_CLOCK_PHASE_INVALID = 0xFFFFFFFFU
} SPI_CLOCK_PHASE;

// *****************************************************************************
/* SPI Clock Polarity
  Summary:
    Identifies SPI Clock Polarity Options
  Description:
    This enumeration identifies possible SPI Clock Polarity Options.
  Remarks:
    None.  */
typedef enum {
    /* The inactive state value of clock is logic level zero */
    SPI_CLOCK_POLARITY_IDLE_LOW = SERCOM_SPIM_CTRLA_CPOL_IDLE_LOW,
    /* The inactive state value of clock is logic level one */
    SPI_CLOCK_POLARITY_IDLE_HIGH = SERCOM_SPIM_CTRLA_CPOL_IDLE_HIGH,
    /* Force the compiler to reserve 32-bit space for each enum value */
    SPI_CLOCK_POLARITY_INVALID = 0xFFFFFFFFU
} SPI_CLOCK_POLARITY;

// *****************************************************************************
/* SPI Data Bits
  Summary:
    Identifies SPI bits per transfer
  Description:
    This enumeration identifies number of bits per SPI transfer.
  Remarks:
    For 9 bit mode, data should be right aligned in the 16 bit
    memory location.  */
typedef enum {
    /* 8 bits per transfer */
    SPI_DATA_BITS_8 = SERCOM_SPIM_CTRLB_CHSIZE_8_BIT,
    /* 9 bits per transfer */
    SPI_DATA_BITS_9 = SERCOM_SPIM_CTRLB_CHSIZE_9_BIT,
    /* Force the compiler to reserve 32-bit space for each enum value */
    SPI_DATA_BITS_INVALID = 0xFFFFFFFFU
} SPI_DATA_BITS;

// *****************************************************************************
/* SPI Transfer Setup Parameters
  Summary:
    Identifies the setup parameters which can be changed dynamically.
  Description
    This structure identifies the possible setup parameters for SPI
    which can be changed dynamically if needed.
  Remarks:
    None.  */
typedef struct {
    uint32_t            clockFrequency; /* Baud Rate or clock frequency */
    SPI_CLOCK_PHASE     clockPhase;     /* Clock Phase */
    SPI_CLOCK_POLARITY  clockPolarity;  /* Clock Polarity */
    SPI_DATA_BITS       dataBits;       /* Number of bits per transfer */
} SPI_TRANSFER_SETUP;

/* samd21 spi dirver class */
struct samd21_spi_t {
    void *regs;
    uint32_t clock;
};

extern struct samd21_spi_t samd21_spi1;

uint8_t SERCOM_SPI_TransferDataSPI (struct samd21_spi_t *sam_spim, uint8_t data);
uint32_t SERCOM_SPI_Transfer (struct samd21_spi_t *sam_spim, uint8_t *txBuffer, uint16_t txLength);

uint32_t SERCOM_SPI_Receive (struct samd21_spi_t *sam_spim,
                             uint8_t *rxBuffer, uint16_t rxMaxLength);

uint32_t SERCOM_SPI_TransferReceive (struct samd21_spi_t *sam_spim,
                                     uint8_t *txBuffer, uint8_t *rxBuffer,
                                     uint16_t txLength, uint16_t rxMaxLength);

void hw_spi_init (void);

#endif
