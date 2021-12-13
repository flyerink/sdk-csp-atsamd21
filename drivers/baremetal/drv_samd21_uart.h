#ifndef __DRV_SAMD21_UART_H__
#define __DRV_SAMD21_UART_H__

#include <stdbool.h>
#include <stdint.h>

#include <sam.h>
#include <system_samd21.h>

#include "board.h"

#define BAUD_RATE_2400                  2400
#define BAUD_RATE_4800                  4800
#define BAUD_RATE_9600                  9600
#define BAUD_RATE_19200                 19200
#define BAUD_RATE_38400                 38400
#define BAUD_RATE_57600                 57600
#define BAUD_RATE_115200                115200
#define BAUD_RATE_230400                230400
#define BAUD_RATE_460800                460800
#define BAUD_RATE_921600                921600
#define BAUD_RATE_2000000               2000000
#define BAUD_RATE_3000000               3000000

#define DATA_BITS_5                     5
#define DATA_BITS_6                     6
#define DATA_BITS_7                     7
#define DATA_BITS_8                     0
#define DATA_BITS_9                     1

#define STOP_BITS_1                     0
#define STOP_BITS_2                     1

#ifdef _WIN32
#include <windows.h>
#else
#define PARITY_NONE                     0
#define PARITY_ODD                      1
#define PARITY_EVEN                     2
#endif

#define BIT_ORDER_LSB                   0
#define BIT_ORDER_MSB                   1

#define NRZ_NORMAL                      0       /* Non Return to Zero : normal mode */
#define NRZ_INVERTED                    1       /* Non Return to Zero : inverted mode */

#ifndef UART_RB_BUFSZ
#define UART_RB_BUFSZ                   64
#endif

#define USART_PARITY_NONE   0x02
#define USART_ERROR_NONE    0x00

/* Default config for serial_configure structure */
#define UART_CONFIG_DEFAULT                 \
{                                           \
    BAUD_RATE_115200, /* 115200 bits/s */   \
    PARITY_NONE,      /* No parity  */      \
    DATA_BITS_8,      /* 8 databits */      \
    STOP_BITS_1,      /* 1 stopbit */       \
}

typedef void (*uart_rx_callback)(void *dev, void *context);

// *****************************************************************************
/* USART Serial Configuration
  Summary:
    Defines the data type for the USART serial configurations.

  Description:
    This may be used to set the serial configurations for USART.

  Remarks:
    None.
*/
typedef struct {
    uint32_t baudRate;
    uint32_t parity;
    uint32_t dataWidth;
    uint32_t stopBits;
} usart_serial_setup_t;

typedef struct _samd2x_uart_t {
    sercom_registers_t *sercom_reg;
    IRQn_Type vector;
    ringbuffer_t *in_rb;
    ringbuffer_t *out_rb;
    uart_rx_callback rx_callback;
    void *context;
} SAMD2x_UART_T;

extern SAMD2x_UART_T uart3;

int hw_usart_init (void);
int uart_putc (SAMD2x_UART_T *uart, char c);
int uart_write (SAMD2x_UART_T *uart, uint8_t *buffer, uint32_t length);
int uart_getc (SAMD2x_UART_T *uart);
void uart_set_rx_indicate (SAMD2x_UART_T *uart, uart_rx_callback callback, void *context);

#endif
