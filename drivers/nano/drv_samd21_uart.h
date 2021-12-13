/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-12-13     A19582       the first version
 */
#ifndef _DRV_SAMD21_UART_H_
#define _DRV_SAMD21_UART_H_

#include <stdbool.h>
#include <rtthread.h>
#include <rthw.h>

#include <sam.h>
#include <system_samd21.h>

typedef void (*uart_rx_callback)(void *dev, void *context);

typedef struct _samd2x_uart_t {
    ringbuffer_t *in_rb;
    ringbuffer_t *out_rb;
    sercom_registers_t *sercom_reg;
    IRQn_Type vector;
    uart_rx_callback rx_callback;
    void *context;
} SAMD2x_UART_T;

int uart_putc (SAMD2x_UART_T *uart, char c);
int uart_write (SAMD2x_UART_T *uart, uint8_t *buffer, uint32_t length);
int uart_getc (SAMD2x_UART_T *uart);
void uart_set_rx_indicate (SAMD2x_UART_T *uart, uart_rx_callback callback, void *context);

int rt_hw_usart_init (void);

#endif /* _DRV_SAMD21_UART_H_ */
