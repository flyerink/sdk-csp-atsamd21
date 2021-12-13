#ifndef _DRV_RINGBUFFER_H
#define _DRV_RINGBUFFER_H

#include <stdint.h>
#include <stdlib.h>

#include "rtthread.h"

typedef enum {
    RINGBUFFER_OK,
    RINGBUFFER_EMPTY,
    RINGBUFFER_FULL
} ringbuffer_state_t;

typedef struct {
    rt_uint8_t *buffer;
    rt_uint32_t buffer_size;
    volatile rt_uint16_t write_mirror  : 1;
    volatile rt_uint16_t write_index   : 15;
    volatile rt_uint16_t read_mirror   : 1;
    volatile rt_uint16_t read_index    : 15;
} ringbuffer_t;

#ifdef RT_USING_HEAP
ringbuffer_t *ringbuffer_create (rt_uint32_t size);
void ringbuffer_release (ringbuffer_t *rb);
#endif

ringbuffer_t *ringbuffer_init (ringbuffer_t *rb, rt_uint8_t *buffer, rt_uint32_t size);

ringbuffer_state_t ringbuffer_get_state (ringbuffer_t *rb);
rt_uint16_t ringbuffer_data_len (ringbuffer_t *rb);
rt_uint8_t ringbuffer_getchar (ringbuffer_t *rb, rt_uint8_t *ch);
rt_uint8_t ringbuffer_putchar (ringbuffer_t *rb, rt_uint8_t ch);

#endif
