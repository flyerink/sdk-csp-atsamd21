#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "rtthread.h"
#include "drv_common.h"
#include "drv_ringbuffer.h"

#ifdef RT_USING_HEAP
ringbuffer_t *ringbuffer_create (rt_uint32_t size)
{
    ringbuffer_t *rb = rt_malloc (sizeof (ringbuffer_t));

    if (rb != NULL) {
        rb->buffer = rt_malloc (size);
        rb->buffer_size = size;

        /* initialize read and write index */
        rb->read_index = 0;
        rb->write_index = 0;
        rb->read_mirror = 0;
        rb->write_mirror = 0;
        return rb;
    } else
        return 0;
}

void ringbuffer_release (ringbuffer_t *rb)
{
    if (rb != NULL) {
        rt_free (rb->buffer);
        rt_free (rb);
    }
}
#endif

ringbuffer_t *ringbuffer_init (ringbuffer_t *rb, rt_uint8_t *buffer, rt_uint32_t size)
{
    if (rb != NULL) {
        rb->buffer = buffer;
        rb->buffer_size = size;

        /* initialize read and write index */
        rb->read_index = 0;
        rb->write_index = 0;
        rb->read_mirror = 0;
        rb->write_mirror = 0;
        return rb;
    } else
        return RT_NULL;
}

ringbuffer_state_t ringbuffer_get_state (ringbuffer_t *rb)
{
    ringbuffer_state_t state = RINGBUFFER_OK;
    rt_enter_critical();

    if (rb->read_index == rb->write_index) {
        if (rb->read_mirror == rb->write_mirror)
            state = RINGBUFFER_EMPTY;
        else
            state = RINGBUFFER_FULL;
    }

    rt_exit_critical ();
    return state;
}

rt_uint16_t ringbuffer_data_len (ringbuffer_t *rb)
{
    rt_uint16_t len = 0;
    rt_enter_critical();

    switch (ringbuffer_get_state (rb)) {
        case RINGBUFFER_EMPTY:
            len = 0;
            break;
        case RINGBUFFER_FULL:
            len = rb->buffer_size;
            break;
        case RINGBUFFER_OK:
        default:
            if (rb->write_index > rb->read_index)
                len = rb->write_index - rb->read_index;
            else
                len = rb->buffer_size - (rb->read_index - rb->write_index);
            break;
    };

    rt_exit_critical ();

    return len;
}

/** return the size of empty space in rb */
#define ringbuffer_space_len(rb) ((rb)->buffer_size - ringbuffer_data_len(rb))

rt_uint8_t ringbuffer_getchar (ringbuffer_t *rb, rt_uint8_t *ch)
{
    if (rb != NULL) {
        /* ringbuffer is empty */
        if (ringbuffer_data_len (rb) == 0)
            return 0;

        rt_enter_critical();

        /* put character */
        *ch = rb->buffer[rb->read_index];

        if (rb->read_index == rb->buffer_size - 1) {
            rb->read_mirror = ~rb->read_mirror;
            rb->read_index = 0;
        } else {
            rb->read_index++;
        }

        rt_exit_critical ();
        return 1;
    }

    return 0;
}

rt_uint8_t ringbuffer_putchar (ringbuffer_t *rb, rt_uint8_t ch)
{
    if (rb != NULL) {
        /* whether has enough space */
        if (ringbuffer_space_len (rb) == 0)
            return 0;

        rt_enter_critical();

        rb->buffer[rb->write_index] = ch;

        /* flip mirror */
        if (rb->write_index == rb->buffer_size - 1) {
            rb->write_mirror = ~rb->write_mirror;
            rb->write_index = 0;
        } else {
            rb->write_index++;
        }

        rt_exit_critical ();
        return 1;
    }

    return 0;
}
