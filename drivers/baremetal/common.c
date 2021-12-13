#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "board.h"
#include "common.h"

volatile uint8_t irq_nest = 0;

uint32_t critical_enter (void)
{
    uint32_t level = __get_PRIMASK();
    __disable_irq();
    irq_nest ++;

    return level;
}

void critical_exit (uint32_t level)
{
    __set_PRIMASK (level);

    if (--irq_nest == 0)
        __enable_irq();
}

ringbuffer_t *ringbuffer_create (uint32_t size)
{
    ringbuffer_t *rb = malloc (sizeof (ringbuffer_t));

    if (rb != NULL) {
        rb->buffer = malloc (size);
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
        free (rb->buffer);
        free (rb);
    }
}

ringbuffer_state_t ringbuffer_get_state (ringbuffer_t *rb)
{
    ringbuffer_state_t state = RINGBUFFER_OK;
    uint32_t level = critical_enter();

    if (rb->read_index == rb->write_index) {
        if (rb->read_mirror == rb->write_mirror)
            state = RINGBUFFER_EMPTY;
        else
            state = RINGBUFFER_FULL;
    }

    critical_exit (level);
    return state;
}

uint16_t ringbuffer_data_len (ringbuffer_t *rb)
{
    uint16_t len = 0;
    uint32_t level = critical_enter();

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

    critical_exit (level);

    return len;
}

/** return the size of empty space in rb */
#define ringbuffer_space_len(rb) ((rb)->buffer_size - ringbuffer_data_len(rb))

uint8_t ringbuffer_getchar (ringbuffer_t *rb, uint8_t *ch)
{
    if (rb != NULL) {
        /* ringbuffer is empty */
        if (ringbuffer_data_len (rb) == 0)
            return 0;

        uint32_t level = critical_enter();

        /* put character */
        *ch = rb->buffer[rb->read_index];

        if (rb->read_index == rb->buffer_size - 1) {
            rb->read_mirror = ~rb->read_mirror;
            rb->read_index = 0;
        } else {
            rb->read_index++;
        }

        critical_exit (level);
        return 1;
    }

    return 0;
}

uint8_t ringbuffer_putchar (ringbuffer_t *rb, uint8_t ch)
{
    if (rb != NULL) {
        /* whether has enough space */
        if (ringbuffer_space_len (rb) == 0)
            return 0;

        uint32_t level = critical_enter();

        rb->buffer[rb->write_index] = ch;

        /* flip mirror */
        if (rb->write_index == rb->buffer_size - 1) {
            rb->write_mirror = ~rb->write_mirror;
            rb->write_index = 0;
        } else {
            rb->write_index++;
        }

        critical_exit (level);
        return 1;
    }

    return 0;
}
