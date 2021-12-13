#ifndef __COMMON_H
#define __COMMON_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/* Error code definitions */
#define RT_EOK                          0               /**< There is no error */
#define RT_ERROR                        1               /**< A generic error happens */
#define RT_ETIMEOUT                     2               /**< Timed out */
#define RT_EFULL                        3               /**< The resource is full */
#define RT_EEMPTY                       4               /**< The resource is empty */
#define RT_ENOMEM                       5               /**< No memory */
#define RT_ENOSYS                       6               /**< No system */
#define RT_EBUSY                        7               /**< Busy */
#define RT_EIO                          8               /**< IO error */
#define RT_EINTR                        9               /**< Interrupted system call */
#define RT_EINVAL                       10              /**< Invalid argument */

#define PINMUX_UNUSED   0xFFFFFFFF

#define ENABLE_INT()    __set_PRIMASK(0)    /* 使能全局中断 */
#define DISABLE_INT()   __set_PRIMASK(1)    /* 禁止全局中断 */

typedef enum {
    RINGBUFFER_OK,
    RINGBUFFER_EMPTY,
    RINGBUFFER_FULL
} ringbuffer_state_t;

typedef struct {
    uint8_t *buffer;
    uint16_t buffer_size;
    volatile uint16_t write_mirror  : 1;
    volatile uint16_t write_index   : 15;
    volatile uint16_t read_mirror   : 1;
    volatile uint16_t read_index    : 15;
} ringbuffer_t;

uint32_t critical_enter (void);
void critical_exit (uint32_t level);

ringbuffer_t *ringbuffer_create (uint32_t size);
void ringbuffer_release (ringbuffer_t *rb);

ringbuffer_state_t ringbuffer_get_state (ringbuffer_t *rb);
uint16_t ringbuffer_data_len (ringbuffer_t *rb);

uint8_t ringbuffer_getchar (ringbuffer_t *rb, uint8_t *ch);
uint8_t ringbuffer_putchar (ringbuffer_t *rb, uint8_t ch);

#endif
