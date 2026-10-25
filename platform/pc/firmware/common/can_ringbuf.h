/* Single-producer, single-consumer ring buffer of CAN frames (sbt CAN.md, 5.3).
 * Shared by drv_fdcan (ISR -> main loop) and the PC mock, so overflow behaviour is identical. */
#ifndef CAN_RINGBUF_H
#define CAN_RINGBUF_H

#include <stdbool.h>
#include <stdint.h>
#include "hal_can.h"

#define CAN_RB_SIZE 32u   /* power of two */

typedef struct {
    hal_can_frame_t buf[CAN_RB_SIZE];
    volatile uint32_t head;   /* written by the producer only */
    volatile uint32_t tail;   /* written by the consumer only */
    uint32_t overflows;
} can_ringbuf_t;

void can_rb_init(can_ringbuf_t *rb);
bool can_rb_push(can_ringbuf_t *rb, const hal_can_frame_t *frame);
bool can_rb_pop(can_ringbuf_t *rb, hal_can_frame_t *frame);
uint32_t can_rb_count(const can_ringbuf_t *rb);

#endif
