#include "can_ringbuf.h"

void can_rb_init(can_ringbuf_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
    rb->overflows = 0;
}

bool can_rb_push(can_ringbuf_t *rb, const hal_can_frame_t *frame)
{
    uint32_t head = rb->head;
    if ((uint32_t)(head - rb->tail) >= CAN_RB_SIZE) {
        rb->overflows++;
        return false;
    }
    rb->buf[head % CAN_RB_SIZE] = *frame;
    rb->head = head + 1u;
    return true;
}

bool can_rb_pop(can_ringbuf_t *rb, hal_can_frame_t *frame)
{
    uint32_t tail = rb->tail;
    if (tail == rb->head) return false;
    *frame = rb->buf[tail % CAN_RB_SIZE];
    rb->tail = tail + 1u;
    return true;
}

uint32_t can_rb_count(const can_ringbuf_t *rb) { return rb->head - rb->tail; }
