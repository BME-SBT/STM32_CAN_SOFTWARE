/* hal_can - raw CAN frames (sbt CAN.md, section 3).
 * Implemented by hal_can_stm32.c on the target and by mock/hal_can_mock.c on the PC. */
#ifndef HAL_CAN_H
#define HAL_CAN_H

#include <stdint.h>

typedef struct {
    uint16_t id;            /* 11-bit standard identifier */
    uint8_t  dlc;           /* 0..8 */
    uint8_t  data[8];
    uint32_t timestamp_ms;  /* set on receive */
} hal_can_frame_t;

typedef enum {
    HAL_CAN_ERROR_ACTIVE = 0,
    HAL_CAN_ERROR_PASSIVE,
    HAL_CAN_BUS_OFF
} hal_can_state_t;

typedef void (*hal_can_error_cb_t)(hal_can_state_t state);

/* accept_ids: one hardware filter per accepted ID, everything else is rejected. */
int hal_can_init(const uint16_t *accept_ids, uint8_t n_ids);
/* Non-blocking: 0 on success, negative if the frame could not be queued (TX FIFO full, bus-off). */
int hal_can_send(const hal_can_frame_t *frame);
/* 1 if a frame was returned, 0 if the RX buffer is empty. */
int hal_can_receive(hal_can_frame_t *frame);
hal_can_state_t hal_can_get_state(void);
/* Leaves bus-off. 0 on success, negative on failure. */
int hal_can_restart(void);
void hal_can_set_error_cb(hal_can_error_cb_t cb);

#endif
