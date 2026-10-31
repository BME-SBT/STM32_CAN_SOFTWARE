/* CAN protocol layer (sbt CAN.md, section 4 and 7.2).
 * Turns raw frames into typed messages and back; the application only sees fresh, plausible data. */
#ifndef CAN_PROTO_H
#define CAN_PROTO_H

#include <stdbool.h>
#include <stdint.h>
#include "can_msgs.h"

#define CAN_PROTO_MAX_RX            8u
#define CAN_PROTO_MAX_TX            8u
#define CAN_PROTO_RECOVERY_FRAMES   3u     /* RECOVERY -> OK after N valid frames (6.5) */
#define CAN_PROTO_BUSOFF_BACKOFF_MS 100u   /* wait before each restart (4.5) */
#define CAN_PROTO_MAX_RESTARTS      3u     /* failed restarts before F_CAN_BUSOFF [?] */

typedef enum {
    CAN_OK = 0,
    CAN_ERR_NO_DATA = -1,
    CAN_ERR_TIMEOUT = -2,
    CAN_ERR_TX_FULL = -3,
    CAN_ERR_PARAM = -4
} can_status_t;

/* Per-message communication state (6.5). */
typedef enum {
    CAN_RX_UNUSED = 0,   /* not configured as an RX message on this node */
    CAN_RX_NO_DATA,      /* nothing received since init */
    CAN_RX_OK,
    CAN_RX_TIMEOUT,
    CAN_RX_RECOVERY
} can_rx_state_t;

typedef bool (*can_provider_fn)(void *msg);   /* fills a periodic message; return false to skip */
typedef void (*can_rx_cb_t)(uint16_t id);
typedef void (*can_timeout_cb_t)(uint16_t id);

typedef struct {
    uint16_t id;
    uint16_t period_ms;        /* 0 = period from the message table */
    can_provider_fn provider;  /* may be NULL and set later with can_proto_set_provider() */
} can_tx_entry_t;

typedef struct {
    const uint16_t *rx_ids;
    uint8_t n_rx;
    const can_tx_entry_t *tx;
    uint8_t n_tx;
} can_proto_cfg_t;

typedef struct {
    uint32_t rx_frames;          /* valid frames stored */
    uint32_t rx_dlc_errors;      /* F_RX_DLC */
    uint32_t rx_range_errors;    /* F_RX_RANGE */
    uint32_t rx_unknown;         /* passed the HW filter but not in the RX table */
    uint32_t tx_frames;
    uint32_t tx_overruns;        /* frame skipped: TX FIFO full or bus-off */
    uint32_t busoff_events;
    uint32_t restart_failures;
    uint32_t error_state_changes;
} can_proto_stats_t;

void can_proto_init(const can_proto_cfg_t *cfg);
void can_proto_process(void);   /* every 1 ms: RX dispatch, bus state, timeout check, periodic TX */

can_status_t can_proto_get(uint16_t id, void *out, uint32_t *age_ms);
can_status_t can_proto_send(uint16_t id, const void *msg);   /* event-driven transmit */
void can_proto_set_provider(uint16_t id, can_provider_fn fn);
void can_proto_register_timeout_cb(uint16_t id, can_timeout_cb_t cb);
void can_proto_register_rx_cb(uint16_t id, can_rx_cb_t cb);
void can_proto_get_stats(can_proto_stats_t *out);
can_rx_state_t can_proto_rx_state(uint16_t id);

/* Typed getters (7.2). */
static inline can_status_t can_proto_get_motor_temps(msg_motor_temps_t *m, uint32_t *age)
{ return can_proto_get(CAN_ID_MOTOR_TEMPS, m, age); }
static inline can_status_t can_proto_get_motor_data(msg_motor_data_t *m, uint32_t *age)
{ return can_proto_get(CAN_ID_MOTOR_DATA, m, age); }
static inline can_status_t can_proto_get_bms_data(msg_bms_data_t *m, uint32_t *age)
{ return can_proto_get(CAN_ID_BMS_DATA, m, age); }
static inline can_status_t can_proto_get_bms_extra(msg_bms_extra_t *m, uint32_t *age)
{ return can_proto_get(CAN_ID_BMS_EXTRA, m, age); }

#endif
