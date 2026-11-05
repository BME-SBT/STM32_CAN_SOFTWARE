#include <stddef.h>
#include <string.h>
#include "can_proto.h"
#include "fault_mgr.h"
#include "hal_can.h"
#include "hal_time.h"

typedef struct {
    const can_msg_def_t *def;
    can_msg_any_t value;
    uint32_t last_ms;
    can_rx_state_t state;
    uint8_t recovery_count;
    can_rx_cb_t rx_cb;
    can_timeout_cb_t timeout_cb;
} rx_slot_t;

typedef struct {
    const can_msg_def_t *def;
    uint16_t period_ms;
    can_provider_fn provider;
    uint32_t next_due_ms;
} tx_slot_t;

static rx_slot_t rx_slots[CAN_PROTO_MAX_RX];
static uint8_t n_rx;
static tx_slot_t tx_slots[CAN_PROTO_MAX_TX];
static uint8_t n_tx;
static can_proto_stats_t stats;
static bool bus_off;
static uint32_t bus_off_since_ms;
static uint8_t failed_restarts;

static rx_slot_t *find_rx(uint16_t id)
{
    for (uint8_t i = 0; i < n_rx; i++)
        if (rx_slots[i].def->id == id) return &rx_slots[i];
    return NULL;
}

static tx_slot_t *find_tx(uint16_t id)
{
    for (uint8_t i = 0; i < n_tx; i++)
        if (tx_slots[i].def->id == id) return &tx_slots[i];
    return NULL;
}

static void on_error_state(hal_can_state_t state)
{
    (void)state;   /* the state itself is polled in can_proto_process() */
    stats.error_state_changes++;
}

void can_proto_init(const can_proto_cfg_t *cfg)
{
    uint16_t accept[CAN_PROTO_MAX_RX];
    uint32_t now = hal_time_tick_ms();

    memset(rx_slots, 0, sizeof rx_slots);
    memset(tx_slots, 0, sizeof tx_slots);
    memset(&stats, 0, sizeof stats);
    n_rx = 0;
    n_tx = 0;
    bus_off = false;
    bus_off_since_ms = 0;
    failed_restarts = 0;

    for (uint8_t i = 0; i < cfg->n_rx && n_rx < CAN_PROTO_MAX_RX; i++) {
        const can_msg_def_t *def = can_msg_def(cfg->rx_ids[i]);
        if (def == NULL) continue;
        rx_slots[n_rx].def = def;
        rx_slots[n_rx].state = CAN_RX_NO_DATA;
        accept[n_rx] = def->id;
        n_rx++;
    }
    for (uint8_t i = 0; i < cfg->n_tx && n_tx < CAN_PROTO_MAX_TX; i++) {
        const can_msg_def_t *def = can_msg_def(cfg->tx[i].id);
        if (def == NULL) continue;
        tx_slots[n_tx].def = def;
        tx_slots[n_tx].period_ms = cfg->tx[i].period_ms ? cfg->tx[i].period_ms : def->period_ms;
        tx_slots[n_tx].provider = cfg->tx[i].provider;
        tx_slots[n_tx].next_due_ms = now;
        n_tx++;
    }
    hal_can_init(accept, n_rx);
    hal_can_set_error_cb(on_error_state);
}

static void handle_frame(const hal_can_frame_t *f)
{
    rx_slot_t *s = find_rx(f->id);
    can_msg_any_t tmp;

    if (s == NULL) {
        stats.rx_unknown++;
        return;
    }
    if (f->dlc != s->def->dlc) {
        stats.rx_dlc_errors++;
        return;
    }
    memset(&tmp, 0, sizeof tmp);
    if (!s->def->unpack(f->data, &tmp)) {
        stats.rx_range_errors++;
        return;
    }
    stats.rx_frames++;
    s->value = tmp;
    s->last_ms = f->timestamp_ms;
    switch (s->state) {
    case CAN_RX_TIMEOUT:
        s->state = CAN_RX_RECOVERY;
        s->recovery_count = 1;
        break;
    case CAN_RX_RECOVERY:
        if (++s->recovery_count >= CAN_PROTO_RECOVERY_FRAMES) s->state = CAN_RX_OK;
        break;
    default:
        s->state = CAN_RX_OK;
        break;
    }
    if (s->rx_cb) s->rx_cb(f->id);
}

static void handle_bus_state(uint32_t now)
{
    if (hal_can_get_state() != HAL_CAN_BUS_OFF) {
        bus_off = false;
        return;
    }
    if (!bus_off) {
        bus_off = true;
        bus_off_since_ms = now;
        stats.busoff_events++;
        return;
    }
    if ((uint32_t)(now - bus_off_since_ms) < CAN_PROTO_BUSOFF_BACKOFF_MS) return;

    if (hal_can_restart() == 0) {
        bus_off = false;
        failed_restarts = 0;
        fault_clear(F_CAN_BUSOFF);
    } else {
        stats.restart_failures++;
        bus_off_since_ms = now;
        if (failed_restarts < 255u) failed_restarts++;
        if (failed_restarts >= CAN_PROTO_MAX_RESTARTS) fault_set(F_CAN_BUSOFF);
    }
}

static void check_timeouts(uint32_t now)
{
    for (uint8_t i = 0; i < n_rx; i++) {
        rx_slot_t *s = &rx_slots[i];
        if (s->state != CAN_RX_OK && s->state != CAN_RX_RECOVERY) continue;
        if ((uint32_t)(now - s->last_ms) > s->def->timeout_ms) {
            s->state = CAN_RX_TIMEOUT;
            if (s->timeout_cb) s->timeout_cb(s->def->id);
        }
    }
}

static can_status_t send_msg(const can_msg_def_t *def, const void *msg)
{
    hal_can_frame_t f;
    memset(&f, 0, sizeof f);
    f.id = def->id;
    f.dlc = def->dlc;
    def->pack(msg, f.data);
    if (hal_can_send(&f) != 0) {
        stats.tx_overruns++;   /* skipped; the next period sends fresh data */
        return CAN_ERR_TX_FULL;
    }
    stats.tx_frames++;
    return CAN_OK;
}

static void periodic_tx(uint32_t now)
{
    for (uint8_t i = 0; i < n_tx; i++) {
        tx_slot_t *s = &tx_slots[i];
        can_msg_any_t m;
        if ((int32_t)(now - s->next_due_ms) < 0) continue;
        s->next_due_ms += s->period_ms;
        if ((int32_t)(now - s->next_due_ms) >= 0) s->next_due_ms = now + s->period_ms;   /* fell behind */
        if (s->provider == NULL) continue;
        memset(&m, 0, sizeof m);
        if (s->provider(&m)) (void)send_msg(s->def, &m);
    }
}

void can_proto_process(void)
{
    uint32_t now = hal_time_tick_ms();
    hal_can_frame_t f;

    while (hal_can_receive(&f) == 1) handle_frame(&f);
    handle_bus_state(now);
    check_timeouts(now);
    periodic_tx(now);
}

can_status_t can_proto_get(uint16_t id, void *out, uint32_t *age_ms)
{
    rx_slot_t *s = find_rx(id);
    if (s == NULL || out == NULL) return CAN_ERR_PARAM;
    if (age_ms) *age_ms = (s->state == CAN_RX_NO_DATA) ? 0u : (uint32_t)(hal_time_tick_ms() - s->last_ms);
    if (s->state == CAN_RX_NO_DATA) return CAN_ERR_NO_DATA;
    if (s->state != CAN_RX_OK) return CAN_ERR_TIMEOUT;
    memcpy(out, &s->value, s->def->size);
    return CAN_OK;
}

can_status_t can_proto_send(uint16_t id, const void *msg)
{
    const can_msg_def_t *def = can_msg_def(id);
    if (def == NULL || msg == NULL) return CAN_ERR_PARAM;
    return send_msg(def, msg);
}

void can_proto_set_provider(uint16_t id, can_provider_fn fn)
{
    tx_slot_t *s = find_tx(id);
    if (s) s->provider = fn;
}

void can_proto_register_timeout_cb(uint16_t id, can_timeout_cb_t cb)
{
    rx_slot_t *s = find_rx(id);
    if (s) s->timeout_cb = cb;
}

void can_proto_register_rx_cb(uint16_t id, can_rx_cb_t cb)
{
    rx_slot_t *s = find_rx(id);
    if (s) s->rx_cb = cb;
}

void can_proto_get_stats(can_proto_stats_t *out) { *out = stats; }

can_rx_state_t can_proto_rx_state(uint16_t id)
{
    rx_slot_t *s = find_rx(id);
    return s ? s->state : CAN_RX_UNUSED;
}
