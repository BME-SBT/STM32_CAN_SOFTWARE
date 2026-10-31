/* Virtual CAN bus. Ports attach to the bus; sim_bus_deliver() sends the queued frames, lowest ID
 * first (arbitration), to every other port whose filter accepts them, and remembers the last frame
 * of each ID so the console can show and reuse it. */
#ifndef SIM_BUS_H
#define SIM_BUS_H

#include <stdbool.h>
#include <stdint.h>
#include "can_ringbuf.h"
#include "hal_can.h"

#define SIM_MAX_PORTS     6
#define SIM_PORT_TXQ_MAX  16
#define SIM_PORT_FILTERS  16

typedef struct sim_port {
    char name[12];
    bool bus_off;                        /* no TX, no RX */
    hal_can_frame_t txq[SIM_PORT_TXQ_MAX];
    uint8_t tx_head, tx_count, tx_depth;
    can_ringbuf_t rx;
    uint16_t accept[SIM_PORT_FILTERS];   /* IDs this port receives */
    uint8_t n_accept;
} sim_port_t;

void sim_bus_reset(void);
sim_port_t *sim_bus_attach(const char *name, uint8_t tx_depth);
uint8_t sim_bus_port_count(void);
sim_port_t *sim_bus_port(uint8_t index);

int sim_port_send(sim_port_t *p, const hal_can_frame_t *f);   /* 0 ok, -1 TX FIFO full, -2 bus-off */
bool sim_port_recv(sim_port_t *p, hal_can_frame_t *f);
void sim_port_set_filter(sim_port_t *p, const uint16_t *ids, uint8_t n);   /* n = 0: receives nothing */
bool sim_port_accepts(const sim_port_t *p, uint16_t id);
void sim_port_flush_tx(sim_port_t *p);

/* Deliver everything queued since the last call. Called once per simulated ms. */
void sim_bus_deliver(uint32_t now);

/* Last frame with this ID that crossed the bus, and when; false if none yet. f and t may be NULL. */
bool sim_bus_last(uint16_t id, hal_can_frame_t *f, uint32_t *t);
uint32_t sim_bus_count(uint16_t id);   /* frames with this ID delivered since reset */

#endif
