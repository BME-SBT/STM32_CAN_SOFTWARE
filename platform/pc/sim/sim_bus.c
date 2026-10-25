#include <stdio.h>
#include <string.h>
#include "sim_bus.h"

#define ID_SPACE 0x800u   /* 11-bit identifiers */

static sim_port_t ports[SIM_MAX_PORTS];
static uint8_t n_ports;
static hal_can_frame_t last_frame[ID_SPACE];
static uint32_t last_time[ID_SPACE];
static uint32_t count[ID_SPACE];

void sim_bus_reset(void)
{
    memset(ports, 0, sizeof ports);
    memset(count, 0, sizeof count);
    n_ports = 0;
}

sim_port_t *sim_bus_attach(const char *name, uint8_t tx_depth)
{
    sim_port_t *p;
    if (n_ports >= SIM_MAX_PORTS) return NULL;
    p = &ports[n_ports++];
    memset(p, 0, sizeof *p);
    snprintf(p->name, sizeof p->name, "%s", name);
    if (tx_depth < 1) tx_depth = 1;
    if (tx_depth > SIM_PORT_TXQ_MAX) tx_depth = SIM_PORT_TXQ_MAX;
    p->tx_depth = tx_depth;
    can_rb_init(&p->rx);
    return p;
}

uint8_t sim_bus_port_count(void) { return n_ports; }
sim_port_t *sim_bus_port(uint8_t index) { return index < n_ports ? &ports[index] : NULL; }

int sim_port_send(sim_port_t *p, const hal_can_frame_t *f)
{
    if (p->bus_off) return -2;
    if (p->tx_count >= p->tx_depth) return -1;
    p->txq[(p->tx_head + p->tx_count) % SIM_PORT_TXQ_MAX] = *f;
    p->tx_count++;
    return 0;
}

bool sim_port_recv(sim_port_t *p, hal_can_frame_t *f) { return can_rb_pop(&p->rx, f); }

void sim_port_set_filter(sim_port_t *p, const uint16_t *ids, uint8_t n)
{
    if (n > SIM_PORT_FILTERS) n = SIM_PORT_FILTERS;
    p->n_accept = n;
    for (uint8_t i = 0; i < n; i++) p->accept[i] = ids[i];
}

bool sim_port_accepts(const sim_port_t *p, uint16_t id)
{
    for (uint8_t i = 0; i < p->n_accept; i++)
        if (p->accept[i] == id) return true;
    return false;
}

void sim_port_flush_tx(sim_port_t *p)
{
    p->tx_count = 0;
    p->tx_head = 0;
}

void sim_bus_deliver(uint32_t now)
{
    for (;;) {
        int best = -1;
        sim_port_t *p;
        hal_can_frame_t f;

        /* Arbitration: lowest ID at the head of any TX FIFO wins. */
        for (uint8_t i = 0; i < n_ports; i++) {
            if (ports[i].tx_count == 0 || ports[i].bus_off) continue;
            if (best < 0 || ports[i].txq[ports[i].tx_head].id < ports[best].txq[ports[best].tx_head].id)
                best = i;
        }
        if (best < 0) break;

        p = &ports[best];
        f = p->txq[p->tx_head];
        p->tx_head = (uint8_t)((p->tx_head + 1) % SIM_PORT_TXQ_MAX);
        p->tx_count--;
        f.timestamp_ms = now;   /* the RX "ISR" timestamp */

        if (f.id < ID_SPACE) {
            last_frame[f.id] = f;
            last_time[f.id] = now;
            count[f.id]++;
        }
        for (uint8_t i = 0; i < n_ports; i++) {   /* no self-reception */
            if ((int)i == best || ports[i].bus_off || !sim_port_accepts(&ports[i], f.id)) continue;
            (void)can_rb_push(&ports[i].rx, &f);
        }
    }
}

bool sim_bus_last(uint16_t id, hal_can_frame_t *f, uint32_t *t)
{
    if (id >= ID_SPACE || count[id] == 0) return false;
    if (f) *f = last_frame[id];
    if (t) *t = last_time[id];
    return true;
}

uint32_t sim_bus_count(uint16_t id) { return id < ID_SPACE ? count[id] : 0; }
