/* Simulation world: owns time, bus, simulated nodes and the DUT, and advances everything in a fixed
 * order, 1 ms per step:
 *   1. time += 1 ms   2. bus delivers the frames queued in the previous step
 *   3. simulated nodes step   4. DUT runs one scheduler pass   5. state changes are reported as events */
#ifndef SIM_WORLD_H
#define SIM_WORLD_H

#include <stdbool.h>
#include <stdint.h>
#include "hal_can.h"
#include "sim_bus.h"
#include "sim_nodes.h"

/* The firmware under test. init() must fully reset the stack and call sched_init(). */
typedef struct {
    const char *name;
    void (*init)(void);
    bool is_control_node;   /* enables the vehicle-state events */
} sim_dut_t;

extern const sim_dut_t sim_dut_control_node;

typedef struct {
    uint32_t now;          /* hal_time value */
    const sim_dut_t *dut;
    sim_port_t *manual;    /* frames typed in by the user / test */
    sim_motor_t motor;
    sim_bms_t bms;
    double pedal_a_pct, pedal_b_pct;
    bool enable_switch;
} sim_world_t;

/* dut NULL = Control node. start_ms lets tests start near the 32-bit clock wrap. */
void sim_world_reset(const sim_dut_t *dut, uint32_t start_ms);
void sim_world_step(void);
void sim_world_run(uint32_t ms);
sim_world_t *sim_world(void);

/* Physical inputs of the Control node. Pedal in %, converted to raw ADC like the real sensor. */
void sim_world_set_pedal(double a_pct, double b_pct);
void sim_world_set_enable(bool on);

/* Queues a frame on the "manual" port; it crosses the bus in the next step. */
int sim_world_send_manual(const hal_can_frame_t *f);

/* Events: one line per notable change (vehicle state, faults, RX state, CAN state, motor state). */
typedef void (*sim_event_sink_t)(uint32_t t, const char *text);
void sim_set_event_sink(sim_event_sink_t sink);
void sim_event(const char *fmt, ...);

#endif
