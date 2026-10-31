#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "can_proto.h"
#include "control_node.h"
#include "fault_mgr.h"
#include "hal_mock.h"
#include "sched.h"
#include "sim_world.h"
#include "vehicle_fsm.h"

const sim_dut_t sim_dut_control_node = { "Control node", control_node_init, true };

static sim_world_t w;
static sim_event_sink_t event_sink;

/* Snapshot for change detection */
static struct {
    vehicle_state_t state;
    uint32_t faults;
    can_rx_state_t rx[8];
    hal_can_state_t can;
} snap;

static const char *const rx_state_names[] = { "UNUSED", "NO_DATA", "OK", "TIMEOUT", "RECOVERY" };
static const char *const can_state_names[] = { "ERROR_ACTIVE", "ERROR_PASSIVE", "BUS_OFF" };

void sim_set_event_sink(sim_event_sink_t sink) { event_sink = sink; }

void sim_event(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    if (event_sink == NULL) return;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    event_sink(w.now, buf);
}

static void take_snapshot(void)
{
    snap.state = vehicle_fsm_state();
    snap.faults = fault_active_mask();
    for (uint8_t i = 0; i < can_msg_def_count() && i < 8; i++) snap.rx[i] = can_proto_rx_state(can_msg_def_at(i)->id);
    snap.can = hal_can_get_state();
}

static void detect_events(void)
{
    uint32_t f = fault_active_mask(), changed = f ^ snap.faults;

    if (w.dut->is_control_node && vehicle_fsm_state() != snap.state) {
        vehicle_state_t s = vehicle_fsm_state();
        if (s == VEH_SAFE_STATE)
            sim_event("vehicle state %s -> %s (latched %s)", vehicle_state_name(snap.state), vehicle_state_name(s),
                      fault_name(vehicle_fsm_latched_fault()));
        else
            sim_event("vehicle state %s -> %s", vehicle_state_name(snap.state), vehicle_state_name(s));
    }
    for (int c = 0; c < F_COUNT; c++) {
        if (!(changed & (1u << c))) continue;
        sim_event("fault %s %s (%s)", (f & (1u << c)) ? "SET  " : "CLEAR", fault_name((fault_code_t)c),
                  fault_severity((fault_code_t)c) == FAULT_SEV_CRITICAL ? "critical" : "warning");
    }
    for (uint8_t i = 0; i < can_msg_def_count() && i < 8; i++) {
        const can_msg_def_t *d = can_msg_def_at(i);
        can_rx_state_t st = can_proto_rx_state(d->id);
        if (st != snap.rx[i]) sim_event("rx %s %s -> %s", d->name, rx_state_names[snap.rx[i]], rx_state_names[st]);
    }
    if (hal_can_get_state() != snap.can)
        sim_event("CAN controller %s -> %s", can_state_names[snap.can], can_state_names[hal_can_get_state()]);
    take_snapshot();
}

void sim_world_reset(const sim_dut_t *dut, uint32_t start_ms)
{
    memset(&w, 0, sizeof w);
    w.dut = dut ? dut : &sim_dut_control_node;
    w.now = start_ms;
    hal_time_mock_set(start_ms);

    sim_bus_reset();
    sim_override_clear_all();
    hal_io_mock_reset();
    hal_can_mock_reset();   /* attaches the DUT's port "dut" */
    sim_motor_init(&w.motor, sim_bus_attach("motor", 8), start_ms);
    sim_bms_init(&w.bms, sim_bus_attach("bms", 8), start_ms);
    w.manual = sim_bus_attach("manual", SIM_PORT_TXQ_MAX);   /* receives nothing */

    sim_world_set_pedal(0.0, 0.0);
    sim_world_set_enable(false);

    w.dut->init();
    take_snapshot();
}

void sim_world_step(void)
{
    w.now++;
    hal_time_mock_set(w.now);
    sim_bus_deliver(w.now);
    sim_motor_step(&w.motor, w.now);
    sim_bms_step(&w.bms, w.now, w.motor.current_a);
    sched_run_once();
    detect_events();
}

void sim_world_run(uint32_t ms)
{
    while (ms--) sim_world_step();
}

sim_world_t *sim_world(void) { return &w; }

static uint16_t pct_to_raw(double pct)
{
    /* Same calibration as the firmware (control_config.h): 0 % = 400, 100 % = 3700. */
    double raw = 400.0 + pct * 33.0;
    if (raw < 0.0) raw = 0.0;
    if (raw > 4095.0) raw = 4095.0;
    return (uint16_t)(raw + 0.5);
}

void sim_world_set_pedal(double a_pct, double b_pct)
{
    w.pedal_a_pct = a_pct;
    w.pedal_b_pct = b_pct;
    hal_adc_mock_set(HAL_ADC_PEDAL_A, pct_to_raw(a_pct));
    hal_adc_mock_set(HAL_ADC_PEDAL_B, pct_to_raw(b_pct));
}

void sim_world_set_enable(bool on)
{
    w.enable_switch = on;
    hal_gpio_mock_set(HAL_GPIO_ENABLE_SWITCH, on);
}

int sim_world_send_manual(const hal_can_frame_t *f) { return sim_port_send(w.manual, f); }
