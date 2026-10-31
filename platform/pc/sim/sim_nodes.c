#include <math.h>
#include <stdio.h>
#include <string.h>
#include "can_msgs.h"
#include "sim_nodes.h"
#include "sim_world.h"

#define DT_S 0.001   /* nodes step once per simulated ms */

/* ------------------------------------------------------------------ overrides */

typedef struct {
    bool used;
    const sim_msg_t *msg;
    int field;
    double value;
} override_t;

static override_t overrides[SIM_OVERRIDE_MAX];

bool sim_override_set(const sim_msg_t *msg, int field, double value)
{
    int free_slot = -1;
    for (int i = 0; i < SIM_OVERRIDE_MAX; i++) {
        if (overrides[i].used && overrides[i].msg == msg && overrides[i].field == field) {
            overrides[i].value = value;
            return true;
        }
        if (!overrides[i].used && free_slot < 0) free_slot = i;
    }
    if (free_slot < 0) return false;
    overrides[free_slot].used = true;
    overrides[free_slot].msg = msg;
    overrides[free_slot].field = field;
    overrides[free_slot].value = value;
    return true;
}

bool sim_override_clear(const sim_msg_t *msg, int field)
{
    bool any = false;
    for (int i = 0; i < SIM_OVERRIDE_MAX; i++) {
        if (overrides[i].used && overrides[i].msg == msg && (field < 0 || overrides[i].field == field)) {
            overrides[i].used = false;
            any = true;
        }
    }
    return any;
}

void sim_override_clear_all(void) { memset(overrides, 0, sizeof overrides); }

int sim_override_count(void)
{
    int n = 0;
    for (int i = 0; i < SIM_OVERRIDE_MAX; i++) n += overrides[i].used;
    return n;
}

bool sim_override_describe(int index, char *buf, size_t n)
{
    for (int i = 0; i < SIM_OVERRIDE_MAX; i++) {
        if (!overrides[i].used) continue;
        if (index-- == 0) {
            const sim_field_t *f = &overrides[i].msg->fields[overrides[i].field];
            char val[32];
            sim_dbc_format_value(f, overrides[i].value, val, sizeof val);
            snprintf(buf, n, "%s.%s = %s %s", overrides[i].msg->name, f->name, val, f->unit);
            return true;
        }
    }
    return false;
}

int sim_node_send(sim_port_t *port, uint16_t id, const void *msg)
{
    const can_msg_def_t *def = can_msg_def(id);
    hal_can_frame_t f;
    if (def == NULL) return -1;
    memset(&f, 0, sizeof f);
    f.id = id;
    f.dlc = def->dlc;
    def->pack(msg, f.data);
    for (int i = 0; i < SIM_OVERRIDE_MAX; i++)
        if (overrides[i].used && overrides[i].msg->id == id)
            sim_dbc_set(&overrides[i].msg->fields[overrides[i].field], f.data, overrides[i].value);
    return sim_port_send(port, &f);
}

static bool due(uint32_t now, uint32_t *next, uint32_t period)
{
    if ((int32_t)(now - *next) < 0) return false;
    *next = now + period;
    return true;
}

static int16_t to_i16(double v)
{
    if (v > 32767.0) return 32767;
    if (v < -32768.0) return -32768;
    return (int16_t)lround(v);
}

/* ------------------------------------------------------------------ motor controller */

static const char *const motor_state_names[SIM_MOTOR_STATE_COUNT] = { "INIT", "IDLE", "RUNNING", "FAULT" };

const char *sim_motor_state_name(sim_motor_state_t s) { return s < SIM_MOTOR_STATE_COUNT ? motor_state_names[s] : "?"; }

static void motor_set_state(sim_motor_t *m, sim_motor_state_t s, const char *reason)
{
    if (m->state == s) return;
    if (reason) sim_event("motor %s -> %s (%s)", motor_state_names[m->state], motor_state_names[s], reason);
    else sim_event("motor %s -> %s", motor_state_names[m->state], motor_state_names[s]);
    m->state = s;
    m->fault_reason = (s == SIM_MOTOR_FAULT) ? reason : NULL;
}

void sim_motor_init(sim_motor_t *m, sim_port_t *port, uint32_t now)
{
    static const uint16_t rx[] = { CAN_ID_CONTROL };
    memset(m, 0, sizeof *m);
    m->port = port;
    m->enabled = true;
    m->state = SIM_MOTOR_INIT;
    m->motor_temp_c = 25.0;
    m->ctrl_temp_c = 25.0;
    m->next_temps_ms = now + 1;
    m->next_data_ms = now + 1;
    sim_port_set_filter(port, rx, 1);
}

void sim_motor_step(sim_motor_t *m, uint32_t now)
{
    hal_can_frame_t f;
    const char *reason = NULL;
    double target_rpm, ambient = 25.0;

    while (sim_port_recv(m->port, &f)) {
        msg_control_t c;
        if (!m->enabled || f.id != CAN_ID_CONTROL || f.dlc != 3 || !can_unpack_control(f.data, &c)) continue;
        if (!m->have_control || c.alive != m->alive) m->alive_change_ms = now;
        m->have_control = true;
        m->last_control_ms = now;
        m->alive = c.alive;
        m->throttle_pct = c.throttle_pct;
        m->enable = c.enable;
    }
    if (!m->enabled) return;

    /* Fault causes (6.3): Control timeout or frozen alive counter, over-temperature, over-current. */
    if (m->have_control && (uint32_t)(now - m->last_control_ms) > SIM_MOTOR_CONTROL_TIMEOUT_MS)
        reason = "Control timeout";
    else if (m->have_control && (uint32_t)(now - m->alive_change_ms) > SIM_MOTOR_CONTROL_TIMEOUT_MS)
        reason = "alive counter frozen";
    else if (m->motor_temp_c > SIM_MOTOR_TEMP_FAULT_C)
        reason = "over-temperature";
    else if (fabs(m->current_a) > SIM_MOTOR_CURRENT_FAULT_A)
        reason = "over-current";

    switch (m->state) {
    case SIM_MOTOR_INIT:
        motor_set_state(m, SIM_MOTOR_IDLE, NULL);
        break;
    case SIM_MOTOR_IDLE:
        if (reason) motor_set_state(m, SIM_MOTOR_FAULT, reason);
        else if (m->have_control && m->enable == 1 && m->throttle_pct == 0)
            motor_set_state(m, SIM_MOTOR_RUNNING, "enable=1, throttle=0");
        break;
    case SIM_MOTOR_RUNNING:
        if (reason) motor_set_state(m, SIM_MOTOR_FAULT, reason);
        else if (m->enable == 0) motor_set_state(m, SIM_MOTOR_IDLE, "enable=0");
        break;
    case SIM_MOTOR_FAULT:
        if (!reason && m->have_control && m->enable == 0) motor_set_state(m, SIM_MOTOR_IDLE, "cause gone, enable=0");
        break;
    default:
        break;
    }

    /* First-order model: current follows throttle, rpm lags (tau 0.3 s), temperatures rise with I^2. */
    if (m->state == SIM_MOTOR_RUNNING) {
        m->current_a = m->throttle_pct * 1.5;     /* 150 A at 100 % */
        target_rpm = m->throttle_pct * 30.0;      /* 3000 rpm at 100 % */
    } else {
        m->current_a = 0.0;
        target_rpm = 0.0;
    }
    m->rpm += (target_rpm - m->rpm) * DT_S / 0.3;
    m->torque_nm = m->current_a * 0.12;
    m->motor_temp_c += DT_S * ((ambient + 0.002 * m->current_a * m->current_a) - m->motor_temp_c) / 30.0;
    m->ctrl_temp_c += DT_S * ((ambient + 0.0015 * m->current_a * m->current_a) - m->ctrl_temp_c) / 20.0;

    if (due(now, &m->next_temps_ms, 100)) {
        msg_motor_temps_t t = { to_i16(m->motor_temp_c * 10.0), to_i16(m->ctrl_temp_c * 10.0) };
        sim_node_send(m->port, CAN_ID_MOTOR_TEMPS, &t);
    }
    if (due(now, &m->next_data_ms, 20)) {
        msg_motor_data_t d = { to_i16(m->rpm), to_i16(m->current_a * 10.0), to_i16(m->torque_nm * 10.0) };
        sim_node_send(m->port, CAN_ID_MOTOR_DATA, &d);
    }
}

/* ------------------------------------------------------------------ BMS */

void sim_bms_init(sim_bms_t *b, sim_port_t *port, uint32_t now)
{
    memset(b, 0, sizeof *b);
    b->port = port;
    b->enabled = true;
    b->soc_pct = 80.0;
    b->temp_c = 25.0;
    b->mppt_a = 5.0;
    b->voltage_v = 44.0 + 0.08 * b->soc_pct;
    b->next_data_ms = now + 1;
    b->next_extra_ms = now + 1;
    sim_port_set_filter(port, NULL, 0);   /* receives nothing */
}

void sim_bms_step(sim_bms_t *b, uint32_t now, double motor_current_a)
{
    hal_can_frame_t f;
    while (sim_port_recv(b->port, &f)) {
    }
    if (!b->enabled) return;

    /* 48 V pack [?], 20 mOhm; positive current = discharge; 2 A auxiliary load. The SOC never drains:
     * it stays at its start value, and a low battery is set by hand (BMSData soc=... voltage=...). */
    b->current_a = motor_current_a + 2.0 - b->mppt_a;
    b->voltage_v = 44.0 + 0.08 * b->soc_pct - 0.02 * b->current_a;
    b->temp_c += DT_S * ((25.0 + 0.0008 * b->current_a * b->current_a) - b->temp_c) / 120.0;
    b->fan_rpm = b->temp_c > 30.0 ? 1500.0 + (b->temp_c - 30.0) * 100.0 : 0.0;

    if (due(now, &b->next_data_ms, 100)) {
        msg_bms_data_t d = { to_i16(b->voltage_v * 10.0), to_i16(b->current_a * 10.0), (uint16_t)lround(b->soc_pct * 10.0) };
        sim_node_send(b->port, CAN_ID_BMS_DATA, &d);
    }
    if (due(now, &b->next_extra_ms, 500)) {
        msg_bms_extra_t e = { to_i16(b->temp_c * 10.0), to_i16(b->mppt_a * 10.0), to_i16(b->fan_rpm) };
        sim_node_send(b->port, CAN_ID_BMS_EXTRA, &e);
    }
}
