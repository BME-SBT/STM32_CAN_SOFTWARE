#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sim_dbc.h"
#include "sim_util.h"

static const char *const veh_states[] = { "INIT", "STANDBY", "READY", "DRIVE", "SAFE_STATE" };

/*                   name            start bits signed scale decimals unit */
static const sim_field_t f_control[] = {
    { "throttle",     0,  8, true,  1.0,  0, "%",    NULL, 0 },
    { "enable",       8,  8, false, 1.0,  0, "",     NULL, 0 },
    { "alive",       16,  4, false, 1.0,  0, "",     NULL, 0 },
};
static const sim_field_t f_vehicle_status[] = {
    { "state",        0,  8, false, 1.0,  0, "",     veh_states, 5 },
    { "flags",        8,  8, false, 1.0,  0, "",     NULL, 0 },
    { "fault_mask",  16, 16, false, 1.0,  0, "",     NULL, 0 },
};
static const sim_field_t f_motor_temps[] = {
    { "motor_temp",   0, 16, true,  0.1,  1, "C",    NULL, 0 },
    { "ctrl_temp",   16, 16, true,  0.1,  1, "C",    NULL, 0 },
};
static const sim_field_t f_motor_data[] = {
    { "rpm",          0, 16, true,  1.0,  0, "rpm",  NULL, 0 },
    { "current",     16, 16, true,  0.1,  1, "A",    NULL, 0 },
    { "torque",      32, 16, true,  0.1,  1, "Nm",   NULL, 0 },
};
static const sim_field_t f_bms_data[] = {
    { "voltage",      0, 16, true,  0.1,  1, "V",    NULL, 0 },
    { "current",     16, 16, true,  0.1,  1, "A",    NULL, 0 },
    { "soc",         32, 16, false, 0.1,  1, "%",    NULL, 0 },
};
static const sim_field_t f_bms_extra[] = {
    { "temp",         0, 16, true,  0.1,  1, "C",    NULL, 0 },
    { "mppt_current",16, 16, true,  0.1,  1, "A",    NULL, 0 },
    { "fan_rpm",     32, 16, true,  1.0,  0, "rpm",  NULL, 0 },
};
#define MSG(id, name, dlc, sender, f) { id, name, dlc, sender, f, (uint8_t)(sizeof f / sizeof f[0]) }

static const sim_msg_t msgs[] = {
    MSG(0x100, "Control",       3, "dut",   f_control),
    MSG(0x101, "VehicleStatus", 4, "dut",   f_vehicle_status),
    MSG(0x200, "MotorTemps",    4, "motor", f_motor_temps),
    MSG(0x201, "MotorData",     6, "motor", f_motor_data),
    MSG(0x300, "BMSData",       6, "bms",   f_bms_data),
    MSG(0x301, "BMSExtra",      6, "bms",   f_bms_extra),
};
#define N_MSGS ((uint8_t)(sizeof msgs / sizeof msgs[0]))

uint8_t sim_dbc_count(void) { return N_MSGS; }
const sim_msg_t *sim_dbc_at(uint8_t index) { return index < N_MSGS ? &msgs[index] : NULL; }

const sim_msg_t *sim_dbc_by_id(uint16_t id)
{
    for (uint8_t i = 0; i < N_MSGS; i++)
        if (msgs[i].id == id) return &msgs[i];
    return NULL;
}

const sim_msg_t *sim_dbc_by_name(const char *name)
{
    for (uint8_t i = 0; i < N_MSGS; i++)
        if (sim_streq(msgs[i].name, name)) return &msgs[i];
    return NULL;
}

int sim_dbc_field_index(const sim_msg_t *msg, const char *field)
{
    for (uint8_t i = 0; i < msg->n_fields; i++)
        if (sim_streq(msg->fields[i].name, field)) return i;
    return -1;
}

static uint64_t load_le(const uint8_t *d)
{
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) v |= (uint64_t)d[i] << (8 * i);
    return v;
}

static void store_le(uint8_t *d, uint64_t v)
{
    for (int i = 0; i < 8; i++) d[i] = (uint8_t)(v >> (8 * i));
}

int64_t sim_dbc_get_raw(const sim_field_t *f, const uint8_t *data)
{
    uint64_t mask = (1ull << f->bits) - 1ull;
    int64_t raw = (int64_t)((load_le(data) >> f->start) & mask);
    if (f->is_signed && (raw & (int64_t)(1ull << (f->bits - 1)))) raw -= (int64_t)(1ull << f->bits);
    return raw;
}

double sim_dbc_get(const sim_field_t *f, const uint8_t *data) { return (double)sim_dbc_get_raw(f, data) * f->scale; }

bool sim_dbc_set(const sim_field_t *f, uint8_t *data, double value)
{
    uint64_t mask = (1ull << f->bits) - 1ull;
    int64_t lo = f->is_signed ? -(int64_t)(1ull << (f->bits - 1)) : 0;
    int64_t hi = f->is_signed ? (int64_t)(1ull << (f->bits - 1)) - 1 : (int64_t)mask;
    int64_t raw = llround(value / f->scale);
    bool ok = true;
    uint64_t v;

    if (raw < lo) { raw = lo; ok = false; }
    if (raw > hi) { raw = hi; ok = false; }
    v = load_le(data);
    v &= ~(mask << f->start);
    v |= ((uint64_t)raw & mask) << f->start;
    store_le(data, v);
    return ok;
}

bool sim_dbc_parse_value(const sim_field_t *f, const char *text, double *out)
{
    for (uint8_t i = 0; i < f->n_enum; i++) {
        if (sim_streq(f->enum_names[i], text)) {
            *out = i;
            return true;
        }
    }
    return sim_parse_double(text, out);
}

void sim_dbc_format_value(const sim_field_t *f, double value, char *buf, size_t n)
{
    long long raw = llround(value / f->scale);
    if (f->n_enum && raw >= 0 && raw < f->n_enum) snprintf(buf, n, "%s", f->enum_names[raw]);
    else snprintf(buf, n, "%.*f", f->decimals, value);
}

void sim_dbc_format(const sim_msg_t *msg, const uint8_t *data, char *buf, size_t n)
{
    size_t used = 0;
    buf[0] = '\0';
    for (uint8_t i = 0; i < msg->n_fields && used < n; i++) {
        const sim_field_t *f = &msg->fields[i];
        char val[32];
        int w;
        sim_dbc_format_value(f, sim_dbc_get(f, data), val, sizeof val);
        w = snprintf(buf + used, n - used, "%s%s=%s%s%s", i ? " " : "", f->name, val, *f->unit ? " " : "", f->unit);
        if (w < 0) break;
        used += (size_t)w;
    }
}
