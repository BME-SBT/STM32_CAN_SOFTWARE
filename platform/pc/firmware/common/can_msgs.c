#include <stddef.h>
#include "can_msgs.h"

/* Range limits are proposals, like the scales in sbt CAN.md 4.1. */
#define IN_RANGE(v, lo, hi) ((v) >= (lo) && (v) <= (hi))

static void put_u16(uint8_t *d, uint16_t v)
{
    d[0] = (uint8_t)v;
    d[1] = (uint8_t)(v >> 8);
}

static uint16_t get_u16(const uint8_t *d) { return (uint16_t)(d[0] | (d[1] << 8)); }

/* Control - 0x100, 3 bytes */
void can_pack_control(const msg_control_t *m, uint8_t *d)
{
    d[0] = (uint8_t)m->throttle_pct;
    d[1] = m->enable;
    d[2] = (uint8_t)(m->alive & 0x0Fu);   /* bits 20-23 reserved */
}

bool can_unpack_control(const uint8_t *d, msg_control_t *m)
{
    m->throttle_pct = (int8_t)d[0];
    m->enable = d[1];
    m->alive = (uint8_t)(d[2] & 0x0Fu);
    return IN_RANGE(m->throttle_pct, -100, 100) && m->enable <= 1u;
}

/* VehicleStatus - 0x101, 4 bytes */
void can_pack_vehicle_status(const msg_vehicle_status_t *m, uint8_t *d)
{
    d[0] = m->state;
    d[1] = m->flags;
    put_u16(d + 2, m->fault_mask);
}

bool can_unpack_vehicle_status(const uint8_t *d, msg_vehicle_status_t *m)
{
    m->state = d[0];
    m->flags = d[1];
    m->fault_mask = get_u16(d + 2);
    return m->state < VEH_STATE_COUNT;
}

/* MotorTemps - 0x200, 4 bytes, 0.1 degC/bit */
void can_pack_motor_temps(const msg_motor_temps_t *m, uint8_t *d)
{
    put_u16(d, (uint16_t)m->motor_temp_dC);
    put_u16(d + 2, (uint16_t)m->ctrl_temp_dC);
}

bool can_unpack_motor_temps(const uint8_t *d, msg_motor_temps_t *m)
{
    m->motor_temp_dC = (int16_t)get_u16(d);
    m->ctrl_temp_dC = (int16_t)get_u16(d + 2);
    return IN_RANGE(m->motor_temp_dC, -400, 2000) && IN_RANGE(m->ctrl_temp_dC, -400, 2000);
}

/* MotorData - 0x201, 6 bytes */
void can_pack_motor_data(const msg_motor_data_t *m, uint8_t *d)
{
    put_u16(d, (uint16_t)m->rpm);
    put_u16(d + 2, (uint16_t)m->current_dA);
    put_u16(d + 4, (uint16_t)m->torque_dNm);
}

bool can_unpack_motor_data(const uint8_t *d, msg_motor_data_t *m)
{
    m->rpm = (int16_t)get_u16(d);
    m->current_dA = (int16_t)get_u16(d + 2);
    m->torque_dNm = (int16_t)get_u16(d + 4);
    return IN_RANGE(m->rpm, -10000, 10000) && IN_RANGE(m->current_dA, -5000, 5000) &&
           IN_RANGE(m->torque_dNm, -5000, 5000);
}

/* BMSData - 0x300, 6 bytes */
void can_pack_bms_data(const msg_bms_data_t *m, uint8_t *d)
{
    put_u16(d, (uint16_t)m->voltage_dV);
    put_u16(d + 2, (uint16_t)m->current_dA);
    put_u16(d + 4, m->soc_dpct);
}

bool can_unpack_bms_data(const uint8_t *d, msg_bms_data_t *m)
{
    m->voltage_dV = (int16_t)get_u16(d);
    m->current_dA = (int16_t)get_u16(d + 2);
    m->soc_dpct = get_u16(d + 4);
    return IN_RANGE(m->voltage_dV, 0, 1500) && IN_RANGE(m->current_dA, -5000, 5000) && m->soc_dpct <= 1000u;
}

/* BMSExtra - 0x301, 6 bytes */
void can_pack_bms_extra(const msg_bms_extra_t *m, uint8_t *d)
{
    put_u16(d, (uint16_t)m->temp_dC);
    put_u16(d + 2, (uint16_t)m->mppt_current_dA);
    put_u16(d + 4, (uint16_t)m->fan_rpm);
}

bool can_unpack_bms_extra(const uint8_t *d, msg_bms_extra_t *m)
{
    m->temp_dC = (int16_t)get_u16(d);
    m->mppt_current_dA = (int16_t)get_u16(d + 2);
    m->fan_rpm = (int16_t)get_u16(d + 4);
    return IN_RANGE(m->temp_dC, -400, 1500) && IN_RANGE(m->mppt_current_dA, -100, 1000) &&
           IN_RANGE(m->fan_rpm, 0, 20000);
}

#define ADAPT(name, type)                                                                          \
    static void pack_##name(const void *m, uint8_t *d) { can_pack_##name((const type *)m, d); }   \
    static bool unpack_##name(const uint8_t *d, void *m) { return can_unpack_##name(d, (type *)m); }

ADAPT(control, msg_control_t)
ADAPT(vehicle_status, msg_vehicle_status_t)
ADAPT(motor_temps, msg_motor_temps_t)
ADAPT(motor_data, msg_motor_data_t)
ADAPT(bms_data, msg_bms_data_t)
ADAPT(bms_extra, msg_bms_extra_t)

#define DEF(id, name, dlc, period, timeout, fn, type) \
    { id, name, dlc, period, timeout, (uint8_t)sizeof(type), pack_##fn, unpack_##fn }

static const can_msg_def_t defs[] = {
    DEF(CAN_ID_CONTROL,        "Control",       3,   20,  100, control,        msg_control_t),
    DEF(CAN_ID_VEHICLE_STATUS, "VehicleStatus", 4,  100,  500, vehicle_status, msg_vehicle_status_t),
    DEF(CAN_ID_MOTOR_TEMPS,    "MotorTemps",    4,  100,  500, motor_temps,    msg_motor_temps_t),
    DEF(CAN_ID_MOTOR_DATA,     "MotorData",     6,   20,  100, motor_data,     msg_motor_data_t),
    DEF(CAN_ID_BMS_DATA,       "BMSData",       6,  100,  500, bms_data,       msg_bms_data_t),
    DEF(CAN_ID_BMS_EXTRA,      "BMSExtra",      6,  500, 2000, bms_extra,      msg_bms_extra_t),
};

#define N_DEFS ((uint8_t)(sizeof defs / sizeof defs[0]))

const can_msg_def_t *can_msg_def(uint16_t id)
{
    for (uint8_t i = 0; i < N_DEFS; i++)
        if (defs[i].id == id) return &defs[i];
    return NULL;
}

const can_msg_def_t *can_msg_def_at(uint8_t index) { return index < N_DEFS ? &defs[index] : NULL; }

uint8_t can_msg_def_count(void) { return N_DEFS; }
