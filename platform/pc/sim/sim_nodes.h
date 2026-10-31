/* Simulated peer nodes (sbt CAN mock.md, 3.4) and field overrides.
 * (motor controller and BMS; V1 has no GPS). Each node owns a sim_bus port, runs a simple
 * behavioural model every ms and sends its periodic
 * messages with the shared firmware codec. Overrides replace single fields in the frames they send. */
#ifndef SIM_NODES_H
#define SIM_NODES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "sim_bus.h"
#include "sim_dbc.h"

/* ---- Overrides: persistent field values in the periodic traffic of the sim nodes ---- */
#define SIM_OVERRIDE_MAX 16

bool sim_override_set(const sim_msg_t *msg, int field, double value);
bool sim_override_clear(const sim_msg_t *msg, int field);   /* field -1: all fields of msg */
void sim_override_clear_all(void);
int sim_override_count(void);
bool sim_override_describe(int index, char *buf, size_t n);   /* index 0..count-1 */

/* Packs msg, applies overrides and queues the frame. Returns sim_port_send()'s result. */
int sim_node_send(sim_port_t *port, uint16_t id, const void *msg);

/* ---- Motor controller (sbt CAN.md, 6.3) ---- */
typedef enum { SIM_MOTOR_INIT = 0, SIM_MOTOR_IDLE, SIM_MOTOR_RUNNING, SIM_MOTOR_FAULT, SIM_MOTOR_STATE_COUNT } sim_motor_state_t;

#define SIM_MOTOR_CONTROL_TIMEOUT_MS 100u
#define SIM_MOTOR_TEMP_FAULT_C       120.0
#define SIM_MOTOR_CURRENT_FAULT_A    250.0

typedef struct {
    sim_port_t *port;
    bool enabled;
    sim_motor_state_t state;
    const char *fault_reason;
    /* last valid Control frame */
    bool have_control;
    uint32_t last_control_ms;
    uint32_t alive_change_ms;
    uint8_t alive;
    int8_t throttle_pct;
    uint8_t enable;
    /* model */
    double rpm, current_a, torque_nm, motor_temp_c, ctrl_temp_c;
    uint32_t next_temps_ms, next_data_ms;
} sim_motor_t;

void sim_motor_init(sim_motor_t *m, sim_port_t *port, uint32_t now);
void sim_motor_step(sim_motor_t *m, uint32_t now);
const char *sim_motor_state_name(sim_motor_state_t s);

/* ---- BMS (+ MPPT) ---- */
typedef struct {
    sim_port_t *port;
    bool enabled;
    double soc_pct, voltage_v, current_a, temp_c, mppt_a, fan_rpm;
    uint32_t next_data_ms, next_extra_ms;
} sim_bms_t;

void sim_bms_init(sim_bms_t *b, sim_port_t *port, uint32_t now);
void sim_bms_step(sim_bms_t *b, uint32_t now, double motor_current_a);

#endif
