/* Vehicle state machine (sbt CAN.md, 6.2). Called from the application step (CFG_APP_STEP_MS). */
#ifndef VEHICLE_FSM_H
#define VEHICLE_FSM_H

#include <stdbool.h>
#include "can_msgs.h"
#include "fault_mgr.h"

typedef struct {
    bool enable_switch;
    bool pedal_ok;              /* pedal_read() returned PEDAL_OK */
    float pedal_filtered_pct;   /* before the deadzone */
    float pedal_pct;            /* after the deadzone */
    float limit_pct;            /* from safety_evaluate() */
    bool data_valid;            /* BMS and motor data valid */
    fault_code_t critical;      /* first active critical fault, F_COUNT if none */
} vehicle_inputs_t;

void vehicle_fsm_init(void);
void vehicle_fsm_step(const vehicle_inputs_t *in);
vehicle_state_t vehicle_fsm_state(void);
float vehicle_fsm_throttle(void);              /* 0..100 %, rate limited */
bool vehicle_fsm_enable(void);                 /* drive enable output */
fault_code_t vehicle_fsm_latched_fault(void);  /* fault that caused SAFE_STATE, F_COUNT if none */
const char *vehicle_state_name(vehicle_state_t s);

#endif
