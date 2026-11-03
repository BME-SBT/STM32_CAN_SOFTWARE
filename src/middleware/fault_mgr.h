/* Fault manager (sbt CAN.md, 8.1). Written only from the main loop.
 * Bits 0-15 of the active mask are published in VehicleStatus.fault_mask [Proposal]. */
#ifndef FAULT_MGR_H
#define FAULT_MGR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    F_RX_TIMEOUT_MOTOR_TEMPS = 0,
    F_RX_TIMEOUT_MOTOR_DATA,
    F_RX_TIMEOUT_BMS_DATA,
    F_RX_TIMEOUT_BMS_EXTRA,
    F_RX_TIMEOUT_GPS,          /* reserved: keeps the fault-mask bit layout of the spec and V2 */
    F_RX_TIMEOUT_POSITION,     /* reserved */
    F_CAN_BUSOFF,
    F_PEDAL_RANGE,
    F_PEDAL_PLAUS,
    F_MOTOR_OVERTEMP_W,
    F_MOTOR_OVERTEMP_C,
    F_BAT_LOW_W,
    F_BAT_LOW_C,
    F_OVERCURRENT,
    F_BAT_TEMP_W,
    F_BAT_TEMP_C,
    F_COMM_LOST,        /* a required message stayed timed out past its grace period (8.2) */
    F_COUNT
} fault_code_t;

typedef enum { FAULT_SEV_WARNING = 0, FAULT_SEV_CRITICAL } fault_severity_t;

void fault_mgr_init(void);
void fault_set(fault_code_t code);
void fault_clear(fault_code_t code);
void fault_update(fault_code_t code, bool active);
bool fault_is_active(fault_code_t code);
uint32_t fault_active_mask(void);
bool fault_is_critical(void);                 /* any critical fault active */
fault_code_t fault_first_critical(void);      /* F_COUNT if none */
fault_severity_t fault_severity(fault_code_t code);
const char *fault_name(fault_code_t code);

#endif
