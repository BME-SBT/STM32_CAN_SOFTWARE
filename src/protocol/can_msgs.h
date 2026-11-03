/* Message definitions and encoding (sbt CAN.md, 0.1 and 4.1).
 * Bus bytes are produced only by the explicit pack/unpack functions below:
 * little-endian, fields back to back, reserved bytes sent as 0 and ignored on receive.
 * Payload structures hold raw bus units (e.g. 0.1 degC per bit); the suffix names the unit.
 * V1 leaves out GPS (0x400) and Position (0x401): telemetry only, not used by the Control node. */
#ifndef CAN_MSGS_H
#define CAN_MSGS_H

#include <stdbool.h>
#include <stdint.h>

#define CAN_ID_CONTROL         0x100u
#define CAN_ID_VEHICLE_STATUS  0x101u
#define CAN_ID_MOTOR_TEMPS     0x200u
#define CAN_ID_MOTOR_DATA      0x201u
#define CAN_ID_BMS_DATA        0x300u
#define CAN_ID_BMS_EXTRA       0x301u

typedef enum {
    VEH_INIT = 0,
    VEH_STANDBY,
    VEH_READY,
    VEH_DRIVE,
    VEH_SAFE_STATE,
    VEH_STATE_COUNT
} vehicle_state_t;

#define VS_FLAG_DERATING 0x01u

typedef struct { int8_t throttle_pct; uint8_t enable; uint8_t alive; } msg_control_t;
typedef struct { uint8_t state; uint8_t flags; uint16_t fault_mask; } msg_vehicle_status_t;
typedef struct { int16_t motor_temp_dC; int16_t ctrl_temp_dC; } msg_motor_temps_t;
typedef struct { int16_t rpm; int16_t current_dA; int16_t torque_dNm; } msg_motor_data_t;
typedef struct { int16_t voltage_dV; int16_t current_dA; uint16_t soc_dpct; } msg_bms_data_t;
typedef struct { int16_t temp_dC; int16_t mppt_current_dA; int16_t fan_rpm; } msg_bms_extra_t;

typedef union {
    msg_control_t        control;
    msg_vehicle_status_t vehicle_status;
    msg_motor_temps_t    motor_temps;
    msg_motor_data_t     motor_data;
    msg_bms_data_t       bms_data;
    msg_bms_extra_t      bms_extra;
} can_msg_any_t;

void can_pack_control(const msg_control_t *m, uint8_t *d);
void can_pack_vehicle_status(const msg_vehicle_status_t *m, uint8_t *d);
void can_pack_motor_temps(const msg_motor_temps_t *m, uint8_t *d);
void can_pack_motor_data(const msg_motor_data_t *m, uint8_t *d);
void can_pack_bms_data(const msg_bms_data_t *m, uint8_t *d);
void can_pack_bms_extra(const msg_bms_extra_t *m, uint8_t *d);

/* Unpack always fills the structure; returns false if a field fails its range check. */
bool can_unpack_control(const uint8_t *d, msg_control_t *m);
bool can_unpack_vehicle_status(const uint8_t *d, msg_vehicle_status_t *m);
bool can_unpack_motor_temps(const uint8_t *d, msg_motor_temps_t *m);
bool can_unpack_motor_data(const uint8_t *d, msg_motor_data_t *m);
bool can_unpack_bms_data(const uint8_t *d, msg_bms_data_t *m);
bool can_unpack_bms_extra(const uint8_t *d, msg_bms_extra_t *m);

typedef void (*can_pack_fn)(const void *msg, uint8_t *data);
typedef bool (*can_unpack_fn)(const uint8_t *data, void *msg);

/* One row of the message table: fixed ID, DLC, period and timeout (0.1). */
typedef struct {
    uint16_t id;
    const char *name;
    uint8_t dlc;
    uint16_t period_ms;
    uint16_t timeout_ms;
    uint8_t size;            /* sizeof the payload structure */
    can_pack_fn pack;
    can_unpack_fn unpack;
} can_msg_def_t;

const can_msg_def_t *can_msg_def(uint16_t id);        /* NULL if unknown */
const can_msg_def_t *can_msg_def_at(uint8_t index);   /* NULL past the end */
uint8_t can_msg_def_count(void);

#endif
