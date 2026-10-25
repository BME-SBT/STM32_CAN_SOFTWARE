#include "can_proto.h"
#include "control_config.h"
#include "fault_mgr.h"
#include "hal_time.h"
#include "safety.h"

/* Watched messages; the first N_REQUIRED raise F_COMM_LOST after CFG_COMM_GRACE_MS in timeout. */
static const struct {
    uint16_t id;
    fault_code_t timeout_fault;
} watched[] = {
    { CAN_ID_MOTOR_TEMPS, F_RX_TIMEOUT_MOTOR_TEMPS },
    { CAN_ID_MOTOR_DATA,  F_RX_TIMEOUT_MOTOR_DATA },
    { CAN_ID_BMS_DATA,    F_RX_TIMEOUT_BMS_DATA },
    { CAN_ID_BMS_EXTRA,   F_RX_TIMEOUT_BMS_EXTRA },
};
#define N_WATCHED (sizeof watched / sizeof watched[0])
#define N_REQUIRED 3u

static uint32_t timed_out_since[N_WATCHED];
static bool timed_out[N_WATCHED];

void safety_init(void)
{
    for (unsigned i = 0; i < N_WATCHED; i++) timed_out[i] = false;
}

/* Threshold with hysteresis: once active, the value must move back past threshold -/+ hysteresis. */
static bool above(bool active, float v, float thr, float hyst) { return active ? v > thr - hyst : v > thr; }
static bool below(bool active, float v, float thr, float hyst) { return active ? v < thr + hyst : v < thr; }

static float min_f(float a, float b) { return a < b ? a : b; }
static float abs_f(float a) { return a < 0.0f ? -a : a; }

void safety_evaluate(safety_out_t *out)
{
    uint32_t now = hal_time_tick_ms();
    float limit = 100.0f;
    bool comm_lost = false;
    msg_motor_temps_t mt;
    msg_motor_data_t md;
    msg_bms_data_t bd;
    msg_bms_extra_t be;
    bool mt_ok = can_proto_get_motor_temps(&mt, NULL) == CAN_OK;
    bool md_ok = can_proto_get_motor_data(&md, NULL) == CAN_OK;
    bool bd_ok = can_proto_get_bms_data(&bd, NULL) == CAN_OK;
    bool be_ok = can_proto_get_bms_extra(&be, NULL) == CAN_OK;

    /* Message timeouts (8.2). TIMEOUT and RECOVERY both count as "not valid". */
    for (unsigned i = 0; i < N_WATCHED; i++) {
        can_rx_state_t st = can_proto_rx_state(watched[i].id);
        bool to = (st == CAN_RX_TIMEOUT || st == CAN_RX_RECOVERY);
        if (to && !timed_out[i]) timed_out_since[i] = now;
        timed_out[i] = to;
        fault_update(watched[i].timeout_fault, to);
        if (to && i < N_REQUIRED && (uint32_t)(now - timed_out_since[i]) >= CFG_COMM_GRACE_MS) comm_lost = true;
    }
    fault_update(F_COMM_LOST, comm_lost);
    if (timed_out[0]) limit = min_f(limit, CFG_DERATE_MOTOR_COMM_PCT);   /* MotorData timeout is critical */
    if (timed_out[3]) limit = min_f(limit, CFG_DERATE_BMS_EXTRA_PCT);

    /* Each check runs only on valid data; without valid data the fault keeps its last state. */
    if (mt_ok) {
        float tm = mt.motor_temp_dC / 10.0f, tc = mt.ctrl_temp_dC / 10.0f;
        bool w = fault_is_active(F_MOTOR_OVERTEMP_W), c = fault_is_active(F_MOTOR_OVERTEMP_C);
        fault_update(F_MOTOR_OVERTEMP_W, above(w, tm, CFG_MOTOR_TEMP_WARN, CFG_TEMP_HYST) ||
                                         above(w, tc, CFG_CTRL_TEMP_WARN, CFG_TEMP_HYST));
        fault_update(F_MOTOR_OVERTEMP_C, above(c, tm, CFG_MOTOR_TEMP_CRIT, CFG_TEMP_HYST) ||
                                         above(c, tc, CFG_CTRL_TEMP_CRIT, CFG_TEMP_HYST));
    }
    if (bd_ok) {
        float v = bd.voltage_dV / 10.0f, soc = bd.soc_dpct / 10.0f;
        bool w = fault_is_active(F_BAT_LOW_W), c = fault_is_active(F_BAT_LOW_C);
        fault_update(F_BAT_LOW_W, below(w, v, CFG_BAT_VOLT_WARN, CFG_BAT_VOLT_HYST) ||
                                  below(w, soc, CFG_BAT_SOC_WARN, CFG_BAT_SOC_HYST));
        fault_update(F_BAT_LOW_C, below(c, v, CFG_BAT_VOLT_CRIT, CFG_BAT_VOLT_HYST));
    }
    if (be_ok) {
        float t = be.temp_dC / 10.0f;
        fault_update(F_BAT_TEMP_W, above(fault_is_active(F_BAT_TEMP_W), t, CFG_BAT_TEMP_WARN, CFG_TEMP_HYST));
        fault_update(F_BAT_TEMP_C, above(fault_is_active(F_BAT_TEMP_C), t, CFG_BAT_TEMP_CRIT, CFG_TEMP_HYST));
    }
    if (md_ok || bd_ok) {
        bool oc = (md_ok && abs_f(md.current_dA / 10.0f) > CFG_MOTOR_CURRENT_MAX) ||
                  (bd_ok && abs_f(bd.current_dA / 10.0f) > CFG_BAT_CURRENT_MAX);
        fault_update(F_OVERCURRENT, oc);
    }

    if (fault_is_active(F_MOTOR_OVERTEMP_W) || fault_is_active(F_BAT_LOW_W) || fault_is_active(F_BAT_TEMP_W))
        limit = min_f(limit, CFG_DERATE_WARNING_PCT);

    out->limit_pct = limit;
    out->derating = limit < 100.0f;
    out->data_valid = mt_ok && md_ok && bd_ok;
}
