#include "control_config.h"
#include "vehicle_fsm.h"

static vehicle_state_t state;
static float throttle;
static bool armed;              /* enable switch seen OFF since boot (start-up interlock, 8.4) */
static fault_code_t latched;

static const char *const names[VEH_STATE_COUNT] = { "INIT", "STANDBY", "READY", "DRIVE", "SAFE_STATE" };

const char *vehicle_state_name(vehicle_state_t s) { return s < VEH_STATE_COUNT ? names[s] : "?"; }

/* Entry actions (6.2 table). */
static void enter(vehicle_state_t s, const vehicle_inputs_t *in)
{
    state = s;
    switch (s) {
    case VEH_STANDBY:
        throttle = 0.0f;
        latched = F_COUNT;   /* clear the latched fault code */
        break;
    case VEH_READY:
        throttle = 0.0f;
        latched = F_COUNT;
        break;
    case VEH_SAFE_STATE:
        throttle = 0.0f;
        latched = in->critical;
        break;
    default:
        break;
    }
}

void vehicle_fsm_init(void)
{
    state = VEH_INIT;
    throttle = 0.0f;
    armed = false;
    latched = F_COUNT;
}

#define RATE_PER_STEP (CFG_THROTTLE_RATE_PCT_PER_S * CFG_APP_STEP_MS / 1000.0f)

static float ramp(float from, float to)
{
    if (to > from + RATE_PER_STEP) return from + RATE_PER_STEP;
    if (to < from - RATE_PER_STEP) return from - RATE_PER_STEP;
    return to;
}

void vehicle_fsm_step(const vehicle_inputs_t *in)
{
    bool pedal_at_zero = in->pedal_ok && in->pedal_filtered_pct <= CFG_PEDAL_DEADZONE_PCT;

    if (!in->enable_switch) armed = true;

    /* any -> SAFE_STATE on a critical fault */
    if (state != VEH_SAFE_STATE && in->critical != F_COUNT) {
        enter(VEH_SAFE_STATE, in);
        return;
    }

    switch (state) {
    case VEH_INIT:
        /* Self-test: the pedal range and the CAN bus are covered by F_PEDAL_RANGE and F_CAN_BUSOFF above. */
        enter(VEH_STANDBY, in);
        break;

    case VEH_STANDBY:
        if (armed && in->enable_switch && pedal_at_zero && in->data_valid) enter(VEH_READY, in);
        break;

    case VEH_READY:
        if (!in->enable_switch) enter(VEH_STANDBY, in);
        else if (in->pedal_ok && in->pedal_pct > 0.0f) enter(VEH_DRIVE, in);
        break;

    case VEH_DRIVE:
        if (!in->enable_switch) {
            /* ramp the throttle to 0 first, then STANDBY */
            throttle = ramp(throttle, 0.0f);
            if (throttle <= 0.0f) enter(VEH_STANDBY, in);
        } else if (!in->pedal_ok ||
                   in->pedal_filtered_pct < CFG_PEDAL_DEADZONE_PCT - CFG_PEDAL_HYST_PCT) {
            enter(VEH_READY, in);
        } else {
            float target = in->pedal_pct < in->limit_pct ? in->pedal_pct : in->limit_pct;
            throttle = ramp(throttle, target);
        }
        break;

    case VEH_SAFE_STATE:
        /* Recovery without a switch action: once no critical fault is active, the pedal at zero is
         * the acknowledge. A pedal still held when the fault clears must be released first, so torque
         * never returns at a moment the driver did not choose. Enable OFF is the driver's "off". */
        if (in->critical != F_COUNT) break;
        if (!in->enable_switch) enter(VEH_STANDBY, in);
        else if (pedal_at_zero && in->data_valid) enter(VEH_READY, in);
        break;

    default:
        break;
    }
}

vehicle_state_t vehicle_fsm_state(void) { return state; }
float vehicle_fsm_throttle(void) { return state == VEH_DRIVE ? throttle : 0.0f; }
bool vehicle_fsm_enable(void) { return state == VEH_READY || state == VEH_DRIVE; }
fault_code_t vehicle_fsm_latched_fault(void) { return latched; }
