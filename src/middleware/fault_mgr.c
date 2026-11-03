#include "fault_mgr.h"

static uint32_t active;

static const struct {
    const char *name;
    fault_severity_t severity;
} info[F_COUNT] = {
    [F_RX_TIMEOUT_MOTOR_TEMPS] = { "F_RX_TIMEOUT_MOTOR_TEMPS", FAULT_SEV_WARNING },
    /* Critical: the motor controller stops on its own Control timeout (also 100 ms) and only leaves
     * FAULT after enable = 0 (6.3), which SAFE_STATE sends. A warning would leave it stuck. */
    [F_RX_TIMEOUT_MOTOR_DATA]  = { "F_RX_TIMEOUT_MOTOR_DATA",  FAULT_SEV_CRITICAL },
    [F_RX_TIMEOUT_BMS_DATA]    = { "F_RX_TIMEOUT_BMS_DATA",    FAULT_SEV_WARNING },
    [F_RX_TIMEOUT_BMS_EXTRA]   = { "F_RX_TIMEOUT_BMS_EXTRA",   FAULT_SEV_WARNING },
    [F_RX_TIMEOUT_GPS]         = { "F_RX_TIMEOUT_GPS",         FAULT_SEV_WARNING },
    [F_RX_TIMEOUT_POSITION]    = { "F_RX_TIMEOUT_POSITION",    FAULT_SEV_WARNING },
    [F_CAN_BUSOFF]             = { "F_CAN_BUSOFF",             FAULT_SEV_CRITICAL },
    [F_PEDAL_RANGE]            = { "F_PEDAL_RANGE",            FAULT_SEV_CRITICAL },
    [F_PEDAL_PLAUS]            = { "F_PEDAL_PLAUS",            FAULT_SEV_CRITICAL },
    [F_MOTOR_OVERTEMP_W]       = { "F_MOTOR_OVERTEMP_W",       FAULT_SEV_WARNING },
    [F_MOTOR_OVERTEMP_C]       = { "F_MOTOR_OVERTEMP_C",       FAULT_SEV_CRITICAL },
    [F_BAT_LOW_W]              = { "F_BAT_LOW_W",              FAULT_SEV_WARNING },
    [F_BAT_LOW_C]              = { "F_BAT_LOW_C",              FAULT_SEV_CRITICAL },
    [F_OVERCURRENT]            = { "F_OVERCURRENT",            FAULT_SEV_CRITICAL },
    [F_BAT_TEMP_W]             = { "F_BAT_TEMP_W",             FAULT_SEV_WARNING },
    [F_BAT_TEMP_C]             = { "F_BAT_TEMP_C",             FAULT_SEV_CRITICAL },
    [F_COMM_LOST]              = { "F_COMM_LOST",              FAULT_SEV_CRITICAL },
};

void fault_mgr_init(void) { active = 0; }

void fault_set(fault_code_t code)
{
    if (code < F_COUNT) active |= (1u << code);
}

void fault_clear(fault_code_t code)
{
    if (code < F_COUNT) active &= ~(1u << code);
}

void fault_update(fault_code_t code, bool on)
{
    if (on) fault_set(code);
    else fault_clear(code);
}

bool fault_is_active(fault_code_t code) { return code < F_COUNT && (active & (1u << code)) != 0; }

uint32_t fault_active_mask(void) { return active; }

fault_code_t fault_first_critical(void)
{
    for (int c = 0; c < F_COUNT; c++)
        if ((active & (1u << c)) && info[c].severity == FAULT_SEV_CRITICAL) return (fault_code_t)c;
    return F_COUNT;
}

bool fault_is_critical(void) { return fault_first_critical() != F_COUNT; }

fault_severity_t fault_severity(fault_code_t code) { return code < F_COUNT ? info[code].severity : FAULT_SEV_WARNING; }

const char *fault_name(fault_code_t code) { return code < F_COUNT ? info[code].name : "-"; }
