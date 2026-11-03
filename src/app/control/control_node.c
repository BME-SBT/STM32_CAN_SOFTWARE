#include <math.h>
#include "can_proto.h"
#include "control_config.h"
#include "control_node.h"
#include "fault_mgr.h"
#include "hal_gpio.h"
#include "hal_time.h"
#include "pedal.h"
#include "safety.h"
#include "sched.h"
#include "vehicle_fsm.h"

static safety_out_t safety;
static pedal_out_t pedal;
static uint8_t alive;

/* Periodic TX providers (4.3) */
static bool provide_control(void *msg)
{
    msg_control_t *m = msg;
    m->throttle_pct = (int8_t)lroundf(vehicle_fsm_throttle());
    m->enable = vehicle_fsm_enable() ? 1u : 0u;
    m->alive = alive;
    alive = (uint8_t)((alive + 1u) & 0x0Fu);
    return true;
}

static bool provide_vehicle_status(void *msg)
{
    msg_vehicle_status_t *m = msg;
    m->state = (uint8_t)vehicle_fsm_state();
    m->flags = safety.derating ? VS_FLAG_DERATING : 0u;
    m->fault_mask = (uint16_t)(fault_active_mask() & 0xFFFFu);
    return true;
}

static const uint16_t rx_ids[] = { CAN_ID_MOTOR_TEMPS, CAN_ID_MOTOR_DATA, CAN_ID_BMS_DATA, CAN_ID_BMS_EXTRA };
static const can_tx_entry_t tx_entries[] = {
    { CAN_ID_CONTROL, 0, provide_control },
    { CAN_ID_VEHICLE_STATUS, 0, provide_vehicle_status },
};
static const can_proto_cfg_t can_cfg = { rx_ids, 4, tx_entries, 2 };

/* Every CFG_APP_STEP_MS: signal processing, safety checks, state machine (5.2) */
static void app_step(void)
{
    vehicle_inputs_t in;
    vehicle_state_t before = vehicle_fsm_state();
    pedal_status_t ps = pedal_read(&pedal);

    fault_update(F_PEDAL_RANGE, ps == PEDAL_ERR_RANGE);
    fault_update(F_PEDAL_PLAUS, ps == PEDAL_ERR_PLAUS);
    safety_evaluate(&safety);

    in.enable_switch = hal_gpio_get(HAL_GPIO_ENABLE_SWITCH);
    in.pedal_ok = (ps == PEDAL_OK);
    in.pedal_filtered_pct = pedal.filtered_pct;
    in.pedal_pct = pedal.pct;
    in.limit_pct = safety.limit_pct;
    in.data_valid = safety.data_valid;
    in.critical = fault_first_critical();
    vehicle_fsm_step(&in);

    /* A state change also changes enable (and throttle): send it at once instead of waiting up to
     * one period for the periodic frames (event-driven transmit, 4.3). */
    if (vehicle_fsm_state() != before) {
        can_msg_any_t m;
        provide_control(&m);
        (void)can_proto_send(CAN_ID_CONTROL, &m);
        provide_vehicle_status(&m);
        (void)can_proto_send(CAN_ID_VEHICLE_STATUS, &m);
    }
}

/* 100 ms: status LED - on in DRIVE, blinking in SAFE_STATE */
static void diag_step(void)
{
    static bool blink;
    vehicle_state_t s = vehicle_fsm_state();
    blink = !blink;
    hal_gpio_set(HAL_GPIO_STATUS_LED, s == VEH_DRIVE || (s == VEH_SAFE_STATE && blink));
}

static sched_task_t tasks[] = {
    { can_proto_process, 1, 0 },
    { app_step, CFG_APP_STEP_MS, 0 },
    { diag_step, 100, 0 },
};

void control_node_init(void)
{
    alive = 0;
    safety.limit_pct = 100.0f;
    safety.derating = false;
    safety.data_valid = false;
    pedal.filtered_pct = 0.0f;
    pedal.pct = 0.0f;

    fault_mgr_init();
    pedal_init();
    safety_init();
    vehicle_fsm_init();
    can_proto_init(&can_cfg);
    sched_init(tasks, (uint8_t)(sizeof tasks / sizeof tasks[0]));
}

float control_node_limit_pct(void) { return safety.limit_pct; }
bool control_node_derating(void) { return safety.derating; }
float control_node_pedal_pct(void) { return pedal.pct; }
