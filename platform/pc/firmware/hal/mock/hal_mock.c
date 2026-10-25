/* PC implementation of the HAL (sbt CAN.md, section 3): time, ADC, GPIO and CAN.
 * hal_can behaves like drv_fdcan (section 2) on top of a sim_bus port. */
#include <stddef.h>
#include <string.h>
#include "hal_adc.h"
#include "hal_can.h"
#include "hal_gpio.h"
#include "hal_mock.h"
#include "hal_time.h"
#include "sim_bus.h"

/* ------------------------------------------------------------------ hal_time */

static uint32_t now_ms;

uint32_t hal_time_tick_ms(void) { return now_ms; }
void hal_time_mock_set(uint32_t ms) { now_ms = ms; }

/* ------------------------------------------------------------------ hal_adc, hal_gpio */

static uint16_t adc[HAL_ADC_COUNT];
static bool gpio[HAL_GPIO_COUNT];

void hal_io_mock_reset(void)
{
    memset(adc, 0, sizeof adc);
    memset(gpio, 0, sizeof gpio);
}

uint16_t hal_adc_read(hal_adc_ch_t ch) { return ch < HAL_ADC_COUNT ? adc[ch] : 0; }
void hal_adc_mock_set(hal_adc_ch_t ch, uint16_t raw) { if (ch < HAL_ADC_COUNT) adc[ch] = raw; }

bool hal_gpio_get(hal_gpio_pin_t pin) { return pin < HAL_GPIO_COUNT ? gpio[pin] : false; }
void hal_gpio_set(hal_gpio_pin_t pin, bool on) { if (pin < HAL_GPIO_COUNT) gpio[pin] = on; }
void hal_gpio_mock_set(hal_gpio_pin_t pin, bool on) { hal_gpio_set(pin, on); }
bool hal_gpio_mock_get(hal_gpio_pin_t pin) { return hal_gpio_get(pin); }

/* ------------------------------------------------------------------ hal_can */

static sim_port_t *port;
static hal_can_state_t state;
static hal_can_error_cb_t error_cb;
static bool initialised;
static bool tx_full_forced;
static uint32_t restart_failures_left;
static uint32_t restart_count;

static void set_state(hal_can_state_t s)
{
    if (s == state) return;
    state = s;
    if (port) {
        port->bus_off = (s == HAL_CAN_BUS_OFF);
        if (s == HAL_CAN_BUS_OFF) sim_port_flush_tx(port);
    }
    if (error_cb) error_cb(s);
}

void hal_can_mock_reset(void)
{
    port = sim_bus_attach("dut", HAL_CAN_MOCK_TX_DEPTH);
    state = HAL_CAN_ERROR_ACTIVE;
    error_cb = NULL;
    initialised = false;
    tx_full_forced = false;
    restart_failures_left = 0;
    restart_count = 0;
}

int hal_can_init(const uint16_t *accept_ids, uint8_t n_ids)
{
    if (!port) return -1;
    sim_port_set_filter(port, accept_ids, n_ids);
    initialised = true;
    return 0;
}

int hal_can_send(const hal_can_frame_t *frame)
{
    if (!initialised || frame == NULL || frame->dlc > 8) return -1;
    if (state == HAL_CAN_BUS_OFF) return -2;
    if (tx_full_forced) return -3;
    return sim_port_send(port, frame) == 0 ? 0 : -3;
}

int hal_can_receive(hal_can_frame_t *frame) { return (port && sim_port_recv(port, frame)) ? 1 : 0; }

hal_can_state_t hal_can_get_state(void) { return state; }

int hal_can_restart(void)
{
    restart_count++;
    if (state != HAL_CAN_BUS_OFF) return 0;
    if (restart_failures_left > 0) {
        restart_failures_left--;
        return -1;
    }
    set_state(HAL_CAN_ERROR_ACTIVE);
    return 0;
}

void hal_can_set_error_cb(hal_can_error_cb_t cb) { error_cb = cb; }

sim_port_t *hal_can_mock_port(void) { return port; }
void hal_can_mock_force_state(hal_can_state_t s) { set_state(s); }
void hal_can_mock_set_tx_full(bool full) { tx_full_forced = full; }
void hal_can_mock_set_restart_failures(uint32_t n) { restart_failures_left = n; }
uint32_t hal_can_mock_restart_count(void) { return restart_count; }
