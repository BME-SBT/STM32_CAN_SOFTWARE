/* Test-side control of the PC mock HAL. Never included by firmware code. */
#ifndef HAL_MOCK_H
#define HAL_MOCK_H

#include <stdbool.h>
#include <stdint.h>
#include "hal_adc.h"
#include "hal_can.h"
#include "hal_gpio.h"

struct sim_port;

/* hal_time */
void hal_time_mock_set(uint32_t ms);

/* hal_adc / hal_gpio */
void hal_io_mock_reset(void);
void hal_adc_mock_set(hal_adc_ch_t ch, uint16_t raw);
void hal_gpio_mock_set(hal_gpio_pin_t pin, bool on);
bool hal_gpio_mock_get(hal_gpio_pin_t pin);

/* hal_can: the DUT's port on the simulated bus */
#define HAL_CAN_MOCK_TX_DEPTH 3u   /* G431 FDCAN has 3 TX buffers [?] */
void hal_can_mock_reset(void);                 /* attaches a new port "dut" to sim_bus */
struct sim_port *hal_can_mock_port(void);
void hal_can_mock_force_state(hal_can_state_t state);
void hal_can_mock_set_tx_full(bool full);
void hal_can_mock_set_restart_failures(uint32_t n);  /* the next n restarts fail */
uint32_t hal_can_mock_restart_count(void);

#endif
