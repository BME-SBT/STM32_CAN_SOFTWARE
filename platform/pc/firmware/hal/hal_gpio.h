/* hal_gpio - logical pins from the board configuration (sbt CAN.md, section 3). */
#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#include <stdbool.h>

typedef enum {
    HAL_GPIO_ENABLE_SWITCH = 0,
    HAL_GPIO_STATUS_LED,
    HAL_GPIO_COUNT
} hal_gpio_pin_t;

bool hal_gpio_get(hal_gpio_pin_t pin);
void hal_gpio_set(hal_gpio_pin_t pin, bool on);

#endif
