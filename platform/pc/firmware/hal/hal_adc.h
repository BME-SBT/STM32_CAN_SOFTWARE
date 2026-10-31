/* hal_adc - latest raw value per channel (sbt CAN.md, section 3). */
#ifndef HAL_ADC_H
#define HAL_ADC_H

#include <stdint.h>

typedef enum {
    HAL_ADC_PEDAL_A = 0,
    HAL_ADC_PEDAL_B,
    HAL_ADC_COUNT
} hal_adc_ch_t;

uint16_t hal_adc_read(hal_adc_ch_t ch);   /* 12-bit raw value */

#endif
