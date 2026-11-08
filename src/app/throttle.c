#include <stdint.h>



#define THROTTLE_ADC_MIN     0



#define THROTTLE_ADC_CENTER  2048


#define THROTTLE_ADC_MAX     4095


#define THROTTLE_DEADBAND    50



int8_t calculate_throttle(uint16_t adc_value)
{
    
    int32_t throttle = 0;


    
    if (adc_value > THROTTLE_ADC_CENTER + THROTTLE_DEADBAND)
    {
        
        
        throttle =
            ((int32_t)(adc_value - THROTTLE_ADC_CENTER - THROTTLE_DEADBAND)
            * 127) /
            (THROTTLE_ADC_MAX - THROTTLE_ADC_CENTER - THROTTLE_DEADBAND);
    }


    
    else if (adc_value < THROTTLE_ADC_CENTER - THROTTLE_DEADBAND)
    {
        
        throttle =
            -((int32_t)(THROTTLE_ADC_CENTER - THROTTLE_DEADBAND - adc_value)
            * 128) /
            (THROTTLE_ADC_CENTER - THROTTLE_ADC_MIN - THROTTLE_DEADBAND);
    }


    
    if (throttle > 127)
        throttle = 127;

    
    if (throttle < -128)
        throttle = -128;


    
    return (int8_t)throttle;
}