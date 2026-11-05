/* Pedal chain (sbt CAN.md, 6.1): ADC -> calibration and plausibility -> low-pass -> deadzone -> 0..100 %. */
#ifndef PEDAL_H
#define PEDAL_H

typedef enum { PEDAL_OK = 0, PEDAL_ERR_RANGE, PEDAL_ERR_PLAUS } pedal_status_t;

typedef struct {
    float filtered_pct;   /* after the low-pass, before the deadzone */
    float pct;            /* after the deadzone, 0..100 */
} pedal_out_t;

void pedal_init(void);
/* Call every application step. On an error both outputs are 0. */
pedal_status_t pedal_read(pedal_out_t *out);

#endif
