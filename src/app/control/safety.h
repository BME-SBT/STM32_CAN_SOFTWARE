/* Safety logic (sbt CAN.md, 6.1 and 8.2): compares received data with the limits in control_config.h,
 * sets and clears faults, and returns the throttle limit. Runs every application step. */
#ifndef SAFETY_H
#define SAFETY_H

#include <stdbool.h>

typedef struct {
    float limit_pct;     /* 0..100, applied to the throttle before the rate limiter */
    bool derating;       /* limit_pct < 100 */
    bool data_valid;     /* MotorTemps, MotorData and BMSData all valid */
} safety_out_t;

void safety_init(void);
void safety_evaluate(safety_out_t *out);

#endif
