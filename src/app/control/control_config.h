/* Control node tuning (sbt CAN.md, 6.1): all gains, limits and thresholds in one place.
 * Every value here is a proposal to be validated on the bench. */
#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

/* Application step: pedal, safety checks and state machine run this often. */
#define CFG_APP_STEP_MS            5

/* Pedal: two redundant 12-bit channels, same direction [?] */
#define CFG_PEDAL_RAW_MIN          400     /* raw at 0 %   */
#define CFG_PEDAL_RAW_MAX          3700    /* raw at 100 % */
#define CFG_PEDAL_RAW_LOW_LIMIT    200     /* below: F_PEDAL_RANGE (broken wire / short) */
#define CFG_PEDAL_RAW_HIGH_LIMIT   3900    /* above: F_PEDAL_RANGE */
#define CFG_PEDAL_PLAUS_DIFF_PCT   10.0f   /* max disagreement between channels */
#define CFG_PEDAL_PLAUS_MS         50      /* ... for this long -> F_PEDAL_PLAUS */
#define CFG_PEDAL_FILTER_ALPHA     0.3f    /* low-pass per 5 ms step (same response as 0.5 per 10 ms) */
#define CFG_PEDAL_DEADZONE_PCT     5.0f    /* also "released": both channels at or below it, unfiltered */
#define CFG_PEDAL_HYST_PCT         2.0f    /* DRIVE -> READY below deadzone - hysteresis */

/* Throttle */
#define CFG_THROTTLE_RATE_PCT_PER_S 1000.0f /* max change: 0 -> 100 % in 100 ms */

/* Motor and controller temperature, degC */
#define CFG_MOTOR_TEMP_WARN        80.0f
#define CFG_MOTOR_TEMP_CRIT        100.0f
#define CFG_CTRL_TEMP_WARN         70.0f
#define CFG_CTRL_TEMP_CRIT         85.0f
#define CFG_TEMP_HYST              2.0f

/* Battery (48 V nominal pack [?]) */
#define CFG_BAT_VOLT_WARN          45.0f   /* V */
#define CFG_BAT_VOLT_CRIT          42.0f   /* V */
#define CFG_BAT_VOLT_HYST          0.5f
#define CFG_BAT_SOC_WARN           15.0f   /* % */
#define CFG_BAT_SOC_HYST           1.0f
#define CFG_BAT_TEMP_WARN          45.0f   /* degC */
#define CFG_BAT_TEMP_CRIT          55.0f

/* Over-current, A */
#define CFG_MOTOR_CURRENT_MAX      200.0f
#define CFG_BAT_CURRENT_MAX        180.0f

/* Derating limits, % of full throttle */
#define CFG_DERATE_WARNING_PCT     50.0f   /* any warning-level temperature / battery fault */
#define CFG_DERATE_MOTOR_COMM_PCT  30.0f   /* MotorTemps timed out (8.2); a MotorData timeout is critical */
#define CFG_DERATE_BMS_EXTRA_PCT   50.0f   /* BMSExtra timed out (8.2) */

/* A required message (MotorTemps, MotorData, BMSData) timed out this long -> F_COMM_LOST [?] */
#define CFG_COMM_GRACE_MS          1000u

#endif
