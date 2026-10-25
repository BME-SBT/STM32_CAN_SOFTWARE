#include <stdbool.h>
#include "control_config.h"
#include "hal_adc.h"
#include "pedal.h"

#define PLAUS_STEPS (CFG_PEDAL_PLAUS_MS / CFG_APP_STEP_MS)

static float filtered;
static bool have_filtered;
static int plaus_count;

void pedal_init(void)
{
    filtered = 0.0f;
    have_filtered = false;
    plaus_count = 0;
}

static float raw_to_pct(unsigned raw)
{
    float pct = ((float)raw - CFG_PEDAL_RAW_MIN) * 100.0f / (float)(CFG_PEDAL_RAW_MAX - CFG_PEDAL_RAW_MIN);
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return pct;
}

static bool raw_in_range(unsigned raw) { return raw >= CFG_PEDAL_RAW_LOW_LIMIT && raw <= CFG_PEDAL_RAW_HIGH_LIMIT; }

pedal_status_t pedal_read(pedal_out_t *out)
{
    unsigned a = hal_adc_read(HAL_ADC_PEDAL_A);
    unsigned b = hal_adc_read(HAL_ADC_PEDAL_B);
    float pa, pb, diff;

    out->filtered_pct = 0.0f;
    out->pct = 0.0f;

    if (!raw_in_range(a) || !raw_in_range(b)) {
        have_filtered = false;
        return PEDAL_ERR_RANGE;
    }

    pa = raw_to_pct(a);
    pb = raw_to_pct(b);
    diff = pa > pb ? pa - pb : pb - pa;
    if (diff > CFG_PEDAL_PLAUS_DIFF_PCT) {
        if (plaus_count < PLAUS_STEPS) plaus_count++;
    } else {
        plaus_count = 0;
    }
    if (plaus_count >= PLAUS_STEPS) {
        have_filtered = false;
        return PEDAL_ERR_PLAUS;
    }

    {
        float p = (pa + pb) * 0.5f;
        bool released = pa <= CFG_PEDAL_DEADZONE_PCT && pb <= CFG_PEDAL_DEADZONE_PCT;
        /* The filter smooths pressing; a release counts at once, without waiting for the filter
         * to decay, so the driver's acknowledge and lifting off register in one step. */
        filtered = (have_filtered && !released) ? filtered + CFG_PEDAL_FILTER_ALPHA * (p - filtered) : p;
        have_filtered = true;
    }
    out->filtered_pct = filtered;
    if (filtered > CFG_PEDAL_DEADZONE_PCT)
        out->pct = (filtered - CFG_PEDAL_DEADZONE_PCT) * 100.0f / (100.0f - CFG_PEDAL_DEADZONE_PCT);
    return PEDAL_OK;
}
