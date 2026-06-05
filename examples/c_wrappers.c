/*
 * c_wrappers.c
 *
 * Demonstrates the C wrappers for the two opt-in helpers and the
 * calculator tuning knobs:
 *
 *   - ffb_metrics_*      : derive scaled position + filtered speed/accel
 *                          from a raw wheel angle (ffb_metrics_c.h)
 *   - ffb_axis_local_*   : idle spring / end-stop / always-on damper etc.
 *                          that the host does NOT request (ffb_axis_local_c.h)
 *   - ffb_set_effect_*   : runtime tuning of the condition-effect tables
 *
 * Build (from the library root):
 *   cmake -B build -DFFB_BUILD_EXAMPLES=ON
 *   cmake --build build
 *   ./build/ffb_example_c_wrappers
 */

#include "ffb/ffb_c.h"
#include "ffb/ffb_metrics_c.h"
#include "ffb/ffb_axis_local_c.h"

#include <stdint.h>
#include <stdio.h>
#include <math.h>

/* ---- USER GLUE: time source --------------------------------------- */
static uint32_t g_ms = 0;
static uint32_t platform_millis(void) { return g_ms; }
static uint32_t platform_micros(void) { return g_ms * 1000u; }

/* ---- USER GLUE: USB send (optional) ------------------------------- */
static bool platform_send_hid_report(const uint8_t* report, uint16_t len) {
    (void)report; (void)len;
    return true;   /* pretend the USB stack accepted the report */
}

/* ---- USER GLUE: motor output -------------------------------------- */
static void platform_set_motor(int32_t torque) {
    (void)torque;  /* map -0x7fff..0x7fff to your PWM / current target */
}

int main(void) {
    /* 1. Core engine. */
    ffb_lib_t* lib = ffb_create(/*axis_count=*/1, platform_millis, platform_micros);
    ffb_set_send_report_callback(lib, platform_send_hid_report);

    uint16_t desc_len = 0;
    const uint8_t* desc = ffb_descriptor_1axis(&desc_len);
    (void)desc;
    printf("descriptor: %u bytes, samplerate %.0f Hz, axes %u\n",
           desc_len, ffb_get_samplerate(lib), ffb_get_axis_count(lib));

    /* 2. Metrics helper: raw wheel degrees -> scaled pos + filtered speed/accel.
     *    900 deg of total travel, control loop at 1 kHz. */
    const float DOR = 900.0f;
    ffb_metrics_t* metrics = ffb_metrics_create(DOR, 1000.0f);

    /* 3. Axis-local "feel" effects (not requested by the host). */
    ffb_axis_local_config_t cfg;
    ffb_axis_local_config_default(&cfg);     /* sensible defaults first */
    cfg.degrees_of_rotation  = DOR;
    cfg.idle_spring_strength = 40;           /* auto-center when FFB is off */
    cfg.endstop_strength     = 127;          /* wall at the rotation limit  */
    cfg.damper_intensity     = 30;           /* always-on damping           */
    ffb_axis_local_t* local = ffb_axis_local_create(&cfg);

    /* 4. Optional runtime tuning of the host-effect tables (defaults already
     *    match upstream OpenFFBoard; shown here just for completeness). */
    ffb_effect_gain_t gains;
    ffb_get_effect_gains(lib, &gains);
    gains.spring = 80;                       /* a touch stronger springs */
    ffb_set_effect_gains(lib, &gains);

    /* 5. Pretend the host enabled FFB (normally a Device Control report does this). */
    ffb_set_active(lib, true);

    /* 6. Control loop: sweep the wheel as a slow sine so speed/accel are real.
     *
     * Update the metrics exactly ONCE per tick and reuse the result for both
     * the engine and the feel helper. (If you don't need the axis-local helper,
     * ffb_metrics_update_and_set(metrics, lib, 0, raw_deg) does the update and
     * feeds the engine in a single call.) */
    for (int tick = 0; tick < 5; ++tick, g_ms += 1) {
        float raw_deg = (DOR * 0.5f) * sinf((float)tick * 0.05f);

        ffb_axis_state_t st = ffb_metrics_update(metrics, raw_deg);
        ffb_set_axis_state_s(lib, 0, &st);

        /* Run the host-requested effects. */
        ffb_calculate(lib);
        int32_t host_torque = ffb_get_axis_torque(lib, 0);

        /* Add the local "feel" torque (idle spring / endstop / damper). */
        int32_t local_torque =
            ffb_axis_local_compute(local, &st, raw_deg, ffb_is_active(lib));

        int32_t total = host_torque + local_torque;
        if (total >  0x7fff) total =  0x7fff;
        if (total < -0x7fff) total = -0x7fff;
        platform_set_motor(total);

        printf("tick %d: raw=%.1f deg  host=%d  local=%d  total=%d\n",
               tick, raw_deg, host_torque, local_torque, total);
    }

    /* 7. Runtime feel adjustment (e.g. from a user menu). */
    ffb_axis_local_set_idle_spring(local, 10);          /* softer auto-center */
    ffb_axis_local_set_intensities(local, /*endstop=*/200,
                                   /*damper=*/50, /*friction=*/0, /*inertia=*/0);

    printf("done\n");
    return 0;
}
