/*
 * SPDX-License-Identifier: MIT
 *
 * MIT License
 *
 * Copyright (c) 2026 Santryan Raffi
 *
 * Part of a standalone force-feedback library derived from OpenFFBoard
 * (https://github.com/Ultrawipf/OpenFFBoard).
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
 * minimal_c.c
 *
 * Same as minimal_cpp.cpp but using the C API. The library can be
 * driven entirely from C code; the C++ internals are hidden.
 */

#include "ffb/ffb_c.h"

#include <stdint.h>
#include <stdio.h>

/* ---- USER GLUE: time source ---------------------------------------- */
static uint32_t platform_millis(void) {
    static uint32_t fake_ms = 0;
    return fake_ms++;
}
static uint32_t platform_micros(void) {
    return platform_millis() * 1000;
}

/* ---- USER GLUE: USB send (optional) ------------------------------- */
static bool platform_send_hid_report(const uint8_t* report, uint16_t len) {
    (void)report; (void)len;
    return true;
}

int main(void) {
    ffb_lib_t* lib = ffb_create(/*axis_count=*/1, platform_millis, platform_micros);
    ffb_set_send_report_callback(lib, platform_send_hid_report);

    /* Get the HID descriptor for your USB stack. */
    uint16_t desc_len = 0;
    const uint8_t* desc = ffb_descriptor_1axis(&desc_len);
    printf("FFB descriptor: %u bytes\n", desc_len);
    (void)desc;

    /* Main loop. */
    for (int tick = 0; tick < 10; ++tick) {
        /* Feed kinematic state (replace with real measurements). */
        ffb_set_axis_state(lib, 0,
                            /*pos_scaled_16b=*/0,
                            /*speed=*/0.0f,
                            /*accel=*/0.0f);

        ffb_calculate(lib);
        int32_t torque = ffb_get_axis_torque(lib, 0);
        (void)torque;
        /* my_set_motor_pwm(torque); */
    }

    return 0;
}
