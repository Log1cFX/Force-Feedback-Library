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
 * minimal_cpp.cpp
 *
 * Smallest possible C++ integration of the FFB library. Replace the
 * three "USER GLUE" stubs with calls into your platform's millis(),
 * micros(), and USB stack. The library doesn't care which USB stack
 * you use - TinyUSB, ST USB Device, LUFA, anything that can deliver
 * HID Set/Get Report bytes will work.
 */

#include "ffb/ffb.h"

#include <cstdint>
#include <cstdio>

/* ---- USER GLUE 1: time source -------------------------------------- */
static uint32_t platform_millis() {
    /* Replace with HAL_GetTick() on STM32, millis() on Arduino,
       time_us_64()/1000 on RP2040 SDK, etc. */
    static uint32_t fake_ms = 0;
    return fake_ms++;
}

static uint32_t platform_micros() {
    /* Replace with the platform's high-res timer. The library only uses
       deltas, so wrap-around is fine. */
    return platform_millis() * 1000;
}

/* ---- USER GLUE 2: send status reports back to host (optional) ----- */
static bool platform_send_hid_report(const uint8_t* report, uint16_t len) {
    /* Replace with tud_hid_report(0, report, len) on TinyUSB,
       USBD_HID_SendReport(...) on ST USBD, etc.
       Return true if the report was accepted. */
    (void)report; (void)len;
    return true;
}

/* ---- USER GLUE 3: invoke library from USB callbacks --------------- */
static ffb::Library* g_lib = nullptr;

extern "C" void on_hid_set_report(uint8_t report_id,
                                   const uint8_t* buf, uint16_t len) {
    if (g_lib) g_lib->hidOut(report_id, buf, len);
}

extern "C" uint16_t on_hid_get_report(uint8_t report_id,
                                       uint8_t* reply, uint16_t maxlen) {
    return g_lib ? g_lib->hidGet(report_id, reply, maxlen) : 0;
}

/* ---- Application main --------------------------------------------- */
int main() {
    ffb::TimeSource ts;
    ts.millis = platform_millis;
    ts.micros = platform_micros;

    ffb::Library lib(/*axis_count=*/1, ts);
    lib.setSendReportCallback(platform_send_hid_report);
    g_lib = &lib;

    /* Hand the HID descriptor to your USB stack once, at startup. */
    uint16_t desc_len = 0;
    const uint8_t* desc = ffb::Library::descriptor1Axis(&desc_len);
    std::printf("FFB descriptor (1-axis): %u bytes\n", desc_len);
    (void)desc;
    uint16_t desc2_len = 0;
    (void)ffb::Library::descriptor2Axis(&desc2_len);
    std::printf("FFB descriptor (2-axis): %u bytes\n", desc2_len);

    /* Main loop: replace the busy-wait with your scheduler / timer ISR.
       The library expects calculate() to be called at the rate you passed
       to setSamplerate (default 1000 Hz). */
    for (int tick = 0; tick < 10; ++tick) {
        /* 1. Read the wheel position and feed kinematic state. */
        int32_t  wheel_pos_16b = 0;        /* -0x7fff..0x7fff */
        float    wheel_speed   = 0.0f;     /* deg/s            */
        float    wheel_accel   = 0.0f;     /* deg/s^2          */

        ffb::AxisState s;
        s.pos_scaled_16b = wheel_pos_16b;
        s.speed          = wheel_speed;
        s.accel          = wheel_accel;
        lib.setAxisState(0, s);

        /* 2. Crunch effects. */
        lib.calculate();

        /* 3. Read out torque and drive the motor. */
        int32_t torque = lib.getAxisTorque(0);  /* -0x7fff..0x7fff */
        /* my_set_motor_pwm(torque); */
        (void)torque;
    }

    return 0;
}
