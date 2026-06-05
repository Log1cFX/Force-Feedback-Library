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
 * tinyusb_glue.cpp
 *
 * Sample wiring of the FFB library to TinyUSB callbacks. Drop this in
 * a TinyUSB-based project (RP2040 SDK, ESP32-S3 Arduino, Pico Arduino,
 * any ST/NXP/Nordic project that bundles TinyUSB) and the library will
 * receive the host's FFB reports automatically.
 *
 * The HID descriptor returned by ffb::Library::descriptor1Axis() must
 * be returned from tud_hid_descriptor_report_cb().
 */

#include "ffb/ffb.h"

/* Forward declarations of TinyUSB types/functions so this file compiles
 * without the TinyUSB headers present (replace with `#include "tusb.h"`
 * in a real integration). */
typedef enum {
    HID_REPORT_TYPE_INVALID = 0,
    HID_REPORT_TYPE_INPUT,
    HID_REPORT_TYPE_OUTPUT,
    HID_REPORT_TYPE_FEATURE
} hid_report_type_t;

extern "C" bool tud_hid_report(uint8_t instance, const void* report, uint16_t len);

/* ---- User-supplied time helpers ----------------------------------- */
extern "C" uint32_t board_millis(void);   /* TinyUSB ships this helper */
extern "C" uint32_t time_us_32(void);     /* RP2040 SDK; substitute as needed */

/* ---- Single FFB instance ------------------------------------------ */
namespace {
ffb::Library& get_lib() {
    static ffb::Library s_lib(/*axes=*/1, { board_millis, time_us_32 });
    return s_lib;
}

bool send_report(const uint8_t* buf, uint16_t len) {
    return tud_hid_report(0, buf, len);
}
} /* anonymous namespace */

extern "C" {

/* Called by TinyUSB to fetch the report descriptor during enumeration. */
const uint8_t* tud_hid_descriptor_report_cb(uint8_t /*itf*/) {
    uint16_t len;
    return ffb::Library::descriptor1Axis(&len);
}

/* Host -> device data (OUT pipe + control-pipe Set Report). */
void tud_hid_set_report_cb(uint8_t /*itf*/, uint8_t report_id,
                            hid_report_type_t report_type,
                            const uint8_t* buffer, uint16_t bufsize) {
    /* The first byte of an OUT report received on the interrupt EP
     * with no report_id may be the report ID itself - mirror the
     * original OpenFFBoard fix-up. */
    if ((report_type == HID_REPORT_TYPE_INVALID ||
         report_type == HID_REPORT_TYPE_OUTPUT) && report_id == 0) {
        if (bufsize > 0) report_id = buffer[0];
    }
    get_lib().hidOut(report_id, buffer, bufsize);
}

/* Device -> host feature reply (control-pipe Get Report). */
uint16_t tud_hid_get_report_cb(uint8_t /*itf*/, uint8_t report_id,
                                hid_report_type_t /*report_type*/,
                                uint8_t* buffer, uint16_t reqlen) {
    return get_lib().hidGet(report_id, buffer, reqlen);
}

/* Application-side: call this at FFB rate (e.g. from a 1 kHz timer ISR). */
void ffb_tick(int32_t wheel_pos_16b, float speed_dps, float accel_dpss) {
    ffb::Library& lib = get_lib();
    static bool once = false;
    if (!once) { lib.setSendReportCallback(send_report); once = true; }

    ffb::AxisState s;
    s.pos_scaled_16b = wheel_pos_16b;
    s.speed          = speed_dps;
    s.accel          = accel_dpss;
    lib.setAxisState(0, s);

    lib.calculate();
}

int32_t ffb_get_torque(void) {
    return get_lib().getAxisTorque(0);
}

} /* extern "C" */
