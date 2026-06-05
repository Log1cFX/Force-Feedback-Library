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
 * smoke_test.cpp
 *
 * End-to-end smoke test that drives a constant force effect through
 * the full pipeline (host -> parser -> calculator -> torque) and
 * verifies the resulting torque value. Run as a sanity check after
 * porting the library to a new platform.
 *
 * Build via CMake: cmake -B build -DFFB_BUILD_TESTS=ON && cmake --build build
 * Run: ./build/ffb_smoke_test
 */

#include "ffb/ffb.h"

#include <cstdint>
#include <cstdio>
#include <cstring>

static uint32_t fake_ms = 0;
static uint32_t fake_us = 0;
static uint32_t my_millis() { return fake_ms; }
static uint32_t my_micros() { return fake_us; }
static void advance(uint32_t ms) { fake_ms += ms; fake_us += ms * 1000; }

static bool last_status_report_seen = false;
static bool send_report_cb(const uint8_t* /*buf*/, uint16_t /*len*/) {
    last_status_report_seen = true;
    return true;
}

int main() {
    ffb::Library lib(1, ffb::TimeSource(my_millis, my_micros));
    lib.setSendReportCallback(send_report_cb);

    /* Stationary axis. */
    lib.setAxisState(0, ffb::AxisState(0, 0.0f, 0.0f));

    /* 1. Host: Create New Effect (Feature SET, ID 0x11). */
    ffb::FFB_CreateNewEffect_Feature_Data_t neweff{};
    neweff.effectType = ffb::FFB_EFFECT_CONSTANT;
    lib.hidOut(ffb::HID_ID_NEWEFREP, reinterpret_cast<uint8_t*>(&neweff), sizeof(neweff));

    if (!last_status_report_seen) {
        std::printf("FAIL: no status report after new effect\n"); return 1;
    }

    /* 2. Host: Feature GET on Block Load (ID 0x12). */
    uint8_t reply[64] = {0};
    uint16_t got = lib.hidGet(ffb::HID_ID_BLKLDREP, reply, sizeof(reply));
    if (got != sizeof(ffb::FFB_BlockLoad_Feature_Data_t)) {
        std::printf("FAIL: blockload reply size %u\n", got); return 1;
    }
    ffb::FFB_BlockLoad_Feature_Data_t bl;
    std::memcpy(&bl, reply, sizeof(bl));
    if (bl.effectBlockIndex != 1 || bl.loadStatus != 1) {
        std::printf("FAIL: blockload idx=%u status=%u\n",
                    bl.effectBlockIndex, bl.loadStatus); return 1;
    }
    std::printf("OK: effect allocated at block index %u\n", bl.effectBlockIndex);

    /* 3. Host: Set Effect (OUT, ID 1). enableAxis=0x01 (X only).
     * directionX = 0 means the unit vector for X = -1 (linear projection). */
    ffb::FFB_SetEffect_t set_eff{};
    set_eff.effectBlockIndex = 1;
    set_eff.effectType       = ffb::FFB_EFFECT_CONSTANT;
    set_eff.duration         = 0;
    set_eff.gain             = 255;
    set_eff.enableAxis       = 0x01;
    set_eff.directionX       = 0;
    set_eff.directionY       = 18000;
    lib.hidOut(ffb::HID_ID_EFFREP, reinterpret_cast<uint8_t*>(&set_eff), sizeof(set_eff));

    /* 4. Host: Set Constant Force (OUT, ID 5). */
    ffb::FFB_SetConstantForce_Data_t cf{};
    cf.effectBlockIndex = 1;
    cf.magnitude        = 5000;
    lib.hidOut(ffb::HID_ID_CONSTREP, reinterpret_cast<uint8_t*>(&cf), sizeof(cf));

    /* 5. Host: Effect Operation = start. */
    ffb::FFB_EffOp_Data_t op{};
    op.effectBlockIndex = 1;
    op.state            = 1;
    lib.hidOut(ffb::HID_ID_EFOPREP, reinterpret_cast<uint8_t*>(&op), sizeof(op));

    /* 6. Tick the calculator. */
    advance(10);
    lib.calculate();
    int32_t torque = lib.getAxisTorque(0);

    /* Math:
     *   forceVector = 5000 * (gain=255) / 255   = 5000
     *   axisforce   = -forceVector * (-1)        = 5000
     *   torque      = 5000 * global_gain(255) / 255 = 5000
     */
    if (torque != 5000) {
        std::printf("FAIL: expected torque 5000, got %d\n", torque); return 1;
    }
    std::printf("OK: constant force 5000 -> torque %d\n", torque);

    /* 7. Stop. */
    op.state = 3;
    lib.hidOut(ffb::HID_ID_EFOPREP, reinterpret_cast<uint8_t*>(&op), sizeof(op));
    lib.calculate();
    torque = lib.getAxisTorque(0);
    if (torque != 0) {
        std::printf("FAIL: after stop expected 0, got %d\n", torque); return 1;
    }
    std::printf("OK: after stop, torque %d\n", torque);

    /* 8. Descriptor byte counts should match upstream OpenFFBoard. */
    uint16_t len1 = 0, len2 = 0;
    (void)ffb::Library::descriptor1Axis(&len1);
    (void)ffb::Library::descriptor2Axis(&len2);
    if (len1 != 1196) { std::printf("FAIL: 1-axis desc %u (want 1196)\n", len1); return 1; }
    if (len2 != 1215) { std::printf("FAIL: 2-axis desc %u (want 1215)\n", len2); return 1; }
    std::printf("OK: descriptor sizes 1-axis=%u 2-axis=%u match upstream\n", len1, len2);

    std::printf("ALL TESTS PASSED\n");
    return 0;
}
