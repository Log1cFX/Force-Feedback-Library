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
 * It then checks the situations a quick bench test never reaches: the
 * time counters wrapping, reports that are too short, an effect started
 * before its parameters arrived, and setup mistakes (too many axes, no
 * clock, a filter preset with a zero in it).
 *
 * Build via CMake: cmake -B build -DFFB_BUILD_TESTS=ON && cmake --build build
 * Run: ./build/ffb_smoke_test   (or: ctest --test-dir build)
 */

#include "ffb/ffb.h"
#include "ffb/ffb_axis_local.h"
#include "ffb/ffb_metrics.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <type_traits>

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

/* A copy of the engine would share the original's effect pool (the parser
 * keeps a reference to it), so the type must refuse to be copied. */
static_assert(!std::is_copy_constructible<ffb::Library>::value &&
              !std::is_copy_assignable<ffb::Library>::value,
              "ffb::Library must not be copyable");

/* ---- Host-side helpers: build a report and feed it to the engine ---- */

/* A report ID as it travels on the wire: the library's own ID shifted by
 * FFB_ID_OFFSET (0 unless your ffb_config.h moves the IDs). */
static uint8_t wire_id(uint8_t id) { return static_cast<uint8_t>(id + FFB_ID_OFFSET); }

template <typename Report>
static void send(ffb::Library& lib, uint8_t id, const Report& r) {
    lib.hidOut(wire_id(id), reinterpret_cast<const uint8_t*>(&r), sizeof(r));
}

/* Create New Effect, then read the block index back like a host does. */
static uint8_t host_create(ffb::Library& lib, uint8_t type, uint16_t* pool_free = nullptr) {
    ffb::FFB_CreateNewEffect_Feature_Data_t neweff{};
    neweff.effectType = type;
    send(lib, ffb::HID_ID_NEWEFREP, neweff);

    uint8_t reply[8] = {0};
    lib.hidGet(wire_id(ffb::HID_ID_BLKLDREP), reply, sizeof(reply));
    ffb::FFB_BlockLoad_Feature_Data_t bl;
    std::memcpy(&bl, reply, sizeof(bl));
    if (pool_free) *pool_free = bl.ramPoolAvailable;
    return bl.effectBlockIndex;
}

/* Set Effect on the X axis with direction 0, so the torque read back equals
 * the effect's force with its sign unchanged. duration 0 = infinite. */
static void host_set_effect(ffb::Library& lib, uint8_t idx, uint8_t type,
                            uint16_t duration_ms, uint16_t start_delay_ms = 0) {
    ffb::FFB_SetEffect_t e{};
    e.effectBlockIndex = idx;
    e.effectType       = type;
    e.duration         = duration_ms;
    e.startDelay       = start_delay_ms;
    e.gain             = 255;
    e.enableAxis       = 0x01;
    send(lib, ffb::HID_ID_EFFREP, e);
}

static void host_set_constant(ffb::Library& lib, uint8_t idx, int16_t magnitude) {
    ffb::FFB_SetConstantForce_Data_t cf{};
    cf.effectBlockIndex = idx;
    cf.magnitude        = magnitude;
    send(lib, ffb::HID_ID_CONSTREP, cf);
}

static void host_start(ffb::Library& lib, uint8_t idx) {
    ffb::FFB_EffOp_Data_t op{};
    op.effectBlockIndex = idx;
    op.state            = 1;
    send(lib, ffb::HID_ID_EFOPREP, op);
}

/* Create + configure + start a constant force in one go. */
static uint8_t host_play_constant(ffb::Library& lib, int16_t magnitude,
                                  uint16_t duration_ms = 0, uint16_t start_delay_ms = 0) {
    uint8_t idx = host_create(lib, ffb::FFB_EFFECT_CONSTANT);
    host_set_effect(lib, idx, ffb::FFB_EFFECT_CONSTANT, duration_ms, start_delay_ms);
    host_set_constant(lib, idx, magnitude);
    host_start(lib, idx);
    return idx;
}

/* Advance the fake clocks by ms milliseconds, run one tick, return the torque. */
static int32_t tick(ffb::Library& lib, uint32_t ms = 1) {
    advance(ms);
    lib.calculate();
    return lib.getAxisTorque(0);
}

/* ---- 1. The basic pipeline ------------------------------------------- */

static int test_pipeline() {
    ffb::Library lib(1, ffb::TimeSource(my_millis, my_micros));
    lib.setSendReportCallback(send_report_cb);

    /* Stationary axis. */
    lib.setAxisState(0, ffb::AxisState(0, 0.0f, 0.0f));

    /* 1. Host: Create New Effect (Feature SET, ID 0x11). */
    ffb::FFB_CreateNewEffect_Feature_Data_t neweff{};
    neweff.effectType = ffb::FFB_EFFECT_CONSTANT;
    lib.hidOut(wire_id(ffb::HID_ID_NEWEFREP), reinterpret_cast<uint8_t*>(&neweff), sizeof(neweff));

    if (!last_status_report_seen) {
        std::printf("FAIL: no status report after new effect\n"); return 1;
    }

    /* 2. Host: Feature GET on Block Load (ID 0x12). */
    uint8_t reply[64] = {0};
    uint16_t got = lib.hidGet(wire_id(ffb::HID_ID_BLKLDREP), reply, sizeof(reply));
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
    lib.hidOut(wire_id(ffb::HID_ID_EFFREP), reinterpret_cast<uint8_t*>(&set_eff), sizeof(set_eff));

    /* 4. Host: Set Constant Force (OUT, ID 5). */
    ffb::FFB_SetConstantForce_Data_t cf{};
    cf.effectBlockIndex = 1;
    cf.magnitude        = 5000;
    lib.hidOut(wire_id(ffb::HID_ID_CONSTREP), reinterpret_cast<uint8_t*>(&cf), sizeof(cf));

    /* 5. Host: Effect Operation = start. */
    ffb::FFB_EffOp_Data_t op{};
    op.effectBlockIndex = 1;
    op.state            = 1;
    lib.hidOut(wire_id(ffb::HID_ID_EFOPREP), reinterpret_cast<uint8_t*>(&op), sizeof(op));

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
    lib.hidOut(wire_id(ffb::HID_ID_EFOPREP), reinterpret_cast<uint8_t*>(&op), sizeof(op));
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
    return 0;
}

/* ---- 2. Effect timing survives both counters wrapping ---------------- */

/* Play a 50 Hz triangle and a 2 s ramp, starting at the current fake uptime,
 * and compare them with the values the waveforms must have. `when` only
 * labels the output. */
static int check_waveforms(const char* when) {
    ffb::Library lib(1, ffb::TimeSource(my_millis, my_micros));

    /* Triangle: +/-10000, 20 ms period -> 2000 per millisecond. */
    uint8_t tri = host_create(lib, ffb::FFB_EFFECT_TRIANGLE);
    host_set_effect(lib, tri, ffb::FFB_EFFECT_TRIANGLE, 0);
    ffb::FFB_SetPeriodic_Data_t per{};
    per.effectBlockIndex = tri;
    per.magnitude        = 10000;
    per.period           = 20;
    send(lib, ffb::HID_ID_PRIDREP, per);
    host_start(lib, tri);

    static const int32_t expect[20] = {
         -8000,  -6000, -4000, -2000,     0,  2000,  4000,  6000,  8000, 10000,
          8000,   6000,  4000,  2000,     0, -2000, -4000, -6000, -8000, -10000 };
    for (int i = 0; i < 60; ++i) {                 /* three periods */
        int32_t torque = tick(lib);
        if (torque != expect[i % 20]) {
            std::printf("FAIL: %s: triangle sample %d is %d, expected %d\n",
                        when, i, torque, expect[i % 20]);
            return 1;
        }
    }
    ffb::FFB_EffOp_Data_t stop{};
    stop.effectBlockIndex = tri;
    stop.state            = 3;
    send(lib, ffb::HID_ID_EFOPREP, stop);

    /* Ramp: -10000 -> +10000 over 2000 ms, so 10 per millisecond. */
    uint8_t ramp = host_create(lib, ffb::FFB_EFFECT_RAMP);
    host_set_effect(lib, ramp, ffb::FFB_EFFECT_RAMP, 2000);
    ffb::FFB_SetRamp_Data_t levels{};
    levels.effectBlockIndex = ramp;
    levels.startLevel       = static_cast<uint16_t>(-10000);
    levels.endLevel         = 10000;
    send(lib, ffb::HID_ID_RAMPREP, levels);
    host_start(lib, ramp);

    for (int ms = 500; ms <= 2000; ms += 500) {
        int32_t torque = tick(lib, 500);
        int32_t want   = -10000 + ms * 10;
        if (torque != want) {
            std::printf("FAIL: %s: ramp at %d ms is %d, expected %d\n", when, ms, torque, want);
            return 1;
        }
    }
    if (tick(lib, 10) != 0) {
        std::printf("FAIL: %s: ramp still playing after its duration\n", when);
        return 1;
    }
    std::printf("OK: triangle and ramp are correct %s\n", when);
    return 0;
}

static int test_counter_wrap() {
    /* micros() wraps every 2^32 us = 71.6 minutes. Jump to 73 minutes. */
    advance(73u * 60u * 1000u);
    if (check_waveforms("after micros() has wrapped")) return 1;

    /* An effect that is playing while micros() wraps: start 30 ms before. */
    fake_ms = 4294967u - 30u;
    fake_us = fake_ms * 1000u;
    if (check_waveforms("while micros() wraps")) return 1;

    /* millis() wraps after 2^32 ms = 49.7 days. A 1 s effect that starts
     * 100 ms before the wrap must still last exactly its second. */
    fake_ms = 0xFFFFFFFFu - 100u;
    fake_us = fake_ms * 1000u;                      /* same clock, modulo 2^32 */
    {
        ffb::Library lib(1, ffb::TimeSource(my_millis, my_micros));
        host_play_constant(lib, 5000, /*duration_ms=*/1000);
        int32_t before = tick(lib, 50);             /* 50 ms in, before the wrap */
        int32_t after  = tick(lib, 450);            /* 500 ms in, after the wrap */
        int32_t late   = tick(lib, 600);            /* 1100 ms in: over          */
        if (before != 5000 || after != 5000 || late != 0) {
            std::printf("FAIL: 1 s effect across the millis() wrap: %d, %d, then %d\n",
                        before, after, late);
            return 1;
        }
    }
    /* Same for a start delay that ends after the wrap. */
    fake_ms = 0xFFFFFFFFu - 100u;
    fake_us = fake_ms * 1000u;
    {
        ffb::Library lib(1, ffb::TimeSource(my_millis, my_micros));
        host_play_constant(lib, 5000, /*duration_ms=*/0, /*start_delay_ms=*/300);
        int32_t waiting = tick(lib, 200);           /* delay still running */
        int32_t playing = tick(lib, 200);           /* 400 ms in: started  */
        if (waiting != 0 || playing != 5000) {
            std::printf("FAIL: start delay across the millis() wrap: %d then %d\n",
                        waiting, playing);
            return 1;
        }
    }
    std::printf("OK: duration and start delay hold across the millis() wrap\n");

    if (check_waveforms("after millis() has wrapped")) return 1;
    return 0;
}

/* ---- 3. Malformed or early host traffic ------------------------------ */

static int test_host_traffic() {
    ffb::Library lib(1, ffb::TimeSource(my_millis, my_micros));
    const uint16_t slot = static_cast<uint16_t>(sizeof(ffb::Effect));

    /* Pool accounting: creating and freeing an effect 45 times (more often
     * than there are slots) must always leave the same space free. */
    for (int i = 0; i < 45; ++i) {
        uint16_t pool_free = 0;
        uint8_t idx = host_create(lib, ffb::FFB_EFFECT_CONSTANT, &pool_free);
        if (idx != 1 || pool_free != (FFB_MAX_EFFECTS - 1) * slot) {
            std::printf("FAIL: create #%d gave block %u with %u pool bytes free\n",
                        i + 1, idx, pool_free);
            return 1;
        }
        uint8_t block_free[2] = { wire_id(ffb::HID_ID_BLKFRREP), idx };
        lib.hidOut(wire_id(ffb::HID_ID_BLKFRREP), block_free, sizeof(block_free));
    }
    std::printf("OK: pool space is reported correctly after 45 create/free cycles\n");

    /* A report that is cut short must be ignored, not half-applied. */
    uint8_t idx = host_play_constant(lib, 5000);
    ffb::FFB_SetConstantForce_Data_t cf{};
    cf.effectBlockIndex = idx;
    cf.magnitude        = -20000;
    lib.hidOut(wire_id(ffb::HID_ID_CONSTREP), reinterpret_cast<const uint8_t*>(&cf), sizeof(cf) - 1);
    if (tick(lib) != 5000) {
        std::printf("FAIL: a truncated Set Constant Force report changed the torque\n");
        return 1;
    }
    /* ...and so must a report for an effect block that does not exist. */
    cf.effectBlockIndex = FFB_MAX_EFFECTS + 1;
    send(lib, ffb::HID_ID_CONSTREP, cf);
    cf.effectBlockIndex = 0;
    send(lib, ffb::HID_ID_CONSTREP, cf);
    if (tick(lib) != 5000) {
        std::printf("FAIL: a report for a non-existent effect block changed the torque\n");
        return 1;
    }
    std::printf("OK: truncated and out-of-range reports are ignored\n");

    /* A periodic effect started before its Set Periodic report arrived has
     * a period of 0. It must simply stay silent (this used to divide by 0). */
    lib.resetAllEffects();
    lib.setActive(true);
    static const uint8_t periodic[] = { ffb::FFB_EFFECT_SQUARE, ffb::FFB_EFFECT_SINE,
                                        ffb::FFB_EFFECT_TRIANGLE, ffb::FFB_EFFECT_SAWTOOTHUP,
                                        ffb::FFB_EFFECT_SAWTOOTHDOWN };
    for (uint8_t type : periodic) {
        uint8_t p = host_create(lib, type);
        host_set_effect(lib, p, type, 0);
        host_start(lib, p);
    }
    int32_t torque = tick(lib);
    if (torque != 0) {
        std::printf("FAIL: periodic effects without parameters gave torque %d\n", torque);
        return 1;
    }
    std::printf("OK: periodic effects without parameters stay silent\n");
    return 0;
}

/* ---- 4. Setup mistakes ----------------------------------------------- */

static int test_setup_mistakes() {
    /* More axes than the build has room for: clamped, not out of bounds. */
    {
        ffb::Library lib(FFB_MAX_AXIS + 1, ffb::TimeSource(my_millis, my_micros));
        if (lib.getCalculator().getAxisCount() != FFB_MAX_AXIS) {
            std::printf("FAIL: axis count %u was not clamped to %d\n",
                        lib.getCalculator().getAxisCount(), FFB_MAX_AXIS);
            return 1;
        }
        host_play_constant(lib, 5000);
        if (tick(lib) != 5000) {
            std::printf("FAIL: engine with a clamped axis count does not run\n");
            return 1;
        }
        std::printf("OK: axis count %d is clamped to FFB_MAX_AXIS=%d\n",
                    FFB_MAX_AXIS + 1, FFB_MAX_AXIS);
    }

    /* No time source at all: effects that need no clock still play. */
    {
        ffb::Library lib(1, ffb::TimeSource());
        host_play_constant(lib, 5000);
        if (tick(lib) != 5000) {
            std::printf("FAIL: engine without a time source does not run\n");
            return 1;
        }
        std::printf("OK: engine runs without a time source\n");
    }

    /* A filter preset with q = 0 must not turn into NaN coefficients. A
     * damper at a steady 50 deg/s settles toward 2031; with the heavy
     * smoothing of the lowest q it has only just started to rise. */
    {
        ffb::Library lib(1, ffb::TimeSource(my_millis, my_micros));
        lib.getCalculator().filterPreset(1).damper.q = 0;
        lib.getCalculator().setFilterProfileId(1);

        uint8_t idx = host_create(lib, ffb::FFB_EFFECT_DAMPER);
        host_set_effect(lib, idx, ffb::FFB_EFFECT_DAMPER, 0);
        ffb::FFB_SetCondition_Data_t cond{};
        cond.effectBlockIndex    = idx;
        cond.positiveCoefficient = 0x7fff;
        cond.negativeCoefficient = 0x7fff;
        cond.positiveSaturation  = 0x7fff;
        cond.negativeSaturation  = 0x7fff;
        send(lib, ffb::HID_ID_CONDREP, cond);
        host_start(lib, idx);

        int32_t torque = 0;
        for (int i = 0; i < 20; ++i) {
            lib.setAxisState(0, ffb::AxisState(0, 50.0f, 0.0f));
            torque = tick(lib);
        }
        if (torque <= 0 || torque > 2031) {
            std::printf("FAIL: damper with a q=0 filter preset gave torque %d\n", torque);
            return 1;
        }
        std::printf("OK: a filter preset with q=0 still gives a sane filter (torque %d)\n", torque);
    }
    return 0;
}

/* ---- 5. Optional helpers --------------------------------------------- */

static int test_helpers() {
    /* A wheel that is not at centre when the first sample arrives must not
     * read as a jump from 0 degrees. */
    ffb::MetricsBuilder metrics(/*degrees_of_rotation=*/900.0f, /*samplerate_hz=*/1000.0f);
    ffb::AxisState st = metrics.update(200.0f);
    for (int i = 0; i < 10; ++i) st = metrics.update(200.0f);
    if (st.speed != 0.0f || st.accel != 0.0f || st.pos_scaled_16b != 14563) {
        std::printf("FAIL: wheel at rest at 200 deg reads pos %d, speed %.1f, accel %.1f\n",
                    st.pos_scaled_16b, static_cast<double>(st.speed),
                    static_cast<double>(st.accel));
        return 1;
    }
    std::printf("OK: metrics helper starts cleanly away from centre\n");

    /* Idle spring strength written through config() must take effect. A
     * quarter turn off centre at strength 40 saturates at 40 * 35 = 1400. */
    ffb::AxisLocalEffects local;
    local.config().damper_intensity     = 0;
    local.config().idle_spring_strength = 40;
    int32_t idle = local.compute(ffb::AxisState(16384, 0.0f, 0.0f), /*ffb_on=*/false);
    if (idle != -1400) {
        std::printf("FAIL: idle spring via config() gave %d, expected -1400\n", idle);
        return 1;
    }
    std::printf("OK: idle spring strength set through config() is applied\n");

    /* End-stop: 10 degrees past either limit of a 900 degree wheel pushes
     * back toward centre with 10 * 127 * 25 = 31750 (less a rounding step). */
    ffb::MetricsBuilder wheel(900.0f, 1000.0f);
    int32_t right = local.compute(wheel.update(460.0f), true);
    wheel.reset(-460.0f);
    int32_t left  = local.compute(wheel.update(-460.0f), true);
    if (right > -31650 || right < -31800 || left != -right) {
        std::printf("FAIL: end-stop 10 deg past the limits gave %d (right) and %d (left)\n",
                    right, left);
        return 1;
    }
    /* It must push the same way even when the config still describes a
     * 900 degree wheel but the axis state comes from a 540 degree one
     * (10 degrees past its 270 degree limit here). */
    ffb::MetricsBuilder small_wheel(540.0f, 1000.0f);
    int32_t small_right = local.compute(small_wheel.update(280.0f), true);
    small_wheel.reset(-280.0f);
    int32_t small_left  = local.compute(small_wheel.update(-280.0f), true);
    if (small_right >= 0 || small_left <= 0) {
        std::printf("FAIL: end-stop with a mismatched degrees_of_rotation gave %d (right) "
                    "and %d (left)\n", small_right, small_left);
        return 1;
    }
    std::printf("OK: end-stop pushes back toward centre from both limits\n");
    return 0;
}

int main() {
    if (test_pipeline())       return 1;
    if (test_counter_wrap())   return 1;
    if (test_host_traffic())   return 1;
    if (test_setup_mistakes()) return 1;
    if (test_helpers())        return 1;

    std::printf("ALL TESTS PASSED\n");
    return 0;
}
