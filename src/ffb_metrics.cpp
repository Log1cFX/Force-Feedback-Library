/*
 * SPDX-License-Identifier: MIT
 *
 * MIT License
 *
 * Copyright (c) 2020 Yannick Richter (OpenFFBoard project and contributors)
 * Copyright (c) 2026 Santryan Raffi
 *
 * Derived from OpenFFBoard (https://github.com/Ultrawipf/OpenFFBoard).
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
 * ffb_metrics.cpp
 *
 * Speed and acceleration derivation, ported from OpenFFBoard's
 * Axis::updateMetrics().
 */

#include "ffb/ffb_metrics.h"

namespace {
template <typename T>
inline T clip_t(T v, T lo, T hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
} /* anonymous namespace */

namespace ffb {

/* Build a metrics helper for a wheel with the given travel (degrees) and
 * control-loop rate. degrees <= 0 falls back to 360, hz <= 0 to the default
 * rate. The constructor builds the speed/accel filter coefficients. */
MetricsBuilder::MetricsBuilder(float degrees, float hz, MetricsFilterPreset p)
    : degrees_of_rotation(degrees > 0.0f ? degrees : 360.0f),
      samplerate(hz > 0.0f ? hz : FFB_DEFAULT_SAMPLERATE_HZ),
      preset(p)
{
    setSamplerate(samplerate);
}

/* (Re)build the speed and accel low-pass coefficients for a new loop rate.
 * Call whenever your control rate changes. */
void MetricsBuilder::setSamplerate(float hz) {
    if (hz <= 0.0f) hz = FFB_DEFAULT_SAMPLERATE_HZ;
    samplerate = hz;
    /* presetFc()/presetQ() keep a zero freq or q from reaching the
     * coefficient math (see ffb_biquad.h). */
    speed_filter.setBiquad(BiquadType::lowpass,
                            presetFc(preset.speed, samplerate),
                            presetQ(preset.speed) / 100.0f, 0.0f);
    accel_filter.setBiquad(BiquadType::lowpass,
                            presetFc(preset.accel, samplerate),
                            presetQ(preset.accel) / 100.0f, 0.0f);
}

/* Restart the history at a known position: speed and acceleration read zero
 * until the wheel actually moves from there. */
void MetricsBuilder::reset(float pos_degrees) {
    last_pos = pos_degrees;
    last_speed_raw = 0;
    has_last_pos = true;
    /* Reinit filters to clear state. */
    speed_filter.calcBiquad();
    accel_filter.calcBiquad();
}

int32_t MetricsBuilder::scalePos(float pos_degrees) const {
    /* Faithful port of OpenFFBoard Axis::scaleEncValue(): the scaled position
     * is deliberately NOT clamped. When the wheel rotates past its travel
     * limit the value exceeds +/-0x7fff, and the optional software endstop in
     * ffb_axis_local relies on that overshoot to detect the wall. Spring
     * conditions clamp via their own saturation, so leaving it unclamped is
     * safe for them too. */
    if (degrees_of_rotation == 0.0f) {
        return 0x7fff;
    }
    return static_cast<int32_t>((0xffff / degrees_of_rotation) * pos_degrees);
}

/* Push a new raw wheel angle (degrees) and return the axis state for this tick:
 * the scaled position plus filtered speed and acceleration. Speed is the
 * per-tick position delta times the rate; accel is the speed delta times the
 * rate; both are low-pass filtered to tame the noise of differentiating an
 * encoder. */
AxisState MetricsBuilder::update(float new_pos_degrees) {
    /* The very first sample has no predecessor to difference against. Treat
     * it as the starting point (OpenFFBoard seeds its metrics with the real
     * angle too) instead of as a jump from 0 degrees, which would read as a
     * huge speed for the first few ticks. */
    if (!has_last_pos) {
        last_pos = new_pos_degrees;
        has_last_pos = true;
    }

    AxisState out;
    out.pos_scaled_16b = scalePos(new_pos_degrees);

    float speed_raw = (new_pos_degrees - last_pos) * samplerate;
    out.speed = speed_filter.process(speed_raw);

    float accel_raw = (speed_raw - last_speed_raw) * samplerate;
    out.accel = accel_filter.process(accel_raw);

    last_pos = new_pos_degrees;
    last_speed_raw = speed_raw;
    return out;
}

} /* namespace ffb */
