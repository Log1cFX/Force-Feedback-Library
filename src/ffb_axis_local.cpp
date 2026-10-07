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
 * ffb_axis_local.cpp
 *
 * Logic ported from OpenFFBoard Axis::calculateAxisEffects() and
 * Axis::updateEndstop(), with the same formulas. Two things are arranged
 * differently so that a setup mistake cannot turn into a wrong force: every
 * tunable is read from the config on each tick (the original caches the
 * idle-spring scale), and the end-stop takes its overshoot from the scaled
 * position alone (the original pairs it with a second copy of the angle).
 */

#include "ffb/ffb_axis_local.h"

#include <cmath>

#ifndef M_PI
#  define M_PI 3.14159265358979323846f
#endif

namespace {

template <typename T>
inline T clip_t(T v, T lo, T hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Match OpenFFBoard's INTERNAL_*_SCALER (effects calculator side). */
constexpr float AXIS_DAMPER_RATIO  = 40.0f * 0.7f / 255.0f;   /* DAMPER * 0.7 / 255 */
constexpr float AXIS_INERTIA_RATIO = 4.0f  * 0.7f / 255.0f;   /* INERTIA * 0.7 / 255 */
constexpr float INTERNAL_AXIS_FRICTION_SCALER = 0.7f;
constexpr int32_t INT_FX_CLIP = 20000;
constexpr float ENDSTOP_GAIN = 25.0f;

constexpr int32_t INTERNAL_SCALER_FRICTION_LOCAL = 45;

/* Like a clamp, but reports WHICH bound was exceeded instead of clamping:
 * -1 below lo, +1 above hi, 0 in range. The end-stop uses it to detect which
 * wall the wheel hit (and therefore which way to push back). */
template <typename T>
inline int8_t cliptest(T v, T lo, T hi) {
    if (v < lo) return -1;
    if (v > hi) return 1;
    return 0;
}

} /* anonymous namespace */

namespace ffb {

/* Construct from a config: build the damper/friction/inertia filters. */
AxisLocalEffects::AxisLocalEffects(const AxisLocalConfig& c) : cfg(c) {
    setSamplerate(c.samplerate_hz);
}

/* Set the idle-spring strength. Equivalent to writing
 * config().idle_spring_strength: both take effect on the next compute(). */
void AxisLocalEffects::setIdleSpringStrength(uint8_t strength) {
    cfg.idle_spring_strength = strength;
}

/* (Re)build the damper/friction/inertia low-pass coefficients for a new
 * control-loop rate. presetFc()/presetQ() keep a zero freq or q from reaching
 * the coefficient math (see ffb_biquad.h). */
void AxisLocalEffects::setSamplerate(float hz) {
    if (hz <= 0.0f) hz = FFB_DEFAULT_SAMPLERATE_HZ;
    cfg.samplerate_hz = hz;
    damper_filter.setBiquad(BiquadType::lowpass,
                             presetFc(cfg.damper_filter, hz),
                             presetQ(cfg.damper_filter) / 100.0f, 0.0f);
    friction_filter.setBiquad(BiquadType::lowpass,
                               presetFc(cfg.friction_filter, hz),
                               presetQ(cfg.friction_filter) / 100.0f, 0.0f);
    inertia_filter.setBiquad(BiquadType::lowpass,
                              presetFc(cfg.inertia_filter, hz),
                              presetQ(cfg.inertia_filter) / 100.0f, 0.0f);
}

/* Idle spring: a gentle auto-centering force, used only while host FFB is off.
 * Proportional to -position (pulls back toward center), clamped so it never
 * exceeds a comfortable strength. Scale and clamp follow from the strength as
 * in Axis::setIdleSpringStrength; they are worked out here, from the current
 * config value, so a strength written through config() is not ignored. */
int32_t AxisLocalEffects::updateIdleSpring(int32_t pos_scaled_16b) const {
    const int32_t strength = cfg.idle_spring_strength;
    const int32_t clip     = clip_t<int32_t>(strength * 35, 0, 10000);
    const float   scale    = 0.5f + (static_cast<float>(strength) * 0.01f);
    int32_t f = static_cast<int32_t>(-pos_scaled_16b * scale);
    return clip_t<int32_t>(f, -clip, clip);
}

/* Software end-stop: 0 while the wheel is inside its travel range, otherwise a
 * stiff restoring torque proportional to how many degrees it has overshot the
 * limit, directed back toward center and clamped to +/-0x7fff. Relies on
 * pos_scaled_16b exceeding +/-0x7fff past the limit, which is why the metrics
 * helper leaves the scaled position un-clamped.
 *
 * Both the side and the size of the overshoot come from pos_scaled_16b, whose
 * limits are +/-0x7fff by definition. The overshoot is therefore never
 * negative and the torque can only point back into the range, whatever the
 * sign convention of the encoder and whatever degrees_of_rotation says (a
 * wrong value there only makes the wall softer or harder). */
int32_t AxisLocalEffects::updateEndstop(int32_t pos_scaled_16b) const {
    int8_t clipdir = cliptest<int32_t>(pos_scaled_16b, -0x7fff, 0x7fff);
    if (clipdir == 0) return 0;
    /* Counts past the limit, then degrees: 0xffff counts span the travel. */
    float addtorque = static_cast<float>(clipdir * pos_scaled_16b - 0x7fff) *
                      (cfg.degrees_of_rotation / 65535.0f);
    addtorque *= static_cast<float>(cfg.endstop_strength) * ENDSTOP_GAIN;
    addtorque *= -clipdir;
    return clip_t<int32_t>(static_cast<int32_t>(addtorque), -0x7fff, 0x7fff);
}

/* Sum every axis-local "feel" effect into one torque to ADD on top of the host
 * torque: idle spring (only when FFB is off) + always-on damper/inertia/friction
 * + end-stop, clamped to +/-0x7fff. Each intensity of 0 skips that effect. The
 * caller is responsible for adding this to getAxisTorque() and clamping again.
 * pos_degrees is not read any more (see updateEndstop); the parameter stays so
 * existing callers keep compiling. */
int32_t AxisLocalEffects::compute(const AxisState& m, float pos_degrees, bool ffb_on) {
    (void)pos_degrees;
    int32_t axisEffectTorque = 0;

    if (!ffb_on) {
        axisEffectTorque += updateIdleSpring(m.pos_scaled_16b);
    }

    if (cfg.damper_intensity != 0) {
        float speedFiltered = m.speed * static_cast<float>(cfg.damper_intensity) * AXIS_DAMPER_RATIO;
        speedFiltered = clip_t<float>(speedFiltered,
                                       static_cast<float>(-INT_FX_CLIP),
                                       static_cast<float>(INT_FX_CLIP));
        axisEffectTorque -= static_cast<int32_t>(damper_filter.process(speedFiltered));
    }

    if (cfg.inertia_intensity != 0) {
        float accelFiltered = m.accel * static_cast<float>(cfg.inertia_intensity) * AXIS_INERTIA_RATIO;
        accelFiltered = clip_t<float>(accelFiltered,
                                       static_cast<float>(-INT_FX_CLIP),
                                       static_cast<float>(INT_FX_CLIP));
        axisEffectTorque -= static_cast<int32_t>(inertia_filter.process(accelFiltered));
    }

    if (cfg.friction_intensity != 0) {
        float speed = m.speed * static_cast<float>(INTERNAL_SCALER_FRICTION_LOCAL);
        float speedRampupCeil = 4096.0f;
        float rampupFactor = 1.0f;
        if (std::fabs(speed) < speedRampupCeil) {
            float phaseRad = static_cast<float>(M_PI) *
                              ((std::fabs(speed) / speedRampupCeil) - 0.5f);
            rampupFactor = (1.0f + std::sin(phaseRad)) / 2.0f;
        }
        int8_t sign = speed >= 0 ? 1 : -1;
        float force = static_cast<float>(cfg.friction_intensity) * rampupFactor *
                       sign * INTERNAL_AXIS_FRICTION_SCALER * 32.0f;
        force = clip_t<float>(force,
                               static_cast<float>(-INT_FX_CLIP),
                               static_cast<float>(INT_FX_CLIP));
        axisEffectTorque -= static_cast<int32_t>(friction_filter.process(force));
    }

    /* Endstop. */
    axisEffectTorque += updateEndstop(m.pos_scaled_16b);

    return clip_t<int32_t>(axisEffectTorque, -0x7fff, 0x7fff);
}

} /* namespace ffb */
