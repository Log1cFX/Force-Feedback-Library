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
 * ffb_calculator.cpp
 *
 * Force calculation. The effect-math functions follow OpenFFBoard's
 * EffectsCalculator.cpp formula for formula. The changes are:
 *
 *   - Axis input/output goes through internal arrays instead of an Axis
 *     vector (so the file has no Axis.h dependency).
 *   - HAL_GetTick() / micros() -> the user's TimeSource, read once per tick
 *     (sampleClocks). An effect's age is the unsigned difference to its
 *     startTime, and the periodic generators reduce it modulo the period in
 *     integer arithmetic before anything is converted to float. The original
 *     subtracts the raw counters in floating point instead, which loses the
 *     waveform once micros() has wrapped (about 71 minutes of uptime).
 *   - Single-precision float throughout, where the original lets a few
 *     terms promote to double. A deliberate trade for speed: results can
 *     differ from the firmware's by a rounding step.
 *   - effect->filter[i] is now an inline Biquad (not unique_ptr); presence
 *     is signalled by effect->filter_active[i].
 *   - setFilters() uses a plain switch instead of std::function lambdas.
 *   - Statistics, monitor thread, LED hook, persistence stripped.
 */

#include "ffb/ffb_calculator.h"

#include <cmath>
#include <cstdlib>

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

constexpr uint8_t EFFECT_STATE_INACTIVE = 0;

/* The longest start delay a Set Effect report can carry, in ms. */
constexpr uint32_t MAX_START_DELAY_MS = 0xFFFF;

/* micros() may run this far ahead of millis() * 1000 and still count as the
 * same clock. With a 1 kHz millis() the gap stays below one millisecond; the
 * margin is for a millis() that advances in coarser steps (a 10 ms RTOS tick,
 * say) or ticks late. */
constexpr uint32_t MAX_CLOCK_GAP_US = 50000;

/* Where a periodic effect is inside its current period, in whole
 * milliseconds: (time since start + phase offset) mod period. Whole periods
 * carry no information, so they are dropped from the millisecond count
 * first; that keeps the microsecond sum inside 32 bits however long the
 * effect has been playing. period_ms must not be 0. */
inline uint32_t periodPositionMs(uint32_t elapsed_ms, uint32_t sub_us,
                                 uint32_t period_ms, float phasetime_ms) {
    const int32_t period_us = static_cast<int32_t>(period_ms * 1000u);
    int32_t pos_us = static_cast<int32_t>((elapsed_ms % period_ms) * 1000u + sub_us) +
                     static_cast<int32_t>(phasetime_ms * 1000.0f);
    pos_us %= period_us;
    if (pos_us < 0) pos_us += period_us;     /* a negative phase offset */
    return static_cast<uint32_t>(pos_us) / 1000u;
}

} /* anonymous namespace */

namespace ffb {

/* Construct with the axis count and the user's time source. Allocates nothing:
 * the effect pool and every per-effect filter are members of this object. An
 * axis count the build cannot hold (see FFB_MAX_AXIS) is clamped, because the
 * per-axis arrays are sized by it. Nothing is logged from here: a global
 * instance is constructed before main(), when the log sink may not be up yet;
 * setActive() reports the setup instead. */
Calculator::Calculator(uint8_t axis_count_, TimeSource ts)
    : axis_count(clip_t<uint8_t>(axis_count_, 1, FFB_MAX_AXIS)),
      axis_count_clamped(axis_count != axis_count_),
      time_source(ts)
{}

/* Enable/disable the engine. Switching it on is also where the setup is
 * reported to the debug log, problems included. */
void Calculator::setActive(bool a) {
    if (!a && effects_active) {
        FFB_LOG("FFB: engine off\n");
    }
    if (a && !effects_active) {
        FFB_LOG("FFB: engine on (axes %d, effect slots %d, %d Hz)\n",
                axis_count, FFB_MAX_EFFECTS, static_cast<int>(calcfrequency));
        if (axis_count_clamped) {
            FFB_LOG("FFB: requested axis count is outside 1..%d, using %d\n",
                    FFB_MAX_AXIS, axis_count);
        }
        if (!time_source.millis) {
            FFB_LOG("FFB: no millis() source - timed effects will not advance\n");
        } else if (!time_source.micros) {
            FFB_LOG("FFB: no micros() source - effects are timed in whole milliseconds\n");
        }
    }
    effects_active = a;
}

void Calculator::setSamplerate(float hz) {
    if (hz <= 0) hz = FFB_DEFAULT_SAMPLERATE_HZ;
    this->calcfrequency = hz;
    FFB_LOG("FFB: samplerate %d Hz\n", static_cast<int>(hz));
    /* Match EffectsCalculator::updateSamplerate(): rebuild filters on every
     * effect that already has one. */
    for (Effect& e : effects) {
        if (e.filter_active[0]) {
            setFilters(&e);
        }
    }
}

/* Friction ramp-up threshold, as a percent (0..100) of full speed. Below this
 * speed the friction force is eased in with a half-sine (see calcComponentForce). */
void Calculator::setFrictionRampupPct(uint8_t pct) {
    frictionPctSpeedToRampup = clip_t<uint8_t>(pct, 0, 100);
}

/* Select the active filter profile (0 = default presets, 1 = custom) and
 * rebuild coefficients on every live damper/friction/inertia effect. */
void Calculator::setFilterProfileId(uint8_t id) {
    filterProfileId = clip_t<uint8_t>(id, 0, 1);
    FFB_LOG("FFB: filter profile %d\n", filterProfileId);
    updateFiltersForType(FFB_EFFECT_DAMPER);
    updateFiltersForType(FFB_EFFECT_FRICTION);
    updateFiltersForType(FFB_EFFECT_INERTIA);
}

/* Access one of the two filter-coefficient presets for editing (index clamped
 * to a valid value). Profile 0 is the defaults, profile 1 the custom slot. */
EffectFilterPreset& Calculator::filterPreset(uint8_t profile) {
    if (profile > 1) profile = 1;
    return filter_presets[profile];
}

/* Store the latest position/speed/accel for an axis (bounds-checked). The
 * calculator reads this on the next calculate(). */
void Calculator::setAxisState(uint8_t axis, const AxisState& s) {
    if (axis < FFB_MAX_AXIS) {
        axis_state[axis] = s;
    }
}

/* Return the torque computed for an axis by the last calculate() (0 if the
 * axis index is out of range). */
int32_t Calculator::getAxisTorque(uint8_t axis) const {
    if (axis < FFB_MAX_AXIS) {
        return axis_torque[axis];
    }
    return 0;
}

/* Find the first free pool slot for a new effect of the given type. Returns the
 * 0-based index, or -1 if the type is invalid or the pool is full. */
int32_t Calculator::findFreeEffect(uint8_t type) {
    if (type > FFB_EFFECT_NONE && type < FFB_EFFECT_CUSTOM + 1) {
        for (uint8_t i = 0; i < effects.size(); ++i) {
            if (effects[i].type == FFB_EFFECT_NONE) {
                return i;
            }
        }
    }
    return -1;
}

/* Return a slot to the pool: reset it to a default (free) Effect and mark its
 * per-axis filters inactive. */
void Calculator::freeEffect(uint16_t idx) {
    if (idx < effects.size()) {
        effects[idx] = Effect();  /* Reset all fields */
        for (int i = 0; i < FFB_MAX_AXIS; ++i) {
            effects[idx].filter_active[i] = false;
        }
    }
}

/* ----------- Main per-tick loop ------------------------------------- */

/* Read both clocks for this tick. The effects get the millisecond counter
 * plus the part of the microsecond counter that lies past it, which is what
 * gives the ramp and periodic waveforms their sub-millisecond resolution.
 * The subtraction is unsigned, so either counter may wrap. When micros() is
 * missing or is not the same clock as millis() the difference means nothing
 * and is dropped: effects are then timed in whole milliseconds. */
Calculator::TickTime Calculator::sampleClocks() {
    TickTime now;
    now.ms     = millisNow();
    now.sub_us = 0;
    if (time_source.micros) {
        now.sub_us = time_source.micros() - now.ms * 1000u;
        if (now.sub_us >= MAX_CLOCK_GAP_US) {
            if (!clock_mismatch_logged) {
                clock_mismatch_logged = true;
                FFB_LOG("FFB: micros() does not track millis() (%lu us apart) - "
                        "timing such ticks in whole milliseconds\n",
                        static_cast<unsigned long>(now.sub_us));
            }
            now.sub_us = 0;
        }
    }
    now.sub_ms = static_cast<float>(now.sub_us) * 0.001f;
    return now;
}

/* Compute the torque for every axis this tick: zero the outputs, bail out if
 * FFB is disabled, then walk the effect pool - skipping effects still inside
 * their start delay, expiring finished ones, computing each one's base force,
 * projecting it onto every axis and summing - and finally clamp each axis sum
 * to +/-0x7fff. This is the engine's heartbeat. */
void Calculator::calculate() {
    /* No active state -> zero all torques. */
    for (uint8_t a = 0; a < axis_count; ++a) {
        axis_torque[a] = 0;
    }

    if (!isActive()) {
        return;
    }

    int32_t force = 0;
    int32_t forces[FFB_MAX_AXIS] = {0};

    const TickTime now = sampleClocks();

    for (uint8_t fxi = 0; fxi < FFB_MAX_EFFECTS; ++fxi) {
        Effect* effect = &effects[fxi];

        if (effect->state == EFFECT_STATE_INACTIVE) {
            continue;
        }

        /* Not infinite (0 or 0xffff)? */
        const bool finite = effect->duration != FFB_EFFECT_DURATION_INFINITE &&
                            effect->duration != 0;

        /* Start delay: the effect waits while its start time lies ahead of
         * the clock. "Ahead" is read as "by no more than the longest delay a
         * report can ask for", which keeps the test valid across a counter
         * wrap. An infinite effect without a delay has nothing to wait for
         * and is never subject to the clock at all. */
        if ((finite || effect->startDelay != 0) &&
            static_cast<uint32_t>(effect->startTime - now.ms - 1u) < MAX_START_DELAY_MS)
        {
            continue;
        }

        /* Unsigned difference: correct whether or not the counter wrapped
         * between the start of the effect and now. */
        const uint32_t elapsed_ms = now.ms - effect->startTime;

        if (finite && elapsed_ms > effect->duration) {
            effect->state = EFFECT_STATE_INACTIVE;
            FFB_LOG("FFB: effect finished, at index %d\n", fxi + 1);
            continue;
        }

        force = calcNonConditionEffectForce(effect, elapsed_ms, now);

        for (uint8_t axis = 0; axis < axis_count; ++axis) {
            int32_t axisforce = calcComponentForce(effect, force, axis);
            forces[axis] += axisforce;
        }
    }

    for (uint8_t a = 0; a < axis_count; ++a) {
        axis_torque[a] = clip_t<int32_t>(forces[a], -0x7fff, 0x7fff);
    }
}

/* ----------- Non-condition effects (constant/ramp/periodic) --------- */

/* Produce the scalar "force vector" for a constant, ramp or periodic effect
 * (condition effects are handled in calcComponentForce instead). Applies the
 * envelope if any, generates the waveform for the effect's type, then scales by
 * the effect's own gain. Returns 0 for unsupported types.
 *
 * Time comes in as elapsed_ms (whole milliseconds since the effect started)
 * plus now.sub_us / now.sub_ms (the fraction of the current millisecond). */
int32_t Calculator::calcNonConditionEffectForce(Effect* effect, uint32_t elapsed_ms,
                                                const TickTime& now) {
    int32_t force_vector = 0;
    int32_t magnitude = effect->magnitude;

    if (effect->useEnvelope) {
        magnitude = getEnvelopeMagnitude(effect, elapsed_ms);
    }

    switch (effect->type) {

    case FFB_EFFECT_CONSTANT: {
        force_vector = static_cast<int32_t>(magnitude);
        break;
    }

    case FFB_EFFECT_RAMP: {
        float elapsed_time = static_cast<float>(elapsed_ms) + now.sub_ms;
        int32_t duration = effect->duration;
        /* A ramp ends at its end level: past the duration it holds that
         * level instead of running on (only an infinite-duration ramp can
         * get there; a finite one has expired by then). */
        if (elapsed_time > static_cast<float>(duration)) {
            elapsed_time = static_cast<float>(duration);
        }
        /* Evaluate the interpolation in floating point and truncate only
         * the final sum, exactly like OpenFFBoard. Casting elapsed_time to
         * int first would quantise the ramp to whole milliseconds. */
        force_vector = static_cast<int32_t>(
            static_cast<float>(effect->startLevel) +
            (elapsed_time * (effect->endLevel - effect->startLevel)) / duration);
        break;
    }

    case FFB_EFFECT_SQUARE: {
        uint32_t elapsed_time = elapsed_ms;
        int32_t force = ((elapsed_time + effect->phase) %
                         (static_cast<uint32_t>(effect->period) + 2))
                        < (static_cast<uint32_t>(effect->period) + 2) / 2
                          ? -magnitude : magnitude;
        force_vector = force + effect->offset;
        break;
    }

    case FFB_EFFECT_TRIANGLE: {
        int32_t force = 0;
        int32_t offset = effect->offset;
        uint32_t phase = effect->phase;
        uint32_t period = effect->period;
        if (period == 0) period = 1;    /* started before any Set Periodic */
        float periodF = period;

        int32_t maxMagnitude = offset + magnitude;
        int32_t minMagnitude = offset - magnitude;
        float phasetime = (phase * period) / 35999.0f;
        float remainder = static_cast<float>(
            periodPositionMs(elapsed_ms, now.sub_us, period, phasetime));
        float slope = ((maxMagnitude - minMagnitude) * 2) / periodF;
        if (remainder > (periodF / 2)) {
            force = static_cast<int32_t>(slope * (periodF - remainder));
        } else {
            force = static_cast<int32_t>(slope * remainder);
        }
        force += minMagnitude;
        force_vector = force;
        break;
    }

    case FFB_EFFECT_SAWTOOTHUP: {
        float offset = effect->offset;
        uint32_t phase = effect->phase;
        uint32_t period = effect->period;
        if (period == 0) period = 1;    /* started before any Set Periodic */
        float periodF = period;

        float maxMagnitude = offset + magnitude;
        float minMagnitude = offset - magnitude;
        float phasetime = (phase * period) / 35999.0f;
        float remainder = static_cast<float>(
            periodPositionMs(elapsed_ms, now.sub_us, period, phasetime));
        float slope = (maxMagnitude - minMagnitude) / periodF;
        force_vector = static_cast<int32_t>(minMagnitude +
                                            slope * (period - remainder));
        break;
    }

    case FFB_EFFECT_SAWTOOTHDOWN: {
        float offset = effect->offset;
        float phase = effect->phase;
        uint32_t period = effect->period;
        if (period == 0) period = 1;    /* started before any Set Periodic */
        float periodF = period;

        float maxMagnitude = offset + magnitude;
        float minMagnitude = offset - magnitude;
        float phasetime = (phase * period) / 35999.0f;
        float remainder = static_cast<float>(
            periodPositionMs(elapsed_ms, now.sub_us, period, phasetime));
        float slope = (maxMagnitude - minMagnitude) / periodF;
        force_vector = static_cast<int32_t>(minMagnitude + slope * remainder);
        break;
    }

    case FFB_EFFECT_SINE: {
        uint16_t period_for_freq = effect->period;
        if (period_for_freq < 2) period_for_freq = 2;
        /* sin() repeats every period, so only the time inside the current
         * period is kept. That leaves t small enough for float to hold it
         * exactly, however long the effect has been playing. */
        float t = static_cast<float>(elapsed_ms % period_for_freq) + now.sub_ms;
        float freq = 1.0f / static_cast<float>(period_for_freq);
        float phase = static_cast<float>(effect->phase) / 35999.0f;
        float sine = std::sin(2.0f * static_cast<float>(M_PI) *
                              (t * freq + phase)) * magnitude;
        force_vector = static_cast<int32_t>(effect->offset + sine);
        break;
    }

    default:
        return 0;
    }

    return (force_vector * effect->gain) / 255;
}

/* ----------- Per-axis routing and condition effects ----------------- */

/* Turn an effect into the torque it contributes to ONE axis. For constant and
 * periodic effects this projects the precomputed forceVector onto the axis
 * (low-pass filtering constant force first). For condition effects it ignores
 * forceVector, reads the axis state (position/speed/accel) directly, and applies
 * the condition law. The result is finally scaled by the global gain. */
int32_t Calculator::calcComponentForce(Effect* effect, int32_t forceVector, uint8_t axis) {
    int32_t result_torque = 0;
    uint8_t con_idx = effect->useSingleCondition ? 0 : axis;

    AxisState& m = axis_state[axis];
    float angle_ratio = effect->axisMagnitudes[axis];

    switch (effect->type) {
    case FFB_EFFECT_CONSTANT: {
        if (effect->filter_active[axis]) {
            if (effect->filter[axis].getFc() < 0.5f &&
                effect->filter[0].getFc() != 0.0f)
            {
                forceVector = static_cast<int32_t>(effect->filter[axis].process(
                                                       static_cast<float>(forceVector)));
            }
        }
    }
    /* fallthrough: filter is a constant-force preprocessing step */
#if defined(__GNUC__) && __GNUC__ >= 7
        __attribute__((fallthrough));
#endif
    case FFB_EFFECT_RAMP:
    case FFB_EFFECT_SQUARE:
    case FFB_EFFECT_TRIANGLE:
    case FFB_EFFECT_SAWTOOTHUP:
    case FFB_EFFECT_SAWTOOTHDOWN:
    case FFB_EFFECT_SINE:
        result_torque = static_cast<int32_t>(-forceVector * angle_ratio);
        break;

    case FFB_EFFECT_SPRING: {
        float pos = static_cast<float>(m.pos_scaled_16b);
        result_torque -= calcConditionEffectForce(effect, pos,
                                                   gain.spring, con_idx,
                                                   scaler.spring, angle_ratio);
        break;
    }

    case FFB_EFFECT_FRICTION: {
        float speed = m.speed * INTERNAL_SCALER_FRICTION;

        int16_t offset   = effect->conditions[con_idx].cpOffset;
        int16_t deadBand = effect->conditions[con_idx].deadBand;
        int32_t force    = 0;

        float speedRampupCeil = speedRampupPct();

        if (std::abs(static_cast<int32_t>(speed) - offset) > deadBand) {

            speed -= (offset + (deadBand * (speed < offset ? -1 : 1)));

            float rampupFactor = 1.0f;
            if (std::fabs(speed) < speedRampupCeil) {
                float phaseRad = static_cast<float>(M_PI) *
                                  ((std::fabs(speed) / speedRampupCeil) - 0.5f);
                rampupFactor = (1.0f + std::sin(phaseRad)) / 2.0f;
            }

            int8_t  sign  = speed >= 0 ? 1 : -1;
            uint16_t coeff = speed < 0
                ? effect->conditions[con_idx].negativeCoefficient
                : effect->conditions[con_idx].positiveCoefficient;
            force = static_cast<int32_t>(coeff * rampupFactor * sign);

            if (effect->conditions[con_idx].negativeSaturation != 0 ||
                effect->conditions[con_idx].positiveSaturation != 0)
            {
                force = clip_t<int32_t>(
                    force,
                    -static_cast<int32_t>(effect->conditions[con_idx].negativeSaturation),
                     static_cast<int32_t>(effect->conditions[con_idx].positiveSaturation));
            }

            if (effect->filter_active[axis]) {
                result_torque -= static_cast<int32_t>(
                    effect->filter[axis].process(
                        (((gain.friction + 1) * force) >> 8) *
                        angle_ratio * scaler.friction));
            } else {
                result_torque -= static_cast<int32_t>(
                    (((gain.friction + 1) * force) >> 8) *
                    angle_ratio * scaler.friction);
            }
        }
        break;
    }

    case FFB_EFFECT_DAMPER: {
        float speed = m.speed * INTERNAL_SCALER_DAMPER;
        int32_t cf = calcConditionEffectForce(effect, speed, gain.damper,
                                               con_idx, scaler.damper, angle_ratio);
        if (effect->filter_active[axis]) {
            result_torque -= static_cast<int32_t>(
                effect->filter[axis].process(static_cast<float>(cf)));
        } else {
            result_torque -= cf;
        }
        break;
    }

    case FFB_EFFECT_INERTIA: {
        float accel = m.accel * INTERNAL_SCALER_INERTIA;
        int32_t cf = calcConditionEffectForce(effect, accel, gain.inertia,
                                               con_idx, scaler.inertia, angle_ratio);
        if (effect->filter_active[axis]) {
            result_torque -= static_cast<int32_t>(
                effect->filter[axis].process(static_cast<float>(cf)));
        } else {
            result_torque -= cf;
        }
        break;
    }

    default:
        break;
    }

    return (result_torque * global_gain) / 255;
}

/* Convert the friction ramp-up percentage into an absolute speed ceiling, on
 * the same 0..32767 scale the friction metric uses. */
float Calculator::speedRampupPct() const {
    return (frictionPctSpeedToRampup / 100.0f) * 32767.0f;
}

/* The classic DirectInput condition law, shared by spring/damper/inertia.
 * `metric` is the axis quantity the condition acts on (position for spring,
 * speed for damper, accel for inertia). Outside the deadband the force is
 * coefficient * gainfactor * scale * (metric - offset), clamped to the per-side
 * saturation, then projected onto the axis by angle_ratio. Inside the deadband
 * (or with no displacement) it returns 0. */
int32_t Calculator::calcConditionEffectForce(Effect* effect, float metric,
                                              uint8_t gain_val, uint8_t idx,
                                              float scale, float angle_ratio)
{
    int16_t offset   = effect->conditions[idx].cpOffset;
    int16_t deadBand = effect->conditions[idx].deadBand;
    int32_t force    = 0;
    float gainfactor = static_cast<float>(gain_val + 1) / 256.0f;

    if (std::abs(metric - offset) > deadBand) {
        float coefficient = effect->conditions[idx].negativeCoefficient;
        if (metric > offset) {
            coefficient = effect->conditions[idx].positiveCoefficient;
        }
        coefficient /= 32767.0f;  /* 0x7fff */

        metric = metric - (offset + (deadBand * (metric < offset ? -1 : 1)));

        force = clip_t<int32_t>(
            static_cast<int32_t>(coefficient * gainfactor * scale * metric),
            -static_cast<int32_t>(effect->conditions[idx].negativeSaturation),
             static_cast<int32_t>(effect->conditions[idx].positiveSaturation));
    }

    return static_cast<int32_t>(force * angle_ratio);
}

/* Reshape an effect's magnitude over time with its attack/sustain/fade
 * envelope: ramp up from attackLevel during attackTime, hold, then ramp down to
 * fadeLevel during the final fadeTime. Infinite-duration effects have no
 * envelope and return the magnitude unchanged. The sign of magnitude is kept. */
int32_t Calculator::getEnvelopeMagnitude(Effect* effect, uint32_t elapsed_ms) {
    if (effect->duration == FFB_EFFECT_DURATION_INFINITE || effect->duration == 0) {
        return effect->magnitude;
    }
    int32_t scaler_v = std::abs(effect->magnitude);
    uint32_t elapsed_time = elapsed_ms;
    if (elapsed_time < effect->attackTime && effect->attackTime != 0) {
        scaler_v = (scaler_v - effect->attackLevel) * static_cast<int32_t>(elapsed_time);
        scaler_v /= static_cast<int32_t>(effect->attackTime);
        scaler_v += effect->attackLevel;
    }
    if (elapsed_time > (effect->duration - effect->fadeTime) && effect->fadeTime != 0) {
        scaler_v = (scaler_v - effect->fadeLevel) *
                   (static_cast<int32_t>(effect->duration) - static_cast<int32_t>(elapsed_time));
        scaler_v /= static_cast<int32_t>(effect->fadeTime);
        scaler_v += effect->fadeLevel;
    }
    if (effect->magnitude < 0) scaler_v = -scaler_v;
    return scaler_v;
}

/* ----------- Per-effect biquad filter setup ------------------------- */

/* Configure the per-axis biquad low-pass filters for an effect, using the
 * cutoff/Q from the active preset (divided by the sample rate) for the effect's
 * type, and mark them active. Constant force always uses preset profile 0.
 * Effect types with no filter (e.g. spring) are left untouched. */
void Calculator::setFilters(Effect* effect) {
    const biquad_constant_t* preset;

    switch (effect->type) {
    case FFB_EFFECT_DAMPER:
        preset = &filter_presets[filterProfileId].damper;
        break;
    case FFB_EFFECT_FRICTION:
        preset = &filter_presets[filterProfileId].friction;
        break;
    case FFB_EFFECT_INERTIA:
        preset = &filter_presets[filterProfileId].inertia;
        break;
    case FFB_EFFECT_CONSTANT:
        /* Constant-force filter always uses profile 0, matching original */
        preset = &filter_presets[0].constant;
        break;
    default:
        /* Other effect types have no per-effect filter. */
        return;
    }

    /* A preset with a zero in it cannot be used as written: presetFc() and
     * presetQ() read it as "no cutoff" and as the lowest q, the way the
     * original firmware does when such a value is entered. */
    if (preset->freq == 0 || preset->q == 0) {
        FFB_LOG("FFB: filter preset {%d Hz, q %d} of effect type %d has a zero - "
                "freq 0 is read as no cutoff, q 0 as q 1\n",
                preset->freq, preset->q, effect->type);
    }

    for (int i = 0; i < FFB_MAX_AXIS; ++i) {
        effect->filter[i].setBiquad(BiquadType::lowpass,
                                     presetFc(*preset, calcfrequency),
                                     presetQ(*preset) * qfloatScaler, 0.0f);
        effect->filter_active[i] = true;
    }
}

/* Rebuild biquad coefficients on every live effect of a given type. Called
 * after the sample rate or a filter preset changes so existing effects pick up
 * the new cutoff. */
void Calculator::updateFiltersForType(uint8_t effect_type) {
    for (uint8_t i = 0; i < FFB_MAX_EFFECTS; ++i) {
        if (effects[i].type == effect_type) {
            setFilters(&effects[i]);
        }
    }
}

} /* namespace ffb */
