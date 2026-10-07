# FFB Library — Documentation

A standalone C++ force-feedback engine extracted from the
[OpenFFBoard](https://github.com/Ultrawipf/OpenFFBoard) firmware. It takes the
USB force-feedback (HID PID) reports a game sends, runs the effect math, and
hands you back a single torque number per axis to drive your motor.

Everything platform-specific in the original firmware — STM32 HAL, TinyUSB,
FreeRTOS, flash storage, the configurator CLI — has been stripped out, so the
engine drops into **any** microcontroller project:

- **No heap.** The whole effect pool, including filters, lives inside the
  `Library` object. Nothing is allocated at runtime.
- **No RTOS.** It is a plain polled engine: you call it once per control tick.
- **No dependencies.** Just C++11. A C wrapper is included for C projects.
- **Faithful math.** The waveform, condition, envelope and filter formulas are
  a direct port of OpenFFBoard and produce the same forces.
- **Single precision.** Every calculation is done in `float`: the engine stays
  on the FPU of a Cortex-M4F-class MCU and links no double-precision code.

This document has **two parts**:

- **[Part I — Using the library](#part-i--using-the-library)**: a practical,
  step-by-step guide to wiring the engine into your firmware, in both C++ and
  C. Start here.
- **[Part II — How the library works internally](#part-ii--how-the-library-works-internally)**:
  a deep dive into the pipeline, for anyone modifying the engine or curious how
  it produces a torque.

---

# Part I — Using the Library

1. [The mental model](#1-the-mental-model)
2. [Quick start in C++](#2-quick-start-in-c)
3. [Quick start in C](#3-quick-start-in-c)
4. [The workflow, step by step](#4-the-workflow-step-by-step)
5. [Units and conventions](#5-units-and-conventions)
6. [Wiring to your hardware](#6-wiring-to-your-hardware)
7. [Optional helper: speed & acceleration from position](#7-optional-helper-speed--acceleration-from-position)
8. [Optional helper: axis-local "feel" effects](#8-optional-helper-axis-local-feel-effects)
9. [Tuning the effect engine](#9-tuning-the-effect-engine)
10. [Compile-time configuration](#10-compile-time-configuration)
11. [Building](#11-building)
12. [API quick reference](#12-api-quick-reference)

---

## 1. The mental model

The library sits between your **USB stack** and your **motor driver**:

```
  Host PC (game / DirectInput)
        │  USB HID PID reports
        ▼
  Your USB stack  ──hidOut()/hidGet()──►  ffb::Library  ──getAxisTorque()──►  Your motor driver
        ▲                                      ▲
        │ status reports (optional)            │ setAxisState()  (wheel position / speed / accel)
        └────── send-report callback ──────────┘
```

You give the library two things every control cycle:

1. the inbound **USB reports** from the host (forwarded verbatim), and
2. the current **axis state** — where the wheel is and how fast it moves.

It gives you back one number per axis: the **torque** the host wants applied,
in the range `-0x7fff .. 0x7fff` (`-32767 .. 32767`). You scale that to your
motor's PWM or current and drive the hardware. **That is the entire contract.**

You never have to understand what any USB report means — the library decodes all
of them. You only ever touch five things:

| Phase | What you call | When |
|---|---|---|
| Setup | construct the library | once, at boot |
| Setup | register a status-report callback *(optional)* | once |
| Enumeration | hand over the HID report descriptor | when the USB stack asks for it |
| USB RX | forward each Set/Get Report | from your USB callbacks |
| Per tick | `setAxisState` → `calculate` → `getAxisTorque` | every control cycle (e.g. 1 kHz) |

The same five touch points exist in both the C++ and the C API.

> **One build-time prerequisite.** The library reads its compile-time options
> from a header named `ffb_config.h` that lives in *your* project, not in the
> library. Copy `examples/ffb_config.h` into your source tree and put its folder
> on the include path before the first build — see
> [§10](#10-compile-time-configuration).

---

## 2. Quick start in C++

A complete single-axis wheel, using only the core engine. It assumes your
project already has its `ffb_config.h` ([§10](#10-compile-time-configuration)):

```cpp
#include "ffb/ffb.h"

// --- things your board provides ---
extern uint32_t board_millis(void);          // ms since boot (may wrap)
extern uint32_t board_micros(void);          // us since boot (may wrap)
extern bool     usb_send_in_report(const uint8_t*, uint16_t);
extern void     motor_set(int32_t torque);   // -0x7fff..0x7fff
extern int32_t  encoder_pos_scaled(void);    // -0x7fff..0x7fff over full travel
extern float    encoder_speed_dps(void);     // degrees / second
extern float    encoder_accel_dpss(void);    // degrees / second^2

// One global engine instance for a 1-axis device.
static ffb::Library lib(1, ffb::TimeSource(board_millis, board_micros));

void ffb_init(void) {
    lib.setSendReportCallback(usb_send_in_report);   // optional
    uint16_t len;
    const uint8_t* desc = ffb::Library::descriptor1Axis(&len);
    usb_register_hid_report_descriptor(desc, len);   // your USB stack
}

// From your USB Set/Get Report callbacks:
void usb_on_set_report(uint8_t id, const uint8_t* b, uint16_t n) { lib.hidOut(id, b, n); }
uint16_t usb_on_get_report(uint8_t id, uint8_t* b, uint16_t n)   { return lib.hidGet(id, b, n); }

// Once per control tick (e.g. a 1 kHz timer ISR):
void ffb_tick(void) {
    lib.setAxisState(0, ffb::AxisState(encoder_pos_scaled(),
                                       encoder_speed_dps(),
                                       encoder_accel_dpss()));
    lib.calculate();
    motor_set(lib.getAxisTorque(0));
}
```

That is a fully working force-feedback wheel. Everything below is detail,
options, and helpers.

> **The wheel position still has to reach the host.** The engine handles the
> force-feedback reports; the joystick *input* report that carries the wheel
> angle and buttons to the game is your code's job. Its layout is in
> [§6.1](#61-the-usb-stack).

---

## 3. Quick start in C

The same device using the C wrapper (`ffb/ffb_c.h`). The wrapper holds one
static `ffb::Library` under the hood, so `ffb_create()` is called exactly once.
A C project needs the same `ffb_config.h`
([§10](#10-compile-time-configuration)): the C headers never include it, but the
library's own sources do.

```c
#include "ffb/ffb_c.h"

extern uint32_t board_millis(void);
extern uint32_t board_micros(void);
extern bool     usb_send_in_report(const uint8_t*, uint16_t);
extern void     motor_set(int32_t torque);
extern int32_t  encoder_pos_scaled(void);
extern float    encoder_speed_dps(void);
extern float    encoder_accel_dpss(void);

static ffb_lib_t* lib;

void ffb_init(void) {
    lib = ffb_create(1, board_millis, board_micros);
    ffb_set_send_report_callback(lib, usb_send_in_report);   // optional

    uint16_t len;
    const uint8_t* desc = ffb_descriptor_1axis(&len);
    usb_register_hid_report_descriptor(desc, len);
}

/* From your USB Set/Get Report callbacks: */
void usb_on_set_report(uint8_t id, const uint8_t* b, uint16_t n) { ffb_hid_out(lib, id, b, n); }
uint16_t usb_on_get_report(uint8_t id, uint8_t* b, uint16_t n)   { return ffb_hid_get(lib, id, b, n); }

/* Once per control tick: */
void ffb_tick(void) {
    ffb_set_axis_state(lib, 0, encoder_pos_scaled(),
                       encoder_speed_dps(), encoder_accel_dpss());
    ffb_calculate(lib);
    motor_set(ffb_get_axis_torque(lib, 0));
}
```

Every core C function maps one-to-one to a C++ method. The returned `ffb_lib_t*`
handle stays valid for the program's lifetime.

---

## 4. The workflow, step by step

The five touch points from §1, in detail. C++ first, C second.

### 4.1 Construct the engine

You tell the library how many physical axes you drive and how to read time.

```cpp
// C++
ffb::Library lib(/*axis_count=*/1, ffb::TimeSource(board_millis, board_micros));
```
```c
/* C */
ffb_lib_t* lib = ffb_create(/*axis_count=*/1, board_millis, board_micros);
```

- `axis_count` — 1 for a wheel or single pedal, 2 for a joystick, etc. It must be
  `<= FFB_MAX_AXIS` (compile-time, default 2; see [§10](#10-compile-time-configuration));
  a value outside `1..FFB_MAX_AXIS` is clamped into that range.
- The two time functions are free-running counters in **milliseconds** and
  **microseconds**. They may overflow — every internal use is delta-based, so
  wraparound is harmless. Keep them cheap (a register read); each is read once
  per tick. See [§6.3](#63-the-time-source).

The constructor allocates nothing. In C, the engine is a single static instance,
so `ffb_create()` may be called **only once** per program. In C++, a `Library`
cannot be copied: keep one instance and pass it by reference or pointer.

### 4.2 Give the host the HID report descriptor

At USB enumeration your stack asks for the HID **report descriptor** — the
~1.2 KB blob that tells the host "I am a force-feedback joystick, and here is the
byte layout of every report." The library ships pre-built, byte-correct ones:

```cpp
// C++  — 1196 bytes for 1 axis, 1215 bytes for 2 axes
uint16_t len;
const uint8_t* desc = ffb::Library::descriptor1Axis(&len);   // or descriptor2Axis
```
```c
/* C */
uint16_t len;
const uint8_t* desc = ffb_descriptor_1axis(&len);            /* or ffb_descriptor_2axis */
```

These are **static** and may be called before the engine is even constructed
(some USB stacks ask very early). Return the bytes from your stack's
"get HID report descriptor" callback. This is *not* the same as your USB
device/configuration descriptor — see [§6.1](#61-the-usb-stack).

### 4.3 Forward USB traffic

The host drives FFB by sending HID **Output reports** (effect parameters,
start/stop, gain) and **Feature reports** (the create-effect handshake). Your USB
stack hands those to its Set/Get Report callbacks; forward them straight through,
passing the report ID exactly as the host sent it.

```cpp
// C++
void on_usb_set_report(uint8_t id, const uint8_t* buf, uint16_t len) {
    lib.hidOut(id, buf, len);                 // host → device
}
uint16_t on_usb_get_report(uint8_t id, uint8_t* buf, uint16_t maxlen) {
    return lib.hidGet(id, buf, maxlen);       // device → host; returns #bytes written
}
```
```c
/* C */
void on_usb_set_report(uint8_t id, const uint8_t* buf, uint16_t len) {
    ffb_hid_out(lib, id, buf, len);
}
uint16_t on_usb_get_report(uint8_t id, uint8_t* buf, uint16_t maxlen) {
    return ffb_hid_get(lib, id, buf, maxlen);
}
```

`hidGet` returns the number of bytes it wrote into your buffer (0 if the report
ID is not one it needs to answer). You never decode anything yourself.

`len` must be the real number of bytes in `buf`: a report shorter than its
layout is ignored. Whether a buffer starts with the report-ID byte depends on
the report type, and USB stacks differ here — [§6.1](#61-the-usb-stack) lists
what the library expects.

> Pass the report ID **exactly as the host sent it**. Some stacks (notably
> TinyUSB on the interrupt OUT endpoint) hand you `report_id == 0` with the real
> ID in the first buffer byte — you must recover it first, or `hidOut` ignores the
> report. See [§6.1](#61-the-usb-stack) for the TinyUSB fix-up.

### 4.4 The per-tick loop

This is the hot path. Run it at a fixed rate — the default math is tuned for
1 kHz. Three calls:

```cpp
// C++
lib.setAxisState(0, ffb::AxisState(pos_scaled, speed_dps, accel_dpss));  // 1. where the wheel is
lib.calculate();                                                         // 2. run the engine
int32_t torque = lib.getAxisTorque(0);                                   // 3. read the result
motor_set(torque);
```
```c
/* C */
ffb_set_axis_state(lib, 0, pos_scaled, speed_dps, accel_dpss);           /* 1 */
ffb_calculate(lib);                                                      /* 2 */
int32_t torque = ffb_get_axis_torque(lib, 0);                            /* 3 */
motor_set(torque);
```

See [§5](#5-units-and-conventions) for what `pos_scaled`, `speed_dps` and the
returned torque mean. If you don't already have filtered speed/accel, the
[metrics helper](#7-optional-helper-speed--acceleration-from-position) computes
them for you.

> Until the host enables FFB, the engine is inactive and `calculate()` simply
> zeroes every torque and returns. You can force it on for testing with
> `setActive(true)` / `ffb_set_active(lib, true)`.

### 4.5 Control and settings

```cpp
// C++
bool on = lib.isActive();          lib.setActive(true);
lib.resetAllEffects();             // wipe every effect slot
lib.setGlobalGain(200);            uint8_t g = lib.getGlobalGain();   // 0..255 master strength
lib.setSamplerate(2000.0f);        float hz = lib.getSamplerate();    // tell it your real tick rate
```
```c
/* C */
bool on = ffb_is_active(lib);      ffb_set_active(lib, true);
ffb_reset_all_effects(lib);
ffb_set_global_gain(lib, 200);     uint8_t g = ffb_get_global_gain(lib);
ffb_set_samplerate(lib, 2000.0f);  float hz = ffb_get_samplerate(lib);
```

`setSamplerate()` matters only if you do **not** run at 1 kHz: the engine's
smoothing filters are tuned from it. Call it once at startup with your real loop
rate. (The host normally controls `setActive` and the global gain itself.)

### 4.6 Status reports back to the host (optional)

DirectInput likes the device to publish a "PID State" input report (report ID 2)
when effects are created or started/stopped. If you want that, register a sender;
return `true` if your USB stack accepted the report.

```cpp
// C++
lib.setSendReportCallback([](const uint8_t* buf, uint16_t len) -> bool {
    return usb_send_in_report(buf, len);
});
```
```c
/* C */
ffb_set_send_report_callback(lib, usb_send_in_report);
```

Leave it unset and status reports simply aren't emitted. Most games work fine
without them.

---

## 5. Units and conventions

Getting these right is most of the integration. Three quantities cross the
boundary.

**Position** — `pos_scaled_16b`, an `int32_t`.
Map your wheel so the two ends of its travel are `-0x7fff` and `+0x7fff`, with
center at `0`. This is what spring effects read. If you only have raw encoder
counts or degrees, the [metrics helper](#7-optional-helper-speed--acceleration-from-position)
does the scaling for you.

**Speed** — `float`, **degrees per second**.
**Acceleration** — `float`, **degrees per second²**.
These drive damper, inertia and friction effects. They must be reasonably
smooth: the raw derivative of an encoder is noisy, so low-pass filter them (the
metrics helper does this).

**Torque out** — `int32_t`, range `-0x7fff .. 0x7fff`.
Sign is direction, magnitude is strength. Map it linearly to your motor command.
Add your own safety limits (current, slew rate, thermal) **outside** the library.

> The position scale is intentionally *not* clamped — if the wheel moves past its
> defined travel, the scaled value goes beyond `±0x7fff`. The optional
> [end-stop](#8-optional-helper-axis-local-feel-effects) relies on that overshoot
> to know it has hit the limit.

---

## 6. Wiring to your hardware

The library never touches hardware. Everything platform-specific is reached
through a function pointer you supply. There are three wiring points.

### 6.1 The USB stack

The library is USB-stack agnostic. You connect three things:

| USB event | What you call |
|---|---|
| Host requests the HID **report** descriptor | return `descriptor1Axis()/2Axis()` bytes |
| Set Report (host → device) | `hidOut(id, buf, len)` |
| Get Report (device → host) | `return hidGet(id, buf, maxlen)` |

**TinyUSB** is the most common target:

```cpp
const uint8_t* tud_hid_descriptor_report_cb(uint8_t /*itf*/) {
    uint16_t len; return ffb::Library::descriptor1Axis(&len);
}

// Host -> device (interrupt-EP OUT data + control-pipe Set Report).
void tud_hid_set_report_cb(uint8_t /*itf*/, uint8_t report_id,
                           hid_report_type_t report_type,
                           const uint8_t* buffer, uint16_t bufsize) {
    // On the interrupt OUT endpoint TinyUSB delivers report_id == 0 and puts
    // the real report ID in the first byte of the buffer. Recover it - this
    // mirrors the original OpenFFBoard fix-up.
    if ((report_type == HID_REPORT_TYPE_INVALID ||
         report_type == HID_REPORT_TYPE_OUTPUT) && report_id == 0) {
        if (bufsize > 0) report_id = buffer[0];
    }
    lib.hidOut(report_id, buffer, bufsize);
}

// Device -> host (control-pipe Get Report: Block Load / PID Pool replies).
uint16_t tud_hid_get_report_cb(uint8_t /*itf*/, uint8_t report_id,
                               hid_report_type_t /*report_type*/,
                               uint8_t* buffer, uint16_t reqlen) {
    return lib.hidGet(report_id, buffer, reqlen);
}
```

`examples/tinyusb_glue.cpp` is a ready-to-copy version. For ST USB, LUFA, or a
hand-rolled stack the idea is identical: find the set-report and get-report entry
points and forward.

> **Why the report-ID fix-up matters.** Force-feedback OUT reports arrive on the
> interrupt OUT endpoint, where TinyUSB passes `report_id == 0` and leaves the
> real ID as the **first byte of the buffer**. If you don't recover it, `hidOut`
> sees ID `0`, matches no handler, and effects silently never play. The
> OUT-report structs in `ffb_defs.h` keep that ID byte as their first field, so
> forwarding the whole buffer is correct. (Feature reports on the control pipe are
> different: TinyUSB already provides the ID in `report_id`, and those structs
> have no ID byte — so the fix-up's `report_type`/`report_id == 0` guard skips
> them.)

**What the buffers must contain.** The library follows TinyUSB's conventions.
On any other stack, check what it hands you and add or strip the ID byte in your
glue:

| Direction | Reports | Buffer contents |
|---|---|---|
| `hidOut` | Output reports (`0x01`–`0x0D`) | the whole report, **report-ID byte first** |
| `hidOut` | Feature report Create New Effect (`0x11`) | the payload only, **no** ID byte |
| `hidGet` | Feature replies Block Load (`0x12`), PID Pool (`0x13`) | the library writes the payload only, **no** ID byte; if your stack wants the ID in front, add it |
| send-report callback | PID State (`0x02`) | the whole report, **report-ID byte first** (2 bytes) |

**The joystick input report is yours.** Besides the force-feedback reports, the
shipped descriptor declares the device's ordinary joystick input report — the
one that tells the game where the wheel is. The library does not build or send
it. It is report ID `1`, 24 bytes after the ID byte:

```c
typedef struct __attribute__((packed)) {
    uint8_t buttons[8];   /* 64 buttons, one bit each                          */
    int16_t axes[8];      /* X, Y, Z, Rx, Ry, Rz, Dial, Slider: -32767..32767  */
} joystick_report_t;      /* 24 bytes                                          */
```

Put the wheel on `axes[0]` (X, the axis the force-feedback effects act on) and
send the report at your USB poll rate — on TinyUSB
`tud_hid_report(1, &report, sizeof(report))`, which adds the ID byte itself.

**Two different descriptors — don't confuse them.** The library owns only one:

| Descriptor | What it is | Who provides it |
|---|---|---|
| **HID report descriptor** | the ~1.2 KB FFB-report blob | **the library** (`descriptor1Axis()/2Axis()`) |
| **USB device / configuration / string descriptors** | VID/PID, interfaces, endpoints, product name | **you** |

The library never sees enumeration, so it cannot ship your configuration
descriptor. Two things are mandatory there:

1. **The HID interface needs an OUT endpoint as well as an IN endpoint** — force
   feedback is the host sending Output reports, so an IN-only interface cannot
   receive effects. On TinyUSB use `TUD_HID_INOUT_DESCRIPTOR`.
2. **The report-descriptor length in your config must match the library's** —
   `1196` for `descriptor1Axis()`, `1215` for `descriptor2Axis()`.

A complete, copy-pasteable HID-only config (device + config + string descriptors
and the three callbacks) is in `examples/usb_descriptors_tinyusb.c`. A pure-FFB
`tusb_config.h` just needs the HID class enabled with a 64-byte endpoint buffer:

```c
#define CFG_TUD_HID            1
#define CFG_TUD_HID_EP_BUFSIZE 64    /* >= largest FFB report; the 1.2 KB
                                        descriptor travels over EP0, not this */
#define CFG_TUD_CDC 0
#define CFG_TUD_MSC 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_VENDOR 0
```

### 6.2 The motor driver

`getAxisTorque(axis)` returns `-0x7fff .. 0x7fff`. You own the mapping to
hardware:

```cpp
// PWM H-bridge: sign = direction, magnitude = duty
int32_t t = lib.getAxisTorque(0);
gpio_set_dir(t >= 0);
pwm_set_duty((abs(t) * MAX_DUTY) / 0x7fff);

// Or a bipolar current target for a BLDC FOC loop
float amps = (lib.getAxisTorque(0) / 32767.0f) * MAX_CURRENT_A;
foc_set_iq_target(amps);
```

### 6.3 The time source

The engine needs elapsed time for effect duration, envelopes and periodic-wave
phase. You supply two free-running counters:

```cpp
uint32_t board_millis(void);   // milliseconds since boot
uint32_t board_micros(void);   // microseconds since boot
```

- Both may **wrap around**. Every use is a delta (`now - start`), so a 32-bit
  counter wrapping (~49 days for millis, ~71 min for micros) causes no glitch.
- If your MCU only has `millis()`, derive `micros()` from any 1 MHz hardware
  timer. The periodic and ramp effects use `micros()` for sub-millisecond phase;
  a coarse `micros` still works, just with coarser waveform timing.
- The two should be views of the **same clock**, so that `micros() / 1000`
  equals `millis()`: the engine takes whole milliseconds from `millis()` and
  only the fraction of the current millisecond from `micros()`. If `micros()` is
  missing (a null pointer) or runs apart from `millis()` — a free-running timer
  started at another moment, say — the engine notices and times the effects in
  whole milliseconds; nothing else changes.
- Each counter is read once per tick, at the top of `calculate()`.

---

## 7. Optional helper: speed & acceleration from position

If you only have a raw wheel position and don't want to compute filtered speed
and acceleration yourself, use `MetricsBuilder` (`ffb/ffb_metrics.h`). It scales
degrees into the `±0x7fff` position range **and** derives filtered speed/accel —
exactly the math OpenFFBoard used.

```cpp
// C++
#include "ffb/ffb_metrics.h"

ffb::MetricsBuilder metrics(/*degrees_of_rotation=*/900.0f, /*samplerate_hz=*/1000.0f);

void tick(float raw_wheel_degrees) {
    ffb::AxisState st = metrics.update(raw_wheel_degrees);  // scales + filters
    lib.setAxisState(0, st);
    lib.calculate();
    motor_set(lib.getAxisTorque(0));
}
```

```c
/* C — include ffb/ffb_metrics_c.h, compile src/ffb_metrics_c.cpp + src/ffb_metrics.cpp */
#include "ffb/ffb_metrics_c.h"

ffb_metrics_t* m = ffb_metrics_create(/*dor=*/900.0f, /*hz=*/1000.0f);

void tick(float raw_wheel_degrees) {
    ffb_metrics_update_and_set(m, lib, 0, raw_wheel_degrees);  /* update + feed engine */
    ffb_calculate(lib);
    motor_set(ffb_get_axis_torque(lib, 0));
}
```

Useful calls:

| Purpose | C++ | C |
|---|---|---|
| Update + return the state | `metrics.update(deg)` | `ffb_metrics_update(m, deg)` |
| Update + feed engine in one call | — | `ffb_metrics_update_and_set(m, lib, axis, deg)` |
| New control-loop rate | `metrics.setSamplerate(hz)` | `ffb_metrics_set_samplerate(m, hz)` |
| Clear history at a position | `metrics.reset(deg)` | `ffb_metrics_reset(m, deg)` |
| Custom filter coefficients | constructor's 3rd arg | `ffb_metrics_create_ex(...)` |

The first angle you pass only seeds the history, so the wheel may sit anywhere
at power-up without producing a burst of speed. Call `reset(deg)` when the
position itself jumps — after re-centering the encoder, for example.

Default filters: speed `{70 Hz, Q 0.55}`, accel `{55 Hz, Q 0.30}`. **Skip this
helper entirely** if you already produce filtered speed/accel — just fill
`AxisState` directly. In C, instances come from a static pool of `FFB_MAX_AXIS`
slots, so call `ffb_metrics_create()` once per axis.

---

## 8. Optional helper: axis-local "feel" effects

Some effects are *not* requested by the host — they're things a wheel does on its
own:

- an **idle spring** that auto-centers the wheel when FFB is off,
- a software **end-stop** that pushes back when the wheel hits its rotation
  limit, and
- always-on **damper / friction / inertia** for mechanical feel.

`AxisLocalEffects` (`ffb/ffb_axis_local.h`) provides them. It is completely
separate from the host engine — **you add its output on top of
`getAxisTorque()`**:

```
motor = clip( host torque  +  axis-local torque , -0x7fff, +0x7fff )
```

Nothing happens automatically: if you never construct it and never add its
output, there is no idle spring and **no end-stop** — the core engine doesn't
know your wheel has a physical range.

### 8.1 C++

```cpp
#include "ffb/ffb_axis_local.h"

ffb::AxisLocalConfig cfg;
cfg.degrees_of_rotation  = 900.0f;   // same value as the metrics helper
cfg.idle_spring_strength = 40;       // auto-center when FFB off (0 = off)
cfg.endstop_strength     = 127;      // wall stiffness at the limit
cfg.damper_intensity     = 30;       // always-on damping
ffb::AxisLocalEffects local(cfg);

void tick(float raw_wheel_degrees) {
    ffb::AxisState st = metrics.update(raw_wheel_degrees);
    lib.setAxisState(0, st);
    lib.calculate();

    int32_t host  = lib.getAxisTorque(0);
    int32_t feel  = local.compute(st, raw_wheel_degrees, lib.isActive());

    int32_t total = host + feel;
    if (total >  0x7fff) total =  0x7fff;
    if (total < -0x7fff) total = -0x7fff;
    motor_set(total);
}

// Runtime tuning (e.g. from a settings menu):
local.config().endstop_strength = 200;   // live, read on every compute()
local.setIdleSpringStrength(10);          // same as config().idle_spring_strength = 10
local.setSamplerate(2000.0f);             // setter: rebuilds filters
```

### 8.2 C

```c
/* include ffb/ffb_axis_local_c.h, compile src/ffb_axis_local_c.cpp + src/ffb_axis_local.cpp */
#include "ffb/ffb_axis_local_c.h"

ffb_axis_local_config_t cfg;
ffb_axis_local_config_default(&cfg);     /* start from defaults */
cfg.degrees_of_rotation  = 900.0f;
cfg.idle_spring_strength = 40;
cfg.endstop_strength     = 127;
cfg.damper_intensity     = 30;
ffb_axis_local_t* local = ffb_axis_local_create(&cfg);   /* NULL if pool full */

void tick(float raw_wheel_degrees) {
    ffb_axis_state_t st = ffb_metrics_update(m, raw_wheel_degrees);
    ffb_set_axis_state_s(lib, 0, &st);
    ffb_calculate(lib);

    int32_t host  = ffb_get_axis_torque(lib, 0);
    int32_t feel  = ffb_axis_local_compute(local, &st, raw_wheel_degrees,
                                           ffb_is_active(lib));
    int32_t total = host + feel;
    if (total >  0x7fff) total =  0x7fff;
    if (total < -0x7fff) total = -0x7fff;
    motor_set(total);
}

/* Runtime tuning: */
ffb_axis_local_set_intensities(local, /*endstop=*/200, /*damper=*/50,
                               /*friction=*/0, /*inertia=*/0);   /* all four at once */
ffb_axis_local_set_idle_spring(local, 10);
ffb_axis_local_set_samplerate(local, 2000.0f);
```

`compute(metrics, pos_degrees, ffb_on)` takes the same `AxisState` you feed the
engine and whether host FFB is active (the idle spring engages only when it is
**off**). The end-stop works from `metrics.pos_scaled_16b`: it engages where that
value leaves `±0x7fff` and measures the overshoot from it, so it always pushes
back toward the travel range. That is why the scaled position must reach the
helper un-clamped. The wheel angle in the middle is no longer used — the
parameter is kept so existing code compiles unchanged.

### 8.3 Parameters

| Field | Default | Meaning |
|---|---|---|
| `idle_spring_strength` | `0` | auto-center force when FFB is off (0 = off) |
| `endstop_strength` | `127` | wall stiffness past the limit (higher = harder) |
| `damper_intensity` | `30` | always-on damping (resists speed) |
| `friction_intensity` | `0` | always-on friction |
| `inertia_intensity` | `0` | always-on inertia (resists acceleration) |
| `degrees_of_rotation` | `900` | full wheel travel, as given to the metrics helper; turns the overshoot into degrees |
| `damper_filter {freq,q}` | `{60,55}` | damper low-pass (Hz, Q×100) |
| `friction_filter {freq,q}` | `{50,20}` | friction low-pass |
| `inertia_filter {freq,q}` | `{20,20}` | inertia low-pass |
| `samplerate_hz` | `1000` | control-loop rate (filter tuning) |

The end-stop is a one-sided spring that exists only *beyond* the limit: torque is
proportional to overshoot in degrees, scaled by `endstop_strength × 25`, directed
back toward center, and clamped to `±0x7fff`. With the default strength it reaches
full motor torque about 10° past the limit. The limit itself is where the scaled
position reaches `±0x7fff` — set by the travel you gave the metrics helper —
and `degrees_of_rotation` converts the overshoot to degrees. Keep the two values
equal; if they differ, the wall is only softer or harder than intended.

### 8.4 Runtime tuning — which path

Most fields are read on every `compute()` so you can change them live; only the
filters are cached and need their setter.

| Parameter | C++ | C | When it applies |
|---|---|---|---|
| `endstop_strength` | `config().endstop_strength = n` | `ffb_axis_local_set_intensities(a, n, …)` | next tick |
| `damper/friction/inertia_intensity` | `config().<field> = n` | `…set_intensities(a, …)` | next tick |
| `idle_spring_strength` | `config().idle_spring_strength = n` or `setIdleSpringStrength(n)` | `ffb_axis_local_set_idle_spring(a, n)` | next tick |
| `samplerate_hz` | `setSamplerate(hz)` | `ffb_axis_local_set_samplerate(a, hz)` | **setter required** (rebuilds filters) |
| `degrees_of_rotation` | `config().degrees_of_rotation = d` | *create-time only* | next tick (C++) |
| filter `freq`/`q` | `config().<f>.freq = …` then `setSamplerate(hz)` | *create-time only* | after rebuild |

Two gotchas:

1. **C `ffb_axis_local_set_intensities()` writes all four intensities at once.**
   To change only the end-stop, pass the current damper/friction/inertia values
   too. There are no per-field getters in C, so track them yourself.
2. **In C, `degrees_of_rotation` and the filter `freq`/`q` are fixed at
   `ffb_axis_local_create()`.** In C++ you can change them live via `config()`
   (call `setSamplerate()` afterward to rebuild filters).

> **Build:** the bundled CMake always compiles `src/ffb_axis_local.cpp` into
> `libffb`; the C wrapper `src/ffb_axis_local_c.cpp` is added when
> `FFB_BUILD_C_WRAPPER=ON` (the default). Because `libffb` is static, a helper
> you never call is not pulled into your binary — zero cost if unused.

---

## 9. Tuning the effect engine

The condition effects (spring/damper/inertia/friction), their smoothing filters,
and the friction ramp-up all have runtime knobs. The defaults already match
OpenFFBoard, so you rarely need these.

In **C++** they live on the `Calculator`, reached with `getCalculator()`:

```cpp
ffb::Calculator& fx = lib.getCalculator();

fx.gains().spring = 80;                 // per-effect master gains (uint8)
fx.scalers().damper = 4.0f;             // per-effect output scalers (float)
fx.setFrictionRampupPct(25);            // % of full speed for the friction half-sine ease-in
fx.setFilterProfileId(1);               // 0 = default filter presets, 1 = custom
fx.filterPreset(1).damper = { 30, 40 }; // edit the custom profile (freq Hz, q×100)
fx.updateFiltersForType(FFB_EFFECT_DAMPER);  // rebuild coeffs on live effects
```

In **C** the same knobs are explicit functions:

```c
ffb_effect_gain_t g;   ffb_get_effect_gains(lib, &g);   g.spring = 80;
ffb_set_effect_gains(lib, &g);                          /* spring/damper/inertia/friction */

ffb_effect_scaler_t s; ffb_get_effect_scalers(lib, &s); ffb_set_effect_scalers(lib, &s);

ffb_set_friction_rampup_pct(lib, 25);   ffb_get_friction_rampup_pct(lib);
ffb_set_filter_profile_id(lib, 1);      ffb_get_filter_profile_id(lib);

ffb_effect_filter_preset_t p;
ffb_get_filter_preset(lib, 1, &p);      ffb_set_filter_preset(lib, 1, &p);
ffb_update_filters_for_type(lib, FFB_EFFECT_DAMPER);
```

Default condition gains `spring 64, damper 64, inertia 127, friction 254`;
scalers `spring 16, damper 4, inertia 2, friction 1`; filter presets
`constant {500,70}, friction {50,20}, damper {30,40}, inertia {15,20}`. What each
value does is explained in [Part II §6](#6-projection-conditions-and-filtering).
In any filter preset, here and in the two helpers, a `freq` of `0` means "no
cutoff" (the filter passes everything) and a `q` of `0` is read as `1`.

---

## 10. Compile-time configuration

The library has a handful of compile-time options. They are **not** set inside
the library: they live in a header named **`ffb_config.h` that you create in your
own project**, outside the library folder — the same arrangement as TinyUSB's
`tusb_config.h` or FreeRTOS's `FreeRTOSConfig.h`.

### 10.1 Creating your `ffb_config.h`

1. Copy `examples/ffb_config.h` into your project. It is a template that lists
   every option at its default value; next to your `tusb_config.h` is a natural
   place for it.
2. Edit the values.
3. Put its folder on the include path of **every** source file that uses the
   library — and that includes the library's own `src/*.cpp`. With the bundled
   CMake that is `-DFFB_CONFIG_DIR=<folder>` ([§11](#11-building)); in any other
   build system, add the folder with `-I` next to `include/`.

A complete config for a single-axis wheel can be this short:

```c
/* my_project/config/ffb_config.h */
#ifndef FFB_CONFIG_H_
#define FFB_CONFIG_H_

#define FFB_MAX_AXIS 1          /* a wheel has one axis */

#endif
```

The library includes the file by name (`#include "ffb_config.h"`, from
`ffb/ffb_options.h`), so the filename is fixed. If the file cannot be found, the
build stops with an error that says so.

### 10.2 The options

| Macro | Default | Purpose | When to change |
|---|---|---|---|
| `FFB_MAX_AXIS` | `2` | max physical axes (1–3) | set to `1` for a wheel/pedal to shrink RAM |
| `FFB_MAX_EFFECTS` | `40` | effect-slot pool size (1–127) | lower on tiny MCUs (the host's pool report advertises it) |
| `FFB_DEFAULT_SAMPLERATE_HZ` | `1000.0f` | initial filter tuning rate | match your loop rate (or call `setSamplerate`) |
| `FFB_ID_OFFSET` | `0` | added to every report ID | composite HID device sharing report IDs |
| `FFB_LOG(...)` | no-op | `printf`-style debug log hook | define to your logger to trace effect lifecycle |

Every option is optional: anything your `ffb_config.h` does not define keeps the
default above. `ffb/ffb_options.h` fills those in and rejects an out-of-range
`FFB_MAX_AXIS` or `FFB_MAX_EFFECTS` at compile time. The ceiling of 127 effects
comes from the HID report descriptor, which declares the pool size as a one-byte
signed maximum.

Memory is roughly `FFB_MAX_EFFECTS × sizeof(Effect)` (each effect carries its own
`FFB_MAX_AXIS` filters) — about **6.7 KB** of static RAM with defaults, fine for
a Cortex-M0+. One axis brings it down to about 4.3 KB.

`FFB_LOG` is called like `printf` (a format string plus arguments), so route it
to something that accepts the same:

```c
/* in ffb_config.h */
#include <stdio.h>
#define FFB_LOG(...) printf(__VA_ARGS__)
```

Once routed, the log traces the life of every effect (created, started, stopped,
finished, freed), Device Control and gain changes, the engine being switched on
together with the setup it runs with, and everything the library corrects or
refuses: a report that is too short or names a block that does not exist, an
axis count that had to be clamped, a missing time source, a `micros()` that does
not track `millis()`, a filter preset with a zero in it. Messages come out in
the context that called the library — your USB callback for the reports, your
control tick for an effect running out — so route `FFB_LOG` to something that is
safe to call there. Nothing is logged from a constructor, and reports that a game
streams continuously (Set Constant Force and the like) are not logged at all.

### 10.3 Why the file is yours

`FFB_MAX_AXIS` and `FFB_MAX_EFFECTS` set the size of the engine's internal
structures, so the library's sources and your code must be compiled with exactly
the same values. A `#define` written above `#include "ffb/ffb.h"` in one of your
files would never reach the library's `.cpp` files; a header that both sides
include cannot disagree with itself. And because the file sits in your project,
updating the library never overwrites your settings.

An option can still be passed on the compiler command line instead
(`-DFFB_MAX_AXIS=1`), as long as the flag reaches every source file and the macro
is then left out of `ffb_config.h`.

### 10.4 Custom report layouts

If you need a different report layout (extra buttons, custom axis count), include
`ffb/ffb_descriptor.h` — it re-exports the `HIDDESC_FFB_*` building-block macros so
you can assemble your own descriptor. If you move report IDs, set `FFB_ID_OFFSET`
to keep the parser in sync.

---

## 11. Building

**With the bundled CMake:**

```sh
cmake -B build -DFFB_BUILD_TESTS=ON
cmake --build build          # produces libffb.a and the smoke test
ctest --test-dir build       # runs it (or run build/ffb_smoke_test directly)
```

Options: `FFB_BUILD_EXAMPLES` (off), `FFB_BUILD_TESTS` (off),
`FFB_BUILD_C_WRAPPER` (on), and `FFB_CONFIG_DIR` — the folder that holds your
`ffb_config.h` ([§10](#10-compile-time-configuration)). The target needs C++11
with extensions off.

Built on its own, as above, the library falls back to the template in
`examples/`. Pulled into your own project it has to be told where your config
is, before `add_subdirectory()`:

```cmake
set(FFB_CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/config)   # holds ffb_config.h
add_subdirectory(Force-Feedback-Library)
target_link_libraries(my_firmware PRIVATE ffb)
```

That folder becomes part of the `ffb` target's public include path, so the
library and everything that links it are compiled against the same file. CMake
prints which `ffb_config.h` it picked up when it configures.

**Without CMake (drop into an existing firmware build):**

1. Add `include/` to your include path.
2. Copy `examples/ffb_config.h` into your project and add **its** folder to the
   include path as well ([§10](#10-compile-time-configuration)).
3. Always compile: `src/ffb_biquad.cpp`, `src/ffb_calculator.cpp`,
   `src/ffb_parser.cpp`, `src/ffb_descriptor.cpp`.
4. Compile `src/ffb_c.cpp` only if you use the C API.
5. Compile `src/ffb_metrics.cpp` / `src/ffb_axis_local.cpp` only if you include
   their headers. From C, also compile the matching `*_c.cpp` wrapper.

A build system that gives you no way to add an include folder for a library's
own sources (the Arduino IDE is one) cannot see a header kept in your project.
There, place your `ffb_config.h` directly in the library's `include/` folder
instead.

No external libraries, no RTOS, no dynamic allocation. C++11 minimum;
C++14/17 also build clean.

---

## 12. API quick reference

**C++ — `ffb::Library` (`ffb/ffb.h`)**

| Method | Purpose |
|---|---|
| `Library(uint8_t axis_count, TimeSource ts)` | construct |
| `void hidOut(id, buf, len)` | feed an inbound Output/Feature report |
| `uint16_t hidGet(id, buf, reqlen)` | produce a Feature reply; returns bytes written |
| `void setAxisState(axis, AxisState)` | provide position/speed/accel |
| `void calculate()` | run one engine tick |
| `int32_t getAxisTorque(axis)` | read torque (`-0x7fff..0x7fff`) |
| `bool isActive()` / `void setActive(bool)` | query / force FFB enable |
| `void resetAllEffects()` | clear all effect slots |
| `void setGlobalGain(u8)` / `uint8_t getGlobalGain()` | master gain |
| `void setSamplerate(float)` / `float getSamplerate()` | engine rate (Hz) |
| `void setDirectionEnableMask(u8)` | advanced direction-bit override |
| `void setSendReportCallback(SendReportFn)` | status-report sender (optional) |
| `static descriptor1Axis(uint16_t*)` / `descriptor2Axis(uint16_t*)` | HID descriptors |
| `Calculator& getCalculator()` / `HidParser& getParser()` | escape hatch to internals |

**C — core (`ffb/ffb_c.h`)**

| Function | Purpose |
|---|---|
| `ffb_create(axis_count, millis_fn, micros_fn)` | construct (once) → `ffb_lib_t*` |
| `ffb_set_send_report_callback(lib, cb)` | status-report sender (optional) |
| `ffb_hid_out(lib, id, buf, len)` | feed an inbound report |
| `ffb_hid_get(lib, id, buf, reqlen)` | produce a Feature reply |
| `ffb_set_axis_state(lib, axis, pos, speed, accel)` | provide state (scalar form) |
| `ffb_set_axis_state_s(lib, axis, &state)` | provide state (struct form) |
| `ffb_calculate(lib)` / `ffb_get_axis_torque(lib, axis)` | run tick / read torque |
| `ffb_set_active / ffb_is_active / ffb_reset_all_effects` | control |
| `ffb_set_global_gain / get`, `ffb_set_samplerate / get`, `ffb_get_axis_count` | settings |
| `ffb_set_direction_enable_mask(lib, mask)` | advanced |
| `ffb_set_friction_rampup_pct / get`, `ffb_set_filter_profile_id / get` | tuning |
| `ffb_set_effect_gains / get`, `ffb_set_effect_scalers / get` | condition tuning |
| `ffb_set_filter_preset / get`, `ffb_update_filters_for_type` | filter tuning |
| `ffb_descriptor_1axis(uint16_t*)` / `ffb_descriptor_2axis(uint16_t*)` | HID descriptors |

Optional helpers: `ffb/ffb_metrics.h` + `ffb/ffb_metrics_c.h`
([§7](#7-optional-helper-speed--acceleration-from-position)),
`ffb/ffb_axis_local.h` + `ffb/ffb_axis_local_c.h`
([§8](#8-optional-helper-axis-local-feel-effects)).

---
---

# Part II — How the Library Works Internally

1. [Architecture](#1-architecture)
2. [The effect pool and an effect's lifecycle](#2-the-effect-pool-and-an-effects-lifecycle)
3. [HidParser — decoding inbound reports](#3-hidparser--decoding-inbound-reports)
4. [Calculator — the per-tick loop](#4-calculator--the-per-tick-loop)
5. [Waveform generators](#5-waveform-generators)
6. [Projection, conditions and filtering](#6-projection-conditions-and-filtering)
7. [Envelopes](#7-envelopes)
8. [Biquad filters and the static-allocation design](#8-biquad-filters-and-the-static-allocation-design)
9. [Time handling and overflow](#9-time-handling-and-overflow)
10. [The HID report descriptor](#10-the-hid-report-descriptor)
11. [The optional helpers, internally](#11-the-optional-helpers-internally)
12. [Relationship to upstream OpenFFBoard](#12-relationship-to-upstream-openffboard)
13. [Appendix: report IDs and effect types](#13-appendix-report-ids-and-effect-types)

---

## 1. Architecture

```
┌──────────────────────────────────────────────────────────┐
│  ffb::Library  (facade, header-only)                     │  ffb.h
│  owns the two components below and forwards calls         │
└───────────────┬───────────────────────┬──────────────────┘
                │                       │
                ▼                       ▼
┌───────────────────────────┐  ┌───────────────────────────┐
│  ffb::HidParser           │  │  ffb::Calculator          │
│  decodes USB reports into │  │  reads AxisState[],       │
│  Effect mutations         │◄─┤  runs the force math,     │
│                           │  │  writes axis_torque[]     │
└───────────────┬───────────┘  └───────────┬───────────────┘
                │  share the same          │ uses
                ▼  Effect array            ▼
┌───────────────────────────┐  ┌───────────────────────────┐
│  std::array<Effect, N>    │  │  ffb::Biquad (low-pass)    │
│  the effect pool          │  │  per-effect, per-axis      │
└───────────────────────────┘  └───────────────────────────┘

Static support files (no dependencies):
  ffb_defs.h        wire-format report structs + report-ID / effect-type constants
  ffb_effect.h      the Effect struct (one effect's parameters + its filter slots)
  ffb_descriptor.*  the pre-built HID report descriptor byte arrays
  ffb_options.h     includes your ffb_config.h, fills in defaults, validates

Supplied by you (lives in your project, not in the library):
  ffb_config.h      compile-time knobs (template: examples/ffb_config.h)

Opt-in modules (never pulled in unless you include them):
  ffb_metrics.*     raw position → scaled position + filtered speed/accel
  ffb_axis_local.*  idle spring / end-stop / always-on damper-friction-inertia
```

**The pipeline in one breath.** The host defines an effect through a sequence of
HID reports → `HidParser` decodes each report and writes the relevant fields into
a shared `Effect` slot → once per tick `Calculator::calculate()` walks every
active slot, computes a raw force, projects it onto each axis, applies
condition/envelope/filter shaping, sums per axis, clamps to `±0x7fff`, and stores
it → you read it with `getAxisTorque()` and drive the motor.

**Each class in one line:**

- **`Library`** — header-only facade; owns a `Calculator` + a `HidParser` and
  forwards. Pure convenience. *(was: the firmware's top-level wheel class)*
- **`HidParser`** — the USB decoder. One switch on report ID turns incoming bytes
  into effect-pool mutations and produces Feature replies. *(was `HidFFB`)*
- **`Calculator`** — the math. Per tick it turns the effect pool plus the current
  axis state into per-axis torque. *(was `EffectsCalculator`)*
- **`Effect`** — a flat struct holding one effect's parameters and its biquad
  filter slots. The pool is a fixed `std::array`. *(was `FFB_Effect`)*
- **`Biquad`** — a low-pass (transposed direct form II) that smooths constant
  force and the condition effects. *(was the `Filters` class)*

---

## 2. The effect pool and an effect's lifecycle

All effects live in one fixed array inside the `Calculator`:

```cpp
std::array<Effect, FFB_MAX_EFFECTS> effects;   // default 40 slots
```

A slot is **free** when its `type == FFB_EFFECT_NONE`. An `Effect`
(`ffb_effect.h`) is a flat struct holding every parameter the math needs: `type`,
`state`, `gain`, `magnitude`, `offset`, `startLevel`/`endLevel` (ramp),
`phase`/`period` (periodic), `duration`, envelope levels/times, an
`axisMagnitudes[]` projection vector, a `conditions[]` block per axis, and —
critically for the heap-free design — an inline `Biquad filter[FFB_MAX_AXIS]`
plus a `bool filter_active[FFB_MAX_AXIS]` flag.

A DirectInput effect is born and dies through this sequence, all driven by the
host; the parser implements each step:

1. **Create New Effect** (Feature report `0x11`) — the host says "I want a
   CONSTANT effect." `HidParser::newEffect()` calls
   `Calculator::findFreeEffect(type)`, which returns the first slot whose
   `type == NONE`. The parser records the block index and the pool space that
   is still free.
2. **Block Load poll** (Feature GET `0x12`) — the host polls "did that allocation
   succeed, and at what index?" `hidGet()` fills a `FFB_BlockLoad_Feature_Data_t`
   with the 1-based index and `loadStatus = 1` (success).
3. **Set Effect** (`0x01`) plus the type-specific parameter reports (constant
   `0x05`, ramp `0x06`, periodic `0x04`, envelope `0x02`, condition `0x03`) — the
   host fills in the numbers. Each maps to a `set*` handler that writes fields.
4. **Effect Operation** (`0x0A`) — start / stop / start-solo. On start the parser
   stamps `startTime = millis() + startDelay` and sets `state = 1`.
5. **Block Free** (`0x0B`) — `HidParser::freeEffect(idx)` has
   `Calculator::freeEffect(idx)` reset the slot to a default `Effect()` and clear
   `filter_active[]`, returning it to the pool.

The host can also poll the **PID Pool** feature report (`0x13`) to learn
`FFB_MAX_EFFECTS`, send **Device Control** (`0x0C`) to enable/disable/reset all
FFB, and **Device Gain** (`0x0D`) to set the master gain.

---

## 3. HidParser — decoding inbound reports

`hidOut(report_id, buffer, bufsize)` is a direct port of upstream
`HidFFB::hidOut`. It subtracts `FFB_ID_OFFSET`, switches on the report ID, and
copies the buffer into the matching packed struct:

| Report ID | Handler | Writes |
|---|---|---|
| `0x01` Set Effect | `setEffect` | type, gain, duration, direction → `axisMagnitudes[]`, flags |
| `0x02` Set Envelope | `setEnvelope` | attack/fade levels + times, `useEnvelope = true` |
| `0x03` Set Condition | `setCondition` | one `conditions[]` block (offset, coeffs, saturations, deadband) |
| `0x04` Set Periodic | `setPeriodic` | magnitude, offset, phase, period |
| `0x05` Set Constant Force | `setConstantForce` | magnitude |
| `0x06` Set Ramp | `setRamp` | startLevel, endLevel |
| `0x0A` Effect Operation | `setEffectOperation` | start (stamp `startTime`, `state=1`) / stop |
| `0x0B` Block Free | `freeEffect` | frees the slot |
| `0x0C` Device Control | `controlCmd` | enable / disable / reset-all |
| `0x0D` Device Gain | `setGain` | global gain |
| `0x11` Create New Effect | `newEffect` | allocate a slot (arrives as a Feature report) |

Because the structs in `ffb_defs.h` are packed and laid out exactly like the USB
wire format, decoding is one `memcpy` and a few field copies — no byte-by-byte
parsing. The layouts line up by construction: the host computes its offsets from
the same report descriptor the library ships, and a `static_assert` per struct
stops the build if a compiler pads one. A report with fewer bytes than its
handler reads is dropped, and so is one that names an effect block outside the
pool.

`hidGet(report_id, reply, reqlen)` handles the two **Feature GET** replies the
host polls — Block Load (`0x12`) and PID Pool (`0x13`) — by `memcpy`-ing the
prepared reply struct and returning its size; any other ID returns 0.

When an effect is created or operated on and a send-report callback is
registered, the parser also assembles a PID State input report (`0x02`) and calls
the callback so the host learns the effect-block state.

> **A few robustness fixes vs. upstream** live in the parser: `setCondition`
> clamps the axis index instead of risking an out-of-bounds write, the 3-axis
> direction vector is written to the correct index, short reports are dropped
> instead of being read past their end, and the Block Load reply reports the pool
> space that is really free (upstream never gives a freed block back in that
> count). They change behavior only in cases the original handled incorrectly.

---

## 4. Calculator — the per-tick loop

The whole tick, with the time source swapped in:

```cpp
void Calculator::calculate() {
    for (axis) axis_torque[axis] = 0;            // start from zero
    if (!isActive()) return;                     // FFB disabled → all zero

    int32_t forces[FFB_MAX_AXIS] = {0};
    TickTime now = sampleClocks();               // both clocks, read once

    for (each effect slot) {
        if (state == INACTIVE) continue;

        // An effect still inside its startDelay is skipped this tick;
        // a finite-duration effect past its duration goes inactive.
        if (startTime lies ahead of now.ms) continue;         // start delay
        uint32_t elapsed_ms = now.ms - startTime;             // unsigned delta
        if (finite_duration && elapsed_ms > duration) { state = INACTIVE; continue; }

        int32_t force = calcNonConditionEffectForce(effect, elapsed_ms, now);  // base scalar
        for (axis) forces[axis] += calcComponentForce(effect, force, axis);
    }

    for (axis) axis_torque[axis] = clip(forces[axis], -0x7fff, 0x7fff);
}
```

So each tick: zero, bail if inactive, read the clocks, walk every slot handling
start delay and expiry, compute a base force, fan it out across axes,
accumulate, clamp. Condition effects
(spring/damper/friction/inertia) ignore the base `force` and read the axis state
directly inside `calcComponentForce`.

---

## 5. Waveform generators

`calcNonConditionEffectForce()` returns a single scalar "force vector" for
constant/ramp/periodic effects, before per-axis projection. If `useEnvelope`, the
magnitude is first reshaped by `getEnvelopeMagnitude()` ([§7](#7-envelopes)). Then
a switch on `type`:

- **Constant** — `force = magnitude`.
- **Ramp** — linear interpolation from `startLevel` to `endLevel` across
  `duration`, using the elapsed time down to the sub-millisecond part; past the
  duration it holds `endLevel`.
- **Square** — `±magnitude` depending on whether `(elapsed + phase) mod period`
  is in the first or second half, plus `offset` (millisecond-aligned).
- **Triangle / Sawtooth Up / Sawtooth Down** — compute a `remainder` within the
  period from the elapsed time and `phase`, then a linear slope between
  `offset ± magnitude`.
- **Sine** — `offset + sin(2π·(t·freq + phase))·magnitude`, with `freq = 1/period`
  and `phase` normalized by 35999.

The result is finally scaled by the effect's own gain:
`return force_vector * effect->gain / 255`.

> These waveform formulas match OpenFFBoard, so a given effect produces the same
> output as the original firmware. Two things are done differently on purpose.
> The elapsed time is an unsigned difference to the effect's start, reduced
> modulo the period in integer arithmetic before it becomes a `float` (see
> [§9](#9-time-handling-and-overflow)), so the result does not depend on how long
> the device has been powered. And everything is evaluated in single precision,
> where the firmware lets a few terms promote to `double`: that keeps the whole
> tick on a single-precision FPU, at the price of a rounding step — a count or
> so out of 32767 — in an occasional sample.

---

## 6. Projection, conditions and filtering

`calcComponentForce()` routes the base force (or the axis state, for conditions)
onto one axis and applies the per-effect filter. `angle_ratio =
effect->axisMagnitudes[axis]` is the projection coefficient the parser computed
from the host's direction field.

- **Constant** — if a filter is active and tuned (`Fc < 0.5`), the force is
  low-pass filtered first; then (shared with all periodic/ramp types)
  `result = -forceVector · angle_ratio`. The sign flip matches DirectInput's
  convention (a +X effect pushes toward −X torque).
- **Spring** — `result -= calcConditionEffectForce(pos, gain.spring, …, scaler.spring, angle_ratio)`:
  force proportional to displacement from the condition's `cpOffset`, outside the
  deadband, clamped to saturation.
- **Damper** — same condition formula, but the metric is `speed · 40`
  (`INTERNAL_SCALER_DAMPER`), then low-pass filtered.
- **Inertia** — metric is `accel · 4` (`INTERNAL_SCALER_INERTIA`), filtered.
- **Friction** — metric is `speed · 45` (`INTERNAL_SCALER_FRICTION`); below a
  configurable speed threshold (`frictionPctSpeedToRampup`) the output is eased in
  with a half-sine ramp so friction doesn't snap on at the zero crossing; then
  gained, scaled, projected and filtered.

Finally `return result_torque · global_gain / 255` applies the master gain.

The condition core, `calcConditionEffectForce()`, is the classic DirectInput
condition law:

```
if |metric - cpOffset| > deadBand:
    coefficient = (metric > cpOffset ? positiveCoefficient : negativeCoefficient) / 0x7fff
    metric     -= cpOffset + deadBand·sign
    force       = clip(coefficient · gainfactor · scale · metric,
                       -negativeSaturation, +positiveSaturation)
return force · angle_ratio
```

`gainfactor = (gain_val + 1) / 256`. The per-type `scale` values
(`spring 16, damper 4, inertia 2, friction 1`) and gains
(`spring 64, damper 64, inertia 127, friction 254`) are the upstream defaults,
exposed for tuning ([Part I §9](#9-tuning-the-effect-engine)).

> **Direction projection.** When the host sends Set Effect it gives either a
> per-axis enable + angle or a polar direction. The parser turns that into
> `axisMagnitudes[axis]`. For the common 1-axis case with `directionX = 0`, the X
> unit vector is `-1`, so a positive constant force becomes a positive torque
> after the sign flip. `setDirectionEnableMask()` tells the parser which bit in
> `enableAxis` selects polar vs. per-axis mode, for custom descriptors.

---

## 7. Envelopes

For finite-duration effects with an envelope, `getEnvelopeMagnitude()` reshapes
the magnitude over time: during `attackTime` it ramps from `attackLevel` up to the
full magnitude; during the final `fadeTime` it ramps down to `fadeLevel`; in
between it holds. Infinite-duration effects bypass the envelope. The sign of the
original magnitude is preserved (important for constant force).

---

## 8. Biquad filters and the static-allocation design

`ffb::Biquad` is a low-pass in transposed direct form II (the upstream `Filters`
class with its one external `clip<>` inlined). Each effect that needs smoothing (constant,
damper, friction, inertia) gets one biquad **per axis**, stored *inline* in the
`Effect`:

```cpp
struct Effect {
    ...
    Biquad filter[FFB_MAX_AXIS];
    bool   filter_active[FFB_MAX_AXIS] = { false };
};
```

This is the single structural rewrite versus upstream, which used
`std::unique_ptr<Biquad> filter[MAX_AXIS]` (heap). Three call sites changed:

1. `setFilters(effect)` — instead of `make_unique`, it calls
   `effect->filter[i].setBiquad(lowpass, fc/calcfrequency, q·0.01, 0)` and sets
   `filter_active[i] = true`. The per-type cutoff frequencies come from the
   filter-preset table (`constant {500,70}, friction {50,20}, damper {30,40},
   inertia {15,20}` by default), divided by the sample rate. A zero in a preset
   is read as upstream's `checkFilterCoeff()` reads it — `freq 0` as no cutoff,
   `q 0` as `1` — and a cutoff at Nyquist makes the filter pass its input
   through.
2. `calcComponentForce()` — every `filter != nullptr` check became
   `filter_active[axis]`, and `filter->process()` became `filter.process()`.
3. `freeEffect()` — clears `filter_active[]` instead of resetting smart pointers.

Because the cutoff is `freq / calcfrequency`, the filters are correct only if
`calcfrequency` matches your real loop rate — hence `setSamplerate()` rebuilds the
coefficients of every active filter (`updateFiltersForType` /
`setFilterProfileId` do the same for a profile change).

The upshot: **no heap is ever touched at runtime.** The entire effect pool,
including all filters, is part of the `Library` object's storage — about 6.7 KB
of BSS on a 2-axis, 40-effect default, fixed at compile time.

---

## 9. Time handling and overflow

Everything time-related flows through the two user counters, which
`calculate()` reads once per tick (`sampleClocks()`):

- Effect **start time** is `millis() + startDelay`, stamped when the host starts
  the effect.
- An effect's age is `elapsed_ms = now_ms - startTime`. Effect **start delay**,
  **duration / expiry**, **envelopes** and the square wave use it directly.
- **Ramp**, **sine**, **triangle** and **sawtooth** add the fraction of the
  current millisecond for fine phase resolution: `sub_us = micros() -
  millis()·1000`.
- The periodic generators take `elapsed_ms` modulo the period before any
  conversion to `float`, so the phase stays exact however long the effect — or
  the device — has been running.

All comparisons are deltas (`now - startTime`), so a 32-bit counter wrapping
(~49 days for millis, ~71 minutes for micros) causes no glitch — the subtraction
wraps consistently in unsigned arithmetic. The "is the start still ahead?" test
is wrap-safe too: a start time counts as pending only while it lies at most one
maximum start delay (65.5 s) ahead of the clock.

`sub_us` is meaningful only when both counters are views of one clock. If it
comes out implausibly large (50 ms or more), or there is no `micros()` at all,
it is taken as `0` and the effects are timed in whole milliseconds. The margin
is that generous so that a `millis()` advancing in coarse steps — a 10 ms RTOS
tick, for instance — still gets its fine timing from `micros()`.

> This is where the engine departs from the firmware it was ported from.
> OpenFFBoard subtracts the raw counters in floating point
> (`micros() - startTime·1000`), which stops producing the waveform once
> `micros()` has wrapped — about 71 minutes after power-up — and it applies the
> start delay only to effects with a finite duration.

---

## 10. The HID report descriptor

At enumeration the host reads the report descriptor to discover the device is a
PID-class force-feedback joystick and to learn the byte layout of every report.
`ffb_descriptor.cpp` holds the fully pre-assembled byte arrays
(`hid_1ffb_desc_bytes[]` = 1196 bytes, `hid_2ffb_desc_bytes[]` = 1215 bytes),
byte-identical to upstream OpenFFBoard's generated descriptors. `descriptor1Axis`
/ `descriptor2Axis` just return the pointer and length.

The descriptor and the packed structs in `ffb_defs.h` are two views of the same
contract: the host parses the descriptor to know where, say, `magnitude` sits in
the Set Constant Force report, and the library copies the incoming bytes into a
struct with that exact layout. If you build a custom descriptor with the
`ffb_descriptor.h` macros, the report structs must still match.

---

## 11. The optional helpers, internally

**`MetricsBuilder`** (`ffb_metrics.cpp`) is OpenFFBoard's `Axis::updateMetrics()`:

```
speed = (new_pos - prev_pos) · samplerate;  speed = speedFilter.process(speed)
accel = (speed_raw - last_speed_raw) · samplerate;  accel = accelFilter.process(accel)
```

and `scalePos()` is `Axis::scaleEncValue()` — `(0xffff / degrees) · angle`,
intentionally **un-clamped** so a wheel past its limit produces a scaled value
beyond `±0x7fff`. That overshoot is what the end-stop reads. The first sample
after construction or `reset()` seeds `prev_pos`, as the firmware seeds its
metrics with the real angle, so it yields zero speed rather than a jump from 0°.

**`AxisLocalEffects`** (`ffb_axis_local.cpp`) merges OpenFFBoard's
`Axis::calculateAxisEffects()` and `Axis::updateEndstop()` into one `compute()`:

- **idle spring** — `clip(-pos · idle_scale, -idle_clip, idle_clip)`, active only
  when FFB is off; `idle_scale = 0.5 + strength·0.01`, `idle_clip = strength·35`,
  both worked out from the current strength on each call.
- **damper / inertia / friction** — always-on, intensity-scaled versions
  (`metric · intensity · ratio`, with no deadband/coefficient/saturation), each
  clipped to `±20000` and low-pass filtered. Friction reuses the host effect's
  half-sine ramp-up near zero speed.
- **end-stop** — gated by `cliptest(pos_scaled, -0x7fff, 0x7fff)`: zero inside the
  range, otherwise a restoring torque proportional to degrees of overshoot, scaled
  by `endstop_strength · 25` and directed back toward center. The overshoot is
  `(|pos_scaled| - 0x7fff) · degrees_of_rotation / 0xffff`. The firmware computes
  the same quantity as `|posDegrees| - degrees_of_rotation / 2` from a second copy
  of the angle; deriving it from the scaled position alone means the two can
  never disagree, so the torque cannot end up pointing out of the range.

The caller sums `compute()`'s result with `getAxisTorque()` and clamps — upstream
did this inside `Axis::updateTorque`; the library leaves the summation to you.

---

## 12. Relationship to upstream OpenFFBoard

The effect **math is a faithful port** — the waveform, condition, envelope and
filter formulas produce the same forces as the original firmware. The structural
edits are only at the boundaries:

| Upstream dependency | Replaced with |
|---|---|
| FreeRTOS `Thread` / `Delay` | removed — the engine is polled |
| `PersistentStorage` / flash | removed — coefficients are runtime defaults + setters |
| `CommandHandler` CLI | removed |
| `UsbHidHandler` singleton + `tud_hid_report()` | `hidOut` / `hidGet` + a send-report callback |
| `HAL_GetTick()` / `micros()` | `TimeSource` function pointers |
| `std::vector<unique_ptr<Axis>>` | internal `AxisState[]` in, `axis_torque[]` out |
| `unique_ptr<Biquad> filter[]` (heap) | inline `Biquad filter[]` + `bool filter_active[]` (static) |
| effect statistics / monitor thread / LED hooks | removed |

Class renames: `EffectsCalculator → Calculator`, `HidFFB → HidParser`,
`FFB_Effect → Effect`, `Filters → Biquad`. The per-axis "feel" effects and the
metrics math moved out of the firmware's `Axis` class into the two opt-in helpers.

Where the library deliberately behaves differently from the firmware:

- **Single precision throughout.** A speed decision: no double-precision
  arithmetic anywhere in the tick. Outputs match the firmware to within a
  rounding step ([§5](#5-waveform-generators)).
- **Effect timing by unsigned differences**, immune to both counters wrapping
  and to uptime; the start delay applies to infinite-duration effects as well;
  a ramp holds its end level once its duration has passed
  ([§9](#9-time-handling-and-overflow)).
- **A periodic effect with no period yet** (started before its Set Periodic
  report) stays silent instead of dividing by zero.
- **Reports are length-checked** and the Block Load reply reports the pool space
  that is really free ([§3](#3-hidparser--decoding-inbound-reports)).
- **Filter presets** with a zero in them, or a cutoff at Nyquist, give a
  well-defined filter ([§8](#8-biquad-filters-and-the-static-allocation-design)).
- **The axis-local helper** reads the idle-spring strength live and takes the
  end-stop overshoot from the scaled position
  ([§11](#11-the-optional-helpers-internally)).

---

## 13. Appendix: report IDs and effect types

**Report IDs** (`ffb_defs.h`):

| ID | Name | Dir | Meaning |
|---|---|---|---|
| `0x01` | `HID_ID_EFFREP` | OUT | Set Effect |
| `0x02` | `HID_ID_STATE` / `ENVREP` | IN / OUT | PID State (in) / Set Envelope (out) |
| `0x03` | `HID_ID_CONDREP` | OUT | Set Condition |
| `0x04` | `HID_ID_PRIDREP` | OUT | Set Periodic |
| `0x05` | `HID_ID_CONSTREP` | OUT | Set Constant Force |
| `0x06` | `HID_ID_RAMPREP` | OUT | Set Ramp |
| `0x0A` | `HID_ID_EFOPREP` | OUT | Effect Operation (start/stop) |
| `0x0B` | `HID_ID_BLKFRREP` | OUT | Block Free |
| `0x0C` | `HID_ID_CTRLREP` | OUT | Device Control (enable/disable/reset) |
| `0x0D` | `HID_ID_GAINREP` | OUT | Device Gain |
| `0x11` | `HID_ID_NEWEFREP` | FEAT | Create New Effect |
| `0x12` | `HID_ID_BLKLDREP` | FEAT | Block Load reply |
| `0x13` | `HID_ID_POOLREP` | FEAT | PID Pool reply |

**Effect types** (`ffb_defs.h`): `CONSTANT 0x01`, `RAMP 0x02`, `SQUARE 0x03`,
`SINE 0x04`, `TRIANGLE 0x05`, `SAWTOOTHUP 0x06`, `SAWTOOTHDOWN 0x07`,
`SPRING 0x08`, `DAMPER 0x09`, `INERTIA 0x0A`, `FRICTION 0x0B`, `CUSTOM 0x0C`.

---

*The FFB effect math, HID descriptor layout, and parsing logic originate from
[OpenFFBoard](https://github.com/Ultrawipf/OpenFFBoard) by Yannick Richter and
contributors. This library is a re-packaging for embedded use; design credit is
upstream.*
