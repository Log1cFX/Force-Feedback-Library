# Force Feedback Library

A standalone C++ force-feedback engine extracted from the
[OpenFFBoard](https://github.com/Ultrawipf/OpenFFBoard) firmware. It decodes the
USB force-feedback reports a game sends, runs the effect math, and hands you a
single torque value per axis to drive your motor. The effect math is a faithful
port of the original and produces the same forces; all the project-specific glue
(STM32 HAL, TinyUSB, FreeRTOS, flash storage, the configurator CLI) has been
removed so the library drops into any bare-metal microcontroller project.

- **No heap** — the effect pool and all filters live inside the `Library` object.
- **No RTOS** — a plain polled engine; you call it once per control tick.
- **No dependencies** — just C++11, with a plain-C wrapper for C projects.
- **USB-stack / motor / encoder agnostic** — you wire those up.

> **Full guide:** see **[DOCUMENTATION.md](DOCUMENTATION.md)**. Part I is a
> step-by-step usage walkthrough (C++ **and** C); Part II explains the internals.

## What it does

- Decodes USB HID PID reports from the host — Constant, Ramp, Periodic
  (Sine / Square / Triangle / Sawtooth), Spring, Damper, Friction and Inertia,
  with envelope shaping and condition blocks.
- Maintains a fixed-size pool of effect slots (no allocation at runtime).
- Computes the torque per axis every tick.
- Ships a ready-to-use HID report descriptor (1-axis and 2-axis variants).

It does **not** care which USB stack (TinyUSB, ST USB, LUFA, …), motor driver
(PWM, BLDC/FOC, CAN, …) or encoder you use, or whether you run an RTOS — those
are yours to wire up.

## Quick start

The whole contract: feed it the host's USB reports and the wheel state, read back
a torque.

```cpp
#include "ffb/ffb.h"

// 1. One-time setup (1-axis device).
ffb::Library lib(/*axis_count=*/1, { my_millis, my_micros });
lib.setSendReportCallback(my_usb_send);            // optional status reports

// 2. Hand the HID report descriptor to your USB stack at enumeration.
uint16_t len;
const uint8_t* desc = ffb::Library::descriptor1Axis(&len);
my_usb_set_hid_descriptor(desc, len);

// 3. Forward the host's reports from your USB Set/Get Report callbacks.
void on_usb_set_report(uint8_t id, const uint8_t* buf, uint16_t n) { lib.hidOut(id, buf, n); }
uint16_t on_usb_get_report(uint8_t id, uint8_t* buf, uint16_t n)   { return lib.hidGet(id, buf, n); }

// 4. Every control tick (1 kHz default).
lib.setAxisState(0, { wheel_pos_int16, wheel_speed_dps, wheel_accel_dpss });
lib.calculate();
int32_t torque = lib.getAxisTorque(0);             // -0x7fff .. 0x7fff
```

The same five steps in C, via `ffb/ffb_c.h`:

```c
#include "ffb/ffb_c.h"

ffb_lib_t* lib = ffb_create(1, my_millis, my_micros);
ffb_set_send_report_callback(lib, my_usb_send);

uint16_t len;
const uint8_t* desc = ffb_descriptor_1axis(&len);
my_usb_set_hid_descriptor(desc, len);

/* in your USB callbacks */
ffb_hid_out(lib, id, buf, n);
uint16_t got = ffb_hid_get(lib, id, buf, n);

/* per tick */
ffb_set_axis_state(lib, 0, wheel_pos_int16, wheel_speed_dps, wheel_accel_dpss);
ffb_calculate(lib);
int32_t torque = ffb_get_axis_torque(lib, 0);
```

See [DOCUMENTATION.md](DOCUMENTATION.md) for the units, the per-step breakdown,
and TinyUSB wiring.

## Library layout

```
include/ffb/
├── ffb.h              Main facade (the only header most users need)
├── ffb_c.h            Plain-C wrapper
├── ffb_config.h       Compile-time knobs (FFB_MAX_AXIS, FFB_MAX_EFFECTS, …)
├── ffb_descriptor.h   Pre-built HID descriptor + assembly macros
├── ffb_calculator.h   The math engine (advanced)
├── ffb_parser.h       The USB report decoder (advanced)
├── ffb_effect.h       Internal Effect struct
├── ffb_biquad.h       Biquad filter
├── ffb_metrics.h      Optional: derive speed/accel from raw position
├── ffb_axis_local.h   Optional: idle spring, end-stop, axis-local effects
├── ffb_metrics_c.h    Optional: C wrapper for ffb_metrics.h
└── ffb_axis_local_c.h Optional: C wrapper for ffb_axis_local.h
src/                   Implementations
examples/
├── minimal_cpp.cpp           Smallest C++ integration
├── minimal_c.c               Same, using the C API
├── c_wrappers.c              C API + metrics/axis-local helpers + tuning
├── tinyusb_glue.cpp          TinyUSB report-callback wiring
└── usb_descriptors_tinyusb.c Device/config/string descriptors you supply
tests/smoke_test.cpp   Smoke test
CMakeLists.txt
DOCUMENTATION.md
```

## Compile-time configuration

Override these before including any `ffb/*` header, or on the compiler command
line (`-DFFB_MAX_AXIS=1`):

| Macro | Default | Purpose |
|---|---|---|
| `FFB_MAX_AXIS` | `2` | Number of physical axes (1, 2, or 3) |
| `FFB_MAX_EFFECTS` | `40` | Effect-slot pool size |
| `FFB_DEFAULT_SAMPLERATE_HZ` | `1000.0f` | Initial calculation rate |
| `FFB_ID_OFFSET` | `0` | Added to every report ID (for composite HID stacks) |
| `FFB_LOG(msg)` | no-op | Hook for debug logging |

## Platform requirements

Supply two free-running counters:

```c
uint32_t millis(void);  // milliseconds since boot
uint32_t micros(void);  // microseconds since boot
```

Both may wrap around — the library only uses deltas. If your MCU has only
`millis()`, derive `micros()` from any hardware timer.

No heap is used at runtime: the effect pool and biquad filters are allocated
statically inside the `Library`. With defaults (`FFB_MAX_EFFECTS=40`,
`FFB_MAX_AXIS=2`) the footprint is about **9 KB** of BSS.

## Building

```sh
cmake -B build
cmake --build build        # produces libffb.a
```

Options: `-DFFB_BUILD_EXAMPLES=ON`, `-DFFB_BUILD_TESTS=ON`,
`-DFFB_BUILD_C_WRAPPER=ON` (on by default). Or just add the `src/*.cpp` files to
your existing build and put `include/` on the include path — compile
`ffb_metrics*.cpp` / `ffb_axis_local*.cpp` only if you use those helpers.

## Optional helpers

Both are independent and cost zero bytes if you don't use them:

- **`ffb_metrics.h`** derives filtered speed and acceleration (and a scaled
  position) from a raw position stream, when you don't already have them. Same
  math as the original `Axis::updateMetrics()`.
- **`ffb_axis_local.h`** adds the "feel" effects the host does *not* request — an
  idle self-centering spring (when FFB is off), a software end-stop at the
  rotation limit, and always-on damper/friction/inertia. Same math as
  `Axis::calculateAxisEffects()` / `Axis::updateEndstop()`. You add its output on
  top of `getAxisTorque()`.

Each has a matching C wrapper (`ffb_metrics_c.h`, `ffb_axis_local_c.h`).

## What the host sees

The shipped HID **report** descriptor enumerates as a standard HID joystick on
the PID (Physical Input Device) usage page, which Windows' DirectInput stack
recognises as a force-feedback device. Verify it by:

1. Plugging in the device.
2. Running `joy.cpl` → Properties → Test.
3. Launching any DirectInput-aware game.

> **You still write your own USB configuration descriptor.** The library only
> provides the HID *report* descriptor; the USB *device*, *configuration*, and
> *string* descriptors (VID/PID, endpoints, product name) are yours to supply.
> The HID interface must expose an **OUT** endpoint as well as an IN endpoint
> (FFB is host-driven), e.g. TinyUSB's `TUD_HID_INOUT_DESCRIPTOR`. See
> `examples/usb_descriptors_tinyusb.c` and DOCUMENTATION.md (Part I §6).

## Supported effects

All standard DirectInput PID effects:

| Effect | Implementation |
|---|---|
| Constant Force | Direct magnitude, optional low-pass filter |
| Ramp | Linear interpolation start → end |
| Sine, Square, Triangle, Saw Up/Down | Periodic generators with phase + offset |
| Spring | Coefficient × (position − cpOffset), deadband + saturation |
| Damper | Same, on filtered velocity |
| Inertia | Same, on filtered acceleration |
| Friction | Velocity-based, with a sinusoidal ramp-up below a configurable speed threshold |
| Envelope | Attack/sustain/fade modulation on top of magnitude |
| Conditions | Single or per-axis condition parameter blocks |

The math is a faithful port of OpenFFBoard's `EffectsCalculator.cpp` and produces
the same forces.

## Testing
This library has been tested on one of my own projects to make an FFB steering 
wheel. All effects seem to work fine when tested with WheelCheck; it is able to
generate a lut curve. I have tested the library on Assetto Corsa; force feedback
was impeccable. However, some functionalities haven't been tested yet, such as 
2-axis force feedback. Theoretically, it should function in the exact same manner
as openFFBoard.

## Credits

The FFB effect math, the HID descriptor layout, and the underlying parsing
logic comes from [OpenFFBoard](https://github.com/Ultrawipf/OpenFFBoard).
This library is a re-packaging for embedded use.

This library wouldn't exist without the incredible work of Yannick Richter.
He managed to make a working force feedback system, and instead of selling
the software, like some other projects do, he made it open for everybody. 
The real work wasn't done by me; the only thing I did was repackaging it with
the help of AI. I personally thank him and other contributors for their hard 
work and for making this possible.
