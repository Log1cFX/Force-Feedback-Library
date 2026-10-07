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
 * ffb_config.h  (template)
 *
 * Compile-time configuration for the FFB library.
 *
 * This file is NOT part of the library - it belongs to your project:
 *
 *   1. Copy it into your own source tree, outside the library folder
 *      (next to your tusb_config.h is a natural spot).
 *   2. Edit the values below.
 *   3. Put its directory on the include path of every source file that
 *      uses the library, the library's own sources included. With the
 *      bundled CMake that is -DFFB_CONFIG_DIR=<that directory>.
 *
 * The library includes it by name ("ffb_config.h"), so keep the filename.
 * Every option is optional: delete a #define to fall back to the default
 * (the values below are the defaults). Keep it to preprocessor directives,
 * since it is pulled into every library source file.
 *
 * The bundled examples and the smoke test are built against this copy.
 */

#ifndef FFB_CONFIG_H_
#define FFB_CONFIG_H_

/* Number of physical axes the device exposes. Must be 1, 2, or 3.
 * Use 1 for a wheel or a single pedal to save RAM. */
#define FFB_MAX_AXIS 2

/* Number of simultaneous effects the device can hold. Must be 1 to 127.
 * The host's PID pool report advertises this value; 40 is the OpenFFBoard
 * default. */
#define FFB_MAX_EFFECTS 40

/* Default effect-calculation rate, in Hz. Used to initialise biquad
 * filter coefficients before the user calls setSamplerate(). */
#define FFB_DEFAULT_SAMPLERATE_HZ 1000.0f

/* Offset added to every HID report ID before transmission. 0 matches
 * the OpenFFBoard descriptor; advanced users with composite HID stacks
 * may shift the IDs to avoid collisions. */
#define FFB_ID_OFFSET 0

/* Optional printf-style debug log hook. Leave it undefined for no
 * logging, or route it to your own logger to capture effect lifecycle
 * events and anything the library has to correct or refuse (a malformed
 * report, a setup mistake), for example:
 *
 *   #include <stdio.h>
 *   #define FFB_LOG(...) printf(__VA_ARGS__)
 *
 * It is called from the context that calls the library (your USB callback,
 * your control tick), so the logger must be safe to use there. */
/* #define FFB_LOG(...) my_log(__VA_ARGS__) */

#endif /* FFB_CONFIG_H_ */
