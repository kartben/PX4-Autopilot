/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


/**
 * @file zephyr_rgbled_backend.h
 *
 * RGB LEDs of the px4,leds devicetree node, with the Zephyr LED API.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ZEPHYR_RGBLED_MAX_LEDS 8

/**
 * Get the number of RGB LEDs of the devicetree.
 */
size_t zephyr_rgbled_count(void);

/**
 * Get the name of the device of an RGB LED.
 */
const char *zephyr_rgbled_name(size_t led);

/**
 * Initialize the device of an RGB LED if it has the zephyr,deferred-init
 * property, and get its color channels.
 * @return 0, -ENODEV if the device is not ready, -ENOTSUP if the LED has no
 *         red, green nor blue channel, or another negative error code
 */
int zephyr_rgbled_init(size_t led);

/**
 * Set the red, green and blue channels of an initialized RGB LED, from 0 to
 * 255. Its channels of other colors are off.
 * @return 0, or a negative error code
 */
int zephyr_rgbled_set(size_t led, uint8_t red, uint8_t green, uint8_t blue);

#ifdef __cplusplus
}
#endif
