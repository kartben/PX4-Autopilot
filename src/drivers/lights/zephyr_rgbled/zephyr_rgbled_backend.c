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


#include "zephyr_rgbled_backend.h"

#include <errno.h>
#include <stdbool.h>

#include <px4_leds.h>

#include <zephyr/device.h>
#include <zephyr/sys/util.h>

#if defined(CONFIG_PX4_LEDS) && DT_NODE_HAS_PROP(PX4_LEDS_NODE, rgb)

#include <zephyr/drivers/led.h>
#include <zephyr/dt-bindings/led/led.h>

/* color channels of an LED */
#define MAX_CHANNELS 8

#define RGB_LED(node, prop, idx) PX4_LED_DT_SPEC_GET(DT_PHANDLE_BY_IDX(node, prop, idx))

static const struct led_dt_spec rgb_leds[] = {
	DT_FOREACH_PROP_ELEM_SEP(PX4_LEDS_NODE, rgb, RGB_LED, (,))
};

BUILD_ASSERT(ARRAY_SIZE(rgb_leds) <= ZEPHYR_RGBLED_MAX_LEDS, "px4,leds: too many RGB LEDs");

/* the color of each channel of the LEDs, LED_COLOR_ID_* */
static uint8_t channel_colors[ARRAY_SIZE(rgb_leds)][MAX_CHANNELS];
static uint8_t channel_count[ARRAY_SIZE(rgb_leds)];

size_t zephyr_rgbled_count(void)
{
	return ARRAY_SIZE(rgb_leds);
}

const char *zephyr_rgbled_name(size_t led)
{
	return rgb_leds[led].dev->name;
}

int zephyr_rgbled_init(size_t led)
{
	const struct led_dt_spec *spec = &rgb_leds[led];

	if (!device_is_ready(spec->dev)) {
		/* a device with zephyr,deferred-init that doesn't respond stays not ready */
		(void)device_init(spec->dev);
	}

	if (!device_is_ready(spec->dev)) {
		return -ENODEV;
	}

	const struct led_info *info;
	const int ret = led_get_info_dt(spec, &info);

	if (ret != 0) {
		return ret;
	}

	if ((info->num_colors > MAX_CHANNELS) || (info->color_mapping == NULL)) {
		return -ENOTSUP;
	}

	bool rgb = false;

	for (uint8_t i = 0; i < info->num_colors; i++) {
		const uint8_t color = info->color_mapping[i];
		channel_colors[led][i] = color;
		rgb = rgb || (color == LED_COLOR_ID_RED) || (color == LED_COLOR_ID_GREEN) || (color == LED_COLOR_ID_BLUE);
	}

	channel_count[led] = info->num_colors;

	return rgb ? 0 : -ENOTSUP;
}

int zephyr_rgbled_set(size_t led, uint8_t red, uint8_t green, uint8_t blue)
{
	uint8_t values[MAX_CHANNELS];

	for (uint8_t i = 0; i < channel_count[led]; i++) {
		switch (channel_colors[led][i]) {
		case LED_COLOR_ID_RED:
			values[i] = red;
			break;

		case LED_COLOR_ID_GREEN:
			values[i] = green;
			break;

		case LED_COLOR_ID_BLUE:
			values[i] = blue;
			break;

		default:
			values[i] = 0;
			break;
		}
	}

	return led_set_color_dt(&rgb_leds[led], channel_count[led], values);
}

#else

size_t zephyr_rgbled_count(void)
{
	return 0;
}

const char *zephyr_rgbled_name(size_t led)
{
	return NULL;
}

int zephyr_rgbled_init(size_t led)
{
	return -ENODEV;
}

int zephyr_rgbled_set(size_t led, uint8_t red, uint8_t green, uint8_t blue)
{
	return -ENODEV;
}

#endif
