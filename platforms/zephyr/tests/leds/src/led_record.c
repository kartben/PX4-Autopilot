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


#include "led_record.h"

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/led/led_fake.h>
#include <zephyr/dt-bindings/led/led.h>
#include <zephyr/fff.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

DEFINE_FFF_GLOBALS;

static const struct led_dt_spec fake_rgb = LED_DT_SPEC_GET(DT_NODELABEL(test_fake_rgb));
static const uint8_t fake_rgb_colors[] = DT_PROP(DT_NODELABEL(test_fake_rgb), color_mapping);

static const struct gpio_dt_spec gpio_leds[] = {
	[TEST_LED_BLUE] = GPIO_DT_SPEC_GET(DT_NODELABEL(test_blue_led), gpios),
	[TEST_LED_RED] = GPIO_DT_SPEC_GET(DT_NODELABEL(test_red_led), gpios),
	[TEST_LED_GREEN] = GPIO_DT_SPEC_GET(DT_NODELABEL(test_green_led), gpios),
	[TEST_LED_RGB_RED] = GPIO_DT_SPEC_GET(DT_NODELABEL(test_rgb_red), gpios),
	[TEST_LED_RGB_GREEN] = GPIO_DT_SPEC_GET(DT_NODELABEL(test_rgb_green), gpios),
	[TEST_LED_RGB_BLUE] = GPIO_DT_SPEC_GET(DT_NODELABEL(test_rgb_blue), gpios),
};

static struct k_spinlock lock;
static struct led_event events[LED_RECORD_MAX_EVENTS];
static size_t event_count;

static int record_set_color(const struct device *dev, uint32_t led, uint8_t num_colors,
			    const uint8_t *color)
{
	if ((dev != fake_rgb.dev) || (led != fake_rgb.index) ||
	    (num_colors != ARRAY_SIZE(fake_rgb_colors))) {
		return -EINVAL;
	}

	struct led_event event = {.time_us = k_ticks_to_us_floor64(k_uptime_ticks())};

	for (size_t i = 0; i < num_colors; i++) {
		switch (fake_rgb_colors[i]) {
		case LED_COLOR_ID_RED:
			event.red = color[i];
			break;

		case LED_COLOR_ID_GREEN:
			event.green = color[i];
			break;

		case LED_COLOR_ID_BLUE:
			event.blue = color[i];
			break;

		case LED_COLOR_ID_WHITE:
			event.white = color[i];
			break;
		}
	}

	K_SPINLOCK(&lock) {
		if (event_count < LED_RECORD_MAX_EVENTS) {
			events[event_count] = event;
		}

		event_count++;
	}

	return 0;
}

void led_record_start(void)
{
	K_SPINLOCK(&lock) {
		event_count = 0;
	}

	/* reset by the fake LED driver before each test */
	fake_led_set_color_fake.custom_fake = record_set_color;
}

size_t led_record_get(struct led_event *copy, size_t max_events)
{
	size_t count;

	K_SPINLOCK(&lock) {
		count = event_count;
		memcpy(copy, events, MIN(MIN(count, LED_RECORD_MAX_EVENTS), max_events) * sizeof(events[0]));
	}

	return count;
}

int test_led_level(enum test_gpio_led led)
{
	return gpio_emul_output_get(gpio_leds[led].port, gpio_leds[led].pin);
}

bool test_led_lit(enum test_gpio_led led)
{
	const bool active_low = (gpio_leds[led].dt_flags & GPIO_ACTIVE_LOW) != 0;

	return (test_led_level(led) == 1) != active_low;
}

void test_led_set(enum test_gpio_led led, bool lit)
{
	(void)gpio_pin_set_dt(&gpio_leds[led], lit ? 1 : 0);
}

bool test_gps_led_ready(void)
{
	return device_is_ready(DEVICE_DT_GET(DT_NODELABEL(test_gps_ncp5623c)));
}

bool test_absent_led_ready(void)
{
	return device_is_ready(DEVICE_DT_GET(DT_NODELABEL(test_absent_ncp5623c)));
}
