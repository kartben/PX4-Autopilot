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


#include "safety_switch.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/input/input.h>
#include <zephyr/sys/atomic.h>

#define BUTTON_NODE DT_NODELABEL(test_safety_button)

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_NODELABEL(test_safety_led), gpios);

static atomic_t key_presses;
static atomic_t key_releases;

void test_button_set(bool pressed)
{
	const bool active_low = (button.dt_flags & GPIO_ACTIVE_LOW) != 0;

	(void)gpio_emul_input_set(button.port, button.pin, (pressed != active_low) ? 1 : 0);
}

static gpio_flags_t button_flags(void)
{
	gpio_flags_t flags = 0;

	(void)gpio_emul_flags_get(button.port, button.pin, &flags);
	return flags;
}

bool test_button_is_input(void)
{
	const gpio_flags_t flags = button_flags();

	return ((flags & GPIO_INPUT) != 0) && ((flags & GPIO_OUTPUT) == 0) && ((flags & GPIO_PULL_DOWN) != 0);
}

bool test_button_has_interrupt(void)
{
	return (button_flags() & GPIO_INT_EDGE_BOTH) == GPIO_INT_EDGE_BOTH;
}

uint32_t test_button_key_presses(void)
{
	return (uint32_t)atomic_get(&key_presses);
}

uint32_t test_button_key_releases(void)
{
	return (uint32_t)atomic_get(&key_releases);
}

int test_led_level(void)
{
	return gpio_emul_output_get(led.port, led.pin);
}

bool test_led_lit(void)
{
	const bool active_low = (led.dt_flags & GPIO_ACTIVE_LOW) != 0;

	return (test_led_level() == 1) != active_low;
}

#if defined(CONFIG_INPUT_GPIO_KEYS)
static void key_event(struct input_event *event, void *user_data)
{
	ARG_UNUSED(user_data);

	if ((event->type == INPUT_EV_KEY) && (event->code == DT_PROP(BUTTON_NODE, zephyr_code))) {
		atomic_inc((event->value != 0) ? &key_presses : &key_releases);
	}
}

INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_PARENT(BUTTON_NODE)), key_event, NULL);
#endif
