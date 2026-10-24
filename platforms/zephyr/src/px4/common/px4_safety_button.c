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
 * @file px4_safety_button.c
 *
 * Safety button of the px4,safety-button devicetree node, read with the Zephyr GPIO API, and safety
 * LED of the px4,leds node, a board LED.
 */

#include <px4_safety_button.h>

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#if defined(CONFIG_PX4_SAFETY_LED)
#include <drivers/drv_board_led.h>
#include <px4_leds.h>
#endif

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(px4_safety_button) == 1, "only one px4,safety-button node is supported");

#define SAFETY_BUTTON_NODE DT_PHANDLE(DT_COMPAT_GET_ANY_STATUS_OKAY(px4_safety_button), button)

BUILD_ASSERT(DT_NODE_HAS_PROP(SAFETY_BUTTON_NODE, gpios), "the button of px4,safety-button has no gpios");

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(SAFETY_BUTTON_NODE, gpios);

int px4_safety_button_init(void)
{
	if (!gpio_is_ready_dt(&button)) {
		return -ENODEV;
	}

#if defined(CONFIG_INPUT_GPIO_KEYS) && DT_NODE_HAS_COMPAT_STATUS(DT_PARENT(SAFETY_BUTTON_NODE), gpio_keys, okay)

	/*
	 * configured by the gpio-keys input driver, with an interrupt, which configuring the GPIO again
	 * could disable
	 */
	if (device_is_ready(DEVICE_DT_GET(DT_PARENT(SAFETY_BUTTON_NODE)))) {
		return 0;
	}

#endif

	return gpio_pin_configure_dt(&button, GPIO_INPUT);
}

bool px4_safety_button_pressed(void)
{
	/* released if the GPIO can't be read */
	return gpio_pin_get_dt(&button) == 1;
}

#if defined(CONFIG_PX4_SAFETY_LED)
void px4_safety_led_set(bool on)
{
	px4_board_led_set(LED_SAFETY, on);
}
#endif
