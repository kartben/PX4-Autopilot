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


#include <px4_leds.h>

#include <drivers/drv_board_led.h>

#include <errno.h>
#include <stddef.h>

#include <zephyr/device.h>
#include <zephyr/drivers/led.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(px4_leds) == 1, "only one px4,leds node is supported");

#define BOARD_LED(prop) \
	COND_CODE_1(DT_NODE_HAS_PROP(PX4_LEDS_NODE, prop), \
	(PX4_LED_DT_SPEC_GET(DT_PHANDLE(PX4_LEDS_NODE, prop))), ({0}))

/* indexed by the LEDs of PX4, without a device for the LEDs that the board doesn't have */
static const struct led_dt_spec board_leds[] = {
	[LED_BLUE] = BOARD_LED(blue),
	[LED_RED] = BOARD_LED(red),
	[LED_SAFETY] = BOARD_LED(safety),
	[LED_GREEN] = BOARD_LED(green),
};

/* the board LEDs that are on */
static atomic_t leds_on;

static const struct led_dt_spec *get_led(int led)
{
	if ((led < 0) || (led >= (int)ARRAY_SIZE(board_leds))) {
		return NULL;
	}

	const struct led_dt_spec *spec = &board_leds[led];

	if ((spec->dev == NULL) || !led_is_ready_dt(spec)) {
		return NULL;
	}

	return spec;
}

static void set_led(const struct led_dt_spec *spec, bool on)
{
	if (on) {
		(void)led_on_dt(spec);

	} else {
		(void)led_off_dt(spec);
	}
}

int px4_board_leds_init(void)
{
	int ret = 0;

	atomic_clear(&leds_on);

	for (size_t i = 0; i < ARRAY_SIZE(board_leds); i++) {
		if (board_leds[i].dev == NULL) {
			continue;
		}

		if (!led_is_ready_dt(&board_leds[i])) {
			ret = -ENODEV;
			continue;
		}

		(void)led_off_dt(&board_leds[i]);
	}

	return ret;
}

void px4_board_led_set(int led, bool on)
{
	const struct led_dt_spec *spec = get_led(led);

	if (spec == NULL) {
		return;
	}

	atomic_set_bit_to(&leds_on, led, on);
	set_led(spec, on);
}

void px4_board_led_toggle(int led)
{
	const struct led_dt_spec *spec = get_led(led);

	if (spec == NULL) {
		return;
	}

	const bool was_on = (atomic_xor(&leds_on, BIT(led)) & BIT(led)) != 0;
	set_led(spec, !was_on);
}
