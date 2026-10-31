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


#include "ZephyrRgbLed.hpp"

#include <errno.h>

ModuleBase::Descriptor ZephyrRgbLed::desc{task_spawn, custom_command, print_usage};

ZephyrRgbLed::ZephyrRgbLed() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool ZephyrRgbLed::init()
{
	for (size_t led = 0; led < zephyr_rgbled_count(); led++) {
		const int ret = zephyr_rgbled_init(led);

		if (ret == 0) {
			_ready |= 1u << led;

		} else if (ret == -ENODEV) {
			PX4_INFO("%s not ready", zephyr_rgbled_name(led));

		} else {
			PX4_ERR("%s: not an RGB LED (%d)", zephyr_rgbled_name(led), ret);
		}
	}

	for (size_t led = 0; led < zephyr_rgbled_count(); led++) {
		if (_ready & (1u << led)) {
			zephyr_rgbled_set(led, 0, 0, 0);
		}
	}

	return _ready != 0;
}

void ZephyrRgbLed::request_stop()
{
	ModuleBase::request_stop();

	// Run() is scheduled up to 100 ms later
	ScheduleNow();
}

void ZephyrRgbLed::Run()
{
	if (should_exit()) {
		ScheduleClear();
		set_color(0, 0, 0);
		exit_and_cleanup(desc);
		return;
	}

	LedControlData led_control_data;

	if (_led_controller.update(led_control_data) == 1) {
		const uint8_t brightness = led_control_data.leds[0].brightness;

		switch (led_control_data.leds[0].color) {
		case led_control_s::COLOR_RED:
			set_color(brightness, 0, 0);
			break;

		case led_control_s::COLOR_GREEN:
			set_color(0, brightness, 0);
			break;

		case led_control_s::COLOR_BLUE:
			set_color(0, 0, brightness);
			break;

		// amber is yellow, as with the other RGB LED drivers
		case led_control_s::COLOR_AMBER:
		case led_control_s::COLOR_YELLOW:
			set_color(brightness, brightness, 0);
			break;

		case led_control_s::COLOR_PURPLE:
			set_color(brightness, 0, brightness);
			break;

		case led_control_s::COLOR_CYAN:
			set_color(0, brightness, brightness);
			break;

		case led_control_s::COLOR_WHITE:
			set_color(brightness, brightness, brightness);
			break;

		default:
			set_color(0, 0, 0);
			break;
		}
	}

	ScheduleDelayed(_led_controller.maximum_update_interval());
}

void ZephyrRgbLed::set_color(uint8_t red, uint8_t green, uint8_t blue)
{
	if ((red == _red) && (green == _green) && (blue == _blue)) {
		return;
	}

	_red = red;
	_green = green;
	_blue = blue;

	for (size_t led = 0; led < zephyr_rgbled_count(); led++) {
		if (_ready & (1u << led)) {
			zephyr_rgbled_set(led, red, green, blue);
		}
	}
}

int ZephyrRgbLed::print_status()
{
	for (size_t led = 0; led < zephyr_rgbled_count(); led++) {
		PX4_INFO("%s: %s", zephyr_rgbled_name(led), (_ready & (1u << led)) ? "ready" : "not ready");
	}

	PX4_INFO("red: %u, green: %u, blue: %u", _red, _green, _blue);

	return 0;
}

int ZephyrRgbLed::task_spawn(int argc, char *argv[])
{
	if (zephyr_rgbled_count() == 0) {
		PX4_INFO("no RGB LED in the devicetree");
		return PX4_OK;
	}

	ZephyrRgbLed *instance = new ZephyrRgbLed();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	if (!instance->init()) {
		PX4_INFO("no RGB LED ready");
		delete instance;
		return PX4_OK;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;

	instance->ScheduleNow();

	return PX4_OK;
}

int ZephyrRgbLed::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ZephyrRgbLed::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Shows the system status of the led_control topic, which Commander publishes,
on the RGB LEDs of the px4,leds devicetree node, with the Zephyr LED API.

An RGB LED is a multi-color LED with red, green or blue channels, such as the
LED of an NCP5623C or a leds-group-multicolor of single-color LEDs. An LED
whose device has the zephyr,deferred-init property, such as the LED of a GPS
module, is initialized when the driver starts, and left out if it doesn't
respond.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("zephyr_rgbled", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int zephyr_rgbled_main(int argc, char *argv[])
{
	return ModuleBase::main(ZephyrRgbLed::desc, argc, argv);
}
