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

#include <drivers/drv_board_led.h>
#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_config.h>
#include <px4_zephyr.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/led_control.h>

#include <fcntl.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/ztest.h>

#if !defined(BOARD_HAS_CONTROL_STATUS_LEDS) || (BOARD_ARMED_STATE_LED != LED_BLUE) || (BOARD_OVERLOAD_LED != LED_RED)
#error "the generic board has the armed state and overload LEDs of most flight controllers"
#endif

// the timing of the LED controller of PX4 (lib/led/led.h): the fast blinks change every 100 ms, and the breathing
// brightness every 25 ms, in 64 steps, from (0 / 32)^2 to (32 / 32)^2 and back
static constexpr uint32_t BLINK_FAST_US{100000};
static constexpr uint32_t BREATHE_INTERVAL_US{25000};
static constexpr int BREATHE_STEPS{64};

// the update interval of the driver, while no LED breathes
static constexpr uint32_t UPDATE_US{BLINK_FAST_US};

// the latency of the work queue
static constexpr uint32_t LATENCY_US{2000};

// time for the driver to start, and to show a new state
static constexpr uint32_t RESPONSE_US{2 * UPDATE_US + 50000};

static constexpr uint8_t FULL{255};

struct color {
	const char *name;
	uint8_t red;
	uint8_t green;
	uint8_t blue;
};

static uORB::Publication<led_control_s> led_control_pub{ORB_ID(led_control)};

// the board LEDs once PX4 has started, and the LED of the GPS module before the driver initializes it
static bool started_lit[3];
static bool gps_led_ready_at_boot;

static int run(const char *command)
{
	char buffer[64];
	strncpy(buffer, command, sizeof(buffer) - 1);
	buffer[sizeof(buffer) - 1] = '\0';

	char *argv[10] {};
	int argc = 0;
	char *saveptr = nullptr;

	for (char *token = strtok_r(buffer, " ", &saveptr); token != nullptr && argc < 10;
	     token = strtok_r(nullptr, " ", &saveptr)) {
		argv[argc++] = token;
	}

	return px4_zephyr_run_command(argc, argv);
}

static void *setup(void)
{
	static bool initialized = false;

	if (!initialized) {
		initialized = true;
		gps_led_ready_at_boot = test_gps_led_ready();

		// as a bootloader could leave it
		test_led_set(TEST_LED_BLUE, true);

		zassert_ok(px4_zephyr_init());

		started_lit[0] = test_led_lit(TEST_LED_BLUE);
		started_lit[1] = test_led_lit(TEST_LED_RED);
		started_lit[2] = test_led_lit(TEST_LED_GREEN);
	}

	return nullptr;
}

// the board LEDs of /dev/led0, as Commander opens it
static int fd_leds{-1};

// as in commander_helper.cpp, for the board_common.h macros
static int led_on(int led)
{
	return px4_ioctl(fd_leds, LED_ON, led);
}

static int led_off(int led)
{
	return px4_ioctl(fd_leds, LED_OFF, led);
}

static int led_toggle(int led)
{
	return px4_ioctl(fd_leds, LED_TOGGLE, led);
}

static void check_board_leds(bool blue, bool red, bool green)
{
	zassert_equal(test_led_lit(TEST_LED_BLUE), blue, "blue LED %s", blue ? "off" : "on");
	zassert_equal(test_led_lit(TEST_LED_RED), red, "red LED %s", red ? "off" : "on");
	zassert_equal(test_led_lit(TEST_LED_GREEN), green, "green LED %s", green ? "off" : "on");
}

static void board_before(void *fixture)
{
	fd_leds = px4_open(LED0_DEVICE_PATH, O_RDWR);
	zassert_true(fd_leds >= 0, "no %s", LED0_DEVICE_PATH);

	zassert_ok(led_off(LED_BLUE));
	zassert_ok(led_off(LED_RED));
	zassert_ok(led_off(LED_GREEN));
}

static void board_after(void *fixture)
{
	px4_close(fd_leds);
	fd_leds = -1;
}

ZTEST(px4_board_leds, test_start)
{
	// PX4 turns the board LEDs off when it starts
	zassert_false(started_lit[0] || started_lit[1] || started_lit[2], "board LED on");
}

ZTEST(px4_board_leds, test_on_off)
{
	zassert_ok(led_on(LED_BLUE));
	check_board_leds(true, false, false);

	// active low
	zassert_equal(test_led_level(TEST_LED_BLUE), 0);

	zassert_ok(led_off(LED_BLUE));
	check_board_leds(false, false, false);
	zassert_equal(test_led_level(TEST_LED_BLUE), 1);

	zassert_ok(led_on(LED_AMBER));
	check_board_leds(false, true, false);

	zassert_ok(led_on(LED_GREEN));
	check_board_leds(false, true, true);

	// active high
	zassert_equal(test_led_level(TEST_LED_GREEN), 1);

	zassert_ok(led_off(LED_RED));
	zassert_ok(led_off(LED_GREEN));
	check_board_leds(false, false, false);
}

ZTEST(px4_board_leds, test_toggle)
{
	zassert_ok(led_toggle(LED_GREEN));
	check_board_leds(false, false, true);

	zassert_ok(led_toggle(LED_GREEN));
	check_board_leds(false, false, false);

	zassert_ok(led_on(LED_RED));
	zassert_ok(led_toggle(LED_RED));
	check_board_leds(false, false, false);
}

ZTEST(px4_board_leds, test_missing_led)
{
	// the board has no safety LED
	zassert_ok(led_on(LED_SAFETY));
	zassert_ok(led_toggle(LED_SAFETY));
	check_board_leds(false, false, false);
}

ZTEST(px4_board_leds, test_commander)
{
	// led_init() of Commander, which leaves the green LED on
	zassert_ok(led_on(LED_GREEN));
	zassert_ok(led_on(LED_BLUE));
	zassert_ok(led_off(LED_BLUE));
	zassert_ok(led_on(LED_AMBER));
	zassert_ok(led_off(LED_AMBER));
	check_board_leds(false, false, true);

	// not ready to arm or ready to arm: the armed state LED blinks
	for (int i = 0; i < 4; i++) {
		BOARD_ARMED_STATE_LED_TOGGLE();
		check_board_leds(i % 2 == 0, false, true);
	}

	BOARD_ARMED_STATE_LED_TOGGLE();
	check_board_leds(true, false, true);

	// armed
	BOARD_ARMED_STATE_LED_OFF();
	check_board_leds(false, false, true);

	// overload
	BOARD_OVERLOAD_LED_TOGGLE();
	check_board_leds(false, true, true);

	BOARD_OVERLOAD_LED_OFF();
	check_board_leds(false, false, true);
}

ZTEST_SUITE(px4_board_leds, NULL, setup, board_before, board_after, NULL);

// as Commander publishes the system status
static void publish_status(uint8_t color, uint8_t mode)
{
	led_control_s led_control{};
	led_control.led_mask = 0xff;
	led_control.color = color;
	led_control.mode = mode;
	led_control.priority = 0;
	led_control.timestamp = hrt_absolute_time();
	led_control_pub.publish(led_control);
}

// the colors set on the fake RGB LED, too many for the stack of the test
static led_event events[LED_RECORD_MAX_EVENTS];

static size_t get_events()
{
	const size_t count = led_record_get(events, LED_RECORD_MAX_EVENTS);
	zassert_true(count <= LED_RECORD_MAX_EVENTS, "%zu events", count);
	return count;
}

static bool is_color(const led_event &event, uint8_t red, uint8_t green, uint8_t blue)
{
	return (event.red == red) && (event.green == green) && (event.blue == blue) && (event.white == 0);
}

// the 32 PWM steps of the NCP5623C
static uint8_t ncp5623_pwm(uint8_t value)
{
	return value / 8;
}

// check the color of the fake RGB LED, of the RGB LED of GPIO LEDs and of the LED of the GPS module
static void check_color(uint8_t red, uint8_t green, uint8_t blue)
{
	const size_t count = get_events();
	zassert_true(count > 0, "no color set");

	const led_event &last = events[count - 1];
	zassert_true(is_color(last, red, green, blue), "%u %u %u %u instead of %u %u %u 0",
		     last.red, last.green, last.blue, last.white, red, green, blue);

	zassert_equal(test_led_lit(TEST_LED_RGB_RED), red > 0);
	zassert_equal(test_led_lit(TEST_LED_RGB_GREEN), green > 0);
	zassert_equal(test_led_lit(TEST_LED_RGB_BLUE), blue > 0);

	zassert_equal(test_gps_led_register(NCP5623_PWM1), ncp5623_pwm(red));
	zassert_equal(test_gps_led_register(NCP5623_PWM2), ncp5623_pwm(green));
	zassert_equal(test_gps_led_register(NCP5623_PWM3), ncp5623_pwm(blue));
}

static void rgb_before(void *fixture)
{
	// the driver shows the last led_control message when it starts
	publish_status(led_control_s::COLOR_OFF, led_control_s::MODE_OFF);

	zassert_ok(run("zephyr_rgbled start"));
	k_sleep(K_USEC(RESPONSE_US));

	led_record_start();
}

static void rgb_after(void *fixture)
{
	run("zephyr_rgbled stop");
	param_reset(param_find("SYS_RGB_MAXBRT"));
}

ZTEST(px4_rgb_leds, test_start)
{
	// the driver initializes the LED of the GPS module, and leaves out the one that doesn't respond
	zassert_false(gps_led_ready_at_boot);
	zassert_true(test_gps_led_ready());
	zassert_false(test_absent_led_ready());

	// the LEDs are off, the GPS module one at its maximum current
	zassert_false(test_led_lit(TEST_LED_RGB_RED) || test_led_lit(TEST_LED_RGB_GREEN) || test_led_lit(TEST_LED_RGB_BLUE));
	zassert_equal(test_gps_led_register(NCP5623_CURRENT), 31);
	zassert_equal(test_gps_led_register(NCP5623_PWM1), 0);
	zassert_equal(test_gps_led_register(NCP5623_PWM2), 0);
	zassert_equal(test_gps_led_register(NCP5623_PWM3), 0);

	zassert_ok(run("zephyr_rgbled status"));
}

ZTEST(px4_rgb_leds, test_colors)
{
	// amber is yellow, as with the other RGB LED drivers
	static const color colors[] {
		{"red", FULL, 0, 0},
		{"green", 0, FULL, 0},
		{"blue", 0, 0, FULL},
		{"yellow", FULL, FULL, 0},
		{"amber", FULL, FULL, 0},
		{"purple", FULL, 0, FULL},
		{"cyan", 0, FULL, FULL},
		{"white", FULL, FULL, FULL},
	};

	for (const color &c : colors) {
		char command[32];
		snprintf(command, sizeof(command), "led_control on -c %s", c.name);
		zassert_ok(run(command));
		k_sleep(K_USEC(RESPONSE_US));
		check_color(c.red, c.green, c.blue);
	}

	zassert_ok(run("led_control off"));
	k_sleep(K_USEC(RESPONSE_US));
	check_color(0, 0, 0);
}

ZTEST(px4_rgb_leds, test_blink)
{
	constexpr unsigned BLINKS = 3;
	zassert_ok(run("led_control blink -c green -s fast -n 3"));
	k_sleep(K_USEC(2 * (BLINKS + 1) * BLINK_FAST_US + RESPONSE_US));

	const size_t count = get_events();
	zassert_equal(count, 2 * BLINKS, "%zu colors set", count);

	for (size_t i = 0; i < count; i++) {
		const bool on = (i % 2) == 0;
		zassert_true(is_color(events[i], 0, on ? FULL : 0, 0), "color %zu", i);

		if (i > 0) {
			// the controller changes the color at its first update after half a period
			const uint32_t interval_us = events[i].time_us - events[i - 1].time_us;
			const uint32_t min_us = BLINK_FAST_US - LATENCY_US;
			const uint32_t max_us = ((i == 1) ? 2 : 1) * BLINK_FAST_US + LATENCY_US;
			zassert_between_inclusive(interval_us, min_us, max_us, "color %zu after %u us", i, interval_us);
		}
	}

	// back to the color of the lower priority
	check_color(0, 0, 0);
}

ZTEST(px4_rgb_leds, test_breathe)
{
	constexpr uint32_t BREATHE_PERIOD_US = BREATHE_INTERVAL_US * BREATHE_STEPS;
	constexpr int MAX_STEP = BREATHE_STEPS / 2;

	zassert_ok(run("led_control breathe -c blue"));
	k_sleep(K_USEC(2 * BREATHE_PERIOD_US + RESPONSE_US));

	const size_t count = get_events();
	zassert_true(count > 2 * MAX_STEP, "%zu colors set", count);

	// the brightness of the controller, (step / 32)^2, and the times it rises above half of it
	uint8_t max_blue = 0;
	uint64_t half_times_us[3] {};
	size_t half_count = 0;

	for (size_t i = 0; i < count; i++) {
		zassert_true(is_color(events[i], 0, 0, events[i].blue), "color %zu", i);

		bool on_curve = false;

		for (int step = 0; step <= MAX_STEP; step++) {
			on_curve = on_curve || (events[i].blue == (step * step * FULL) / (MAX_STEP * MAX_STEP));
		}

		zassert_true(on_curve, "brightness %u", events[i].blue);

		if (events[i].blue > max_blue) {
			max_blue = events[i].blue;
		}

		if ((i > 0) && (events[i - 1].blue < FULL / 2) && (events[i].blue >= FULL / 2) && (half_count < 3)) {
			half_times_us[half_count++] = events[i].time_us;
		}
	}

	// the brightest step, or the one before if an update was late
	constexpr uint8_t ALMOST_FULL = ((MAX_STEP - 1) * (MAX_STEP - 1) * FULL) / (MAX_STEP * MAX_STEP);
	zassert_true(max_blue >= ALMOST_FULL, "brightest %u", max_blue);

	zassert_true(half_count >= 2, "%zu rises", half_count);
	const uint32_t period_us = half_times_us[1] - half_times_us[0];
	zassert_between_inclusive(period_us, BREATHE_PERIOD_US - BREATHE_INTERVAL_US - LATENCY_US,
				  BREATHE_PERIOD_US + BREATHE_INTERVAL_US + LATENCY_US, "period of %u us", period_us);
}

ZTEST(px4_rgb_leds, test_max_brightness)
{
	zassert_ok(run("zephyr_rgbled stop"));
	const float max_brightness = 0.5f;
	zassert_ok(param_set(param_find("SYS_RGB_MAXBRT"), &max_brightness));
	zassert_ok(run("zephyr_rgbled start"));
	k_sleep(K_USEC(RESPONSE_US));

	zassert_ok(run("led_control on -c white"));
	k_sleep(K_USEC(RESPONSE_US));
	check_color(128, 128, 128);
}

ZTEST(px4_rgb_leds, test_commander)
{
	// disarmed and ready to arm, with a global position
	publish_status(led_control_s::COLOR_GREEN, led_control_s::MODE_BREATHE);
	k_sleep(K_USEC(BREATHE_INTERVAL_US * BREATHE_STEPS));

	size_t count = get_events();
	zassert_true(count > 2);

	for (size_t i = 0; i < count; i++) {
		zassert_true(is_color(events[i], 0, events[i].green, 0), "color %zu", i);
	}

	// armed
	publish_status(led_control_s::COLOR_GREEN, led_control_s::MODE_ON);
	k_sleep(K_USEC(RESPONSE_US));
	check_color(0, FULL, 0);

	// failsafe
	publish_status(led_control_s::COLOR_PURPLE, led_control_s::MODE_ON);
	k_sleep(K_USEC(RESPONSE_US));
	check_color(FULL, 0, FULL);

	// a blink of a higher priority, after which the LED is back to the failsafe color
	led_record_start();
	zassert_ok(run("led_control blink -c yellow -s fast -n 1"));
	k_sleep(K_USEC(4 * BLINK_FAST_US + RESPONSE_US));

	count = get_events();
	zassert_equal(count, 3, "%zu colors set", count);
	zassert_true(is_color(events[0], FULL, FULL, 0));
	zassert_true(is_color(events[1], 0, 0, 0));
	check_color(FULL, 0, FULL);
}

ZTEST(px4_rgb_leds, test_stop)
{
	zassert_ok(run("led_control on -c red"));
	k_sleep(K_USEC(RESPONSE_US));
	check_color(FULL, 0, 0);

	// the LEDs are turned off right away
	const hrt_abstime stop_time = hrt_absolute_time();
	zassert_ok(run("zephyr_rgbled stop"));
	zassert_true(hrt_elapsed_time(&stop_time) < UPDATE_US / 2, "stopped in %u us",
		     (uint32_t)hrt_elapsed_time(&stop_time));
	check_color(0, 0, 0);
}

ZTEST_SUITE(px4_rgb_leds, NULL, setup, rgb_before, rgb_after, NULL);
