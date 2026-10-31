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
 * @file main.cpp
 *
 * Tests of the safety_button driver with the safety switch of a GPS module on an emulated GPIO controller: a button,
 * also read by the gpio-keys input driver, as on the VMU RT1170, and its LED.
 */

#include "safety_switch.h"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_config.h>
#include <px4_zephyr.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/button_event.h>
#include <uORB/topics/led_control.h>
#include <uORB/topics/tune_control.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>

#include <stdlib.h>
#include <string.h>

#include <zephyr/ztest.h>

using namespace time_literals;

#if !defined(BOARD_SAFETY_BUTTON)
#error "the safety button of the devicetree is BOARD_SAFETY_BUTTON()"
#endif

#if defined(CONFIG_PX4_SAFETY_LED) != defined(BOARD_SAFETY_LED)
#error "the safety LED of the devicetree is BOARD_SAFETY_LED()"
#endif

// the driver samples the button every 33 ms, and publishes a safety button event once it has been pressed for 30
// samples, about 1 second
static constexpr hrt_abstime INTERVAL{33_ms};
static constexpr unsigned PRESSED_SAMPLES{30};

// the latency of the work queue
static constexpr hrt_abstime LATENCY{2_ms};

// a press or a release of this length is sampled at least once
static constexpr hrt_abstime SAMPLED{INTERVAL + 17_ms};

// the presses that request pairing are within this time
static constexpr hrt_abstime PAIRING_WINDOW{2_s};

// the gpio-keys input driver reports a press once the button has been stable for its debounce interval
static constexpr hrt_abstime KEY_DEBOUNCE{30_ms};

static bool led_lit_at_start;

static int run(const char *command)
{
	char buffer[64];
	strncpy(buffer, command, sizeof(buffer) - 1);
	buffer[sizeof(buffer) - 1] = '\0';

	char *argv[4] {};
	int argc = 0;
	char *saveptr = nullptr;

	for (char *token = strtok_r(buffer, " ", &saveptr); token != nullptr && argc < 4;
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

		zassert_ok(px4_zephyr_init());
		zassert_ok(run("safety_button start"));

		// the driver leaves the LED off until the armed state is published
		k_sleep(K_USEC(4 * INTERVAL));
		led_lit_at_start = test_led_lit();
	}

	return nullptr;
}

static void sleep_for(hrt_abstime time)
{
	k_sleep(K_USEC(time));
}

// skip the messages published so far
template<typename T>
static void skip_messages(uORB::Subscription &subscription)
{
	T message;

	while (subscription.update(&message)) {
	}
}

// as Commander publishes them
static void publish_state(bool safety_off, bool armed)
{
	static uORB::Publication<vehicle_status_s> vehicle_status_pub{ORB_ID(vehicle_status)};
	static uORB::Publication<actuator_armed_s> actuator_armed_pub{ORB_ID(actuator_armed)};

	vehicle_status_s vehicle_status{};
	vehicle_status.arming_state = armed ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_DISARMED;
	vehicle_status.safety_button_available = true;
	vehicle_status.safety_off = safety_off;
	vehicle_status.timestamp = hrt_absolute_time();
	vehicle_status_pub.publish(vehicle_status);

	actuator_armed_s actuator_armed{};
	actuator_armed.armed = armed;
	actuator_armed.timestamp = hrt_absolute_time();
	actuator_armed_pub.publish(actuator_armed);
}

// the time of the next safety button event, or 0 if there is none within the timeout
static hrt_abstime wait_for_event(uORB::Subscription &subscription, hrt_abstime timeout)
{
	const hrt_abstime start = hrt_absolute_time();

	while (hrt_elapsed_time(&start) < timeout) {
		button_event_s event;

		if (subscription.update(&event)) {
			zassert_true(event.triggered);
			return event.timestamp;
		}

		k_sleep(K_MSEC(1));
	}

	return 0;
}

// press the button, and check the safety button event of the driver after 30 samples
static void press_for_event(uORB::Subscription &subscription)
{
	const hrt_abstime pressed = hrt_absolute_time();
	test_button_set(true);

	const hrt_abstime event = wait_for_event(subscription, PRESSED_SAMPLES * INTERVAL + 100_ms);
	zassert_true(event != 0, "no safety button event");

	// the 30th sample, 29 intervals after the first one, within an interval of the press
	const hrt_abstime delay = event - pressed;
	zassert_between_inclusive(delay, (PRESSED_SAMPLES - 1) * INTERVAL - LATENCY, PRESSED_SAMPLES * INTERVAL + LATENCY,
				  "event %u us after the press", (uint32_t)delay);
}

static void button_before(void *fixture)
{
	test_button_set(false);
	sleep_for(SAMPLED);
}

static void release_button(void *fixture)
{
	test_button_set(false);
}

ZTEST(px4_safety_button, test_input)
{
	zassert_true(test_button_is_input(), "the button isn't an input with a pull-down");

#if defined(CONFIG_INPUT_GPIO_KEYS)
	// configured by the gpio-keys input driver, whose interrupt PX4 leaves enabled
	zassert_true(test_button_has_interrupt(), "the interrupt of the button is disabled");
#endif
}

ZTEST(px4_safety_button, test_press)
{
	uORB::Subscription safety_button_sub{ORB_ID(safety_button)};
	skip_messages<button_event_s>(safety_button_sub);

	const uint32_t key_presses = test_button_key_presses();
	const uint32_t key_releases = test_button_key_releases();

	press_for_event(safety_button_sub);

	// no other event while the button is held
	zassert_equal(wait_for_event(safety_button_sub, 2_s), 0, "event while the button is held");

	// another one when it is pressed again
	test_button_set(false);
	sleep_for(SAMPLED);
	press_for_event(safety_button_sub);

	test_button_set(false);
	sleep_for(KEY_DEBOUNCE + SAMPLED);

#if defined(CONFIG_INPUT_GPIO_KEYS)
	// the gpio-keys input driver still reports the button
	zassert_equal(test_button_key_presses() - key_presses, 2);
	zassert_equal(test_button_key_releases() - key_releases, 2);
#else
	ARG_UNUSED(key_presses);
	ARG_UNUSED(key_releases);
#endif
}

ZTEST(px4_safety_button, test_interrupted_press)
{
	uORB::Subscription safety_button_sub{ORB_ID(safety_button)};
	skip_messages<button_event_s>(safety_button_sub);

	// presses shorter than 1 second, interrupted by releases that the driver samples, as with a bad contact
	for (int i = 0; i < 3; i++) {
		test_button_set(true);
		sleep_for(700_ms);
		test_button_set(false);
		sleep_for(SAMPLED);
	}

	zassert_false(safety_button_sub.updated(), "event for interrupted presses");

	// the event 1 second after the last press
	press_for_event(safety_button_sub);
}

ZTEST(px4_safety_button, test_pairing)
{
	uORB::Subscription safety_button_sub{ORB_ID(safety_button)};
	uORB::Subscription vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription led_control_sub{ORB_ID(led_control)};
	uORB::Subscription tune_control_sub{ORB_ID(tune_control)};

	// the presses of the other tests are out of the window
	sleep_for(PAIRING_WINDOW + SAMPLED);
	skip_messages<button_event_s>(safety_button_sub);
	skip_messages<vehicle_command_s>(vehicle_command_sub);
	skip_messages<led_control_s>(led_control_sub);
	skip_messages<tune_control_s>(tune_control_sub);

	// 3 presses within 2 seconds request a pairing to the ground station
	for (int i = 0; i < 3; i++) {
		test_button_set(true);
		sleep_for(100_ms);
		test_button_set(false);
		sleep_for(100_ms);
	}

	vehicle_command_s vehicle_command;
	zassert_true(vehicle_command_sub.update(&vehicle_command), "no pairing request");
	zassert_equal(vehicle_command.command, vehicle_command_s::VEHICLE_CMD_START_RX_PAIR);
	zassert_equal(vehicle_command.param1, 10.f);
	zassert_false(vehicle_command_sub.updated());

	led_control_s led_control;
	zassert_true(led_control_sub.update(&led_control));
	zassert_equal(led_control.mode, led_control_s::MODE_BLINK_FAST);
	zassert_equal(led_control.color, led_control_s::COLOR_WHITE);
	zassert_equal(led_control.num_blinks, 20);

	tune_control_s tune_control;
	zassert_true(tune_control_sub.update(&tune_control));
	zassert_equal(tune_control.tune_id, tune_control_s::TUNE_ID_NOTIFY_POSITIVE);

	// presses more than 2 seconds apart don't
	sleep_for(PAIRING_WINDOW + SAMPLED);

	for (int i = 0; i < 3; i++) {
		test_button_set(true);
		sleep_for(100_ms);
		test_button_set(false);
		sleep_for(1_s);
	}

	zassert_false(vehicle_command_sub.updated(), "pairing request for presses more than 2 seconds apart");
	zassert_false(safety_button_sub.updated(), "safety button event for short presses");
}

#if !defined(CONFIG_PX4_SAFETY_LED)
ZTEST(px4_safety_button, test_no_led)
{
	// the driver has no LED to blink
	publish_state(true, true);
	sleep_for(500_ms);
	zassert_false(test_led_lit());
}
#endif

ZTEST_SUITE(px4_safety_button, NULL, setup, button_before, release_button, NULL);

#if defined(CONFIG_PX4_SAFETY_LED)

// the patterns of the safety LED: the driver shows each of the 16 bits for 3 samples (99 ms), so that a pattern lasts
// 48 samples
static constexpr uint16_t SLOW_BLINK{0x0003};   // safety on, disarmed
static constexpr uint16_t QUAD_BLINK{0x5500};   // safety on, armed
static constexpr uint16_t DOUBLE_BLINK{0x5050}; // safety off, disarmed
static constexpr uint16_t SOLID{0xffff};        // safety off, armed
static constexpr int SAMPLES_PER_BIT{3};
static constexpr int PATTERN_SAMPLES{48};
static constexpr hrt_abstime PATTERN_PERIOD{PATTERN_SAMPLES * INTERVAL};

// the LED is sampled every millisecond
static constexpr hrt_abstime LED_SAMPLE{1_ms};
static constexpr hrt_abstime LED_TOLERANCE{2 * LATENCY + LED_SAMPLE};

static bool pattern_lit(uint16_t pattern, int sample)
{
	return (pattern & (1 << ((sample % PATTERN_SAMPLES) / SAMPLES_PER_BIT))) != 0;
}

struct led_change {
	hrt_abstime time;
	bool lit;
};

static constexpr size_t MAX_CHANGES{64};

// the changes of the LED for a time, too many for the stack of the test
static led_change changes[MAX_CHANGES];

static size_t record_led(hrt_abstime duration)
{
	size_t count = 0;
	bool lit = test_led_lit();
	const hrt_abstime start = hrt_absolute_time();

	while (hrt_elapsed_time(&start) < duration) {
		sleep_for(LED_SAMPLE);

		if (test_led_lit() != lit) {
			lit = !lit;

			if (count < MAX_CHANGES) {
				changes[count] = {hrt_absolute_time(), lit};
			}

			count++;
		}
	}

	zassert_true(count <= MAX_CHANGES, "%zu changes", count);
	return count;
}

// check that the LED blinks the pattern, over two periods
static void check_pattern(uint16_t pattern)
{
	// the driver has seen the new state
	sleep_for(2 * INTERVAL);

	const size_t count = record_led(2 * PATTERN_PERIOD + 4 * INTERVAL);

	if (pattern == SOLID) {
		zassert_equal(count, 0, "%zu changes", count);
		zassert_true(test_led_lit());
		return;
	}

	// the samples during which the LED stays lit or off between two changes, a period of them at least
	int runs[MAX_CHANGES] {};
	int samples = 0;

	for (size_t i = 0; i + 1 < count; i++) {
		const hrt_abstime duration = changes[i + 1].time - changes[i].time;
		runs[i] = (int)((duration + INTERVAL / 2) / INTERVAL);
		samples += runs[i];

		const int64_t error = (int64_t)duration - (int64_t)(runs[i] * INTERVAL);
		zassert_true(llabs(error) <= (int64_t)LED_TOLERANCE, "%s for %u us", changes[i].lit ? "lit" : "off",
			     (uint32_t)duration);
	}

	zassert_true(samples >= PATTERN_SAMPLES, "%d samples", samples);

	// the sample of the pattern at the first change
	bool found = false;

	for (int first = 0; (first < PATTERN_SAMPLES) && !found; first++) {
		bool match = pattern_lit(pattern, first + PATTERN_SAMPLES - 1) != changes[0].lit;
		int sample = first;

		for (size_t i = 0; (i + 1 < count) && match; i++) {
			for (int run = 0; run < runs[i]; run++) {
				match = match && (pattern_lit(pattern, sample + run) == changes[i].lit);
			}

			sample += runs[i];
			match = match && (pattern_lit(pattern, sample) != changes[i].lit);
		}

		found = match;
	}

	zassert_true(found, "the LED doesn't blink %04x", pattern);
}

ZTEST(px4_safety_led, test_start)
{
	zassert_false(led_lit_at_start, "LED lit before the armed state is published");
}

ZTEST(px4_safety_led, test_safety_on)
{
	publish_state(false, false);
	check_pattern(SLOW_BLINK);
}

ZTEST(px4_safety_led, test_safety_off)
{
	publish_state(true, false);
	check_pattern(DOUBLE_BLINK);
}

ZTEST(px4_safety_led, test_armed)
{
	publish_state(false, true);
	check_pattern(QUAD_BLINK);
}

ZTEST(px4_safety_led, test_armed_safety_off)
{
	publish_state(true, true);
	check_pattern(SOLID);

	// active low
	zassert_equal(test_led_level(), 0);
}

ZTEST(px4_safety_led, test_button)
{
	// the LED shows the safety state, not the button
	publish_state(false, false);
	test_button_set(true);
	check_pattern(SLOW_BLINK);
}

ZTEST_SUITE(px4_safety_led, NULL, setup, NULL, release_button, NULL);

#endif // CONFIG_PX4_SAFETY_LED
