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
 * Tests of zephyr_rc_input with the SBUS driver of Zephyr on an emulated UART.
 */

#include "sbus_receiver.h"

#include <drivers/drv_hrt.h>
#include <px4_zephyr.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/input_rc.h>

#include <math.h>
#include <string.h>

#include <zephyr/ztest.h>

using namespace time_literals;

// the 11-bit values of the SBUS channels: the 5th one above the threshold of a pressed key, the
// 6th one between the thresholds, where the SBUS driver doesn't report its key
static constexpr uint16_t CHANNELS[SBUS_CHANNELS] {1024, 200, 1800, 172, 1900, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1500};

// the RC channels of the devicetree, from SBUS channels 1 to 5, 16 and 6, in microseconds as
// sbus_decode() decodes them, the key pressed, and the key never reported, released
static constexpr uint8_t CHANNEL_COUNT{7};
static constexpr uint16_t PULSE_WIDTHS[CHANNEL_COUNT] {1514, 999, 1999, 982, 2000, 1812, 1000};
static constexpr size_t KEY_CHANNEL{4};
static constexpr uint16_t KEY_RELEASED{1000};
static constexpr uint16_t KEY_RELEASED_SBUS{100};

static constexpr int32_t RSSI_UNKNOWN{255};

// a frame received this long after a change was sent after it
static constexpr hrt_abstime FRAME_DELAY{30_ms};
static constexpr int FRAME_TIMEOUT_MS{500};

static constexpr hrt_abstime RC_LOST_TIMEOUT{200_ms};
// the driver checks for a lost RC every 50 ms
static constexpr hrt_abstime RC_LOST_MAX_DELAY{RC_LOST_TIMEOUT + 100_ms};

template<typename Condition>
static bool wait_for_input_rc(input_rc_s &input_rc, int timeout_ms, Condition condition)
{
	uORB::Subscription subscription{ORB_ID(input_rc)};

	for (int i = 0; i < timeout_ms; i++) {
		if (subscription.update(&input_rc) && condition(input_rc)) {
			return true;
		}

		k_sleep(K_MSEC(1));
	}

	return false;
}

// the first frame sent after the receiver changed
static bool wait_for_frame(input_rc_s &input_rc)
{
	const hrt_abstime after = hrt_absolute_time() + FRAME_DELAY;

	return wait_for_input_rc(input_rc, FRAME_TIMEOUT_MS, [after](const input_rc_s & rc) {
		return !rc.rc_lost && (rc.timestamp_last_signal > after);
	});
}

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
	zassert_ok(px4_zephyr_init());
	zassert_ok(run("zephyr_rc_input start"));

	// before the first SBUS frame: no frame is published until each channel that isn't a key has
	// been reported
	sbus_receiver_report_sticks(CHANNELS[0]);

	input_rc_s input_rc{};
	zassert_false(wait_for_input_rc(input_rc, FRAME_TIMEOUT_MS, [](const input_rc_s &) {
		return true;
	}));

	return nullptr;
}

static void before(void *fixture)
{
	sbus_receiver_set_channels(CHANNELS);
	sbus_receiver_start();
}

static void after(void *fixture)
{
	sbus_receiver_stop();
	sbus_receiver_report_failsafe(false);
}

static void teardown(void *fixture)
{
	run("zephyr_rc_input stop");
}

ZTEST(px4_rc_input, test_channels)
{
	input_rc_s input_rc{};
	zassert_true(wait_for_frame(input_rc));

	zassert_equal(input_rc.channel_count, CHANNEL_COUNT);

	for (size_t i = 0; i < CHANNEL_COUNT; i++) {
		zassert_equal(input_rc.values[i], PULSE_WIDTHS[i], "channel %zu: %u us", i + 1, input_rc.values[i]);
	}

	for (size_t i = CHANNEL_COUNT; i < input_rc_s::RC_INPUT_MAX_CHANNELS; i++) {
		zassert_equal(input_rc.values[i], 0);
	}

	zassert_equal(input_rc.input_source, input_rc_s::RC_INPUT_SOURCE_PX4FMU_SBUS);
	zassert_false(input_rc.rc_lost);
	zassert_false(input_rc.rc_failsafe);
	zassert_equal(input_rc.rssi, RSSI_UNKNOWN);
	zassert_equal(input_rc.link_quality, -1);
	zassert_true(isnan(input_rc.rssi_dbm));
	zassert_true(input_rc.timestamp_last_signal <= input_rc.timestamp);
}

ZTEST(px4_rc_input, test_key)
{
	uint16_t channels[SBUS_CHANNELS];
	memcpy(channels, CHANNELS, sizeof(channels));
	channels[KEY_CHANNEL] = KEY_RELEASED_SBUS;
	sbus_receiver_set_channels(channels);

	input_rc_s input_rc{};
	zassert_true(wait_for_frame(input_rc));

	zassert_equal(input_rc.values[KEY_CHANNEL], KEY_RELEASED, "%u us", input_rc.values[KEY_CHANNEL]);
	zassert_equal(input_rc.values[0], PULSE_WIDTHS[0]);
}

ZTEST(px4_rc_input, test_other_device)
{
	uint16_t channels[SBUS_CHANNELS];
	memcpy(channels, CHANNELS, sizeof(channels));
	channels[KEY_CHANNEL] = KEY_RELEASED_SBUS;
	sbus_receiver_set_channels(channels);

	input_rc_s input_rc{};
	zassert_true(wait_for_frame(input_rc));
	zassert_equal(input_rc.values[KEY_CHANNEL], KEY_RELEASED);

	// the same key pressed on another input device
	sbus_receiver_press_other_switch(true);

	zassert_true(wait_for_frame(input_rc));
	zassert_equal(input_rc.values[KEY_CHANNEL], KEY_RELEASED, "%u us", input_rc.values[KEY_CHANNEL]);
}

ZTEST(px4_rc_input, test_failsafe)
{
	sbus_receiver_report_failsafe(true);

	input_rc_s input_rc{};
	zassert_true(wait_for_frame(input_rc));
	zassert_true(input_rc.rc_failsafe);
	zassert_false(input_rc.rc_lost);

	sbus_receiver_report_failsafe(false);

	zassert_true(wait_for_frame(input_rc));
	zassert_false(input_rc.rc_failsafe);
}

ZTEST(px4_rc_input, test_rc_lost)
{
	input_rc_s input_rc{};
	zassert_true(wait_for_frame(input_rc));

	sbus_receiver_stop();

	zassert_true(wait_for_input_rc(input_rc, RC_LOST_MAX_DELAY / 1000, [](const input_rc_s & rc) {
		return rc.rc_lost;
	}));

	const hrt_abstime without_frame = input_rc.timestamp - input_rc.timestamp_last_signal;
	zassert_true((without_frame > RC_LOST_TIMEOUT) && (without_frame < RC_LOST_MAX_DELAY),
		     "lost after %" PRIu64 " us", without_frame);
	zassert_equal(input_rc.rssi, 0);
	zassert_equal(input_rc.channel_count, CHANNEL_COUNT);
	zassert_equal(input_rc.values[0], PULSE_WIDTHS[0]);

	// RC is back with the next frame
	sbus_receiver_start();

	zassert_true(wait_for_frame(input_rc));
	zassert_equal(input_rc.rssi, RSSI_UNKNOWN);
}

ZTEST(px4_rc_input, test_restart)
{
	zassert_ok(run("zephyr_rc_input stop"));
	zassert_ok(run("zephyr_rc_input start"));

	input_rc_s input_rc{};
	zassert_true(wait_for_frame(input_rc));
	zassert_equal(input_rc.values[0], PULSE_WIDTHS[0]);
}

ZTEST_SUITE(px4_rc_input, NULL, setup, before, after, teardown);
