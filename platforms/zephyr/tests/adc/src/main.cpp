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


#include "adc_inputs.h"

#include <drivers/drv_adc.h>
#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <px4_platform_common/px4_config.h>
#include <px4_zephyr.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/system_power.h>

#include <math.h>

#include <zephyr/ztest.h>

// the emulated ADC converts with a 3.3 V reference and 12 bits, to mV * (2^12 - 1) / 3300
static constexpr uint32_t REFERENCE_MV{3300};
static constexpr uint32_t MAX_RAW{4095};
static constexpr float V_REF{3.3f};
static constexpr uint32_t RESOLUTION{4096};

// the channels of the px4,adc node of app.overlay, on these channels of the emulated ADC
static constexpr unsigned CHANNEL_COUNT{7};
static constexpr unsigned ADC_CHANNEL[CHANNEL_COUNT] {7, 0, 1, 2, 3, 4, 5};
static constexpr uint32_t INPUT_MV[CHANNEL_COUNT] {1000, 2500, 1650, 1600, 1200, 500, 800};

static constexpr float BAT1_V_DIV{12.5f};
static constexpr float BAT1_A_PER_V{40.f};
static constexpr float BAT2_V_DIV{15.f};

static constexpr float TOLERANCE{0.001f};

// longer than the 10 ms between the samples of board_adc
static constexpr int SAMPLE_TIMEOUT_MS{100};

// three samples of board_adc
static constexpr int SETTLE_MS{30};

// battery_status reads the parameters at most once per second
static constexpr int PARAMETER_TIMEOUT_MS{2000};

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

static int32_t raw(uint32_t millivolts)
{
	return millivolts * MAX_RAW / REFERENCE_MV;
}

// the voltage at the input of a channel, as PX4 computes it from adc_report
static float voltage(uint32_t millivolts)
{
	return raw(millivolts) * V_REF / RESOLUTION;
}

static void set_inputs()
{
	for (unsigned i = 0; i < CHANNEL_COUNT; i++) {
		zassert_ok(adc_inputs_set(ADC_CHANNEL[i], INPUT_MV[i]));
	}
}

template<typename T, typename Condition>
static bool wait_for(const orb_metadata *meta, T &sample, int timeout_ms, Condition condition)
{
	uORB::Subscription subscription{meta};

	for (int i = 0; i < timeout_ms; i++) {
		if (subscription.update(&sample) && condition(sample)) {
			return true;
		}

		k_sleep(K_MSEC(1));
	}

	return false;
}

// a battery_status sample of a battery, from any instance of the topic
template<typename Condition>
static bool wait_for_battery(uint8_t id, battery_status_s &battery, int timeout_ms, Condition condition)
{
	uORB::SubscriptionMultiArray<battery_status_s> subscriptions{ORB_ID::battery_status};

	for (int i = 0; i < timeout_ms; i++) {
		for (auto &subscription : subscriptions) {
			if (subscription.update(&battery) && (battery.id == id) && condition(battery)) {
				return true;
			}
		}

		k_sleep(K_MSEC(1));
	}

	return false;
}

static void *setup(void)
{
	zassert_ok(px4_zephyr_init());
	set_inputs();

	zassert_ok(param_set(param_find("BAT1_V_DIV"), &BAT1_V_DIV));
	zassert_ok(param_set(param_find("BAT1_A_PER_V"), &BAT1_A_PER_V));
	zassert_ok(param_set(param_find("BAT2_V_DIV"), &BAT2_V_DIV));

	// battery 2 is published if its source is a power module
	const int32_t power_module{0};
	zassert_ok(param_set(param_find("BAT2_SOURCE"), &power_module));

	zassert_ok(run("board_adc start"));
	zassert_ok(run("battery_status start"));

	return nullptr;
}

// the inputs of app.overlay, in the latest samples
static void before(void *fixture)
{
	set_inputs();
	k_sleep(K_MSEC(SETTLE_MS));
}

ZTEST(px4_adc, test_channels)
{
	// numbered in the order of io-channels, with the macros of their names
	zassert_equal(ADC_CHANNELS, 0x7f);
	zassert_equal(ADC_ADC_3V3_CHANNEL, 0);
	zassert_equal(ADC_SCALED_V5_CHANNEL, 1);
	zassert_equal(ADC_SCALED_VDD_3V3_SENSORS1_CHANNEL, 2);
	zassert_equal(ADC_SCALED_VDD_3V3_SENSORS2_CHANNEL, 3);
	zassert_equal(ADC_BATTERY1_VOLTAGE_CHANNEL, 4);
	zassert_equal(ADC_BATTERY1_CURRENT_CHANNEL, 5);
	zassert_equal(ADC_BATTERY2_VOLTAGE_CHANNEL, 6);
	zassert_equal(ADC_BATTERY2_CURRENT_CHANNEL, -1);
	zassert_equal(BOARD_NUMBER_BRICKS, 2);
}

ZTEST(px4_adc, test_adc_report)
{
	adc_report_s adc_report{};
	zassert_true(wait_for(ORB_ID(adc_report), adc_report, SAMPLE_TIMEOUT_MS, [](const adc_report_s &) {
		return true;
	}));

	zassert_equal(adc_report.device_id, BUILTIN_ADC_DEVID);
	zassert_equal(adc_report.resolution, RESOLUTION);
	zassert_within(adc_report.v_ref, V_REF, 1e-6f, "v_ref %f", (double)adc_report.v_ref);

	for (unsigned i = 0; i < CHANNEL_COUNT; i++) {
		zassert_equal(adc_report.channel_id[i], i);
		zassert_equal(adc_report.raw_data[i], raw(INPUT_MV[i]), "channel %u: %d", i, adc_report.raw_data[i]);
	}

	for (unsigned i = CHANNEL_COUNT; i < PX4_MAX_ADC_CHANNELS; i++) {
		zassert_equal(adc_report.channel_id[i], -1);
	}
}

ZTEST(px4_adc, test_input_change)
{
	const uint32_t input_mv = 3000;
	zassert_ok(adc_inputs_set(ADC_CHANNEL[ADC_ADC_3V3_CHANNEL], input_mv));

	adc_report_s adc_report{};
	zassert_true(wait_for(ORB_ID(adc_report), adc_report, SAMPLE_TIMEOUT_MS, [input_mv](const adc_report_s & report) {
		return report.raw_data[ADC_ADC_3V3_CHANNEL] == raw(input_mv);
	}));
}

ZTEST(px4_adc, test_read_error)
{
	// a channel that can't be read is -1, and the others are still read
	zassert_ok(adc_inputs_set_failing(ADC_CHANNEL[ADC_ADC_3V3_CHANNEL]));

	adc_report_s adc_report{};
	zassert_true(wait_for(ORB_ID(adc_report), adc_report, SAMPLE_TIMEOUT_MS, [](const adc_report_s & report) {
		return report.raw_data[ADC_ADC_3V3_CHANNEL] == -1;
	}));

	for (unsigned i = 1; i < CHANNEL_COUNT; i++) {
		zassert_equal(adc_report.raw_data[i], raw(INPUT_MV[i]), "channel %u: %d", i, adc_report.raw_data[i]);
	}
}

ZTEST(px4_adc, test_stop)
{
	// board_adc stops at its next sample, and starts again
	const int64_t start_ms = k_uptime_get();
	zassert_ok(run("board_adc stop"));
	const int64_t stop_ms = k_uptime_get() - start_ms;
	zassert_true(stop_ms < SAMPLE_TIMEOUT_MS, "board_adc stop took %lld ms", stop_ms);

	const hrt_abstime restart = hrt_absolute_time();
	zassert_ok(run("board_adc start"));

	adc_report_s adc_report{};
	zassert_true(wait_for(ORB_ID(adc_report), adc_report, SAMPLE_TIMEOUT_MS, [restart](const adc_report_s & report) {
		return report.timestamp > restart;
	}));
}

ZTEST(px4_adc, test_system_power)
{
	system_power_s system_power{};
	zassert_true(wait_for(ORB_ID(system_power), system_power, SAMPLE_TIMEOUT_MS, [](const system_power_s &) {
		return true;
	}));

	// the rails, through voltage dividers of 2
	const float voltage5v = 2.f * voltage(INPUT_MV[ADC_SCALED_V5_CHANNEL]);
	const float sensors1 = 2.f * voltage(INPUT_MV[ADC_SCALED_VDD_3V3_SENSORS1_CHANNEL]);
	const float sensors2 = 2.f * voltage(INPUT_MV[ADC_SCALED_VDD_3V3_SENSORS2_CHANNEL]);
	zassert_within(system_power.voltage5v_v, voltage5v, TOLERANCE, "5 V %f", (double)system_power.voltage5v_v);
	zassert_within(system_power.sensors3v3[0], sensors1, TOLERANCE, "3.3 V %f", (double)system_power.sensors3v3[0]);
	zassert_within(system_power.sensors3v3[1], sensors2, TOLERANCE, "3.3 V %f", (double)system_power.sensors3v3[1]);
	zassert_equal(system_power.sensors3v3_valid, 0x3);

	// USB isn't detected, and both power modules are valid
	zassert_equal(system_power.usb_connected, 0);
	zassert_equal(system_power.usb_valid, 0);
	zassert_equal(system_power.brick_valid, 0x3);
}

ZTEST(px4_adc, test_batteries)
{
	battery_status_s battery{};
	zassert_true(wait_for_battery(1, battery, SAMPLE_TIMEOUT_MS, [](const battery_status_s &) {
		return true;
	}));

	const float voltage1 = voltage(INPUT_MV[ADC_BATTERY1_VOLTAGE_CHANNEL]) * BAT1_V_DIV;
	const float current1 = voltage(INPUT_MV[ADC_BATTERY1_CURRENT_CHANNEL]) * BAT1_A_PER_V;
	zassert_true(battery.connected);
	zassert_within(battery.voltage_v, voltage1, TOLERANCE, "voltage %f", (double)battery.voltage_v);
	zassert_within(battery.current_a, current1, TOLERANCE, "current %f", (double)battery.current_a);
	zassert_equal(battery.source, battery_status_s::SOURCE_POWER_MODULE);

	// battery 2 has no current channel
	zassert_true(wait_for_battery(2, battery, SAMPLE_TIMEOUT_MS, [](const battery_status_s &) {
		return true;
	}));

	const float voltage2 = voltage(INPUT_MV[ADC_BATTERY2_VOLTAGE_CHANNEL]) * BAT2_V_DIV;
	zassert_true(battery.connected);
	zassert_within(battery.voltage_v, voltage2, TOLERANCE, "voltage %f", (double)battery.voltage_v);
	zassert_within(battery.current_a, 0.f, TOLERANCE, "current %f", (double)battery.current_a);
}

ZTEST(px4_adc, test_battery_disconnected)
{
	// below BOARD_ADC_OPEN_CIRCUIT_V at the input of the voltage divider
	zassert_ok(adc_inputs_set(ADC_CHANNEL[ADC_BATTERY1_VOLTAGE_CHANNEL], 0));

	battery_status_s battery{};
	zassert_true(wait_for_battery(1, battery, SAMPLE_TIMEOUT_MS, [](const battery_status_s & status) {
		return !status.connected;
	}));

	zassert_within(battery.voltage_v, 0.f, TOLERANCE, "voltage %f", (double)battery.voltage_v);
}

ZTEST(px4_adc, test_voltage_channel)
{
	// BAT1_V_CHANNEL selects another channel for the voltage of battery 1
	const int32_t channel{ADC_ADC_3V3_CHANNEL};
	zassert_ok(param_set(param_find("BAT1_V_CHANNEL"), &channel));

	const float voltage1 = voltage(INPUT_MV[ADC_ADC_3V3_CHANNEL]) * BAT1_V_DIV;
	battery_status_s battery{};
	zassert_true(wait_for_battery(1, battery, PARAMETER_TIMEOUT_MS, [voltage1](const battery_status_s & status) {
		return fabsf(status.voltage_v - voltage1) < TOLERANCE;
	}), "voltage %f", (double)battery.voltage_v);

	// and the default channel, battery1-voltage, again
	param_reset(param_find("BAT1_V_CHANNEL"));

	const float default_voltage1 = voltage(INPUT_MV[ADC_BATTERY1_VOLTAGE_CHANNEL]) * BAT1_V_DIV;
	zassert_true(wait_for_battery(1, battery, PARAMETER_TIMEOUT_MS, [default_voltage1](const battery_status_s & status) {
		return fabsf(status.voltage_v - default_voltage1) < TOLERANCE;
	}), "voltage %f", (double)battery.voltage_v);
}

ZTEST_SUITE(px4_adc, NULL, setup, before, NULL, NULL);
