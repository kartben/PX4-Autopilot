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

#include "sensor_values.h"

#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <mathlib/mathlib.h>
#include <parameters/param.h>
#include <px4_zephyr.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_accel_fifo.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_gyro_fifo.h>
#include <uORB/topics/sensor_mag.h>

#include <zephyr/ztest.h>

static constexpr float ACCEL[3] {1.f, -2.f, 9.81f};
static constexpr float GYRO[3] {0.5f, -0.25f, 1.f};
static constexpr float IMU_TEMPERATURE{35.f};
static constexpr float MAG[3] {0.2f, -0.1f, -0.4f};
static constexpr float PRESSURE_KPA{101.325f};
static constexpr float TEMPERATURE{21.5f};
static constexpr float VOLTAGE{16.f};
static constexpr float CURRENT{12.5f};
static constexpr float FUEL_GAUGE_VOLTAGE{22.2f};
static constexpr float FUEL_GAUGE_CURRENT{30.f};

// the FIFO of the first IMU is read at IMU_GYRO_RATEMAX
static constexpr uint32_t IMU_ODR{8000};
static constexpr uint32_t IMU_GYRO_RATEMAX{400};
static constexpr float ACCEL_FULL_SCALE{16.f * 9.80665f};
static constexpr float GYRO_FULL_SCALE{math::radians(2000.f)};

// sensor resolutions: ICM42688 at +-16 g and +-2000 dps, AK09918 0.15 uT
static constexpr float ACCEL_TOLERANCE{0.01f};
static constexpr float GYRO_TOLERANCE{0.002f};
static constexpr float MAG_TOLERANCE{0.003f};
static constexpr float PRESSURE_TOLERANCE{1.f};
static constexpr float TEMPERATURE_TOLERANCE{0.01f};
static constexpr float POWER_TOLERANCE{0.001f};

// longer than the 100 ms between the readings of a power monitor
static constexpr int SAMPLE_TIMEOUT_MS{200};

template<typename T>
static bool wait_for_sample(const orb_metadata *meta, T &sample, hrt_abstime after = 0, uint8_t instance = 0)
{
	uORB::Subscription subscription{meta, instance};

	for (int i = 0; i < SAMPLE_TIMEOUT_MS; i++) {
		if (subscription.update(&sample) && sample.timestamp >= after) {
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

static uint32_t device_id(device::Device::DeviceBusType bus_type, uint8_t bus, uint8_t address, uint8_t devtype)
{
	device::Device::DeviceId id{};
	id.devid_s.bus_type = bus_type;
	id.devid_s.bus = bus;
	id.devid_s.address = address;
	id.devid_s.devtype = devtype;
	return id.devid;
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

	// the FIFO of the second IMU isn't read, and it is polled
	sensor_values_set_imu_interrupt(1, false);

	// the power monitor measures battery 2, which is published if its source is a power module
	const int32_t power_module = 0;
	zassert_ok(param_set(param_find("BAT2_SOURCE"), &power_module));

	// the magnetometers are initialized, and reset, when zephyr_sensors starts
	zassert_ok(run("zephyr_sensors start"));

	sensor_values_set_imu(ACCEL, GYRO, IMU_TEMPERATURE);
	zassert_ok(sensor_values_set_mag(MAG));
	sensor_values_set_baro(PRESSURE_KPA, TEMPERATURE);
	sensor_values_set_power(VOLTAGE, CURRENT);

	// wait for the sensors to be read with these values
	k_sleep(K_MSEC(SAMPLE_TIMEOUT_MS));

	return nullptr;
}

static void teardown(void *fixture)
{
	run("zephyr_sensors stop");
}

static void check_imu(uint8_t instance, uint8_t address, hrt_abstime after = 0)
{
	sensor_accel_s accel{};
	zassert_true(wait_for_sample(ORB_ID(sensor_accel), accel, after, instance));

	// rotated by PX4_ROTATION_YAW_90
	zassert_within(accel.x, -ACCEL[1], ACCEL_TOLERANCE, "x %f", (double)accel.x);
	zassert_within(accel.y, ACCEL[0], ACCEL_TOLERANCE, "y %f", (double)accel.y);
	zassert_within(accel.z, ACCEL[2], ACCEL_TOLERANCE, "z %f", (double)accel.z);
	zassert_equal(accel.device_id, device_id(device::Device::DeviceBusType_SPI, 1, address, DRV_IMU_DEVTYPE_ICM42688P));
	zassert_false(accel.is_external);
	zassert_true(accel.timestamp_sample <= accel.timestamp);
	zassert_within(accel.temperature, IMU_TEMPERATURE, TEMPERATURE_TOLERANCE, "temperature %f",
		       (double)accel.temperature);

	sensor_gyro_s gyro{};
	zassert_true(wait_for_sample(ORB_ID(sensor_gyro), gyro, after, instance));

	zassert_within(gyro.x, -GYRO[1], GYRO_TOLERANCE, "x %f", (double)gyro.x);
	zassert_within(gyro.y, GYRO[0], GYRO_TOLERANCE, "y %f", (double)gyro.y);
	zassert_within(gyro.z, GYRO[2], GYRO_TOLERANCE, "z %f", (double)gyro.z);
	zassert_equal(gyro.device_id, accel.device_id);
}

// the samples of an IMU during SAMPLE_TIMEOUT_MS, and the longest time between them
static int imu_samples(uint8_t instance, hrt_abstime &max_interval)
{
	uORB::Subscription subscription{ORB_ID(sensor_accel), instance};
	sensor_accel_s accel{};
	const hrt_abstime start = hrt_absolute_time();
	hrt_abstime last_sample = 0;
	int samples = 0;

	max_interval = 0;

	// the queue of sensor_accel holds the samples of more than 4 ms
	for (int i = 0; i < SAMPLE_TIMEOUT_MS / 4; i++) {
		while (subscription.update(&accel)) {
			// the samples before the test, while the test framework printed to the console, which
			// blocks the other threads on hardware, don't count
			if (accel.timestamp_sample < start) {
				continue;
			}

			if (last_sample != 0) {
				max_interval = math::max(max_interval, accel.timestamp_sample - last_sample);
			}

			last_sample = accel.timestamp_sample;
			samples++;
		}

		k_sleep(K_MSEC(4));
	}

	return samples;
}

ZTEST(px4_sensors, test_imu)
{
	check_imu(0, 0);
}

ZTEST(px4_sensors, test_imu_fifo)
{
	// all the samples of the FIFO, read at IMU_GYRO_RATEMAX
	sensor_accel_fifo_s accel{};
	zassert_true(wait_for_sample(ORB_ID(sensor_accel_fifo), accel));

	zassert_equal(accel.samples, IMU_ODR / IMU_GYRO_RATEMAX, "%u samples", accel.samples);
	zassert_within(accel.dt, 1e6f / IMU_ODR, 0.01f, "dt %f", (double)accel.dt);
	zassert_within(accel.scale, ACCEL_FULL_SCALE / 32768.f, 1e-7f, "scale %f", (double)accel.scale);
	zassert_equal(accel.device_id, device_id(device::Device::DeviceBusType_SPI, 1, 0, DRV_IMU_DEVTYPE_ICM42688P));
	zassert_true(accel.timestamp_sample <= accel.timestamp);

	for (int i = 0; i < accel.samples; i++) {
		// rotated by PX4_ROTATION_YAW_90
		zassert_within(accel.x[i] * accel.scale, -ACCEL[1], ACCEL_TOLERANCE, "x %d", accel.x[i]);
		zassert_within(accel.y[i] * accel.scale, ACCEL[0], ACCEL_TOLERANCE, "y %d", accel.y[i]);
		zassert_within(accel.z[i] * accel.scale, ACCEL[2], ACCEL_TOLERANCE, "z %d", accel.z[i]);
	}

	sensor_gyro_fifo_s gyro{};
	zassert_true(wait_for_sample(ORB_ID(sensor_gyro_fifo), gyro));

	zassert_equal(gyro.samples, IMU_ODR / IMU_GYRO_RATEMAX, "%u samples", gyro.samples);
	zassert_within(gyro.dt, 1e6f / IMU_ODR, 0.01f, "dt %f", (double)gyro.dt);
	zassert_within(gyro.scale, GYRO_FULL_SCALE / 32768.f, 1e-7f, "scale %f", (double)gyro.scale);
	zassert_equal(gyro.device_id, accel.device_id);

	for (int i = 0; i < gyro.samples; i++) {
		zassert_within(gyro.x[i] * gyro.scale, -GYRO[1], GYRO_TOLERANCE, "x %d", gyro.x[i]);
		zassert_within(gyro.y[i] * gyro.scale, GYRO[0], GYRO_TOLERANCE, "y %d", gyro.y[i]);
		zassert_within(gyro.z[i] * gyro.scale, GYRO[2], GYRO_TOLERANCE, "z %d", gyro.z[i]);
	}
}

ZTEST(px4_sensors, test_imu_fifo_rate)
{
	// every sample is published once, at IMU_GYRO_RATEMAX
	uORB::Subscription subscription{ORB_ID(sensor_gyro_fifo)};
	sensor_gyro_fifo_s gyro{};
	const hrt_abstime start = hrt_absolute_time();
	hrt_abstime first_sample = 0;
	hrt_abstime last_sample = 0;
	hrt_abstime max_interval = 0;
	uint32_t samples = 0;
	int messages = 0;

	// the queue of sensor_gyro_fifo holds the samples of 10 ms
	for (int i = 0; i < SAMPLE_TIMEOUT_MS; i++) {
		while (subscription.update(&gyro)) {
			if (gyro.timestamp_sample < start) {
				continue;
			}

			if (last_sample == 0) {
				first_sample = gyro.timestamp_sample;

			} else {
				// the samples after the first message
				samples += gyro.samples;
				max_interval = math::max(max_interval, gyro.timestamp_sample - last_sample);
			}

			last_sample = gyro.timestamp_sample;
			messages++;
		}

		k_sleep(K_MSEC(1));
	}

	zassert_true(messages > 10, "%d messages", messages);
	zassert_true(max_interval < 1.1f * 1e6f / IMU_GYRO_RATEMAX, "%" PRIu64 " us between messages", max_interval);

	// the time between the first and last samples, from the timestamps and from the samples
	const float interval = last_sample - first_sample;
	zassert_within(interval, samples * gyro.dt, gyro.dt, "%f us, %" PRIu32 " samples", (double)interval, samples);
}

ZTEST(px4_sensors, test_imu_fifo_stop)
{
	// the IMU is polled when its FIFO stops being read
	sensor_values_set_imu_interrupt(0, false);
	k_sleep(K_MSEC(SAMPLE_TIMEOUT_MS));
	const hrt_abstime polled = hrt_absolute_time();
	check_imu(0, 0, polled);

	uORB::Subscription fifo{ORB_ID(sensor_gyro_fifo)};
	sensor_gyro_fifo_s gyro{};
	fifo.copy(&gyro);
	zassert_true(gyro.timestamp < polled);

	// the FIFO is read again when the driver restarts
	sensor_values_set_imu_interrupt(0, true);
}

ZTEST(px4_sensors, test_imu_polled)
{
	// the IMU whose FIFO isn't read is polled at IMU_GYRO_RATEMAX
	check_imu(1, 1);

	hrt_abstime max_interval = 0;
	const int samples = imu_samples(1, max_interval);
	zassert_true(samples > static_cast<int>(SAMPLE_TIMEOUT_MS * IMU_GYRO_RATEMAX / 1000 * 9 / 10), "%d samples", samples);
	zassert_false(uORB::Subscription(ORB_ID(sensor_accel_fifo), 1).advertised());
	zassert_false(uORB::Subscription(ORB_ID(sensor_gyro_fifo), 1).advertised());
}

ZTEST(px4_sensors, test_imu_interval)
{
	// the IMUs are read on time while the barometer, on another bus, converts
	for (uint8_t instance = 0; instance < 2; instance++) {
		hrt_abstime max_interval = 0;
		const int samples = imu_samples(instance, max_interval);

		zassert_true(samples > 10, "IMU %u: %d samples", instance, samples);
		zassert_true(max_interval < BARO_CONVERSION_TIME_MS * 1000, "IMU %u: %" PRIu64 " us between samples", instance,
			     max_interval);
	}
}

static void check_mag(hrt_abstime after = 0)
{
	sensor_mag_s mag{};
	zassert_true(wait_for_sample(ORB_ID(sensor_mag), mag, after));

	zassert_within(mag.x, MAG[0], MAG_TOLERANCE, "x %f", (double)mag.x);
	zassert_within(mag.y, MAG[1], MAG_TOLERANCE, "y %f", (double)mag.y);
	zassert_within(mag.z, MAG[2], MAG_TOLERANCE, "z %f", (double)mag.z);
	zassert_equal(mag.device_id, device_id(device::Device::DeviceBusType_I2C, 1, 0xc, DRV_MAG_DEVTYPE_AK09918));
	zassert_true(mag.is_external);
}

ZTEST(px4_sensors, test_mag)
{
	check_mag();
}

ZTEST(px4_sensors, test_absent_sensor)
{
	zassert_false(device_is_ready(DEVICE_DT_GET(DT_NODELABEL(ist8310))));

	k_sleep(K_MSEC(SAMPLE_TIMEOUT_MS));
	zassert_false(uORB::Subscription(ORB_ID(sensor_mag), 1).advertised());
}

ZTEST(px4_sensors, test_restart)
{
	// the optional sensors are found again, without a second initialization
	zassert_ok(run("zephyr_sensors stop"));

	const hrt_abstime restart = hrt_absolute_time();
	zassert_ok(run("zephyr_sensors start"));

	// the emulator has a new sample when it is set
	zassert_ok(sensor_values_set_mag(MAG));
	check_mag(restart);

	// the FIFO is read again
	sensor_gyro_fifo_s gyro{};
	zassert_true(wait_for_sample(ORB_ID(sensor_gyro_fifo), gyro, restart));
	zassert_equal(gyro.samples, IMU_ODR / IMU_GYRO_RATEMAX, "%u samples", gyro.samples);
}

ZTEST(px4_sensors, test_baro)
{
	sensor_baro_s baro{};
	zassert_true(wait_for_sample(ORB_ID(sensor_baro), baro));

	zassert_within(baro.pressure, PRESSURE_KPA * 1000.f, PRESSURE_TOLERANCE, "pressure %f", (double)baro.pressure);
	zassert_within(baro.temperature, TEMPERATURE, TEMPERATURE_TOLERANCE, "temperature %f", (double)baro.temperature);
	zassert_equal(baro.device_id, device_id(device::Device::DeviceBusType_UNKNOWN, 0, 0, DRV_DEVTYPE_UNUSED),
		      "device id 0x%" PRIx32, baro.device_id);
}

ZTEST(px4_sensors, test_power_monitor)
{
	const hrt_abstime after = hrt_absolute_time();
	battery_status_s battery{};
	zassert_true(wait_for_battery(2, battery, SAMPLE_TIMEOUT_MS, [after](const battery_status_s & status) {
		return status.timestamp >= after;
	}));

	zassert_true(battery.connected);
	zassert_within(battery.voltage_v, VOLTAGE, POWER_TOLERANCE, "voltage %f", (double)battery.voltage_v);
	zassert_within(battery.current_a, CURRENT, POWER_TOLERANCE, "current %f", (double)battery.current_a);
	zassert_equal(battery.id, 2);
	zassert_equal(battery.source, battery_status_s::SOURCE_POWER_MODULE);
}

ZTEST(px4_sensors, test_power_monitor_lost)
{
	sensor_values_set_power_lost(true);

	// the battery is disconnected when its power monitor can't be read for 2 s
	battery_status_s battery{};
	const bool disconnected = wait_for_battery(2, battery, 3000, [](const battery_status_s & status) {
		return !status.connected;
	});

	sensor_values_set_power_lost(false);

	zassert_true(disconnected);
	zassert_equal(battery.voltage_v, 0.f);
	zassert_equal(battery.current_a, -1.f);
}

ZTEST(px4_sensors, test_fuel_gauge)
{
	// the emulated fuel gauge is reset after each test
	zassert_ok(sensor_values_set_fuel_gauge(FUEL_GAUGE_VOLTAGE, -FUEL_GAUGE_CURRENT));

	const hrt_abstime after = hrt_absolute_time();
	battery_status_s battery{};
	zassert_true(wait_for_battery(1, battery, SAMPLE_TIMEOUT_MS, [after](const battery_status_s & status) {
		return status.timestamp >= after;
	}));

	zassert_true(battery.connected);
	zassert_within(battery.voltage_v, FUEL_GAUGE_VOLTAGE, POWER_TOLERANCE, "voltage %f", (double)battery.voltage_v);
	zassert_within(battery.current_a, FUEL_GAUGE_CURRENT, POWER_TOLERANCE, "current %f", (double)battery.current_a);

	// the other registers of the emulated fuel gauge read 1
	zassert_within(battery.remaining, 0.01f, 1e-6f, "remaining %f", (double)battery.remaining);
	zassert_equal(battery.warning, battery_status_s::WARNING_EMERGENCY);
	zassert_equal(battery.capacity, 1);
	zassert_equal(battery.cycle_count, 1);
	zassert_within(battery.time_remaining_s, 60.f, 1e-3f, "time remaining %f", (double)battery.time_remaining_s);
	zassert_within(battery.discharged_mah, 0.f, 1e-6f, "discharged %f", (double)battery.discharged_mah);
}

ZTEST_SUITE(px4_sensors, NULL, setup, NULL, NULL, teardown);
