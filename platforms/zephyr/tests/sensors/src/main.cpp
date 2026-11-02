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
#include <parameters/param.h>
#include <px4_zephyr.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_mag.h>

#include <zephyr/ztest.h>

static constexpr float ACCEL[3] {1.f, -2.f, 9.81f};
static constexpr float GYRO[3] {0.5f, -0.25f, 1.f};
static constexpr float MAG[3] {0.2f, -0.1f, -0.4f};
static constexpr float PRESSURE_KPA{101.325f};
static constexpr float TEMPERATURE{21.5f};
static constexpr float VOLTAGE{16.f};
static constexpr float CURRENT{12.5f};

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
static bool wait_for_sample(const orb_metadata *meta, T &sample, hrt_abstime after = 0)
{
	uORB::Subscription subscription{meta};

	for (int i = 0; i < SAMPLE_TIMEOUT_MS; i++) {
		if (subscription.update(&sample) && sample.timestamp >= after) {
			return true;
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

	// the power monitor measures battery 2, which is published if its source is a power module
	const int32_t power_module = 0;
	zassert_ok(param_set(param_find("BAT2_SOURCE"), &power_module));

	// the magnetometers are initialized, and reset, when zephyr_sensors starts
	zassert_ok(run("zephyr_sensors start"));

	zassert_ok(sensor_values_set_imu(ACCEL, GYRO));
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

ZTEST(px4_sensors, test_imu)
{
	sensor_accel_s accel{};
	zassert_true(wait_for_sample(ORB_ID(sensor_accel), accel));

	// rotated by PX4_ROTATION_YAW_90
	zassert_within(accel.x, -ACCEL[1], ACCEL_TOLERANCE, "x %f", (double)accel.x);
	zassert_within(accel.y, ACCEL[0], ACCEL_TOLERANCE, "y %f", (double)accel.y);
	zassert_within(accel.z, ACCEL[2], ACCEL_TOLERANCE, "z %f", (double)accel.z);
	zassert_equal(accel.device_id, device_id(device::Device::DeviceBusType_SPI, 1, 0, DRV_IMU_DEVTYPE_ICM42688P));
	zassert_false(accel.is_external);
	zassert_true(accel.timestamp_sample <= accel.timestamp);
	zassert_true(PX4_ISFINITE(accel.temperature));

	sensor_gyro_s gyro{};
	zassert_true(wait_for_sample(ORB_ID(sensor_gyro), gyro));

	zassert_within(gyro.x, -GYRO[1], GYRO_TOLERANCE, "x %f", (double)gyro.x);
	zassert_within(gyro.y, GYRO[0], GYRO_TOLERANCE, "y %f", (double)gyro.y);
	zassert_within(gyro.z, GYRO[2], GYRO_TOLERANCE, "z %f", (double)gyro.z);
	zassert_equal(gyro.device_id, accel.device_id);
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
	battery_status_s battery{};
	zassert_true(wait_for_sample(ORB_ID(battery_status), battery, hrt_absolute_time()));

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
	uORB::Subscription subscription{ORB_ID(battery_status)};
	battery_status_s battery{};
	bool disconnected = false;

	for (int i = 0; i < 3000 && !disconnected; i++) {
		disconnected = subscription.update(&battery) && !battery.connected;
		k_sleep(K_MSEC(1));
	}

	sensor_values_set_power_lost(false);

	zassert_true(disconnected);
	zassert_equal(battery.voltage_v, 0.f);
	zassert_equal(battery.current_a, -1.f);
}

ZTEST_SUITE(px4_sensors, NULL, setup, NULL, NULL, teardown);
