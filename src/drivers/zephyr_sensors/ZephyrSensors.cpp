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

#include "ZephyrSensors.hpp"

#include <dt-bindings/px4/rotation.h>
#include <lib/conversion/rotation.h>
#include <lib/drivers/device/Device.hpp>
#include <mathlib/mathlib.h>

static_assert(PX4_ROTATION_ROLL_90_PITCH_315 == ROTATION_MAX - 1, "dt-bindings/px4/rotation.h is out of date");

ModuleBase::Descriptor ZephyrSensors::desc{task_spawn, custom_command, print_usage};

ZephyrSensors::ZephyrSensors() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

ZephyrSensors::~ZephyrSensors()
{
	for (size_t i = 0; i < _sensor_count; i++) {
		delete _sensors[i].accel;
		delete _sensors[i].gyro;
		delete _sensors[i].mag;
		delete _sensors[i].baro;
		delete _sensors[i].battery;
	}

	perf_free(_cycle_perf);
	perf_free(_read_error_perf);
}

static uint32_t device_id(const zephyr_sensor_info &info)
{
	device::Device::DeviceId id{};

	switch (info.bus_type) {
	case ZEPHYR_SENSOR_BUS_TYPE_I2C:
		id.devid_s.bus_type = device::Device::DeviceBusType_I2C;
		break;

	case ZEPHYR_SENSOR_BUS_TYPE_SPI:
		id.devid_s.bus_type = device::Device::DeviceBusType_SPI;
		break;

	default:
		id.devid_s.bus_type = device::Device::DeviceBusType_UNKNOWN;
		break;
	}

	id.devid_s.bus = info.bus;
	id.devid_s.address = info.address;
	id.devid_s.devtype = info.devtype;

	return id.devid;
}

bool ZephyrSensors::add_sensor(size_t index)
{
	Sensor &sensor = _sensors[_sensor_count];
	const int ret = zephyr_sensors_get_info(index, &sensor.info);

	if (ret != 0) {
		if (!sensor.info.optional) {
			PX4_ERR("%s not ready (%d)", sensor.info.name, ret);
		}

		return false;
	}

	if (sensor.info.rotation >= ROTATION_MAX) {
		PX4_ERR("%s: invalid rotation %u", sensor.info.name, sensor.info.rotation);
		return false;
	}

	const uint32_t id = device_id(sensor.info);
	const Rotation rotation = static_cast<Rotation>(sensor.info.rotation);
	const bool external = sensor.info.external;

	switch (sensor.info.type) {
	case ZEPHYR_SENSOR_TYPE_IMU:
		sensor.accel = new PX4Accelerometer(id, rotation, external);
		sensor.gyro = new PX4Gyroscope(id, rotation, external);
		sensor.interval = IMU_INTERVAL;
		break;

	case ZEPHYR_SENSOR_TYPE_ACCEL:
		sensor.accel = new PX4Accelerometer(id, rotation, external);
		sensor.interval = IMU_INTERVAL;
		break;

	case ZEPHYR_SENSOR_TYPE_GYRO:
		sensor.gyro = new PX4Gyroscope(id, rotation, external);
		sensor.interval = IMU_INTERVAL;
		break;

	case ZEPHYR_SENSOR_TYPE_MAG:
		sensor.mag = new PX4Magnetometer(id, rotation, external);
		sensor.interval = MAG_INTERVAL;
		break;

	case ZEPHYR_SENSOR_TYPE_BARO:
		sensor.baro = new PX4Barometer(id, external);
		sensor.interval = BARO_INTERVAL;
		break;

	case ZEPHYR_SENSOR_TYPE_POWER:
		sensor.battery = new Battery(sensor.info.battery, this, BATTERY_INTERVAL, battery_status_s::SOURCE_POWER_MODULE);
		sensor.interval = BATTERY_INTERVAL;
		break;
	}

	sensor.index = index;
	_sensor_count++;

	return true;
}

int ZephyrSensors::init()
{
	for (size_t index = 0; index < zephyr_sensors_count(); index++) {
		if (_sensor_count >= MAX_SENSORS) {
			PX4_WARN("only %zu sensors supported", MAX_SENSORS);
			break;
		}

		add_sensor(index);
	}

	if (_sensor_count == 0) {
		return PX4_ERROR;
	}

	// the sensors are read in turn, on the work queue of the first one's bus
	ChangeWorkQueue(px4::device_bus_to_wq(device_id(_sensors[0].info)));

	hrt_abstime interval = _sensors[0].interval;

	for (size_t i = 1; i < _sensor_count; i++) {
		interval = math::min(interval, _sensors[i].interval);
	}

	_interval = interval;
	ScheduleOnInterval(interval);

	return PX4_OK;
}

void ZephyrSensors::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update;
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
	}

	perf_begin(_cycle_perf);

	for (size_t i = 0; i < _sensor_count; i++) {
		Sensor &sensor = _sensors[i];
		const hrt_abstime start = hrt_absolute_time();

		// a read due before the middle of the period of the bus is done now: the jitter of the
		// work queue would otherwise skip reads at the rate of the bus
		if (start + _interval / 2 < sensor.next_read) {
			continue;
		}

		sensor.next_read = start + sensor.interval;

		zephyr_sensor_sample sample{};

		if (zephyr_sensors_read(sensor.index, &sample) == 0) {
			// the sample was taken during the read
			sample.timestamp_us = math::constrain(sample.timestamp_us, start, hrt_absolute_time());
			sensor.last_sample = sample.timestamp_us;
			publish(sensor, sample);

		} else {
			sensor.error_count++;
			perf_count(_read_error_perf);

			if (sensor.battery && (hrt_elapsed_time(&sensor.last_sample) > BATTERY_TIMEOUT)) {
				sensor.battery->setConnected(false);
				sensor.battery->updateVoltage(0.f);
				sensor.battery->updateCurrent(-1.f);
				sensor.battery->updateAndPublishBatteryStatus(hrt_absolute_time());
			}
		}
	}

	perf_end(_cycle_perf);
}

void ZephyrSensors::publish(Sensor &sensor, const zephyr_sensor_sample &sample)
{
	const bool has_temperature = PX4_ISFINITE(sample.temperature);

	if (sensor.accel) {
		sensor.accel->set_error_count(sensor.error_count);

		if (has_temperature) {
			sensor.accel->set_temperature(sample.temperature);
		}

		sensor.accel->update(sample.timestamp_us, sample.accel[0], sample.accel[1], sample.accel[2]);
	}

	if (sensor.gyro) {
		sensor.gyro->set_error_count(sensor.error_count);

		if (has_temperature) {
			sensor.gyro->set_temperature(sample.temperature);
		}

		sensor.gyro->update(sample.timestamp_us, sample.gyro[0], sample.gyro[1], sample.gyro[2]);
	}

	if (sensor.mag) {
		sensor.mag->set_error_count(sensor.error_count);
		sensor.mag->update(sample.timestamp_us, sample.mag[0], sample.mag[1], sample.mag[2]);
	}

	if (sensor.baro) {
		sensor.baro->set_error_count(sensor.error_count);

		if (has_temperature) {
			sensor.baro->set_temperature(sample.temperature);
		}

		sensor.baro->update(sample.timestamp_us, sample.pressure);
	}

	if (sensor.battery) {
		sensor.battery->setConnected(true);
		sensor.battery->updateVoltage(sample.voltage);
		sensor.battery->updateCurrent(sample.current);
		sensor.battery->updateAndPublishBatteryStatus(sample.timestamp_us);
	}
}

int ZephyrSensors::print_status()
{
	perf_print_counter(_cycle_perf);
	perf_print_counter(_read_error_perf);

	for (size_t i = 0; i < _sensor_count; i++) {
		const Sensor &sensor = _sensors[i];
		PX4_INFO("%s: device id %" PRIu32 ", every %" PRIu64 " us, %" PRIu32 " errors", sensor.info.name,
			 device_id(sensor.info), sensor.interval, sensor.error_count);
	}

	return 0;
}

int ZephyrSensors::task_spawn(int argc, char *argv[])
{
	if (zephyr_sensors_count() == 0) {
		PX4_INFO("no sensors in the devicetree");
		return PX4_OK;
	}

	ZephyrSensors *instance = new ZephyrSensors();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int ZephyrSensors::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ZephyrSensors::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Publishes the Zephyr sensors listed in the px4,sensors devicetree node to uORB.

Each sensor is read through the Zephyr sensor API, at 1 kHz for IMUs, 100 Hz for
magnetometers, 50 Hz for barometers and 10 Hz for power monitors, which publish
the status of a battery.

The sensors whose device has the zephyr,deferred-init devicetree property are
optional, like the compass of a GPS module: they are initialized when the driver
starts, and left out if they don't respond.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("zephyr_sensors", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int zephyr_sensors_main(int argc, char *argv[])
{
	return ModuleBase::main(ZephyrSensors::desc, argc, argv);
}
