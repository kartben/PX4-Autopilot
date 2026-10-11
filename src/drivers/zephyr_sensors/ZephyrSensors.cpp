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

// IMUs are read at the rate of the rate controller, as the IMU drivers of PX4 read their FIFO
static hrt_abstime imu_interval()
{
	int32_t rate_hz = 0;
	param_get(param_find("IMU_GYRO_RATEMAX"), &rate_hz);

	// the limits of PX4Gyroscope::get_max_rate_hz()
	return 1_s / math::constrain(rate_hz, static_cast<int32_t>(100), static_cast<int32_t>(4000));
}

SensorBus::SensorBus(ZephyrSensors &module, const zephyr_sensor_info &info) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(device_id(info))),
	_module(module),
	_bus_type(info.bus_type),
	_bus(info.bus),
	_queue_name(px4::device_bus_to_wq(device_id(info)).name)
{
}

SensorBus::~SensorBus()
{
	for (size_t i = 0; i < _sensor_count; i++) {
		delete _sensors[i]->accel;
		delete _sensors[i]->gyro;
		delete _sensors[i]->mag;
		delete _sensors[i]->baro;
		delete _sensors[i]->battery;
		delete _sensors[i]->fuel_gauge;
		delete _sensors[i];
	}

	perf_free(_cycle_perf);
	perf_free(_read_error_perf);
}

bool SensorBus::has(const zephyr_sensor_info &info) const
{
	return (info.bus_type == _bus_type) && (info.bus == _bus);
}

bool SensorBus::add_sensor(size_t index, const zephyr_sensor_info &info)
{
	Sensor *sensor = new Sensor{};

	if ((sensor == nullptr) || (_sensor_count >= MAX_SENSORS)) {
		delete sensor;
		return false;
	}

	sensor->index = index;
	sensor->info = info;

	const uint32_t id = device_id(info);
	const Rotation rotation = static_cast<Rotation>(info.rotation);
	const bool external = info.external;

	switch (info.type) {
	case ZEPHYR_SENSOR_TYPE_IMU:
		sensor->accel = new PX4Accelerometer(id, rotation, external);
		sensor->gyro = new PX4Gyroscope(id, rotation, external);
		sensor->interval = imu_interval();
		break;

	case ZEPHYR_SENSOR_TYPE_ACCEL:
		sensor->accel = new PX4Accelerometer(id, rotation, external);
		sensor->interval = imu_interval();
		break;

	case ZEPHYR_SENSOR_TYPE_GYRO:
		sensor->gyro = new PX4Gyroscope(id, rotation, external);
		sensor->interval = imu_interval();
		break;

	case ZEPHYR_SENSOR_TYPE_MAG:
		sensor->mag = new PX4Magnetometer(id, rotation, external);
		sensor->interval = MAG_INTERVAL;
		break;

	case ZEPHYR_SENSOR_TYPE_BARO:
		sensor->baro = new PX4Barometer(id, external);
		sensor->interval = BARO_INTERVAL;
		break;

	case ZEPHYR_SENSOR_TYPE_POWER:
		sensor->battery = new Battery(info.battery, this, BATTERY_INTERVAL, battery_status_s::SOURCE_POWER_MODULE);
		sensor->interval = BATTERY_INTERVAL;
		break;

	case ZEPHYR_SENSOR_TYPE_FUEL_GAUGE:
		sensor->fuel_gauge = new uORB::PublicationMulti<battery_status_s> {ORB_ID(battery_status)};
		sensor->interval = BATTERY_INTERVAL;
		break;
	}

	_sensors[_sensor_count++] = sensor;

	return true;
}

void SensorBus::start()
{
	hrt_abstime interval = _sensors[0]->interval;

	for (size_t i = 1; i < _sensor_count; i++) {
		interval = math::min(interval, _sensors[i]->interval);
	}

	_interval = interval;
	ScheduleOnInterval(interval);
}

void SensorBus::Run()
{
	if (_module.should_exit()) {
		ScheduleClear();

		// the module deletes the buses when the last one stops
		_module.bus_stopped();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update;
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
	}

	perf_begin(_cycle_perf);

	for (size_t i = 0; i < _sensor_count; i++) {
		Sensor &sensor = *_sensors[i];
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

			if (sensor.fuel_gauge && (hrt_elapsed_time(&sensor.last_sample) > BATTERY_TIMEOUT)) {
				publish_fuel_gauge(sensor, nullptr, hrt_absolute_time());
			}
		}
	}

	perf_end(_cycle_perf);
}

void SensorBus::publish(Sensor &sensor, const zephyr_sensor_sample &sample)
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

	if (sensor.fuel_gauge) {
		publish_fuel_gauge(sensor, &sample, sample.timestamp_us);
	}
}

// the battery status measured by a fuel gauge, or a disconnected battery without a sample
void SensorBus::publish_fuel_gauge(Sensor &sensor, const zephyr_sensor_sample *sample, hrt_abstime timestamp)
{
	battery_status_s status{};
	status.timestamp = timestamp;
	status.id = sensor.info.battery;
	status.current_a = -1.f;
	status.current_average_a = -1.f;
	status.discharged_mah = -1.f;
	status.remaining = -1.f;
	status.time_remaining_s = NAN;
	status.temperature = NAN;

	if (sample) {
		status.connected = true;
		status.voltage_v = sample->voltage;
		status.temperature = sample->temperature;
		status.time_remaining_s = sample->time_remaining;
		status.cycle_count = sample->cycle_count;
		status.state_of_health = sample->state_of_health;

		if (PX4_ISFINITE(sample->current)) {
			status.current_a = sample->current;
		}

		if (PX4_ISFINITE(sample->average_current)) {
			status.current_average_a = sample->average_current;
		}

		if (PX4_ISFINITE(sample->full_capacity)) {
			status.capacity = static_cast<uint16_t>(math::constrain(sample->full_capacity, 0.f, (float)UINT16_MAX));
		}

		if (PX4_ISFINITE(sample->remaining_capacity)) {
			if (!PX4_ISFINITE(sensor.initial_remaining_capacity)) {
				sensor.initial_remaining_capacity = sample->remaining_capacity;
			}

			status.discharged_mah = sensor.initial_remaining_capacity - sample->remaining_capacity;
		}

		if (PX4_ISFINITE(sample->remaining)) {
			status.remaining = math::constrain(sample->remaining, 0.f, 1.f);

			if (status.remaining > _param_bat_low_thr.get()) {
				status.warning = battery_status_s::WARNING_NONE;

			} else if (status.remaining > _param_bat_crit_thr.get()) {
				status.warning = battery_status_s::WARNING_LOW;

			} else if (status.remaining > _param_bat_emergen_thr.get()) {
				status.warning = battery_status_s::WARNING_CRITICAL;

			} else {
				status.warning = battery_status_s::WARNING_EMERGENCY;
			}
		}
	}

	sensor.fuel_gauge->publish(status);
}

void SensorBus::print_status() const
{
	PX4_INFO("%s", _queue_name);
	perf_print_counter(_cycle_perf);
	perf_print_counter(_read_error_perf);

	for (size_t i = 0; i < _sensor_count; i++) {
		const Sensor &sensor = *_sensors[i];
		PX4_INFO("%s: device id %" PRIu32 ", every %" PRIu64 " us, %" PRIu32 " errors", sensor.info.name,
			 device_id(sensor.info), sensor.interval, sensor.error_count);
	}
}

ZephyrSensors::~ZephyrSensors()
{
	for (size_t i = 0; i < _bus_count; i++) {
		delete _buses[i];
	}
}

int ZephyrSensors::init()
{
	for (size_t index = 0; index < zephyr_sensors_count(); index++) {
		if (_sensor_count >= SensorBus::MAX_SENSORS) {
			PX4_WARN("only %zu sensors supported", SensorBus::MAX_SENSORS);
			break;
		}

		zephyr_sensor_info info{};
		const int ret = zephyr_sensors_get_info(index, &info);

		if (ret != 0) {
			if (!info.optional) {
				PX4_ERR("%s not ready (%d)", info.name, ret);
			}

			continue;
		}

		if (info.rotation >= ROTATION_MAX) {
			PX4_ERR("%s: invalid rotation %u", info.name, info.rotation);
			continue;
		}

		SensorBus *bus = nullptr;

		for (size_t i = 0; (i < _bus_count) && (bus == nullptr); i++) {
			if (_buses[i]->has(info)) {
				bus = _buses[i];
			}
		}

		if (bus == nullptr) {
			bus = new SensorBus(*this, info);

			if (bus == nullptr) {
				PX4_ERR("alloc failed");
				return PX4_ERROR;
			}

			_buses[_bus_count++] = bus;
		}

		if (!bus->add_sensor(index, info)) {
			PX4_ERR("alloc failed");
			return PX4_ERROR;
		}

		_sensor_count++;
	}

	if (_sensor_count == 0) {
		return PX4_ERROR;
	}

	// the sensors of each bus are read in turn, on the work queue of the bus
	_running_buses.store(_bus_count);

	for (size_t i = 0; i < _bus_count; i++) {
		_buses[i]->start();
	}

	return PX4_OK;
}

int ZephyrSensors::print_status()
{
	for (size_t i = 0; i < _bus_count; i++) {
		_buses[i]->print_status();
	}

	return 0;
}

void ZephyrSensors::bus_stopped()
{
	if (_running_buses.fetch_sub(1) == 1) {
		exit_and_cleanup(desc);
	}
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

Each sensor is read through the Zephyr sensor API, IMUs at the rate of the rate
controller (IMU_GYRO_RATEMAX), magnetometers at 100 Hz, barometers at 50 Hz and
power monitors, which publish the status of a battery, at 10 Hz. Smart batteries
are read at 10 Hz through the Zephyr fuel gauge API, and publish the state of
charge measured by their fuel gauge.

The sensors of each bus are read in turn on the work queue of the bus, so that a
slow bus doesn't delay the sensors of the other buses.

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
