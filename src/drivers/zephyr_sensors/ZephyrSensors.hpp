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
 * @file ZephyrSensors.hpp
 *
 * Publishes the Zephyr sensors listed in the px4,sensors devicetree node to uORB.
 */

#pragma once

#include "zephyr_sensors_backend.h"

#include <drivers/drv_hrt.h>
#include <lib/battery/battery.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/barometer/PX4Barometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel_fifo.h>
#include <uORB/topics/sensor_gyro_fifo.h>

using namespace time_literals;

class ZephyrSensors;

/**
 * The sensors of a bus, read in turn on the work queue of the bus.
 */
class SensorBus : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static constexpr size_t MAX_SENSORS{16};

	SensorBus(ZephyrSensors &module, const zephyr_sensor_info &info);
	~SensorBus() override;

	bool has(const zephyr_sensor_info &info) const;

	bool add_sensor(size_t index, const zephyr_sensor_info &info);

	void start();

	void print_status() const;

private:
	static constexpr hrt_abstime MAG_INTERVAL{10_ms};
	static constexpr hrt_abstime BARO_INTERVAL{20_ms};
	static constexpr hrt_abstime BATTERY_INTERVAL{100_ms};

	// an IMU whose FIFO isn't read for this long is polled instead
	static constexpr hrt_abstime FIFO_TIMEOUT{100_ms};

	// a battery is disconnected when its power monitor or fuel gauge can't be read for this long
	static constexpr hrt_abstime BATTERY_TIMEOUT{2_s};

	struct Sensor {
		size_t index{0};
		zephyr_sensor_info info{};
		hrt_abstime interval{0};
		hrt_abstime next_read{0};
		hrt_abstime last_sample{0};
		uint32_t error_count{0};

		PX4Accelerometer *accel{nullptr};
		PX4Gyroscope *gyro{nullptr};
		PX4Magnetometer *mag{nullptr};
		PX4Barometer *baro{nullptr};
		Battery *battery{nullptr};

		uORB::PublicationMulti<battery_status_s> *fuel_gauge{nullptr};
		float initial_remaining_capacity{NAN};

		// the FIFO is streamed instead of polling the IMU
		zephyr_sensor_fifo *fifo{nullptr};
		bool fifo_data{false};
	};

	void Run() override;

	void schedule();

	bool start_fifo(Sensor &sensor);
	void stop_fifo(Sensor &sensor);
	void read_fifo(Sensor &sensor);
	static void fifo_ready(void *arg);

	void publish(Sensor &sensor, const zephyr_sensor_sample &sample);
	void publish_fifo(Sensor &sensor, const zephyr_sensor_fifo &fifo);
	void publish_fuel_gauge(Sensor &sensor, const zephyr_sensor_sample *sample, hrt_abstime timestamp);

	ZephyrSensors &_module;
	const zephyr_sensor_bus_type _bus_type;
	const uint8_t _bus;
	const char *const _queue_name;

	Sensor *_sensors[MAX_SENSORS] {};
	size_t _sensor_count{0};
	hrt_abstime _interval{0};

	bool _started{false};
	bool _stopped{false};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::BAT_LOW_THR>) _param_bat_low_thr,
		(ParamFloat<px4::params::BAT_CRIT_THR>) _param_bat_crit_thr,
		(ParamFloat<px4::params::BAT_EMERGEN_THR>) _param_bat_emergen_thr
	)

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _read_error_perf{perf_alloc(PC_COUNT, MODULE_NAME": read error")};
	perf_counter_t _fifo_overflow_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO overflow")};
};

class ZephyrSensors : public ModuleBase
{
public:
	static Descriptor desc;

	~ZephyrSensors() override;

	static int task_spawn(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[]);

	static int print_usage(const char *reason = nullptr);

	int init();

	int print_status() override;

	// called by each bus when it stops after a stop request, the module exits with the last one
	void bus_stopped();

private:
	SensorBus *_buses[SensorBus::MAX_SENSORS] {};
	size_t _bus_count{0};
	size_t _sensor_count{0};

	px4::atomic<size_t> _running_buses{0};
};
