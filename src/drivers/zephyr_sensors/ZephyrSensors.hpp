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
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/barometer/PX4Barometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

using namespace time_literals;

class ZephyrSensors : public ModuleBase, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	ZephyrSensors();
	~ZephyrSensors() override;

	static int task_spawn(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[]);

	static int print_usage(const char *reason = nullptr);

	int init();

	int print_status() override;

private:
	static constexpr size_t MAX_SENSORS{8};

	static constexpr hrt_abstime IMU_INTERVAL{1_ms};
	static constexpr hrt_abstime MAG_INTERVAL{10_ms};
	static constexpr hrt_abstime BARO_INTERVAL{20_ms};

	struct Sensor {
		size_t index{0};
		zephyr_sensor_info info{};
		hrt_abstime interval{0};
		hrt_abstime next_read{0};
		uint32_t error_count{0};

		PX4Accelerometer *accel{nullptr};
		PX4Gyroscope *gyro{nullptr};
		PX4Magnetometer *mag{nullptr};
		PX4Barometer *baro{nullptr};
	};

	void Run() override;

	bool add_sensor(size_t index);
	void publish(Sensor &sensor, const zephyr_sensor_sample &sample);

	Sensor _sensors[MAX_SENSORS] {};
	size_t _sensor_count{0};
	hrt_abstime _interval{0};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _read_error_perf{perf_alloc(PC_COUNT, MODULE_NAME": read error")};
};
