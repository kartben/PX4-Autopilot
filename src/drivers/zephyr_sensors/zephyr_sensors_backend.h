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
 * @file zephyr_sensors_backend.h
 *
 * Access to the Zephyr sensors listed in the px4,sensors devicetree node.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* same order as the type property in px4,sensors.yaml */
enum zephyr_sensor_type {
	ZEPHYR_SENSOR_TYPE_IMU,
	ZEPHYR_SENSOR_TYPE_ACCEL,
	ZEPHYR_SENSOR_TYPE_GYRO,
	ZEPHYR_SENSOR_TYPE_MAG,
	ZEPHYR_SENSOR_TYPE_BARO,
	ZEPHYR_SENSOR_TYPE_POWER,
	ZEPHYR_SENSOR_TYPE_FUEL_GAUGE,
};

enum zephyr_sensor_bus_type {
	ZEPHYR_SENSOR_BUS_TYPE_UNKNOWN,
	ZEPHYR_SENSOR_BUS_TYPE_I2C,
	ZEPHYR_SENSOR_BUS_TYPE_SPI,
};

struct zephyr_sensor_info {
	const char *name;
	enum zephyr_sensor_type type;
	enum zephyr_sensor_bus_type bus_type;
	uint8_t bus;
	uint8_t address;
	uint8_t devtype;
	uint8_t rotation;
	uint8_t battery;
	bool external;
	bool optional;
};

struct zephyr_sensor_sample {
	uint64_t timestamp_us;
	float accel[3];		/* m/s^2 */
	float gyro[3];		/* rad/s */
	float mag[3];		/* Gauss */
	float pressure;		/* Pa */
	float voltage;		/* V */
	float current;		/* A, positive while discharging, NAN if not measured */
	float temperature;	/* degrees Celsius, NAN if not measured */

	/* measured by fuel gauges, NAN or 0 if not measured */
	float average_current;		/* A, positive while discharging */
	float remaining;		/* state of charge, from 0 to 1 */
	float remaining_capacity;	/* mAh */
	float full_capacity;		/* mAh */
	float time_remaining;		/* s */
	uint16_t cycle_count;
	uint8_t state_of_health;	/* % */
};

size_t zephyr_sensors_count(void);

/**
 * Get the description of a sensor, after initializing its device if the
 * initialization is deferred in the devicetree, which makes the sensor optional.
 * @return 0, or -ENODEV if the sensor device is not ready
 */
int zephyr_sensors_get_info(size_t index, struct zephyr_sensor_info *info);

/**
 * Read a sample from a sensor, blocking until the read completes. Different
 * sensors can be read concurrently, but each from a single thread.
 * @return 0 or a negative error code
 */
int zephyr_sensors_read(size_t index, struct zephyr_sensor_sample *sample);

#ifdef __cplusplus
}
#endif
