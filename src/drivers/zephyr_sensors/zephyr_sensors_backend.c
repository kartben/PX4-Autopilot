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

#include "zephyr_sensors_backend.h"

#include <drivers/drv_sensor.h>

#include <errno.h>
#include <math.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/rtio/rtio.h>

#ifdef CONFIG_FUEL_GAUGE
#include <zephyr/drivers/fuel_gauge.h>
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(px4_sensors)

#define SENSORS_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(px4_sensors)
#define SENSOR_DEVICE_NODE(node) DT_PHANDLE(node, sensor)
#define SENSOR_IODEV(node) UTIL_CAT(px4_sensor_iodev_, DT_DEP_ORD(node))
#define IS_FUEL_GAUGE(node) DT_ENUM_HAS_VALUE(node, type, fuel_gauge)

#define IMU_CHANNELS {SENSOR_CHAN_ACCEL_XYZ, 0}, {SENSOR_CHAN_GYRO_XYZ, 0}, {SENSOR_CHAN_DIE_TEMP, 0}
#define ACCEL_CHANNELS {SENSOR_CHAN_ACCEL_XYZ, 0}
#define GYRO_CHANNELS {SENSOR_CHAN_GYRO_XYZ, 0}
#define MAG_CHANNELS {SENSOR_CHAN_MAGN_XYZ, 0}
#define BARO_CHANNELS {SENSOR_CHAN_PRESS, 0}, {SENSOR_CHAN_AMBIENT_TEMP, 0}
#define POWER_CHANNELS {SENSOR_CHAN_VOLTAGE, 0}, {SENSOR_CHAN_CURRENT, 0}

/* fuel gauges are read with the fuel gauge API instead */
#define SENSOR_IODEV_DEFINE(node)                                                                  \
	COND_CODE_1(IS_FUEL_GAUGE(node), (),                                                       \
		    (SENSOR_DT_READ_IODEV(SENSOR_IODEV(node), SENSOR_DEVICE_NODE(node),            \
					  UTIL_CAT(DT_STRING_UPPER_TOKEN(node, type), _CHANNELS));))

DT_FOREACH_CHILD_STATUS_OKAY(SENSORS_NODE, SENSOR_IODEV_DEFINE)

#define FUEL_GAUGE_CHECK(node)                                                                     \
	BUILD_ASSERT(!IS_FUEL_GAUGE(node) || IS_ENABLED(CONFIG_FUEL_GAUGE),                         \
		     DT_NODE_FULL_NAME(node) " is a fuel gauge, which needs CONFIG_FUEL_GAUGE");

DT_FOREACH_CHILD_STATUS_OKAY(SENSORS_NODE, FUEL_GAUGE_CHECK)

#define HAS_COMPAT(node, compat) DT_NODE_HAS_COMPAT(SENSOR_DEVICE_NODE(node), compat)

#define SENSOR_DEVTYPE(node)                                                                       \
	(HAS_COMPAT(node, invensense_icm42686)    ? DRV_IMU_DEVTYPE_ICM42686P                     \
	 : HAS_COMPAT(node, invensense_icm42688)  ? DRV_IMU_DEVTYPE_ICM42688P                     \
	 : HAS_COMPAT(node, bosch_bmi08x_accel)   ? DRV_ACC_DEVTYPE_BMI088                        \
	 : HAS_COMPAT(node, bosch_bmi08x_gyro)    ? DRV_GYR_DEVTYPE_BMI088                        \
	 : HAS_COMPAT(node, asahi_kasei_akm09918c) ? DRV_MAG_DEVTYPE_AK09918                      \
	 : HAS_COMPAT(node, bosch_bmm150)         ? DRV_MAG_DEVTYPE_BMM150                        \
	 : HAS_COMPAT(node, isentek_ist8310)      ? DRV_MAG_DEVTYPE_IST8310                       \
	 : HAS_COMPAT(node, pni_rm3100)           ? DRV_MAG_DEVTYPE_RM3100                        \
	 : HAS_COMPAT(node, st_lis3mdl_magn)      ? DRV_MAG_DEVTYPE_LIS3MDL                       \
	 : HAS_COMPAT(node, bosch_bmp388)         ? DRV_BARO_DEVTYPE_BMP388                       \
	 : HAS_COMPAT(node, bosch_bmp390)         ? DRV_BARO_DEVTYPE_BMP390                       \
	 : HAS_COMPAT(node, bosch_bmp581)         ? DRV_BARO_DEVTYPE_BMP581                       \
	 : DRV_DEVTYPE_UNUSED)

#define SENSOR_BUS_TYPE(node)                                                                      \
	(DT_ON_BUS(SENSOR_DEVICE_NODE(node), i2c)   ? ZEPHYR_SENSOR_BUS_TYPE_I2C                   \
	 : DT_ON_BUS(SENSOR_DEVICE_NODE(node), spi) ? ZEPHYR_SENSOR_BUS_TYPE_SPI                   \
	 : ZEPHYR_SENSOR_BUS_TYPE_UNKNOWN)

#define SENSOR_ADDRESS(node)                                                                       \
	COND_CODE_1(DT_REG_HAS_IDX(SENSOR_DEVICE_NODE(node), 0),                                   \
		    (DT_REG_ADDR(SENSOR_DEVICE_NODE(node))), (0))

struct sensor_entry {
	const char *name;
	const struct device *dev;
	uint32_t bus_ord;
	struct rtio_iodev *iodev;
	enum zephyr_sensor_type type;
	enum zephyr_sensor_bus_type bus_type;
	uint8_t address;
	uint8_t devtype;
	uint8_t rotation;
	bool external;
	bool deferred;
	uint8_t battery;
};

#define SENSOR_ENTRY(node)                                                                         \
	{                                                                                          \
		.name = DT_NODE_FULL_NAME(node),                                                   \
			.dev = DEVICE_DT_GET(SENSOR_DEVICE_NODE(node)),                                    \
			       .bus_ord = DT_DEP_ORD(DT_PARENT(SENSOR_DEVICE_NODE(node))),                        \
					  .iodev = COND_CODE_1(IS_FUEL_GAUGE(node), (NULL), (&SENSOR_IODEV(node))),         \
						   .type = (enum zephyr_sensor_type)DT_ENUM_IDX(node, type),                          \
								   .bus_type = SENSOR_BUS_TYPE(node),                                                 \
										   .address = SENSOR_ADDRESS(node),                                                   \
												   .devtype = SENSOR_DEVTYPE(node),                                                   \
														   .rotation = DT_PROP(node, rotation),                                               \
																   .external = DT_PROP(node, external),                                               \
																		   .deferred = DT_PROP_OR(SENSOR_DEVICE_NODE(node), zephyr_deferred_init, false),     \
																				   .battery = DT_PROP(node, battery),                                                \
	},

static const struct sensor_entry sensors[] = {
	DT_FOREACH_CHILD_STATUS_OKAY(SENSORS_NODE, SENSOR_ENTRY)
};

/* reads are synchronous and come from a single work queue */
RTIO_DEFINE(px4_sensors_rtio, 1, 1);

static uint8_t read_buffer[128] __aligned(8);

size_t zephyr_sensors_count(void)
{
	return ARRAY_SIZE(sensors);
}

/* PX4 numbers the buses of each type from 1, here in devicetree order */
static uint8_t bus_number(size_t index)
{
	uint8_t bus = 0;

	for (size_t i = 0; i <= index; i++) {
		if (sensors[i].bus_type != sensors[index].bus_type) {
			continue;
		}

		bool first_on_bus = true;

		for (size_t j = 0; j < i; j++) {
			if (sensors[j].bus_ord == sensors[i].bus_ord) {
				first_on_bus = false;
				break;
			}
		}

		if (first_on_bus) {
			bus++;

			if (sensors[i].bus_ord == sensors[index].bus_ord) {
				break;
			}
		}
	}

	return bus;
}

#ifdef CONFIG_FUEL_GAUGE
static float fuel_gauge_value(const struct device *dev, fuel_gauge_prop_t prop, float scale)
{
	union fuel_gauge_prop_val val;

	if (fuel_gauge_get_prop(dev, prop, &val) != 0) {
		return NAN;
	}

	switch (prop) {
	case FUEL_GAUGE_CURRENT_UA:
		return val.current_ua * scale;

	case FUEL_GAUGE_AVG_CURRENT_UA:
		return val.avg_current_ua * scale;

	case FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE_PCT:
		return val.relative_state_of_charge_pct * scale;

	case FUEL_GAUGE_REMAINING_CAPACITY_UAH:
		return val.remaining_capacity_uah * scale;

	case FUEL_GAUGE_FULL_CHARGE_CAPACITY_UAH:
		return val.full_charge_capacity_uah * scale;

	case FUEL_GAUGE_RUNTIME_TO_EMPTY_MINS:
		/* the largest value means that the battery isn't discharging */
		return (val.runtime_to_empty_mins < UINT16_MAX) ? val.runtime_to_empty_mins * scale : NAN;

	case FUEL_GAUGE_TEMPERATURE_DK:
		return val.temperature_dk * scale - 273.15f;

	case FUEL_GAUGE_CYCLE_COUNT:
		return val.cycle_count * scale;

	case FUEL_GAUGE_STATE_OF_HEALTH:
		return val.state_of_health * scale;

	default:
		return NAN;
	}
}

/* the properties that a fuel gauge doesn't support are not measured, except the voltage */
static int read_fuel_gauge(const struct device *dev, struct zephyr_sensor_sample *sample)
{
	union fuel_gauge_prop_val voltage;
	const int ret = fuel_gauge_get_prop(dev, FUEL_GAUGE_VOLTAGE_UV, &voltage);

	if (ret != 0) {
		return ret;
	}

	const float cycle_count = fuel_gauge_value(dev, FUEL_GAUGE_CYCLE_COUNT, 1.f);
	const float state_of_health = fuel_gauge_value(dev, FUEL_GAUGE_STATE_OF_HEALTH, 1.f);

	/* the fuel gauge current is positive while charging */
	sample->voltage = voltage.voltage_uv * 1e-6f;
	sample->current = fuel_gauge_value(dev, FUEL_GAUGE_CURRENT_UA, -1e-6f);
	sample->average_current = fuel_gauge_value(dev, FUEL_GAUGE_AVG_CURRENT_UA, -1e-6f);
	sample->remaining = fuel_gauge_value(dev, FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE_PCT, 0.01f);
	sample->remaining_capacity = fuel_gauge_value(dev, FUEL_GAUGE_REMAINING_CAPACITY_UAH, 1e-3f);
	sample->full_capacity = fuel_gauge_value(dev, FUEL_GAUGE_FULL_CHARGE_CAPACITY_UAH, 1e-3f);
	sample->time_remaining = fuel_gauge_value(dev, FUEL_GAUGE_RUNTIME_TO_EMPTY_MINS, 60.f);
	sample->temperature = fuel_gauge_value(dev, FUEL_GAUGE_TEMPERATURE_DK, 0.1f);
	sample->cycle_count = isfinite(cycle_count) ? (uint16_t)cycle_count : 0;
	sample->state_of_health = isfinite(state_of_health) ? (uint8_t)state_of_health : 0;

	return 0;
}
#else
static int read_fuel_gauge(const struct device *dev, struct zephyr_sensor_sample *sample)
{
	return -ENOTSUP;
}
#endif

int zephyr_sensors_get_info(size_t index, struct zephyr_sensor_info *info)
{
	if (index >= ARRAY_SIZE(sensors)) {
		return -EINVAL;
	}

	const struct sensor_entry *sensor = &sensors[index];

	info->name = sensor->name;
	info->type = sensor->type;
	info->bus_type = sensor->bus_type;
	info->bus = (sensor->bus_type == ZEPHYR_SENSOR_BUS_TYPE_UNKNOWN) ? 0 : bus_number(index);
	info->address = sensor->address;
	info->devtype = sensor->devtype;
	info->rotation = sensor->rotation;
	info->battery = sensor->battery;
	info->external = sensor->external;
	info->optional = sensor->deferred;

	if (sensor->deferred && !device_is_ready(sensor->dev)) {
		/* a device that doesn't respond stays not ready */
		(void)device_init(sensor->dev);
	}

	if (!device_is_ready(sensor->dev)) {
		return -ENODEV;
	}

	/* fuel gauge drivers don't check that the gauge responds when they are initialized */
	if (sensor->deferred && (sensor->type == ZEPHYR_SENSOR_TYPE_FUEL_GAUGE)) {
		struct zephyr_sensor_sample sample;

		return (read_fuel_gauge(sensor->dev, &sample) == 0) ? 0 : -ENODEV;
	}

	return 0;
}

static float q31_to_float(q31_t value, int8_t shift)
{
	return ldexpf((float)value, shift - 31);
}

static int decode_three_axis(const struct sensor_decoder_api *decoder, enum sensor_channel channel,
			     float values[3], uint64_t *timestamp_ns)
{
	const struct sensor_chan_spec spec = {channel, 0};
	struct sensor_three_axis_data data;
	uint32_t fit = 0;
	const int ret = decoder->decode(read_buffer, spec, &fit, 1, &data);

	if (ret <= 0) {
		return (ret < 0) ? ret : -ENODATA;
	}

	for (int axis = 0; axis < 3; axis++) {
		values[axis] = q31_to_float(data.readings[0].values[axis], data.shift);
	}

	*timestamp_ns = data.header.base_timestamp_ns + data.readings[0].timestamp_delta;
	return 0;
}

static int decode_q31(const struct sensor_decoder_api *decoder, enum sensor_channel channel,
		      float *value, uint64_t *timestamp_ns)
{
	const struct sensor_chan_spec spec = {channel, 0};
	struct sensor_q31_data data;
	uint32_t fit = 0;
	const int ret = decoder->decode(read_buffer, spec, &fit, 1, &data);

	if (ret <= 0) {
		return (ret < 0) ? ret : -ENODATA;
	}

	*value = q31_to_float(data.readings[0].value, data.shift);
	*timestamp_ns = data.header.base_timestamp_ns + data.readings[0].timestamp_delta;
	return 0;
}

int zephyr_sensors_read(size_t index, struct zephyr_sensor_sample *sample)
{
	if (index >= ARRAY_SIZE(sensors)) {
		return -EINVAL;
	}

	const struct sensor_entry *sensor = &sensors[index];

	if (sensor->type == ZEPHYR_SENSOR_TYPE_FUEL_GAUGE) {
		return read_fuel_gauge(sensor->dev, sample);
	}

	const struct sensor_decoder_api *decoder;

	int ret = sensor_get_decoder(sensor->dev, &decoder);

	if (ret == 0) {
		ret = sensor_read(sensor->iodev, &px4_sensors_rtio, read_buffer, sizeof(read_buffer));
	}

	if (ret != 0) {
		return ret;
	}

	uint64_t timestamp_ns = 0;
	uint64_t temperature_timestamp_ns = 0;
	float pressure_kpa = NAN;

	sample->temperature = NAN;

	switch (sensor->type) {
	case ZEPHYR_SENSOR_TYPE_IMU:
		ret = decode_three_axis(decoder, SENSOR_CHAN_ACCEL_XYZ, sample->accel, &timestamp_ns);

		if (ret == 0) {
			ret = decode_three_axis(decoder, SENSOR_CHAN_GYRO_XYZ, sample->gyro, &timestamp_ns);
		}

		if (ret == 0) {
			decode_q31(decoder, SENSOR_CHAN_DIE_TEMP, &sample->temperature,
				   &temperature_timestamp_ns);
		}

		break;

	case ZEPHYR_SENSOR_TYPE_ACCEL:
		ret = decode_three_axis(decoder, SENSOR_CHAN_ACCEL_XYZ, sample->accel, &timestamp_ns);
		break;

	case ZEPHYR_SENSOR_TYPE_GYRO:
		ret = decode_three_axis(decoder, SENSOR_CHAN_GYRO_XYZ, sample->gyro, &timestamp_ns);
		break;

	case ZEPHYR_SENSOR_TYPE_MAG:
		ret = decode_three_axis(decoder, SENSOR_CHAN_MAGN_XYZ, sample->mag, &timestamp_ns);
		break;

	case ZEPHYR_SENSOR_TYPE_BARO:
		ret = decode_q31(decoder, SENSOR_CHAN_PRESS, &pressure_kpa, &timestamp_ns);
		sample->pressure = pressure_kpa * 1000.f;

		if (ret == 0) {
			decode_q31(decoder, SENSOR_CHAN_AMBIENT_TEMP, &sample->temperature,
				   &temperature_timestamp_ns);
		}

		break;

	case ZEPHYR_SENSOR_TYPE_POWER:
		ret = decode_q31(decoder, SENSOR_CHAN_VOLTAGE, &sample->voltage, &timestamp_ns);

		if (ret == 0) {
			ret = decode_q31(decoder, SENSOR_CHAN_CURRENT, &sample->current, &timestamp_ns);
		}

		break;

	case ZEPHYR_SENSOR_TYPE_FUEL_GAUGE:
		/* read with the fuel gauge API */
		ret = -EINVAL;
		break;
	}

	sample->timestamp_us = timestamp_ns / NSEC_PER_USEC;

	return ret;
}

#else

size_t zephyr_sensors_count(void)
{
	return 0;
}

int zephyr_sensors_get_info(size_t index, struct zephyr_sensor_info *info)
{
	return -EINVAL;
}

int zephyr_sensors_read(size_t index, struct zephyr_sensor_sample *sample)
{
	return -EINVAL;
}

#endif
