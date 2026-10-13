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

#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>

#include <errno.h>
#include <math.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_clock.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/spinlock.h>

#ifdef CONFIG_FUEL_GAUGE
#include <zephyr/drivers/fuel_gauge.h>
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(px4_sensors)

#define SENSORS_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(px4_sensors)
#define SENSOR_DEVICE_NODE(node) DT_PHANDLE(node, sensor)
#define SENSOR_IODEV(node) UTIL_CAT(px4_sensor_iodev_, DT_DEP_ORD(node))
#define SENSOR_RTIO(node) UTIL_CAT(px4_sensor_rtio_, DT_DEP_ORD(node))
#define SENSOR_BUFFER(node) UTIL_CAT(px4_sensor_buffer_, DT_DEP_ORD(node))
#define IS_FUEL_GAUGE(node) DT_ENUM_HAS_VALUE(node, type, fuel_gauge)

#define IMU_CHANNELS {SENSOR_CHAN_ACCEL_XYZ, 0}, {SENSOR_CHAN_GYRO_XYZ, 0}, {SENSOR_CHAN_DIE_TEMP, 0}
#define ACCEL_CHANNELS {SENSOR_CHAN_ACCEL_XYZ, 0}
#define GYRO_CHANNELS {SENSOR_CHAN_GYRO_XYZ, 0}
#define MAG_CHANNELS {SENSOR_CHAN_MAGN_XYZ, 0}
#define BARO_CHANNELS {SENSOR_CHAN_PRESS, 0}, {SENSOR_CHAN_AMBIENT_TEMP, 0}
#define POWER_CHANNELS {SENSOR_CHAN_VOLTAGE, 0}, {SENSOR_CHAN_CURRENT, 0}

#define FIFO_TRIGGERS {SENSOR_TRIG_FIFO_WATERMARK, SENSOR_STREAM_DATA_INCLUDE}

#define READ_BUFFER_SIZE 128

/*
 * Each sensor has its own RTIO context and buffer, as the sensors of different buses are read
 * concurrently. Fuel gauges are read with the fuel gauge API instead.
 */
#define SENSOR_IODEV_DEFINE(node)                                                                  \
	COND_CODE_1(IS_FUEL_GAUGE(node), (),                                                       \
		    (SENSOR_DT_READ_IODEV(SENSOR_IODEV(node), SENSOR_DEVICE_NODE(node),            \
					  UTIL_CAT(DT_STRING_UPPER_TOKEN(node, type), _CHANNELS));   \
		     RTIO_DEFINE(SENSOR_RTIO(node), 1, 1);                                         \
		     static uint8_t SENSOR_BUFFER(node)[READ_BUFFER_SIZE] __aligned(8);))

DT_FOREACH_CHILD_STATUS_OKAY(SENSORS_NODE, SENSOR_IODEV_DEFINE)

#define SENSOR_HAS_PROP(node, prop) DT_NODE_HAS_PROP(SENSOR_DEVICE_NODE(node), prop)
#define IS_IMU(node)                                                                               \
	UTIL_OR(DT_ENUM_HAS_VALUE(node, type, imu),                                                \
		UTIL_OR(DT_ENUM_HAS_VALUE(node, type, accel), DT_ENUM_HAS_VALUE(node, type, gyro)))
#define HAS_INTERRUPT(node)                                                                        \
	UTIL_OR(UTIL_OR(SENSOR_HAS_PROP(node, int_gpios), SENSOR_HAS_PROP(node, irq_gpios)),       \
		UTIL_OR(SENSOR_HAS_PROP(node, int1_gpios), SENSOR_HAS_PROP(node, int2_gpios)))

/* the FIFO of an IMU is streamed if its driver supports it, which needs the interrupt pin */
#define HAS_FIFO(node) UTIL_AND(IS_IMU(node), HAS_INTERRUPT(node))

#define SENSOR_FIFO(node) UTIL_CAT(px4_sensor_fifo_, DT_DEP_ORD(node))
#define SENSOR_FIFO_IODEV(node) UTIL_CAT(px4_sensor_fifo_iodev_, DT_DEP_ORD(node))
#define SENSOR_FIFO_RTIO(node) UTIL_CAT(px4_sensor_fifo_rtio_, DT_DEP_ORD(node))

/*
 * Each FIFO read of a stream takes a buffer of whole blocks until it is read, with a completion
 * queue event, and fails when there are no blocks left.
 */
#define FIFO_BLOCK_SIZE 64
#define FIFO_BLOCK_COUNT DIV_ROUND_UP(CONFIG_PX4_SENSORS_FIFO_BUFFER_SIZE, FIFO_BLOCK_SIZE)
#define FIFO_CQ_SIZE (FIFO_BLOCK_COUNT + 1)

#define DECODED_SIZE                                                                               \
	(sizeof(struct sensor_three_axis_data) +                                                   \
	 (ZEPHYR_SENSOR_FIFO_MAX_SAMPLES - 1) * sizeof(struct sensor_three_axis_sample_data))

BUILD_ASSERT(sizeof(struct sensor_q31_data) +
	     (ZEPHYR_SENSOR_FIFO_MAX_SAMPLES - 1) * sizeof(struct sensor_q31_sample_data) <= DECODED_SIZE);

struct fifo_channel {
	float full_scale;		/* reported by the driver, 0 if unknown */
	float scale;			/* per count */
	int8_t shift;
	uint64_t last_timestamp_ns;
	float dt_us;
};

struct sensor_fifo {
	struct k_spinlock lock;
	zephyr_sensor_fifo_callback_t callback;
	void *arg;
	bool running;
	struct fifo_channel accel;
	struct fifo_channel gyro;
	union {
		struct sensor_three_axis_data three_axis;
		struct sensor_q31_data q31;
		uint8_t buffer[DECODED_SIZE];
	} decoded;
};

#define FIFO_RTIO_DEFINE(name)                                                                     \
	RTIO_DEFINE_WITH_MEMPOOL(name, 1, FIFO_CQ_SIZE, FIFO_BLOCK_COUNT, FIFO_BLOCK_SIZE,          \
				 sizeof(uint64_t))

#define SENSOR_FIFO_DEFINE(node)                                                                   \
	IF_ENABLED(HAS_FIFO(node),                                                                 \
		   (SENSOR_DT_STREAM_IODEV(SENSOR_FIFO_IODEV(node), SENSOR_DEVICE_NODE(node),        \
					   FIFO_TRIGGERS);                                       \
		    FIFO_RTIO_DEFINE(SENSOR_FIFO_RTIO(node));                                       \
		    static struct sensor_fifo SENSOR_FIFO(node);))

DT_FOREACH_CHILD_STATUS_OKAY(SENSORS_NODE, SENSOR_FIFO_DEFINE)

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
	struct rtio *rtio;
	uint8_t *buffer;
	struct rtio_iodev *fifo_iodev;
	struct rtio *fifo_rtio;
	struct sensor_fifo *fifo;
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
																						   .rtio = COND_CODE_1(IS_FUEL_GAUGE(node), (NULL), (&SENSOR_RTIO(node))),           \
																								   .buffer = COND_CODE_1(IS_FUEL_GAUGE(node), (NULL), (SENSOR_BUFFER(node))),        \
																										   .fifo_iodev = COND_CODE_1(HAS_FIFO(node), (&SENSOR_FIFO_IODEV(node)), (NULL)),     \
																												   .fifo_rtio = COND_CODE_1(HAS_FIFO(node), (&SENSOR_FIFO_RTIO(node)), (NULL)),       \
																														   .fifo = COND_CODE_1(HAS_FIFO(node), (&SENSOR_FIFO(node)), (NULL)),                 \
	},

static const struct sensor_entry sensors[] = {
	DT_FOREACH_CHILD_STATUS_OKAY(SENSORS_NODE, SENSOR_ENTRY)
};

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

static int decode_three_axis(const struct sensor_decoder_api *decoder, const uint8_t *buffer,
			     enum sensor_channel channel, float values[3], uint64_t *timestamp_ns)
{
	const struct sensor_chan_spec spec = {channel, 0};
	struct sensor_three_axis_data data;
	uint32_t fit = 0;
	const int ret = decoder->decode(buffer, spec, &fit, 1, &data);

	if (ret <= 0) {
		return (ret < 0) ? ret : -ENODATA;
	}

	for (int axis = 0; axis < 3; axis++) {
		values[axis] = q31_to_float(data.readings[0].values[axis], data.shift);
	}

	*timestamp_ns = data.header.base_timestamp_ns + data.readings[0].timestamp_delta;
	return 0;
}

static int decode_q31(const struct sensor_decoder_api *decoder, const uint8_t *buffer,
		      enum sensor_channel channel, float *value, uint64_t *timestamp_ns)
{
	const struct sensor_chan_spec spec = {channel, 0};
	struct sensor_q31_data data;
	uint32_t fit = 0;
	const int ret = decoder->decode(buffer, spec, &fit, 1, &data);

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
		ret = sensor_read(sensor->iodev, sensor->rtio, sensor->buffer, READ_BUFFER_SIZE);
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
		ret = decode_three_axis(decoder, sensor->buffer, SENSOR_CHAN_ACCEL_XYZ, sample->accel, &timestamp_ns);

		if (ret == 0) {
			ret = decode_three_axis(decoder, sensor->buffer, SENSOR_CHAN_GYRO_XYZ, sample->gyro, &timestamp_ns);
		}

		if (ret == 0) {
			decode_q31(decoder, sensor->buffer, SENSOR_CHAN_DIE_TEMP, &sample->temperature,
				   &temperature_timestamp_ns);
		}

		break;

	case ZEPHYR_SENSOR_TYPE_ACCEL:
		ret = decode_three_axis(decoder, sensor->buffer, SENSOR_CHAN_ACCEL_XYZ, sample->accel, &timestamp_ns);
		break;

	case ZEPHYR_SENSOR_TYPE_GYRO:
		ret = decode_three_axis(decoder, sensor->buffer, SENSOR_CHAN_GYRO_XYZ, sample->gyro, &timestamp_ns);
		break;

	case ZEPHYR_SENSOR_TYPE_MAG:
		ret = decode_three_axis(decoder, sensor->buffer, SENSOR_CHAN_MAGN_XYZ, sample->mag, &timestamp_ns);
		break;

	case ZEPHYR_SENSOR_TYPE_BARO:
		ret = decode_q31(decoder, sensor->buffer, SENSOR_CHAN_PRESS, &pressure_kpa, &timestamp_ns);
		sample->pressure = pressure_kpa * 1000.f;

		if (ret == 0) {
			decode_q31(decoder, sensor->buffer, SENSOR_CHAN_AMBIENT_TEMP, &sample->temperature,
				   &temperature_timestamp_ns);
		}

		break;

	case ZEPHYR_SENSOR_TYPE_POWER:
		ret = decode_q31(decoder, sensor->buffer, SENSOR_CHAN_VOLTAGE, &sample->voltage, &timestamp_ns);

		if (ret == 0) {
			ret = decode_q31(decoder, sensor->buffer, SENSOR_CHAN_CURRENT, &sample->current, &timestamp_ns);
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

static void fifo_cqe_callback(struct rtio *r, void *arg)
{
	struct sensor_fifo *fifo = arg;

	K_SPINLOCK(&fifo->lock) {
		if (fifo->callback != NULL) {
			fifo->callback(fifo->arg);
		}
	}
}

/*
 * A FIFO watermark of at most 1 / rate_hz, for the drivers that set it from the batch duration,
 * which they round up to whole samples.
 * @return 0, or -ENOTSUP if the kernel tick is too long for the batch duration
 */
static int set_batch_duration(const struct sensor_entry *sensor, uint32_t rate_hz)
{
	const enum sensor_channel channel = (sensor->type == ZEPHYR_SENSOR_TYPE_ACCEL) ?
					    SENSOR_CHAN_ACCEL_XYZ : SENSOR_CHAN_GYRO_XYZ;
	struct sensor_value odr;

	if ((sensor_attr_get(sensor->dev, channel, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr) != 0) ||
	    (odr.val1 <= 0) || (rate_hz == 0)) {
		/* a watermark of the devicetree */
		return 0;
	}

	const uint64_t ticks_per_second = CONFIG_SYS_CLOCK_TICKS_PER_SEC;
	const uint32_t odr_hz = odr.val1;
	uint32_t samples = CLAMP(odr_hz / rate_hz, 1, ZEPHYR_SENSOR_FIFO_MAX_SAMPLES);

	while ((samples > 1) && ((samples * ticks_per_second) % odr_hz != 0)) {
		samples--;
	}

	const uint64_t duration = samples * ticks_per_second / odr_hz;
	const struct sensor_value ticks = {.val1 = (int32_t)duration};

	if (duration == 0) {
		return -ENOTSUP;
	}

	struct sensor_value current;

	if ((sensor_attr_get(sensor->dev, SENSOR_CHAN_ALL, SENSOR_ATTR_BATCH_DURATION, &current) != 0) ||
	    (current.val1 != ticks.val1)) {
		(void)sensor_attr_set(sensor->dev, SENSOR_CHAN_ALL, SENSOR_ATTR_BATCH_DURATION, &ticks);
	}

	return 0;
}

static void fifo_channel_init(struct fifo_channel *channel, const struct device *dev,
			      enum sensor_channel sensor_channel)
{
	struct sensor_value full_scale;

	*channel = (struct fifo_channel) {};

	if (sensor_attr_get(dev, sensor_channel, SENSOR_ATTR_FULL_SCALE, &full_scale) == 0) {
		channel->full_scale = sensor_value_to_float(&full_scale);
	}
}

/* the FIFO data that is read before a stream starts or fails is dropped */
static void fifo_drain(const struct sensor_entry *sensor)
{
	struct rtio *rtio = sensor->fifo_rtio;
	struct rtio_cqe *cqe;

	while ((cqe = rtio_cqe_consume(rtio)) != NULL) {
		const int result = cqe->result;
		uint8_t *buffer = NULL;
		uint32_t buffer_len = 0;

		if (result >= 0) {
			(void)rtio_cqe_get_mempool_buffer(rtio, cqe, &buffer, &buffer_len);
		}

		rtio_cqe_release(rtio, cqe);

		if (result < 0) {
			/* the buffer of a failed stream is released by RTIO */
			sensor->fifo->running = false;

		} else if (buffer != NULL) {
			rtio_release_buffer(rtio, buffer, buffer_len);
		}
	}
}

int zephyr_sensors_fifo_start(size_t index, uint32_t rate_hz, zephyr_sensor_fifo_callback_t callback,
			      void *arg)
{
	if ((index >= ARRAY_SIZE(sensors)) || (sensors[index].fifo == NULL)) {
		return -ENOTSUP;
	}

	const struct sensor_entry *sensor = &sensors[index];
	struct sensor_fifo *fifo = sensor->fifo;

	fifo_drain(sensor);

	if (!fifo->running) {
		int ret = set_batch_duration(sensor, rate_hz);

		if (ret != 0) {
			return ret;
		}

		fifo_channel_init(&fifo->accel, sensor->dev, SENSOR_CHAN_ACCEL_XYZ);
		fifo_channel_init(&fifo->gyro, sensor->dev, SENSOR_CHAN_GYRO_XYZ);
		(void)rtio_set_cqe_callback(sensor->fifo_rtio, fifo_cqe_callback, fifo);

		ret = sensor_stream(sensor->fifo_iodev, sensor->fifo_rtio, NULL, NULL);

		if (ret != 0) {
			return ret;
		}

		/* drivers without streaming fail when the stream is submitted */
		fifo->running = true;
		fifo_drain(sensor);

		if (!fifo->running) {
			return -ENOTSUP;
		}
	}

	K_SPINLOCK(&fifo->lock) {
		fifo->callback = callback;
		fifo->arg = arg;
	}

	return 0;
}

void zephyr_sensors_fifo_stop(size_t index)
{
	if ((index >= ARRAY_SIZE(sensors)) || (sensors[index].fifo == NULL)) {
		return;
	}

	struct sensor_fifo *fifo = sensors[index].fifo;

	K_SPINLOCK(&fifo->lock) {
		fifo->callback = NULL;
		fifo->arg = NULL;
	}
}

struct time_reference {
	uint64_t hrt_us;
	uint64_t sensor_ns;
};

static void time_reference_get(struct time_reference *reference)
{
	uint64_t cycles;

	reference->hrt_us = hrt_absolute_time();
	reference->sensor_ns = (sensor_clock_get_cycles(&cycles) == 0) ? sensor_clock_cycles_to_ns(cycles) : 0;
}

/* the sensor clock, which drivers timestamp the FIFO reads with, wraps if it has 32 bits */
static uint64_t time_reference_to_hrt(const struct time_reference *reference, uint64_t timestamp_ns)
{
	const uint64_t age_us = (reference->sensor_ns > timestamp_ns) ?
				(reference->sensor_ns - timestamp_ns) / NSEC_PER_USEC : 0;

	return (reference->hrt_us > age_us) ? (reference->hrt_us - age_us) : reference->hrt_us;
}

static int16_t to_count(float value)
{
	return (int16_t)CLAMP(lroundf(value), INT16_MIN, INT16_MAX);
}

static void decode_fifo_samples(struct sensor_fifo *fifo, const struct sensor_decoder_api *decoder,
				const uint8_t *buffer, enum sensor_channel sensor_channel,
				struct fifo_channel *channel, const struct time_reference *reference,
				struct zephyr_sensor_fifo_samples *samples, bool *overflow)
{
	const struct sensor_chan_spec spec = {sensor_channel, 0};
	struct sensor_three_axis_data *data = &fifo->decoded.three_axis;
	uint16_t frames = ZEPHYR_SENSOR_FIFO_MAX_SAMPLES;
	uint32_t fit = 0;

	/* some decoders read past the frames of the FIFO read */
	if ((decoder->get_frame_count(buffer, spec, &frames) == 0) && (frames > ZEPHYR_SENSOR_FIFO_MAX_SAMPLES)) {
		/* more samples than a FIFO message of PX4 */
		*overflow = true;
		frames = ZEPHYR_SENSOR_FIFO_MAX_SAMPLES;
	}

	const int count = (frames > 0) ? decoder->decode(buffer, spec, &fit, frames, data) : 0;

	samples->count = 0;

	if (count <= 0) {
		return;
	}

	if (!(channel->scale > 0.f) || (data->shift != channel->shift)) {
		float range = ldexpf(1.f, data->shift);

		if (channel->full_scale > 0.f) {
			/*
			 * the drivers take the shift from the full scale of the data, which is larger than
			 * the configured one for the high resolution FIFO data of some IMUs
			 */
			range = channel->full_scale;

			while (range <= ldexpf(1.f, data->shift - 1)) {
				range *= 2.f;
			}
		}

		channel->scale = range / 32768.f;
		channel->shift = data->shift;
	}

	const float to_counts = ldexpf(1.f, data->shift - 31) / channel->scale;

	for (int i = 0; i < count; i++) {
		samples->x[i] = to_count(data->readings[i].x * to_counts);
		samples->y[i] = to_count(data->readings[i].y * to_counts);
		samples->z[i] = to_count(data->readings[i].z * to_counts);
	}

	const uint64_t first_ns = data->header.base_timestamp_ns + data->readings[0].timestamp_delta;
	const uint64_t last_ns = data->header.base_timestamp_ns + data->readings[count - 1].timestamp_delta;

	if ((count > 1) && (last_ns > first_ns)) {
		channel->dt_us = (float)(last_ns - first_ns) / (float)(count - 1) / 1e3f;

	} else if ((channel->last_timestamp_ns != 0) && (last_ns > channel->last_timestamp_ns)) {
		channel->dt_us = (float)(last_ns - channel->last_timestamp_ns) / (float)count / 1e3f;
	}

	channel->last_timestamp_ns = last_ns;

	samples->timestamp_us = time_reference_to_hrt(reference, last_ns);
	samples->dt_us = channel->dt_us;
	samples->scale = channel->scale;

	/* the time between samples is known after the second sample */
	samples->count = (channel->dt_us > 0.f) ? count : 0;
}

static float decode_fifo_temperature(struct sensor_fifo *fifo, const struct sensor_decoder_api *decoder,
				     const uint8_t *buffer)
{
	const struct sensor_chan_spec spec = {SENSOR_CHAN_DIE_TEMP, 0};
	struct sensor_q31_data *data = &fifo->decoded.q31;
	uint16_t frames = ZEPHYR_SENSOR_FIFO_MAX_SAMPLES;
	uint32_t fit = 0;

	if (decoder->get_frame_count(buffer, spec, &frames) != 0) {
		return NAN;
	}

	const int count = decoder->decode(buffer, spec, &fit, MIN(frames, ZEPHYR_SENSOR_FIFO_MAX_SAMPLES), data);

	if (count <= 0) {
		return NAN;
	}

	float sum = 0.f;

	for (int i = 0; i < count; i++) {
		sum += q31_to_float(data->readings[i].temperature, data->shift);
	}

	return sum / (float)count;
}

int zephyr_sensors_fifo_read(size_t index, struct zephyr_sensor_fifo *samples)
{
	if ((index >= ARRAY_SIZE(sensors)) || (sensors[index].fifo == NULL)) {
		return -EINVAL;
	}

	const struct sensor_entry *sensor = &sensors[index];
	struct sensor_fifo *fifo = sensor->fifo;
	struct rtio_cqe *cqe = rtio_cqe_consume(sensor->fifo_rtio);

	if (cqe == NULL) {
		return 0;
	}

	const int result = cqe->result;
	uint8_t *buffer = NULL;
	uint32_t buffer_len = 0;

	if (result >= 0) {
		(void)rtio_cqe_get_mempool_buffer(sensor->fifo_rtio, cqe, &buffer, &buffer_len);
	}

	rtio_cqe_release(sensor->fifo_rtio, cqe);

	if (result < 0) {
		/* the stream stops when it fails, and its buffer is released by RTIO */
		fifo->running = false;
		return result;
	}

	const struct sensor_decoder_api *decoder;
	struct time_reference reference;

	time_reference_get(&reference);

	samples->accel.count = 0;
	samples->gyro.count = 0;
	samples->temperature = NAN;
	samples->overflow = false;

	if ((buffer != NULL) && (sensor_get_decoder(sensor->dev, &decoder) == 0)) {
		if ((decoder->has_trigger != NULL) && decoder->has_trigger(buffer, SENSOR_TRIG_FIFO_FULL)) {
			samples->overflow = true;
		}

		if (sensor->type != ZEPHYR_SENSOR_TYPE_GYRO) {
			decode_fifo_samples(fifo, decoder, buffer, SENSOR_CHAN_ACCEL_XYZ, &fifo->accel, &reference,
					    &samples->accel, &samples->overflow);
		}

		if (sensor->type != ZEPHYR_SENSOR_TYPE_ACCEL) {
			decode_fifo_samples(fifo, decoder, buffer, SENSOR_CHAN_GYRO_XYZ, &fifo->gyro, &reference,
					    &samples->gyro, &samples->overflow);
		}

		if (sensor->type == ZEPHYR_SENSOR_TYPE_IMU) {
			samples->temperature = decode_fifo_temperature(fifo, decoder, buffer);
		}
	}

	if (buffer != NULL) {
		rtio_release_buffer(sensor->fifo_rtio, buffer, buffer_len);
	}

	return 1;
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

int zephyr_sensors_fifo_start(size_t index, uint32_t rate_hz, zephyr_sensor_fifo_callback_t callback,
			      void *arg)
{
	return -ENOTSUP;
}

void zephyr_sensors_fifo_stop(size_t index)
{
}

int zephyr_sensors_fifo_read(size_t index, struct zephyr_sensor_fifo *samples)
{
	return -EINVAL;
}

#endif
