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

/*
 * Barometer that only implements the sensor fetch and get API, to test the
 * RTIO fallback of the Zephyr sensor API.
 */

#define DT_DRV_COMPAT vnd_baro

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

struct fake_baro_data {
	struct sensor_value pressure;
	struct sensor_value temperature;
};

void fake_baro_set(const struct device *dev, float pressure_kpa, float temperature)
{
	struct fake_baro_data *data = dev->data;

	sensor_value_from_float(&data->pressure, pressure_kpa);
	sensor_value_from_float(&data->temperature, temperature);
}

static int fake_baro_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	return 0;
}

static int fake_baro_channel_get(const struct device *dev, enum sensor_channel chan,
				 struct sensor_value *val)
{
	const struct fake_baro_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_PRESS:
		*val = data->pressure;
		return 0;

	case SENSOR_CHAN_AMBIENT_TEMP:
		*val = data->temperature;
		return 0;

	default:
		return -ENOTSUP;
	}
}

static DEVICE_API(sensor, fake_baro_api) = {
	.sample_fetch = fake_baro_sample_fetch,
	.channel_get = fake_baro_channel_get,
};

#define FAKE_BARO_DEFINE(inst)                                                                     \
	static struct fake_baro_data fake_baro_data_##inst;                                        \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, NULL, NULL, &fake_baro_data_##inst, NULL, POST_KERNEL,  \
				     CONFIG_SENSOR_INIT_PRIORITY, &fake_baro_api);

DT_INST_FOREACH_STATUS_OKAY(FAKE_BARO_DEFINE)
