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
 * Power monitor that only implements the sensor fetch and get API, as the
 * Zephyr INA2xx drivers.
 */

#define DT_DRV_COMPAT vnd_power_monitor

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

struct fake_power_monitor_data {
	struct sensor_value voltage;
	struct sensor_value current;
	bool lost;
};

void fake_power_monitor_set(const struct device *dev, float voltage, float current)
{
	struct fake_power_monitor_data *data = dev->data;

	sensor_value_from_float(&data->voltage, voltage);
	sensor_value_from_float(&data->current, current);
}

void fake_power_monitor_set_lost(const struct device *dev, bool lost)
{
	struct fake_power_monitor_data *data = dev->data;

	data->lost = lost;
}

static int fake_power_monitor_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct fake_power_monitor_data *data = dev->data;

	return data->lost ? -EIO : 0;
}

static int fake_power_monitor_channel_get(const struct device *dev, enum sensor_channel chan,
		struct sensor_value *val)
{
	const struct fake_power_monitor_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_VOLTAGE:
		*val = data->voltage;
		return 0;

	case SENSOR_CHAN_CURRENT:
		*val = data->current;
		return 0;

	default:
		return -ENOTSUP;
	}
}

static DEVICE_API(sensor, fake_power_monitor_api) = {
	.sample_fetch = fake_power_monitor_sample_fetch,
	.channel_get = fake_power_monitor_channel_get,
};

#define FAKE_POWER_MONITOR_DEFINE(inst)                                                            \
	static struct fake_power_monitor_data fake_power_monitor_data_##inst;                      \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, NULL, NULL, &fake_power_monitor_data_##inst, NULL,      \
				     POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,                     \
				     &fake_power_monitor_api);

DT_INST_FOREACH_STATUS_OKAY(FAKE_POWER_MONITOR_DEFINE)
