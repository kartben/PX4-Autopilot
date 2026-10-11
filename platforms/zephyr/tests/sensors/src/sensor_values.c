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

#include <math.h>

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/emul_fuel_gauge.h>
#include <zephyr/drivers/emul_sensor.h>

void fake_baro_set(const struct device *dev, float pressure_kpa, float temperature);
void fake_power_monitor_set(const struct device *dev, float voltage, float current);
void fake_power_monitor_set_lost(const struct device *dev, bool lost);

static int set_channel(const struct emul *emul, enum sensor_channel channel, float value)
{
	const struct sensor_chan_spec spec = {channel, 0};
	q31_t lower;
	q31_t upper;
	q31_t epsilon;
	int8_t shift;

	int ret = emul_sensor_backend_get_sample_range(emul, spec, &lower, &upper, &epsilon, &shift);

	if (ret != 0) {
		return ret;
	}

	const q31_t q31 = (q31_t)llroundf(ldexpf(value, 31 - shift));

	return emul_sensor_backend_set_channel(emul, spec, &q31, shift);
}

static int set_three_axis(const struct emul *emul, enum sensor_channel x, const float values[3])
{
	int ret = 0;

	/* the Y and Z channels follow X */
	for (int axis = 0; axis < 3 && ret == 0; axis++) {
		ret = set_channel(emul, x + axis, values[axis]);
	}

	return ret;
}

int sensor_values_set_imu(const float accel[3], const float gyro[3])
{
	const struct emul *emul = EMUL_DT_GET(DT_NODELABEL(icm42688));
	int ret = set_three_axis(emul, SENSOR_CHAN_ACCEL_X, accel);

	if (ret == 0) {
		ret = set_three_axis(emul, SENSOR_CHAN_GYRO_X, gyro);
	}

	return ret;
}

int sensor_values_set_mag(const float mag[3])
{
	return set_three_axis(EMUL_DT_GET(DT_NODELABEL(akm09918c)), SENSOR_CHAN_MAGN_X, mag);
}

void sensor_values_set_baro(float pressure_kpa, float temperature)
{
	fake_baro_set(DEVICE_DT_GET(DT_NODELABEL(baro)), pressure_kpa, temperature);
}

void sensor_values_set_power(float voltage, float current)
{
	fake_power_monitor_set(DEVICE_DT_GET(DT_NODELABEL(power_monitor)), voltage, current);
}

void sensor_values_set_power_lost(bool lost)
{
	fake_power_monitor_set_lost(DEVICE_DT_GET(DT_NODELABEL(power_monitor)), lost);
}

int sensor_values_set_fuel_gauge(float voltage, float current)
{
	return emul_fuel_gauge_set_battery_charging(EMUL_DT_GET(DT_NODELABEL(sbs_gauge)),
			(uint32_t)lroundf(voltage * 1e6f),
			(int)lroundf(current * 1e6f));
}
