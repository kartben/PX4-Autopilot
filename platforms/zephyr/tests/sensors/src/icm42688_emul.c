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
 * ICM-42688-P emulator with a FIFO, instead of the Zephyr emulator, which has none, to test the
 * streaming of the FIFO by the icm4268x driver. The FIFO is filled at the output data rate of the
 * gyroscope with the values set by the test, as from the hardware. Its INT1 pin pulses each time
 * the FIFO watermark is reached, unless the test disables it.
 */

#define DT_DRV_COMPAT invensense_icm42688

#include <math.h>

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi_emul.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

/* registers of bank 0 */
#define REG_DEVICE_CONFIG     0x11
#define REG_FIFO_CONFIG       0x16
#define REG_TEMP_DATA1        0x1d
#define REG_ACCEL_DATA_X1     0x1f
#define REG_GYRO_DATA_X1      0x25
#define REG_INT_STATUS        0x2d
#define REG_FIFO_COUNTH       0x2e
#define REG_FIFO_COUNTL       0x2f
#define REG_FIFO_DATA         0x30
#define REG_SIGNAL_PATH_RESET 0x4b
#define REG_GYRO_CONFIG0      0x4f
#define REG_ACCEL_CONFIG0     0x50
#define REG_FIFO_CONFIG1      0x5f
#define REG_FIFO_CONFIG2      0x60
#define REG_FIFO_CONFIG3      0x61
#define REG_INT_SOURCE0       0x65
#define REG_WHO_AM_I          0x75
#define REG_BANK_SEL          0x76
#define REG_COUNT             0x80

#define READ_BIT          BIT(7)
#define WHO_AM_I_ICM42688 0x47

#define SOFT_RESET_CONFIG BIT(0)
#define FIFO_MODE_MASK    GENMASK(7, 6)
#define FIFO_MODE_STREAM  BIT(6)
#define FIFO_FULL_INT     BIT(1)
#define FIFO_THS_INT      BIT(2)
#define DATA_RDY_INT      BIT(3)
#define RESET_DONE_INT    BIT(4)
#define FIFO_FLUSH        BIT(1)
#define FIFO_HIRES_EN     BIT(4)
#define FIFO_THS_INT1_EN  BIT(2)
#define ODR_MASK          GENMASK(3, 0)
#define FS_SEL_MASK       GENMASK(7, 5)

/* FIFO packets of accelerometer, gyroscope, temperature and timestamp */
#define FIFO_SIZE          2048
#define PACKET_SIZE        16
#define PACKET_SIZE_HIRES  20
#define PACKET_HEADER      (BIT(6) | BIT(5) | BIT(3))
#define PACKET_HEADER_20   BIT(4)
#define PACKET_EMPTY       0x80

#define G                  9.80665f
#define RADIANS_PER_DEGREE 0.0174532925f
#define HIRES_FULL_SCALE   BIT(19)
#define TEMP_SENSITIVITY   132.48f
#define TEMP_SENSITIVITY_8 2.07f

struct icm42688_emul_config {
	struct gpio_dt_spec int_gpio;
};

struct icm42688_emul_data {
	const struct emul *target;
	struct k_spinlock lock;
	struct k_timer watermark_timer;
	uint8_t reg[REG_COUNT];
	uint8_t bank;
	float accel[3];		/* m/s^2 */
	float gyro[3];		/* rad/s */
	float temperature;	/* degrees Celsius */
	bool interrupt;
	int64_t fifo_start;	/* ticks */
	uint32_t fifo_packets;	/* read or flushed since the FIFO started */
	uint32_t fifo_offset;	/* bytes read of the next packet */
	uint8_t packet[PACKET_SIZE_HIRES];
};

static uint32_t odr_hz(uint8_t config0)
{
	static const uint32_t odr[16] = {
		[1] = 32000, [2] = 16000, [3] = 8000, [4] = 4000, [5] = 2000, [6] = 1000,
		[7] = 200, [8] = 100, [9] = 50, [10] = 25, [15] = 500,
	};

	return odr[FIELD_GET(ODR_MASK, config0)];
}

static float accel_full_scale(const struct icm42688_emul_data *data)
{
	return (float)(16 >> FIELD_GET(FS_SEL_MASK, data->reg[REG_ACCEL_CONFIG0])) * G;
}

static float gyro_full_scale(const struct icm42688_emul_data *data)
{
	return 2000.f / (float)BIT(FIELD_GET(FS_SEL_MASK, data->reg[REG_GYRO_CONFIG0])) * RADIANS_PER_DEGREE;
}

static int32_t to_raw(float value, float full_scale, int32_t counts)
{
	/* the lowest count marks invalid samples */
	return CLAMP(lroundf(value / full_scale * (float)counts), 1 - counts, counts - 1);
}

static bool fifo_hires(const struct icm42688_emul_data *data)
{
	return (data->reg[REG_FIFO_CONFIG1] & FIFO_HIRES_EN) != 0;
}

static uint32_t packet_size(const struct icm42688_emul_data *data)
{
	return fifo_hires(data) ? PACKET_SIZE_HIRES : PACKET_SIZE;
}

static uint32_t watermark_packets(const struct icm42688_emul_data *data)
{
	const uint32_t watermark = data->reg[REG_FIFO_CONFIG2] | ((data->reg[REG_FIFO_CONFIG3] & 0x0f) << 8);

	return DIV_ROUND_UP(watermark, packet_size(data));
}

static bool fifo_running(const struct icm42688_emul_data *data)
{
	return (data->reg[REG_FIFO_CONFIG] & FIFO_MODE_MASK) == FIFO_MODE_STREAM;
}

/* packets in the FIFO, of which the oldest are lost when it is full */
static uint32_t fifo_count(struct icm42688_emul_data *data)
{
	if (!fifo_running(data)) {
		return 0;
	}

	const uint64_t generated = (uint64_t)(k_uptime_ticks() - data->fifo_start) *
				   odr_hz(data->reg[REG_GYRO_CONFIG0]) / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
	const uint32_t capacity = FIFO_SIZE / packet_size(data);
	uint32_t count = (uint32_t)(generated - data->fifo_packets);

	if (count > capacity) {
		data->reg[REG_INT_STATUS] |= FIFO_FULL_INT;
		data->fifo_packets += count - capacity;
		data->fifo_offset = 0;
		count = capacity;
	}

	return count;
}

/* the FIFO restarts empty, and reaches its watermark every watermark samples */
static void fifo_restart(struct icm42688_emul_data *data)
{
	const uint32_t odr = odr_hz(data->reg[REG_GYRO_CONFIG0]);
	const uint32_t watermark = watermark_packets(data);

	data->fifo_start = k_uptime_ticks();
	data->fifo_packets = 0;
	data->fifo_offset = 0;

	if (fifo_running(data) && (odr > 0) && (watermark > 0)) {
		const k_timeout_t period = K_TICKS((uint64_t)watermark * CONFIG_SYS_CLOCK_TICKS_PER_SEC / odr);

		k_timer_start(&data->watermark_timer, period, period);

	} else {
		k_timer_stop(&data->watermark_timer);
	}
}

static void encode_packet(const struct icm42688_emul_data *data, uint8_t *packet)
{
	const float temperature = data->temperature - 25.f;

	if (fifo_hires(data)) {
		/* 20 bits, at the largest full scale */
		packet[0] = PACKET_HEADER | PACKET_HEADER_20;

		for (int axis = 0; axis < 3; axis++) {
			const int32_t accel = to_raw(data->accel[axis], 16.f * G, HIRES_FULL_SCALE);
			const int32_t gyro = to_raw(data->gyro[axis], 2000.f * RADIANS_PER_DEGREE, HIRES_FULL_SCALE);

			sys_put_be16((uint16_t)(accel >> 4), &packet[1 + 2 * axis]);
			sys_put_be16((uint16_t)(gyro >> 4), &packet[7 + 2 * axis]);
			packet[17 + axis] = ((accel & 0x0f) << 4) | (gyro & 0x0f);
		}

		sys_put_be16((uint16_t)lroundf(temperature * TEMP_SENSITIVITY), &packet[13]);
		sys_put_be16(0, &packet[15]);

	} else {
		packet[0] = PACKET_HEADER;

		for (int axis = 0; axis < 3; axis++) {
			sys_put_be16((uint16_t)to_raw(data->accel[axis], accel_full_scale(data), BIT(15)),
				     &packet[1 + 2 * axis]);
			sys_put_be16((uint16_t)to_raw(data->gyro[axis], gyro_full_scale(data), BIT(15)),
				     &packet[7 + 2 * axis]);
		}

		packet[13] = (uint8_t)lroundf(temperature * TEMP_SENSITIVITY_8);
		sys_put_be16(0, &packet[14]);
	}
}

static uint8_t fifo_read(struct icm42688_emul_data *data)
{
	if (data->fifo_offset == 0) {
		if (fifo_count(data) == 0) {
			return PACKET_EMPTY;
		}

		encode_packet(data, data->packet);
	}

	const uint8_t value = data->packet[data->fifo_offset++];

	if (data->fifo_offset == packet_size(data)) {
		data->fifo_offset = 0;
		data->fifo_packets++;
	}

	return value;
}

static uint8_t reg_read(struct icm42688_emul_data *data, uint8_t reg)
{
	uint8_t value[2];

	switch (reg) {
	case REG_TEMP_DATA1:
	case REG_TEMP_DATA1 + 1:
		sys_put_be16((uint16_t)lroundf((data->temperature - 25.f) * TEMP_SENSITIVITY), value);
		return value[reg - REG_TEMP_DATA1];

	case REG_ACCEL_DATA_X1 ... REG_ACCEL_DATA_X1 + 5:
		sys_put_be16((uint16_t)to_raw(data->accel[(reg - REG_ACCEL_DATA_X1) / 2], accel_full_scale(data),
					      BIT(15)), value);
		return value[(reg - REG_ACCEL_DATA_X1) % 2];

	case REG_GYRO_DATA_X1 ... REG_GYRO_DATA_X1 + 5:
		sys_put_be16((uint16_t)to_raw(data->gyro[(reg - REG_GYRO_DATA_X1) / 2], gyro_full_scale(data),
					      BIT(15)), value);
		return value[(reg - REG_GYRO_DATA_X1) % 2];

	case REG_INT_STATUS: {
			/* the interrupts are cleared when they are read */
			const uint8_t status = data->reg[REG_INT_STATUS] | DATA_RDY_INT;

			data->reg[REG_INT_STATUS] = 0;
			return status;
		}

	case REG_FIFO_COUNTH:
	case REG_FIFO_COUNTL:
		sys_put_be16((uint16_t)(fifo_count(data) * packet_size(data)), value);
		return value[reg - REG_FIFO_COUNTH];

	default:
		return (reg < REG_COUNT) ? data->reg[reg] : 0;
	}
}

static void reg_write(struct icm42688_emul_data *data, uint8_t reg, uint8_t value)
{
	if (reg == REG_BANK_SEL) {
		data->bank = value;
		return;
	}

	if ((data->bank != 0) || (reg >= REG_COUNT)) {
		return;
	}

	switch (reg) {
	case REG_DEVICE_CONFIG:
		if (value & SOFT_RESET_CONFIG) {
			memset(data->reg, 0, sizeof(data->reg));
			data->reg[REG_WHO_AM_I] = WHO_AM_I_ICM42688;
			data->reg[REG_INT_STATUS] = RESET_DONE_INT;
			fifo_restart(data);
		}

		break;

	case REG_SIGNAL_PATH_RESET:
		if (value & FIFO_FLUSH) {
			fifo_restart(data);
		}

		break;

	case REG_FIFO_CONFIG:
		data->reg[reg] = value;
		fifo_restart(data);
		break;

	default:
		data->reg[reg] = value;
		break;
	}
}

static void icm42688_emul_watermark(struct k_timer *timer)
{
	struct icm42688_emul_data *data = CONTAINER_OF(timer, struct icm42688_emul_data, watermark_timer);
	const struct icm42688_emul_config *config = data->target->cfg;
	bool pulse = false;

	K_SPINLOCK(&data->lock) {
		if (fifo_count(data) >= watermark_packets(data)) {
			data->reg[REG_INT_STATUS] |= FIFO_THS_INT;
			pulse = data->interrupt && (data->reg[REG_INT_SOURCE0] & FIFO_THS_INT1_EN);
		}
	}

	if (pulse) {
		gpio_emul_input_set(config->int_gpio.port, config->int_gpio.pin, 1);
		gpio_emul_input_set(config->int_gpio.port, config->int_gpio.pin, 0);
	}
}

void icm42688_emul_set(const struct emul *target, const float accel[3], const float gyro[3],
		       float temperature)
{
	struct icm42688_emul_data *data = target->data;

	K_SPINLOCK(&data->lock) {
		memcpy(data->accel, accel, sizeof(data->accel));
		memcpy(data->gyro, gyro, sizeof(data->gyro));
		data->temperature = temperature;
	}
}

void icm42688_emul_set_interrupt(const struct emul *target, bool enabled)
{
	struct icm42688_emul_data *data = target->data;

	K_SPINLOCK(&data->lock) {
		data->interrupt = enabled;
	}
}

static int icm42688_emul_io(const struct emul *target, const struct spi_config *config,
			    const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
	struct icm42688_emul_data *data = target->data;

	if ((tx_bufs == NULL) || (tx_bufs->count < 1) || (tx_bufs->buffers[0].len != 1)) {
		return -EINVAL;
	}

	const uint8_t address = *(const uint8_t *)tx_bufs->buffers[0].buf;
	const uint8_t reg = address & ~READ_BIT;

	/* the register address is followed by the data, as a second buffer */
	if (address & READ_BIT) {
		if ((rx_bufs == NULL) || (rx_bufs->count < 2)) {
			return -EINVAL;
		}

		uint8_t *rx = rx_bufs->buffers[1].buf;

		K_SPINLOCK(&data->lock) {
			for (size_t i = 0; i < rx_bufs->buffers[1].len; i++) {
				/* the FIFO is read from a single register */
				rx[i] = (reg == REG_FIFO_DATA) ? fifo_read(data) : reg_read(data, reg + i);
			}
		}

	} else {
		if (tx_bufs->count < 2) {
			return -EINVAL;
		}

		const uint8_t *tx = tx_bufs->buffers[1].buf;

		K_SPINLOCK(&data->lock) {
			for (size_t i = 0; i < tx_bufs->buffers[1].len; i++) {
				reg_write(data, reg + i, tx[i]);
			}
		}
	}

	return 0;
}

static const struct spi_emul_api icm42688_emul_api = {
	.io = icm42688_emul_io,
};

static int icm42688_emul_init(const struct emul *target, const struct device *parent)
{
	struct icm42688_emul_data *data = target->data;

	data->target = target;
	data->reg[REG_WHO_AM_I] = WHO_AM_I_ICM42688;
	data->interrupt = true;
	k_timer_init(&data->watermark_timer, icm42688_emul_watermark, NULL);

	return 0;
}

#define ICM42688_EMUL_DEFINE(n)                                                                    \
	static const struct icm42688_emul_config icm42688_emul_config_##n = {                      \
		.int_gpio = GPIO_DT_SPEC_INST_GET(n, int_gpios),                                   \
	};                                                                                         \
	static struct icm42688_emul_data icm42688_emul_data_##n;                                   \
	EMUL_DT_INST_DEFINE(n, icm42688_emul_init, &icm42688_emul_data_##n,                        \
			    &icm42688_emul_config_##n, &icm42688_emul_api, NULL)

DT_INST_FOREACH_STATUS_OKAY(ICM42688_EMUL_DEFINE)
