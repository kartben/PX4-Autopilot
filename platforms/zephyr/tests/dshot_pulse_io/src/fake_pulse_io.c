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
 * pulse_io controller of the test, with the capabilities of its devicetree node, which records the
 * configuration of its channels and the last frame each sent
 */

#define DT_DRV_COMPAT vnd_pulse_io

#include "fake_pulse_io.h"

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

struct fake_pio_config {
	uint8_t num_channels;
	uint32_t modes;
	uint8_t cell_duty_bits;
};

struct fake_pio_data {
	struct fake_pio_channel channels[FAKE_PIO_MAX_CHANNELS];
};

static uint32_t sequence;

static struct fake_pio_channel *get_channel(const struct device *dev, struct pulse_io_channel *chan)
{
	const struct fake_pio_config *cfg = dev->config;
	struct fake_pio_data *data = dev->data;
	const uintptr_t idx = (uintptr_t)chan - 1U;

	if ((idx >= cfg->num_channels) || !data->channels[idx].reserved) {
		return NULL;
	}

	return &data->channels[idx];
}

static int fake_pio_get_capabilities(const struct device *dev, struct pulse_io_caps *caps)
{
	const struct fake_pio_config *cfg = dev->config;

	*caps = (struct pulse_io_caps) {
		.tx_max_streaming = FAKE_PIO_MAX_ENTRIES,
		.modes = cfg->modes,
		.min_tick_ns = 1,
		.max_tick_ns = 1000000,
		.max_duration_ticks = UINT16_MAX,
		.tx_channel_mask = BIT_MASK(cfg->num_channels),
		.cell_duty_bits = cfg->cell_duty_bits,
		.num_channels = cfg->num_channels,
		.supports_tx = true,
	};

	return 0;
}

static int fake_pio_channel_get(const struct device *dev, uint8_t channel_idx, struct pulse_io_channel **chan)
{
	const struct fake_pio_config *cfg = dev->config;
	struct fake_pio_data *data = dev->data;

	if (channel_idx >= cfg->num_channels) {
		return -ENODEV;
	}

	if (data->channels[channel_idx].reserved) {
		return -EBUSY;
	}

	data->channels[channel_idx].reserved = true;
	*chan = (struct pulse_io_channel *)(uintptr_t)(channel_idx + 1U);

	return 0;
}

static int fake_pio_channel_release(const struct device *dev, struct pulse_io_channel *chan)
{
	struct fake_pio_channel *ch = get_channel(dev, chan);

	if (ch == NULL) {
		return -EINVAL;
	}

	ch->reserved = false;
	ch->configured = false;

	return 0;
}

static int fake_pio_channel_configure(const struct device *dev, struct pulse_io_channel *chan,
				      const struct pulse_io_config *pio_cfg)
{
	const struct fake_pio_config *cfg = dev->config;
	struct fake_pio_channel *ch = get_channel(dev, chan);

	if (ch == NULL) {
		return -EINVAL;
	}

	if (((pio_cfg->mode & cfg->modes) == 0) || (pio_cfg->dir != PULSE_IO_DIR_TX)) {
		return -ENOTSUP;
	}

	ch->cfg = *pio_cfg;
	ch->configured = true;

	return 0;
}

static int fake_pio_transmit_sync(const struct device *dev, struct pulse_io_channel *chan,
				  const struct pulse_io_tx_req *req, k_timeout_t timeout)
{
	struct fake_pio_channel *ch = get_channel(dev, chan);

	ARG_UNUSED(timeout);

	if ((ch == NULL) || !ch->configured) {
		return -EINVAL;
	}

	if (req->count > FAKE_PIO_MAX_ENTRIES) {
		return -ENOMEM;
	}

	ch->sequence = ++sequence;

	if (ch->transmit_error != 0) {
		return ch->transmit_error;
	}

	if (ch->cfg.mode == PULSE_IO_MODE_CELL) {
		memcpy(ch->cells, req->cells, req->count * sizeof(req->cells[0]));

	} else {
		memcpy(ch->symbols, req->symbols, req->count * sizeof(req->symbols[0]));
	}

	ch->count = req->count;
	ch->transmits++;

	return 0;
}

static int fake_pio_receive_sync(const struct device *dev, struct pulse_io_channel *chan,
				 const struct pulse_io_rx_req *req, size_t *count, k_timeout_t timeout)
{
	return -ENOTSUP;
}

static int fake_pio_stop(const struct device *dev, struct pulse_io_channel *chan)
{
	return 0;
}

static DEVICE_API(pulse_io, fake_pio_api) = {
	.get_capabilities = fake_pio_get_capabilities,
	.channel_get = fake_pio_channel_get,
	.channel_release = fake_pio_channel_release,
	.channel_configure = fake_pio_channel_configure,
	.transmit_sync = fake_pio_transmit_sync,
	.receive_sync = fake_pio_receive_sync,
	.stop = fake_pio_stop,
};

#define FAKE_PIO_DEFINE(inst)                                                                                  \
	BUILD_ASSERT(DT_INST_PROP(inst, num_channels) <= FAKE_PIO_MAX_CHANNELS);                              \
	static const struct fake_pio_config fake_pio_config_##inst = {                                         \
		.num_channels = DT_INST_PROP(inst, num_channels),                                              \
		.modes = DT_INST_PROP(inst, modes),                                                            \
		.cell_duty_bits = DT_INST_PROP(inst, cell_duty_bits),                                          \
	};                                                                                                     \
	static struct fake_pio_data fake_pio_data_##inst;                                                      \
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL, &fake_pio_data_##inst, &fake_pio_config_##inst, POST_KERNEL,   \
			      CONFIG_PULSE_IO_INIT_PRIORITY, &fake_pio_api);

DT_INST_FOREACH_STATUS_OKAY(FAKE_PIO_DEFINE)

#define FAKE_PIO_DATA(inst) &fake_pio_data_##inst,

static struct fake_pio_data *const all_data[] = {DT_INST_FOREACH_STATUS_OKAY(FAKE_PIO_DATA)};

void fake_pio_clear(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(all_data); i++) {
		for (size_t c = 0; c < FAKE_PIO_MAX_CHANNELS; c++) {
			struct fake_pio_channel *ch = &all_data[i]->channels[c];

			ch->count = 0;
			ch->transmits = 0;
			ch->sequence = 0;
			ch->transmit_error = 0;
		}
	}
}

struct fake_pio_channel *fake_pio_channel(const struct device *dev, uint8_t channel)
{
	struct fake_pio_data *data = dev->data;

	__ASSERT_NO_MSG(channel < FAKE_PIO_MAX_CHANNELS);

	return &data->channels[channel];
}
