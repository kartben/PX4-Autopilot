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
 * @file dshot_pulse_io.c
 *
 * DShot device on pulse_io channels (px4,dshot-pulse-io), with the driver API of px4_dshot.h. A frame is 16 bits,
 * the most significant first, each a bit period whose high time is 3/4 of the period for a 1 and 3/8 for a 0,
 * then the line rests low. It is sent as 16 cells and an idle cell, or as 32 symbols on a controller without
 * cells with a duty cycle. The pulse_io API has no transmit that starts several channels at once, so the frames
 * of the channels are sent one after the other, and the replies of bidirectional DShot aren't received.
 */

#define DT_DRV_COMPAT px4_dshot_pulse_io

#include <px4_dshot.h>

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pulse_io.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/sys/util.h>

/* the ticks of a bit period, and the high times of a 1 and a 0 */
#define DSHOT_BIT_TICKS  40U
#define DSHOT_ONE_TICKS  30U
#define DSHOT_ZERO_TICKS 15U

#define DSHOT_FRAME_BITS 16U

/* a frame takes 29 us at 600 kbit/s and 114 us at 150 kbit/s, which the timeout leaves room for */
#define DSHOT_SEND_TIMEOUT K_MSEC(2)

struct dshot_pio_channel_config {
	const struct device *dev;
	uint8_t channel;
};

struct dshot_pio_channel {
	/* NULL while the pulse_io channel isn't reserved */
	struct pulse_io_channel *chan;
	enum pulse_io_mode mode;
};

struct dshot_pio_config {
	const struct dshot_pio_channel_config *channels;
	uint8_t num_channels;
};

struct dshot_pio_data {
	struct k_mutex lock;
	struct dshot_pio_channel *channels;
	uint32_t configured;
	uint32_t bit_rate;
};

static void release_channel(const struct dshot_pio_channel_config *channel_cfg, struct dshot_pio_channel *ch)
{
	if (ch->chan != NULL) {
		(void)pulse_io_channel_release(channel_cfg->dev, ch->chan);
		ch->chan = NULL;
	}
}

static int configure_channel(const struct dshot_pio_channel_config *channel_cfg, struct dshot_pio_channel *ch,
			     uint32_t bit_rate)
{
	struct pulse_io_caps caps;
	int ret;

	if (ch->chan == NULL) {
		ret = pulse_io_channel_get(channel_cfg->dev, channel_cfg->channel, &ch->chan);

		if (ret < 0) {
			ch->chan = NULL;
			return ret;
		}
	}

	ret = pulse_io_get_capabilities(channel_cfg->dev, &caps);

	if (ret < 0) {
		return ret;
	}

	/* cells need a duty cycle for the high time of the bits */
	if (((caps.modes & PULSE_IO_MODE_CELL) != 0) && (caps.cell_duty_bits > 0)) {
		ch->mode = PULSE_IO_MODE_CELL;

	} else if ((caps.modes & PULSE_IO_MODE_SYMBOL) != 0) {
		ch->mode = PULSE_IO_MODE_SYMBOL;

	} else {
		return -EINVAL;
	}

	const struct pulse_io_config pio_cfg = {
		.resolution_hz = bit_rate * DSHOT_BIT_TICKS,
		.cell_period_ticks = DSHOT_BIT_TICKS,
		.mode = ch->mode,
		.dir = PULSE_IO_DIR_TX,
		.idle_high = false,
	};

	return pulse_io_channel_configure(channel_cfg->dev, ch->chan, &pio_cfg);
}

static int dshot_pio_configure(const struct device *dev, uint32_t channels, uint32_t bidirectional_channels,
			       uint32_t bit_rate)
{
	const struct dshot_pio_config *cfg = dev->config;
	struct dshot_pio_data *data = dev->data;
	int ret = 0;

	if ((bidirectional_channels != 0) || ((channels & ~BIT_MASK(cfg->num_channels)) != 0) ||
	    ((bit_rate != 150000) && (bit_rate != 300000) && (bit_rate != 600000))) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	data->configured = 0;

	/* the channels of a controller may share their timing, as those of an STM32 timer */
	if (bit_rate != data->bit_rate) {
		for (uint8_t i = 0; i < cfg->num_channels; i++) {
			release_channel(&cfg->channels[i], &data->channels[i]);
		}

		data->bit_rate = bit_rate;
	}

	for (uint8_t i = 0; i < cfg->num_channels; i++) {
		if ((channels & BIT(i)) == 0) {
			release_channel(&cfg->channels[i], &data->channels[i]);
			continue;
		}

		ret = configure_channel(&cfg->channels[i], &data->channels[i], bit_rate);

		if (ret < 0) {
			break;
		}
	}

	if (ret < 0) {
		for (uint8_t i = 0; i < cfg->num_channels; i++) {
			release_channel(&cfg->channels[i], &data->channels[i]);
		}

		/* the API has a single error for what the device can't do */
		data->bit_rate = 0;
		ret = -EINVAL;

	} else {
		data->configured = channels;
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

static int send_frame(const struct dshot_pio_channel_config *channel_cfg, const struct dshot_pio_channel *ch,
		      uint16_t frame)
{
	struct pulse_io_tx_req req = {0};

	if (ch->mode == PULSE_IO_MODE_CELL) {
		/* the idle cell ends the high time of the last bit on every controller */
		struct pulse_cell cells[DSHOT_FRAME_BITS + 1] = {0};

		for (unsigned i = 0; i < DSHOT_FRAME_BITS; i++) {
			const bool one = (frame & BIT(DSHOT_FRAME_BITS - 1 - i)) != 0;

			cells[i].duty = one ? DSHOT_ONE_TICKS : DSHOT_ZERO_TICKS;
			cells[i].level = 1;
		}

		req.cells = cells;
		req.count = ARRAY_SIZE(cells);

		return pulse_io_transmit_sync(channel_cfg->dev, ch->chan, &req, DSHOT_SEND_TIMEOUT);
	}

	struct pulse_symbol symbols[2 * DSHOT_FRAME_BITS];

	for (unsigned i = 0; i < DSHOT_FRAME_BITS; i++) {
		const uint32_t high = ((frame & BIT(DSHOT_FRAME_BITS - 1 - i)) != 0) ? DSHOT_ONE_TICKS : DSHOT_ZERO_TICKS;

		symbols[2 * i] = (struct pulse_symbol) {.duration = high, .level = 1};
		symbols[2 * i + 1] = (struct pulse_symbol) {.duration = DSHOT_BIT_TICKS - high, .level = 0};
	}

	req.symbols = symbols;
	req.count = ARRAY_SIZE(symbols);

	return pulse_io_transmit_sync(channel_cfg->dev, ch->chan, &req, DSHOT_SEND_TIMEOUT);
}

static int dshot_pio_send(const struct device *dev, uint32_t channels, const uint16_t *frames)
{
	const struct dshot_pio_config *cfg = dev->config;
	struct dshot_pio_data *data = dev->data;
	int ret = 0;

	k_mutex_lock(&data->lock, K_FOREVER);

	if ((channels & ~data->configured) != 0) {
		ret = -EINVAL;

	} else {
		/* the channels one after the other, as the API can't start them together */
		uint32_t remaining = channels;

		while (remaining != 0) {
			const unsigned i = u32_count_trailing_zeros(remaining);
			const int err = send_frame(&cfg->channels[i], &data->channels[i], frames[i]);

			remaining &= ~BIT(i);

			if ((err < 0) && (ret == 0)) {
				ret = err;
			}
		}
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

static int dshot_pio_get_reply(const struct device *dev, uint32_t channel, uint32_t *reply)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel);
	ARG_UNUSED(reply);

	/* no channel is bidirectional */
	return -ENODATA;
}

static const struct px4_dshot_driver_api dshot_pio_api = {
	.configure = dshot_pio_configure,
	.send = dshot_pio_send,
	.get_reply = dshot_pio_get_reply,
};

static int dshot_pio_init(const struct device *dev)
{
	const struct dshot_pio_config *cfg = dev->config;
	struct dshot_pio_data *data = dev->data;

	k_mutex_init(&data->lock);

	for (uint8_t i = 0; i < cfg->num_channels; i++) {
		if (!device_is_ready(cfg->channels[i].dev)) {
			return -ENODEV;
		}
	}

	return 0;
}

#define DSHOT_PIO_CHANNEL(node_id, prop, idx) \
	{.dev = DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx)), .channel = DT_PHA_BY_IDX(node_id, prop, idx, channel)},

#define DSHOT_PIO_DEFINE(inst)                                                                                \
	BUILD_ASSERT(DT_INST_PROP_LEN(inst, pulse_ios) <= 32, "at most 32 DShot channels");                   \
	static const struct dshot_pio_channel_config dshot_pio_channels_##inst[] = {                          \
		DT_INST_FOREACH_PROP_ELEM(inst, pulse_ios, DSHOT_PIO_CHANNEL)};                               \
	static struct dshot_pio_channel dshot_pio_channel_data_##inst[DT_INST_PROP_LEN(inst, pulse_ios)];     \
	static const struct dshot_pio_config dshot_pio_config_##inst = {                                      \
		.channels = dshot_pio_channels_##inst,                                                        \
		.num_channels = DT_INST_PROP_LEN(inst, pulse_ios),                                            \
	};                                                                                                    \
	static struct dshot_pio_data dshot_pio_data_##inst = {                                                \
		.channels = dshot_pio_channel_data_##inst,                                                    \
	};                                                                                                    \
	DEVICE_DT_INST_DEFINE(inst, dshot_pio_init, NULL, &dshot_pio_data_##inst, &dshot_pio_config_##inst,   \
			      POST_KERNEL, CONFIG_PX4_DSHOT_PULSE_IO_INIT_PRIORITY, &dshot_pio_api);

DT_INST_FOREACH_STATUS_OKAY(DSHOT_PIO_DEFINE)
