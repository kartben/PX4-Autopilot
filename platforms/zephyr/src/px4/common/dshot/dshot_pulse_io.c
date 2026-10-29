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
 * cells with a duty cycle. The frames of the channels of a controller that can start several channels together
 * (tx_sync_multi_channel) start together, and the others are sent one after the other.
 *
 * A bidirectional channel is a half duplex pulse_io channel (PULSE_IO_DIR_TX_RX) whose line idles high: its
 * frame is inverted, the line low for 3/4 of a 1 and 3/8 of a 0, and a receive chained to the transmit through
 * RTIO captures the reply of the ESC, at 5/4 of the bit rate, without waiting for it. Its 20 bits after the start
 * bit are the line levels at the middle of each bit.
 */

#define DT_DRV_COMPAT px4_dshot_pulse_io

#include <px4_dshot.h>

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pulse_io.h>
#include <zephyr/kernel.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/sys/util.h>

/* the ticks of a bit period, and the high times of a 1 and a 0, as sent and inverted */
#define DSHOT_BIT_TICKS          40U
#define DSHOT_ONE_TICKS          30U
#define DSHOT_ZERO_TICKS         15U
#define DSHOT_INVERTED_ONE_TICKS  (DSHOT_BIT_TICKS - DSHOT_ONE_TICKS)
#define DSHOT_INVERTED_ZERO_TICKS (DSHOT_BIT_TICKS - DSHOT_ZERO_TICKS)

#define DSHOT_FRAME_BITS 16U

/* the reply: 20 bits after its start bit, at 5/4 of the bit rate */
#define DSHOT_REPLY_BITS      20U
#define DSHOT_REPLY_BIT_TICKS (DSHOT_BIT_TICKS * 4U / 5U)

/*
 * The receive ends once the line has not changed for 60 us: the ESC replies about 30 us after the frame, and the
 * reply has at most 3 bits without an edge, 16 us at 150 kbit/s.
 */
#define DSHOT_REPLY_IDLE_US 60U

/* the edges of a reply and the line levels around them */
#define DSHOT_REPLY_SYMBOLS 32U

/* a frame takes 29 us at 600 kbit/s and 114 us at 150 kbit/s, which the timeout leaves room for */
#define DSHOT_SEND_TIMEOUT K_MSEC(2)

/* user data of the RTIO submissions: the channel, and whether it is the transmit */
#define DSHOT_PIO_TX_FLAG BIT(8)

enum dshot_pio_reply {
	/* no frame was sent, or its reply was returned */
	DSHOT_PIO_REPLY_NONE,
	/* the reply can still arrive */
	DSHOT_PIO_REPLY_PENDING,
	DSHOT_PIO_REPLY_DONE,
};

struct dshot_pio_channel_config {
	const struct device *dev;
	uint8_t channel;
	struct rtio_iodev *iodev;
};

struct dshot_pio_channel {
	/* NULL while the pulse_io channel isn't reserved */
	struct pulse_io_channel *chan;
	enum pulse_io_mode mode;
	/* the controller starts several channels together */
	bool multi;
	bool bidirectional;
	/* the frame being sent */
	union {
		/* the idle cell ends the last bit on every controller */
		struct pulse_cell cells[DSHOT_FRAME_BITS + 1];
		struct pulse_symbol symbols[2 * DSHOT_FRAME_BITS];
	};
	enum dshot_pio_reply reply;
	/* bytes of the reply, or the error of its receive */
	int reply_result;
	struct pulse_symbol reply_symbols[DSHOT_REPLY_SYMBOLS];
};

struct dshot_pio_config {
	const struct dshot_pio_channel_config *channels;
	uint8_t num_channels;
	struct rtio *rtio;
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

	pulse_io_iodev_set_channel(channel_cfg->iodev, NULL);
	ch->bidirectional = false;
	ch->reply = DSHOT_PIO_REPLY_NONE;
}

static int configure_channel(const struct dshot_pio_channel_config *channel_cfg, struct dshot_pio_channel *ch,
			     uint32_t bit_rate, bool bidirectional)
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

	if (bidirectional && (!caps.supports_rx || ((caps.rx_channel_mask & BIT(channel_cfg->channel)) == 0))) {
		return -EINVAL;
	}

	ch->multi = caps.tx_sync_multi_channel;
	ch->bidirectional = bidirectional;
	ch->reply = DSHOT_PIO_REPLY_NONE;

	const struct pulse_io_config pio_cfg = {
		.resolution_hz = bit_rate * DSHOT_BIT_TICKS,
		.cell_period_ticks = DSHOT_BIT_TICKS,
		.rx_idle_threshold_ticks = bidirectional ? bit_rate * DSHOT_BIT_TICKS / USEC_PER_SEC * DSHOT_REPLY_IDLE_US : 0,
		.mode = ch->mode,
		.dir = bidirectional ? PULSE_IO_DIR_TX_RX : PULSE_IO_DIR_TX,
		.idle_high = bidirectional,
	};

	ret = pulse_io_channel_configure(channel_cfg->dev, ch->chan, &pio_cfg);

	if (ret == 0) {
		pulse_io_iodev_set_channel(channel_cfg->iodev, bidirectional ? ch->chan : NULL);
	}

	return ret;
}

static int dshot_pio_configure(const struct device *dev, uint32_t channels, uint32_t bidirectional_channels,
			       uint32_t bit_rate)
{
	const struct dshot_pio_config *cfg = dev->config;
	struct dshot_pio_data *data = dev->data;
	int ret = 0;

	if (((channels & ~BIT_MASK(cfg->num_channels)) != 0) || ((bidirectional_channels & ~channels) != 0) ||
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

		ret = configure_channel(&cfg->channels[i], &data->channels[i], bit_rate,
					(bidirectional_channels & BIT(i)) != 0);

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

/* the request of a frame in the buffer of its channel, inverted on a bidirectional channel */
static struct pulse_io_tx_req encode_frame(struct dshot_pio_channel *ch, uint16_t frame)
{
	struct pulse_io_tx_req req = {0};
	const uint32_t one = ch->bidirectional ? DSHOT_INVERTED_ONE_TICKS : DSHOT_ONE_TICKS;
	const uint32_t zero = ch->bidirectional ? DSHOT_INVERTED_ZERO_TICKS : DSHOT_ZERO_TICKS;

	if (ch->mode == PULSE_IO_MODE_CELL) {
		/* a cell starts at the level opposite to the idle level */
		for (unsigned i = 0; i < DSHOT_FRAME_BITS; i++) {
			const bool bit = (frame & BIT(DSHOT_FRAME_BITS - 1 - i)) != 0;

			ch->cells[i] = (struct pulse_cell) {.duty = bit ? one : zero, .level = 1};
		}

		ch->cells[DSHOT_FRAME_BITS] = (struct pulse_cell) {
			.duty = ch->bidirectional ? DSHOT_BIT_TICKS : 0, .level = ch->bidirectional};
		req.cells = ch->cells;
		req.count = ARRAY_SIZE(ch->cells);

	} else {
		for (unsigned i = 0; i < DSHOT_FRAME_BITS; i++) {
			const uint32_t high = ((frame & BIT(DSHOT_FRAME_BITS - 1 - i)) != 0) ? one : zero;
			const struct pulse_symbol high_part = {.duration = high, .level = 1};
			const struct pulse_symbol low_part = {.duration = DSHOT_BIT_TICKS - high, .level = 0};

			ch->symbols[2 * i] = ch->bidirectional ? low_part : high_part;
			ch->symbols[2 * i + 1] = ch->bidirectional ? high_part : low_part;
		}

		req.symbols = ch->symbols;
		req.count = ARRAY_SIZE(ch->symbols);
	}

	return req;
}

/* the transmit and the receive of the reply of a bidirectional channel, chained */
static int submit_bidirectional(const struct dshot_pio_config *cfg, unsigned i, struct dshot_pio_channel *ch,
				uint16_t frame)
{
	if (rtio_sqe_acquirable(cfg->rtio) < 2) {
		return -ENOMEM;
	}

	const struct pulse_io_tx_req req = encode_frame(ch, frame);
	const size_t size = (ch->mode == PULSE_IO_MODE_CELL) ? sizeof(req.cells[0]) : sizeof(req.symbols[0]);
	struct rtio_sqe *tx = rtio_sqe_acquire(cfg->rtio);
	struct rtio_sqe *rx = rtio_sqe_acquire(cfg->rtio);

	rtio_sqe_prep_write(tx, cfg->channels[i].iodev, RTIO_PRIO_HIGH, (const uint8_t *)req.cells, req.count * size,
			    (void *)(uintptr_t)(i | DSHOT_PIO_TX_FLAG));
	tx->flags |= RTIO_SQE_CHAINED;
	rtio_sqe_prep_read(rx, cfg->channels[i].iodev, RTIO_PRIO_HIGH, (uint8_t *)ch->reply_symbols,
			   sizeof(ch->reply_symbols), (void *)(uintptr_t)i);
	ch->reply = DSHOT_PIO_REPLY_PENDING;

	return 0;
}

/* the receives that completed, of which only the receive gives the reply */
static void process_completions(const struct dshot_pio_config *cfg, struct dshot_pio_data *data)
{
	struct rtio_cqe *cqe;

	while ((cqe = rtio_cqe_consume(cfg->rtio)) != NULL) {
		const uintptr_t user = (uintptr_t)cqe->userdata;
		const unsigned i = user & BIT_MASK(8);

		/* a failed transmit cancels the receive chained to it, which completes too */
		if (((user & DSHOT_PIO_TX_FLAG) == 0) && (i < cfg->num_channels)) {
			data->channels[i].reply_result = cqe->result;
			data->channels[i].reply = DSHOT_PIO_REPLY_DONE;
		}

		rtio_cqe_release(cfg->rtio, cqe);
	}
}

static int dshot_pio_send(const struct device *dev, uint32_t channels, const uint16_t *frames)
{
	const struct dshot_pio_config *cfg = dev->config;
	struct dshot_pio_data *data = dev->data;
	int ret = 0;

	k_mutex_lock(&data->lock, K_FOREVER);

	if ((channels & ~data->configured) != 0) {
		ret = -EINVAL;
		goto unlock;
	}

	process_completions(cfg, data);

	uint32_t remaining = channels;
	bool submit = false;

	while (remaining != 0) {
		/* the first channel left, and the other channels of its controller if they start together */
		const unsigned first = u32_count_trailing_zeros(remaining);
		const struct device *pio = cfg->channels[first].dev;
		struct pulse_io_channel *chans[PULSE_IO_TX_MULTI_MAX];
		struct pulse_io_tx_req reqs[PULSE_IO_TX_MULTI_MAX];
		size_t num = 0;

		if (data->channels[first].bidirectional) {
			remaining &= ~BIT(first);

			if (data->channels[first].reply == DSHOT_PIO_REPLY_PENDING) {
				ret = (ret == 0) ? -EBUSY : ret;
				continue;
			}

			const int err = submit_bidirectional(cfg, first, &data->channels[first], frames[first]);

			if (err < 0) {
				ret = (ret == 0) ? err : ret;

			} else {
				submit = true;
			}

			continue;
		}

		for (unsigned i = first; (i < cfg->num_channels) && (num < ARRAY_SIZE(chans)); i++) {
			if (((remaining & BIT(i)) == 0) || data->channels[i].bidirectional ||
			    ((i != first) && (!data->channels[first].multi || (cfg->channels[i].dev != pio)))) {
				continue;
			}

			chans[num] = data->channels[i].chan;
			reqs[num] = encode_frame(&data->channels[i], frames[i]);
			num++;
			remaining &= ~BIT(i);
		}

		const int err = (num > 1) ? pulse_io_transmit_multi_sync(pio, chans, reqs, num, DSHOT_SEND_TIMEOUT)
				: pulse_io_transmit_sync(pio, chans[0], &reqs[0], DSHOT_SEND_TIMEOUT);

		if ((err < 0) && (ret == 0)) {
			ret = err;
		}
	}

	/* the bidirectional channels send without waiting for the replies */
	if (submit) {
		const int err = rtio_submit(cfg->rtio, 0);

		if ((err < 0) && (ret == 0)) {
			ret = err;
		}
	}

unlock:
	k_mutex_unlock(&data->lock);

	return ret;
}

/* the line level at a time of the reply, after its last edge the level that follows it */
static bool level_at(const struct pulse_symbol *symbols, size_t num, uint32_t time)
{
	uint32_t end = 0;

	for (size_t i = 0; i < num; i++) {
		end += symbols[i].duration;

		if (time < end) {
			return symbols[i].level;
		}
	}

	return symbols[num - 1].level;
}

static int decode_reply(const struct pulse_symbol *symbols, size_t num, uint32_t *reply)
{
	uint32_t start = 0;
	size_t i = 0;

	/* the start bit is the first low level */
	while ((i < num) && (symbols[i].level != 0)) {
		start += symbols[i].duration;
		i++;
	}

	if (i == num) {
		return -ENODATA;
	}

	*reply = 0;

	for (unsigned bit = 1; bit <= DSHOT_REPLY_BITS; bit++) {
		const uint32_t middle = start + bit * DSHOT_REPLY_BIT_TICKS + DSHOT_REPLY_BIT_TICKS / 2;

		*reply = (*reply << 1) | (level_at(symbols, num, middle) ? 1 : 0);
	}

	return 0;
}

static int dshot_pio_get_reply(const struct device *dev, uint32_t channel, uint32_t *reply)
{
	const struct dshot_pio_config *cfg = dev->config;
	struct dshot_pio_data *data = dev->data;
	int ret;

	if (channel >= cfg->num_channels) {
		return -ENODATA;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	process_completions(cfg, data);

	struct dshot_pio_channel *ch = &data->channels[channel];

	switch (ch->reply) {
	case DSHOT_PIO_REPLY_PENDING:
		ret = -EBUSY;
		break;

	case DSHOT_PIO_REPLY_DONE:
		ch->reply = DSHOT_PIO_REPLY_NONE;
		ret = (ch->reply_result > 0)
		      ? decode_reply(ch->reply_symbols, ch->reply_result / sizeof(ch->reply_symbols[0]), reply)
		      : -ENODATA;
		break;

	default:
		ret = -ENODATA;
		break;
	}

	k_mutex_unlock(&data->lock);

	return ret;
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

/* an RTIO iodev for each channel, named after the node and the index of the channel */
#define DSHOT_PIO_IODEV_NAME(node_id, idx) UTIL_CAT(dshot_pio_iodev_, UTIL_CAT(DT_DEP_ORD(node_id), UTIL_CAT(_, idx)))

#define DSHOT_PIO_IODEV(node_id, prop, idx) 	PULSE_IO_IODEV_DEFINE(DSHOT_PIO_IODEV_NAME(node_id, idx), DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx)));

#define DSHOT_PIO_CHANNEL(node_id, prop, idx) 	{.dev = DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx)), .channel = DT_PHA_BY_IDX(node_id, prop, idx, channel), 	 .iodev = &DSHOT_PIO_IODEV_NAME(node_id, idx)},

#define DSHOT_PIO_DEFINE(inst)                                                                                \
	BUILD_ASSERT(DT_INST_PROP_LEN(inst, pulse_ios) <= 32, "at most 32 DShot channels");                   \
	DT_INST_FOREACH_PROP_ELEM(inst, pulse_ios, DSHOT_PIO_IODEV)                                           \
	/* a transmit and a receive for each channel */                                                       \
	RTIO_DEFINE(dshot_pio_rtio_##inst, 2 * DT_INST_PROP_LEN(inst, pulse_ios), 2 * DT_INST_PROP_LEN(inst, pulse_ios)); \
	static const struct dshot_pio_channel_config dshot_pio_channels_##inst[] = {                          \
		DT_INST_FOREACH_PROP_ELEM(inst, pulse_ios, DSHOT_PIO_CHANNEL)};                               \
	static struct dshot_pio_channel dshot_pio_channel_data_##inst[DT_INST_PROP_LEN(inst, pulse_ios)];     \
	static const struct dshot_pio_config dshot_pio_config_##inst = {                                      \
		.channels = dshot_pio_channels_##inst,                                                        \
		.num_channels = DT_INST_PROP_LEN(inst, pulse_ios),                                            \
		.rtio = &dshot_pio_rtio_##inst,                                                               \
	};                                                                                                    \
	static struct dshot_pio_data dshot_pio_data_##inst = {                                                \
		.channels = dshot_pio_channel_data_##inst,                                                    \
	};                                                                                                    \
	DEVICE_DT_INST_DEFINE(inst, dshot_pio_init, NULL, &dshot_pio_data_##inst, &dshot_pio_config_##inst,   \
			      POST_KERNEL, CONFIG_PX4_DSHOT_PULSE_IO_INIT_PRIORITY, &dshot_pio_api);

DT_INST_FOREACH_STATUS_OKAY(DSHOT_PIO_DEFINE)
