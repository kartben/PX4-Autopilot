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
 * DShot device with the driver API of px4_dshot.h, which records the frames sent on its channels and gives
 * the replies set by the test to the frames of its bidirectional channels.
 */

#define DT_DRV_COMPAT vnd_dshot

#include "fake_dshot.h"

#include <errno.h>
#include <string.h>

#include <px4_dshot.h>

#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>

struct fake_dshot_channel {
	uint32_t replies[FAKE_DSHOT_MAX_REPLIES];
	size_t reply_count;
	size_t next_reply;
	bool busy;
	/* the reply to the last frame, until it is read */
	bool has_reply;
	uint32_t reply;
	uint32_t replied;
	uint32_t unreplied;
};

struct fake_dshot_data {
	struct fake_dshot_config config;
	struct fake_dshot_channel channels[FAKE_DSHOT_CHANNELS];
	struct fake_dshot_send sends[FAKE_DSHOT_HISTORY];
	uint32_t send_count;
};

static struct k_spinlock lock;

static struct fake_dshot_data *get_data(enum fake_dshot dshot);

static int fake_dshot_configure(const struct device *dev, uint32_t channels, uint32_t bidirectional_channels,
				uint32_t bit_rate)
{
	struct fake_dshot_data *data = dev->data;
	int ret = 0;

	K_SPINLOCK(&lock) {
		data->config.configure_calls++;

		if ((channels & ~BIT_MASK(FAKE_DSHOT_CHANNELS)) || (bidirectional_channels & ~channels)) {
			data->config.invalid_calls++;
			ret = -EINVAL;
			K_SPINLOCK_BREAK;
		}

		data->config.channels = channels;
		data->config.bidirectional_channels = bidirectional_channels;
		data->config.bit_rate = bit_rate;
	}

	return ret;
}

static int fake_dshot_send(const struct device *dev, uint32_t channels, const uint16_t *frames)
{
	struct fake_dshot_data *data = dev->data;
	int ret = 0;

	K_SPINLOCK(&lock) {
		if ((channels == 0) || (channels & ~data->config.channels)) {
			data->config.invalid_calls++;
			ret = -EINVAL;
			K_SPINLOCK_BREAK;
		}

		struct fake_dshot_send *send = &data->sends[data->send_count % FAKE_DSHOT_HISTORY];

		send->channels = channels;

		for (uint32_t channel = 0; channel < FAKE_DSHOT_CHANNELS; channel++) {
			struct fake_dshot_channel *ch = &data->channels[channel];

			if ((channels & BIT(channel)) == 0) {
				send->frames[channel] = 0;
				continue;
			}

			send->frames[channel] = frames[channel];

			if ((data->config.bidirectional_channels & BIT(channel)) == 0) {
				continue;
			}

			if (ch->busy) {
				data->config.invalid_calls++;
			}

			ch->has_reply = ch->reply_count > 0;

			if (ch->has_reply) {
				ch->reply = ch->replies[ch->next_reply];
				ch->next_reply = (ch->next_reply + 1) % ch->reply_count;
				ch->replied++;

			} else {
				ch->unreplied++;
			}
		}

		data->send_count++;
	}

	return ret;
}

static int fake_dshot_get_reply(const struct device *dev, uint32_t channel, uint32_t *reply)
{
	struct fake_dshot_data *data = dev->data;
	int ret = 0;

	K_SPINLOCK(&lock) {
		if ((channel >= FAKE_DSHOT_CHANNELS) || ((data->config.bidirectional_channels & BIT(channel)) == 0)) {
			data->config.invalid_calls++;
			ret = -EINVAL;
			K_SPINLOCK_BREAK;
		}

		struct fake_dshot_channel *ch = &data->channels[channel];

		if (ch->busy) {
			ret = -EBUSY;

		} else if (!ch->has_reply) {
			ret = -ENODATA;

		} else {
			*reply = ch->reply;
			ch->has_reply = false;
		}
	}

	return ret;
}

static const struct px4_dshot_driver_api fake_dshot_api = {
	.configure = fake_dshot_configure,
	.send = fake_dshot_send,
	.get_reply = fake_dshot_get_reply,
};

void fake_dshot_reset(enum fake_dshot dshot)
{
	struct fake_dshot_data *data = get_data(dshot);

	K_SPINLOCK(&lock) {
		memset(data, 0, sizeof(*data));
	}
}

void fake_dshot_get_config(enum fake_dshot dshot, struct fake_dshot_config *config)
{
	const struct fake_dshot_data *data = get_data(dshot);

	K_SPINLOCK(&lock) {
		*config = data->config;
	}
}

uint32_t fake_dshot_send_count(enum fake_dshot dshot)
{
	const struct fake_dshot_data *data = get_data(dshot);
	uint32_t count = 0;

	K_SPINLOCK(&lock) {
		count = data->send_count;
	}

	return count;
}

bool fake_dshot_get_send(enum fake_dshot dshot, uint32_t number, struct fake_dshot_send *send)
{
	const struct fake_dshot_data *data = get_data(dshot);
	bool found = false;

	K_SPINLOCK(&lock) {
		found = (number < data->send_count) && (data->send_count - number <= FAKE_DSHOT_HISTORY);

		if (found) {
			*send = data->sends[number % FAKE_DSHOT_HISTORY];
		}
	}

	return found;
}

void fake_dshot_set_replies(enum fake_dshot dshot, uint32_t channel, const uint32_t *replies, size_t count,
			    uint32_t *replied, uint32_t *unreplied)
{
	__ASSERT_NO_MSG(channel < FAKE_DSHOT_CHANNELS);
	__ASSERT_NO_MSG(count <= FAKE_DSHOT_MAX_REPLIES);

	struct fake_dshot_channel *ch = &get_data(dshot)->channels[channel];

	K_SPINLOCK(&lock) {
		if (replied != NULL) {
			*replied = ch->replied;
		}

		if (unreplied != NULL) {
			*unreplied = ch->unreplied;
		}

		if (count > 0) {
			memcpy(ch->replies, replies, count * sizeof(replies[0]));
		}

		ch->reply_count = count;
		ch->next_reply = 0;
		ch->replied = 0;
		ch->unreplied = 0;
	}
}

void fake_dshot_set_busy(enum fake_dshot dshot, uint32_t channel, bool busy)
{
	struct fake_dshot_data *data = get_data(dshot);

	__ASSERT_NO_MSG(channel < FAKE_DSHOT_CHANNELS);

	K_SPINLOCK(&lock) {
		data->channels[channel].busy = busy;
	}
}

#define FAKE_DSHOT_DEFINE(inst)                                                                                \
	static struct fake_dshot_data fake_dshot_data_##inst;                                                  \
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL, &fake_dshot_data_##inst, NULL, POST_KERNEL,                    \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &fake_dshot_api);

DT_INST_FOREACH_STATUS_OKAY(FAKE_DSHOT_DEFINE)

static struct fake_dshot_data *get_data(enum fake_dshot dshot)
{
	static const struct device *const devices[] = {
		[FAKE_DSHOT1] = DEVICE_DT_GET(DT_NODELABEL(fake_dshot1)),
		[FAKE_DSHOT2] = DEVICE_DT_GET(DT_NODELABEL(fake_dshot2)),
	};

	__ASSERT_NO_MSG(dshot < ARRAY_SIZE(devices));

	return devices[dshot]->data;
}
