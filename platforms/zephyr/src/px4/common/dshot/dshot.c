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
 * @file dshot.c
 *
 * DShot outputs of the dshot driver, on the channels of the DShot devices of the dshot property of the
 * outputs of the px4,pwm-outputs devicetree node, see px4_dshot.h.
 */

#include <drivers/drv_dshot.h>
#include <lib/dshot/dshot_protocol.h>
#include <px4_dshot.h>
#include <px4_platform_common/log.h>

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/sys/util.h>

/* an ESC is online after this many valid replies in a row, and offline after as many missing or invalid ones */
#define BDSHOT_OFFLINE_COUNT 200

/* the channels of a device are bits of the masks of its API */
#define DSHOT_DEVICE_MAX_CHANNELS 32

struct dshot_output {
	/* NULL for an output without a dshot property */
	const struct device *dev;
	uint8_t channel;
};

#define DSHOT_DEVICE_CHANNEL(node) {.dev = DEVICE_DT_GET(DT_PHANDLE(node, dshot)), .channel = DT_PHA(node, dshot, channel)}

#define DSHOT_OUTPUT(node) \
	COND_CODE_1(DT_NODE_HAS_PROP(node, dshot), (DSHOT_DEVICE_CHANNEL(node)), ({.dev = NULL})),

#define DSHOT_CHANNEL_CHECK(node) \
	COND_CODE_1(DT_NODE_HAS_PROP(node, dshot), \
		    (BUILD_ASSERT(DT_PHA(node, dshot, channel) < DSHOT_DEVICE_MAX_CHANNELS, \
				  DT_NODE_PATH(node) ": the DShot channel is out of range");), ())

#if DT_HAS_COMPAT_STATUS_OKAY(px4_pwm_outputs)
#define PWM_OUTPUTS_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(px4_pwm_outputs)
DT_FOREACH_CHILD_STATUS_OKAY(PWM_OUTPUTS_NODE, DSHOT_CHANNEL_CHECK)
#define DSHOT_OUTPUTS DT_FOREACH_CHILD_STATUS_OKAY(PWM_OUTPUTS_NODE, DSHOT_OUTPUT)
#else
#define DSHOT_OUTPUTS
#endif

static const struct dshot_output outputs[DIRECT_PWM_OUTPUT_CHANNELS] = {DSHOT_OUTPUTS};

struct edt_sample {
	uint8_t value;
	bool ready;
};

struct dshot_channel {
	bool init;
	bool bidirectional;
	bool frame_set;
	/* a frame was sent, whose reply has not been processed */
	bool reply_pending;
	uint16_t frame;
	int32_t erpm;
	bool erpm_valid;
	struct edt_sample edt_temperature;
	struct edt_sample edt_voltage;
	struct edt_sample edt_current;
	struct edt_sample edt_state_event;
	uint32_t crc_errors;
	uint32_t frame_errors;
	uint32_t no_replies;
	uint16_t successes;
	uint16_t failures;
	bool online;
};

/* indexed by output, as the channels of drv_dshot.h */
static struct dshot_channel channels[DIRECT_PWM_OUTPUT_CHANNELS];
static uint32_t bdshot_ready_mask;
static bool edt_enabled;
static bool dshot_armed;

static void note_success(struct dshot_channel *ch)
{
	ch->failures = 0;

	if (ch->successes < BDSHOT_OFFLINE_COUNT) {
		ch->successes++;
	}

	if (ch->successes >= BDSHOT_OFFLINE_COUNT) {
		ch->online = true;
	}
}

static void note_failure(struct dshot_channel *ch)
{
	ch->successes = 0;

	if (ch->failures < BDSHOT_OFFLINE_COUNT) {
		ch->failures++;
	}

	if (ch->failures >= BDSHOT_OFFLINE_COUNT) {
		ch->online = false;
	}
}

static void set_edt_sample(struct edt_sample *sample, int32_t value)
{
	sample->value = value;
	sample->ready = true;
}

static void process_reply(unsigned output, int ret, uint32_t reply)
{
	struct dshot_channel *ch = &channels[output];

	/* DShot.cpp waits for all the bidirectional outputs, whose replies can be missing or invalid */
	bdshot_ready_mask |= BIT(output);

	if (ret != 0) {
		ch->no_replies++;
		note_failure(ch);
		return;
	}

	uint16_t payload = 0;
	const enum dshot_reply_status status = dshot_decode_reply(reply, &payload);

	if (status != DSHOT_REPLY_OK) {
		if (status == DSHOT_REPLY_FRAME_ERROR) {
			ch->frame_errors++;

		} else {
			ch->crc_errors++;
		}

		note_failure(ch);
		return;
	}

	struct BDShotTelemetry telemetry = {};

	dshot_decode_telemetry(payload, edt_enabled, &telemetry);

	switch (telemetry.type) {
	case DSHOT_EDT_ERPM:
		ch->erpm = telemetry.value;
		ch->erpm_valid = true;
		break;

	case DSHOT_EDT_TEMPERATURE:
		set_edt_sample(&ch->edt_temperature, telemetry.value);
		break;

	case DSHOT_EDT_VOLTAGE:
		set_edt_sample(&ch->edt_voltage, telemetry.value);
		break;

	case DSHOT_EDT_CURRENT:
		set_edt_sample(&ch->edt_current, telemetry.value);
		break;

	case DSHOT_EDT_STATE_EVENT:
		set_edt_sample(&ch->edt_state_event, telemetry.value);
		break;

	default:
		break;
	}

	note_success(ch);
}

int up_dshot_init(uint32_t channel_mask, uint32_t bdshot_channel_mask, unsigned dshot_pwm_freq, bool edt_enable)
{
	memset(channels, 0, sizeof(channels));
	bdshot_ready_mask = 0;
	edt_enabled = edt_enable;
	dshot_armed = false;

	uint32_t output_mask = 0;

	for (unsigned i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; i++) {
		if (((channel_mask | bdshot_channel_mask) & BIT(i)) == 0) {
			continue;
		}

		if (outputs[i].dev == NULL) {
			PX4_WARN("output %u: no DShot device", i + 1);
			/* no replies, which must not hold the replies of the other outputs */
			bdshot_ready_mask |= BIT(i) & bdshot_channel_mask;
			continue;
		}

		output_mask |= BIT(i);
	}

	/* the channels of each device, configured together */
	uint32_t unconfigured = output_mask;

	while (unconfigured != 0) {
		const struct device *dev = outputs[u32_count_trailing_zeros(unconfigured)].dev;
		uint32_t device_channels = 0;
		uint32_t device_bidirectional_channels = 0;
		uint32_t device_outputs = 0;

		for (unsigned i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; i++) {
			if (((unconfigured & BIT(i)) == 0) || (outputs[i].dev != dev)) {
				continue;
			}

			const uint32_t channel = BIT(outputs[i].channel);

			unconfigured &= ~BIT(i);

			if (device_channels & channel) {
				PX4_ERR("output %u: channel %u of %s already used", i + 1, outputs[i].channel, dev->name);
				output_mask &= ~BIT(i);
				bdshot_ready_mask |= BIT(i) & bdshot_channel_mask;
				continue;
			}

			device_channels |= channel;
			device_outputs |= BIT(i);

			if (bdshot_channel_mask & BIT(i)) {
				device_bidirectional_channels |= channel;
			}
		}

		const int ret = device_is_ready(dev) ? px4_dshot_configure(dev, device_channels,
				device_bidirectional_channels, dshot_pwm_freq) : -ENODEV;

		if (ret != 0) {
			PX4_ERR("%s: configuration failed (%d)", dev->name, ret);
			output_mask &= ~device_outputs;
			bdshot_ready_mask |= device_outputs & bdshot_channel_mask;
		}
	}

	for (unsigned i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; i++) {
		if (output_mask & BIT(i)) {
			channels[i].init = true;
			channels[i].bidirectional = (bdshot_channel_mask & BIT(i)) != 0;
		}
	}

	return output_mask;
}

void dshot_motor_data_set(uint8_t channel, uint16_t throttle, bool telemetry)
{
	if (channel >= DIRECT_PWM_OUTPUT_CHANNELS || !channels[channel].init) {
		return;
	}

	channels[channel].frame = dshot_frame(throttle, telemetry, channels[channel].bidirectional);
	channels[channel].frame_set = true;
}

void up_dshot_trigger(void)
{
	if (!dshot_armed) {
		return;
	}

	uint32_t send_mask = 0;

	for (unsigned i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; i++) {
		struct dshot_channel *ch = &channels[i];

		if (!ch->init || !ch->frame_set) {
			continue;
		}

		if (ch->reply_pending) {
			uint32_t reply = 0;
			const int ret = px4_dshot_get_reply(outputs[i].dev, outputs[i].channel, &reply);

			/* the ESC can still be replying on the line */
			if (ret == -EBUSY) {
				continue;
			}

			process_reply(i, ret, reply);
			ch->reply_pending = false;
		}

		send_mask |= BIT(i);
	}

	/* the channels of each device start together */
	while (send_mask != 0) {
		const struct device *dev = outputs[u32_count_trailing_zeros(send_mask)].dev;
		uint16_t frames[DSHOT_DEVICE_MAX_CHANNELS] = {};
		uint32_t device_channels = 0;
		uint32_t device_outputs = 0;

		for (unsigned i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; i++) {
			if ((send_mask & BIT(i)) && (outputs[i].dev == dev)) {
				frames[outputs[i].channel] = channels[i].frame;
				device_channels |= BIT(outputs[i].channel);
				device_outputs |= BIT(i);
			}
		}

		send_mask &= ~device_outputs;

		const int ret = px4_dshot_send(dev, device_channels, frames);

		for (unsigned i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; i++) {
			if ((device_outputs & BIT(i)) && channels[i].bidirectional) {
				if (ret == 0) {
					channels[i].reply_pending = true;

				} else {
					/* a frame that was not sent gets no reply */
					process_reply(i, ret, 0);
				}
			}
		}
	}
}

int up_dshot_arm(bool armed)
{
	dshot_armed = armed;

	/* no reply is expected to the first frame after arming */
	for (unsigned i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; i++) {
		channels[i].reply_pending = false;
	}

	return 0;
}

uint16_t up_bdshot_get_ready_mask(void)
{
	return (uint16_t)bdshot_ready_mask;
}

int up_bdshot_num_errors(uint8_t channel)
{
	if (channel >= DIRECT_PWM_OUTPUT_CHANNELS) {
		return 0;
	}

	return channels[channel].crc_errors + channels[channel].frame_errors + channels[channel].no_replies;
}

int up_bdshot_get_erpm(uint8_t channel, int *erpm)
{
	/* the outputs without replies keep their ready bit */
	if (channel >= DIRECT_PWM_OUTPUT_CHANNELS || !channels[channel].bidirectional) {
		return PX4_ERROR;
	}

	int status = PX4_ERROR;

	if (channels[channel].erpm_valid) {
		*erpm = channels[channel].erpm;
		status = PX4_OK;
	}

	bdshot_ready_mask &= ~BIT(channel);
	return status;
}

int up_bdshot_get_extended_telemetry(uint8_t channel, int type, uint8_t *value)
{
	if (channel >= DIRECT_PWM_OUTPUT_CHANNELS) {
		return PX4_ERROR;
	}

	struct edt_sample *sample = NULL;

	switch (type) {
	case DSHOT_EDT_TEMPERATURE:
		sample = &channels[channel].edt_temperature;
		break;

	case DSHOT_EDT_VOLTAGE:
		sample = &channels[channel].edt_voltage;
		break;

	case DSHOT_EDT_CURRENT:
		sample = &channels[channel].edt_current;
		break;

	case DSHOT_EDT_STATE_EVENT:
		sample = &channels[channel].edt_state_event;
		break;

	default:
		return PX4_ERROR;
	}

	if (!sample->ready) {
		return PX4_ERROR;
	}

	*value = sample->value;
	sample->ready = false;
	return PX4_OK;
}

int up_bdshot_channel_online(uint8_t channel)
{
	if (channel >= DIRECT_PWM_OUTPUT_CHANNELS) {
		return 0;
	}

	return channels[channel].online;
}

int up_bdshot_channel_capture_supported(uint8_t channel)
{
	if (channel >= DIRECT_PWM_OUTPUT_CHANNELS) {
		return 0;
	}

	return channels[channel].init && channels[channel].bidirectional;
}

void up_bdshot_status(void)
{
	for (unsigned i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; i++) {
		const struct dshot_channel *ch = &channels[i];

		if (!ch->init || !ch->bidirectional) {
			continue;
		}

		PX4_INFO("Output %u (%s channel %u): %s, eRPM %ld", i + 1, outputs[i].dev->name, outputs[i].channel,
			 ch->online ? "online" : "offline", (long)ch->erpm * 100);

		if (edt_enabled) {
			PX4_INFO("EDT temperature %u C, voltage %.2f V, current %u A", ch->edt_temperature.value,
				 (double)ch->edt_voltage.value * 0.25, ch->edt_current.value);
		}

		PX4_INFO("CRC errors %lu, frame errors %lu, no replies %lu", (unsigned long)ch->crc_errors,
			 (unsigned long)ch->frame_errors, (unsigned long)ch->no_replies);
	}
}
