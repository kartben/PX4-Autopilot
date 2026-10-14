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

#include "zephyr_rc_input_backend.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#ifdef CONFIG_PX4_RC_INPUT

#include <zephyr/input/input.h>

#define RC_INPUT_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(px4_rc_input)
#define INPUT_NODE DT_PHANDLE(RC_INPUT_NODE, input)
#define CHANNEL_COUNT (DT_PROP_LEN(RC_INPUT_NODE, channels) / 2)
#define INPUT_MIN DT_PROP(RC_INPUT_NODE, input_min)
#define INPUT_MAX DT_PROP(RC_INPUT_NODE, input_max)
#define HAS_FAILSAFE DT_NODE_HAS_PROP(RC_INPUT_NODE, failsafe)
#define IS_CRSF DT_NODE_HAS_COMPAT(INPUT_NODE, tbs_crsf)

#if IS_CRSF
#include <zephyr/input/input_crsf.h>
#endif

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(px4_rc_input) == 1, "only one px4,rc-input node is supported");
BUILD_ASSERT((DT_PROP_LEN(RC_INPUT_NODE, channels) % 2) == 0,
	     "px4,rc-input: each channel is the type and the code of an input event");
BUILD_ASSERT(CHANNEL_COUNT <= ZEPHYR_RC_INPUT_MAX_CHANNELS, "px4,rc-input: too many channels");
BUILD_ASSERT(INPUT_MAX > INPUT_MIN, "px4,rc-input: input-max must be greater than input-min");
BUILD_ASSERT(!HAS_FAILSAFE || (DT_PROP_LEN_OR(RC_INPUT_NODE, failsafe, 0) == 2),
	     "px4,rc-input: failsafe is the type and the code of an input event");

#define RC_MIN_US 1000
#define RC_MAX_US 2000

/* the scaling of the SBUS decoder of PX4, see src/lib/rc/sbus.cpp */
#define RC_SCALE ((float)(RC_MAX_US - RC_MIN_US) / (float)(INPUT_MAX - INPUT_MIN))
#define RC_OFFSET ((int32_t)(RC_MIN_US - (RC_SCALE * INPUT_MIN + 0.5f)))

/* the type and the code of the input event of each channel */
static const uint32_t channel_events[] = DT_PROP(RC_INPUT_NODE, channels);

static struct k_spinlock lock;

/* the last value of each channel, the state of the keys */
static int32_t values[CHANNEL_COUNT];
static uint32_t reported;
static bool failsafe;

/* the values of the last frame */
static int32_t frame_values[CHANNEL_COUNT];
static bool frame_failsafe;
static uint64_t frame_timestamp_us;
static uint32_t frame_count;

static zephyr_rc_input_event_t frame_event;
static void *frame_event_arg;

/* events called without the lock, which clearing the event waits for */
static atomic_t events_running;

static bool is_key(size_t channel)
{
	return channel_events[2 * channel] == INPUT_EV_KEY;
}

static bool is_failsafe(const struct input_event *evt)
{
	return COND_CODE_1(HAS_FAILSAFE,
			   ((evt->type == DT_PROP_BY_IDX(RC_INPUT_NODE, failsafe, 0)) &&
			    (evt->code == DT_PROP_BY_IDX(RC_INPUT_NODE, failsafe, 1))),
			   (false));
}

/* called with the lock held: keys are released until their first event */
static bool all_reported(void)
{
	for (size_t i = 0; i < CHANNEL_COUNT; i++) {
		if (!is_key(i) && ((reported & BIT(i)) == 0)) {
			return false;
		}
	}

	return true;
}

static void rc_input_callback(struct input_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);

	k_spinlock_key_t key = k_spin_lock(&lock);

	for (size_t i = 0; i < CHANNEL_COUNT; i++) {
		if ((evt->type == channel_events[2 * i]) && (evt->code == channel_events[2 * i + 1])) {
			values[i] = evt->value;
			reported |= BIT(i);
		}
	}

	if (is_failsafe(evt)) {
		failsafe = (evt->value != 0);
	}

	zephyr_rc_input_event_t event = NULL;
	void *event_arg = NULL;

	if (evt->sync && all_reported()) {
		memcpy(frame_values, values, sizeof(frame_values));
		frame_failsafe = failsafe;
		frame_timestamp_us = k_ticks_to_us_floor64(k_uptime_ticks());
		frame_count++;

		event = frame_event;
		event_arg = frame_event_arg;

		if (event != NULL) {
			atomic_inc(&events_running);
		}
	}

	k_spin_unlock(&lock, key);

	/* with interrupts enabled, as the event wakes up a thread */
	if (event != NULL) {
		event(event_arg);
		atomic_dec(&events_running);
	}
}

INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(INPUT_NODE), rc_input_callback, NULL);

static uint16_t pulse_width_us(size_t channel, int32_t value)
{
	if (is_key(channel)) {
		return (value != 0) ? RC_MAX_US : RC_MIN_US;
	}

	const long pulse_width = lroundf((float)value * RC_SCALE) + RC_OFFSET;

	return (uint16_t)CLAMP(pulse_width, 0, UINT16_MAX);
}

int zephyr_rc_input_get_info(struct zephyr_rc_input_info *info)
{
	const struct device *dev = DEVICE_DT_GET(INPUT_NODE);

	info->device = dev->name;
	info->channel_count = CHANNEL_COUNT;
	info->sbus = DT_NODE_HAS_COMPAT(INPUT_NODE, futaba_sbus);
	info->crsf = IS_CRSF;

	return device_is_ready(dev) ? 0 : -ENODEV;
}

void zephyr_rc_input_set_event(zephyr_rc_input_event_t event, void *arg)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	frame_event = event;
	frame_event_arg = arg;
	k_spin_unlock(&lock, key);

	/* no event is called once it is cleared: wait for one that is being called, in the thread of
	 * the input driver, which may have a lower priority
	 */
	if (event == NULL) {
		while (atomic_get(&events_running) != 0) {
			k_sleep(K_MSEC(1));
		}
	}
}

int zephyr_rc_input_get_frame(struct zephyr_rc_input_frame *frame)
{
	int32_t raw[CHANNEL_COUNT];

	k_spinlock_key_t key = k_spin_lock(&lock);
	memcpy(raw, frame_values, sizeof(raw));
	frame->failsafe = frame_failsafe;
	frame->timestamp_us = frame_timestamp_us;
	frame->count = frame_count;
	k_spin_unlock(&lock, key);

	if (frame->count == 0) {
		return -ENODATA;
	}

	for (size_t i = 0; i < CHANNEL_COUNT; i++) {
		frame->values[i] = pulse_width_us(i, raw[i]);
	}

	return 0;
}

int zephyr_rc_input_get_link_stats(struct zephyr_rc_input_link_stats *stats)
{
#if IS_CRSF
	const struct crsf_link_stats link_stats = input_crsf_get_link_stats(DEVICE_DT_GET(INPUT_NODE));

	/* the driver clears them until the receiver sends its first link statistics */
	if ((link_stats.uplink_rssi_1 == 0) && (link_stats.uplink_rssi_2 == 0)
	    && (link_stats.uplink_link_quality == 0)) {
		return -ENODATA;
	}

	stats->rssi_dbm = -(int16_t)((link_stats.active_antenna != 0) ? link_stats.uplink_rssi_2
				     : link_stats.uplink_rssi_1);
	stats->link_quality = link_stats.uplink_link_quality;
	stats->snr = link_stats.uplink_snr;
	stats->rf_mode = link_stats.rf_mode;

	return 0;
#else
	return -ENOTSUP;
#endif
}

int zephyr_rc_input_send_telemetry(uint8_t type, const uint8_t *payload, size_t length)
{
#if IS_CRSF
	/* the driver copies the payload */
	return input_crsf_send_telemetry(DEVICE_DT_GET(INPUT_NODE), type, (uint8_t *)payload, length);
#else
	return -ENOTSUP;
#endif
}

#else

int zephyr_rc_input_get_info(struct zephyr_rc_input_info *info)
{
	return -ENOENT;
}

void zephyr_rc_input_set_event(zephyr_rc_input_event_t event, void *arg)
{
}

int zephyr_rc_input_get_frame(struct zephyr_rc_input_frame *frame)
{
	return -ENODATA;
}

int zephyr_rc_input_get_link_stats(struct zephyr_rc_input_link_stats *stats)
{
	return -ENOTSUP;
}

int zephyr_rc_input_send_telemetry(uint8_t type, const uint8_t *payload, size_t length)
{
	return -ENOTSUP;
}

#endif
