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
 * @file zephyr_rc_input_backend.h
 *
 * RC channels of the px4,rc-input devicetree node, from the events of a Zephyr
 * input device.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ZEPHYR_RC_INPUT_MAX_CHANNELS 18

struct zephyr_rc_input_info {
	/** name of the input device */
	const char *device;
	uint8_t channel_count;
	/** the input device is an SBUS receiver */
	bool sbus;
	/** the input device is a CRSF receiver, with link statistics and telemetry */
	bool crsf;
};

struct zephyr_rc_input_link_stats {
	/** RSSI of the active antenna, in dBm */
	int16_t rssi_dbm;
	/** uplink link quality, in % */
	uint8_t link_quality;
	/** uplink signal-to-noise ratio, in dB */
	int8_t snr;
	/** RF mode of the link */
	uint8_t rf_mode;
};

struct zephyr_rc_input_frame {
	/** time of the sync event that ended the frame, see hrt_absolute_time() */
	uint64_t timestamp_us;
	/** frames received since boot */
	uint32_t count;
	/** pulse widths, in microseconds */
	uint16_t values[ZEPHYR_RC_INPUT_MAX_CHANNELS];
	bool failsafe;
};

typedef void (*zephyr_rc_input_event_t)(void *arg);

/**
 * Get the description of the RC input.
 * @return 0, -ENOENT if there is no px4,rc-input node, or -ENODEV if its input
 *         device is not ready
 */
int zephyr_rc_input_get_info(struct zephyr_rc_input_info *info);

/**
 * Call event with arg after each frame, from the thread that reports the input
 * events, or from interrupts with some input drivers. No event is called once
 * this returns with a NULL event.
 */
void zephyr_rc_input_set_event(zephyr_rc_input_event_t event, void *arg);

/**
 * Get the last frame.
 * @return 0, or -ENODATA if no frame was received
 */
int zephyr_rc_input_get_frame(struct zephyr_rc_input_frame *frame);

/**
 * Get the last link statistics of a CRSF receiver.
 *
 * @return 0, -ENOTSUP if the input device is not a CRSF receiver, or -ENODATA before its first
 *         link statistics
 */
int zephyr_rc_input_get_link_stats(struct zephyr_rc_input_link_stats *stats);

/**
 * Send a telemetry frame to a CRSF receiver, without waiting for its transmission.
 *
 * @param type CRSF frame type
 * @param payload big-endian payload of the frame
 * @return 0, -ENOTSUP if the input device is not a CRSF receiver, -EBUSY while the previous frame
 *         is sent, or another negative errno
 */
int zephyr_rc_input_send_telemetry(uint8_t type, const uint8_t *payload, size_t length);

#ifdef __cplusplus
}

#endif
