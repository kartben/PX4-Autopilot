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

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* the fake DShot devices of app.overlay */
enum fake_dshot {
	FAKE_DSHOT1,
	FAKE_DSHOT2,
};

/* channels of a fake DShot device */
#define FAKE_DSHOT_CHANNELS 4

/* sends kept by a fake DShot device */
#define FAKE_DSHOT_HISTORY 512

/* replies of a channel, given in turn to its frames */
#define FAKE_DSHOT_MAX_REPLIES 8

struct fake_dshot_config {
	uint32_t channels;
	uint32_t bidirectional_channels;
	uint32_t bit_rate;
	/* calls of configure, and calls that broke the driver API */
	uint32_t configure_calls;
	uint32_t invalid_calls;
};

struct fake_dshot_send {
	uint32_t channels;
	uint16_t frames[FAKE_DSHOT_CHANNELS];
};

/* clear the configuration, the sends and the replies of a device */
void fake_dshot_reset(enum fake_dshot dshot);

void fake_dshot_get_config(enum fake_dshot dshot, struct fake_dshot_config *config);

/* sends since the reset */
uint32_t fake_dshot_send_count(enum fake_dshot dshot);

/* get a send by its number since the reset, from 0: false if it is not among the last FAKE_DSHOT_HISTORY */
bool fake_dshot_get_send(enum fake_dshot dshot, uint32_t number, struct fake_dshot_send *send);

/*
 * Set the replies to the next frames of a bidirectional channel, given in turn, or no replies with count 0.
 * Return the frames that got a reply and the frames that got none since the last call.
 */
void fake_dshot_set_replies(enum fake_dshot dshot, uint32_t channel, const uint32_t *replies, size_t count,
			    uint32_t *replied, uint32_t *unreplied);

/* while busy, the reply to the last frame of a channel can still arrive */
void fake_dshot_set_busy(enum fake_dshot dshot, uint32_t channel, bool busy);

#ifdef __cplusplus
}
#endif
