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
 * @file dshot_protocol.c
 *
 * DShot frames, and the replies of the ESCs with bidirectional DShot.
 */

#include "dshot_protocol.h"

#define DSHOT_VALUE_POSITION		5u
#define DSHOT_TELEMETRY_POSITION	4u
#define NIBBLES_SIZE 			4u
#define DSHOT_NUMBER_OF_NIBBLES		3u
#define DSHOT_REPLY_MASK		0xFFFFFu

static const uint32_t gcr_decode[32] = {
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x9, 0xA, 0xB, 0x0, 0xD, 0xE, 0xF,
	0x0, 0x0, 0x2, 0x3, 0x0, 0x5, 0x6, 0x7,
	0x0, 0x0, 0x8, 0x1, 0x0, 0x4, 0xC, 0x0
};

/**
* bits 	1-11	- throttle value (0-47 are reserved, 48-2047 give 2000 steps of throttle resolution)
* bit 	12		- dshot telemetry enable/disable
* bits 	13-16	- XOR checksum
**/
uint16_t dshot_frame(uint16_t value, bool telemetry, bool bidirectional)
{
	uint16_t packet = 0;
	uint16_t checksum = 0;

	packet |= value << DSHOT_VALUE_POSITION;
	packet |= ((uint16_t)telemetry & 0x01) << DSHOT_TELEMETRY_POSITION;

	uint16_t csum_data = bidirectional ? ~packet : packet;

	/* XOR checksum calculation */
	csum_data >>= NIBBLES_SIZE;

	for (unsigned i = 0; i < DSHOT_NUMBER_OF_NIBBLES; i++) {
		checksum ^= (csum_data & 0x0F); // XOR data by nibbles
		csum_data >>= NIBBLES_SIZE;
	}

	packet |= (checksum & 0x0F);

	return packet;
}

enum dshot_reply_status dshot_decode_reply(uint32_t reply, uint16_t *payload)
{
	uint32_t value = reply & DSHOT_REPLY_MASK;
	uint32_t data;
	uint32_t csum_data;

	/* if lowest significant bit isn't 1 we've got a framing error */
	if ((value & 0x1) == 0) {
		return DSHOT_REPLY_FRAME_ERROR;
	}

	/* Decode RLL */
	value = value ^ (value >> 1);

	/* Decode GCR */
	data = gcr_decode[value & 0x1fU];
	data |= gcr_decode[(value >> 5U) & 0x1fU] << 4U;
	data |= gcr_decode[(value >> 10U) & 0x1fU] << 8U;
	data |= gcr_decode[(value >> 15U) & 0x1fU] << 12U;

	/* Calculate checksum */
	csum_data = data;
	csum_data = csum_data ^ (csum_data >> 8U);
	csum_data = csum_data ^ (csum_data >> NIBBLES_SIZE);

	if ((csum_data & 0xFU) != 0xFU) {
		return DSHOT_REPLY_CRC_ERROR;
	}

	*payload = (data >> 4) & 0xFFF;
	return DSHOT_REPLY_OK;
}

void dshot_decode_telemetry(uint16_t payload, bool edt_enabled, struct BDShotTelemetry *telemetry)
{
	// MSB of the 9-bit mantissa clear means extended telemetry, not eRPM
	uint32_t mantissa = payload & 0x01FF;
	bool is_telemetry = !(mantissa & 0x0100);

	if (edt_enabled && is_telemetry) {
		telemetry->type = (payload & 0x0F00) >> 8;
		telemetry->value = payload & 0x00FF;

	} else {
		uint8_t exponent = ((payload >> 9) & 0x7); /* 3 bit: exponent */
		uint16_t period = (payload & 0x1FF); /* 9 bit: period base */
		period = period << exponent; /* Period in usec */

		telemetry->type = DSHOT_EDT_ERPM;

		if (period == 65408 || period == 0) {
			telemetry->value = 0;

		} else {
			telemetry->value = (1000000U * 60U / 100U + period / 2U) / period;
		}
	}
}
