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
 * @file dshot_protocol.h
 *
 * DShot frames, and the replies of the ESCs with bidirectional DShot, whatever hardware sends and
 * receives them.
 */

#pragma once

#include <drivers/drv_dshot.h>

#include <stdbool.h>
#include <stdint.h>

__BEGIN_DECLS

enum dshot_reply_status {
	DSHOT_REPLY_OK = 0,
	DSHOT_REPLY_FRAME_ERROR,
	DSHOT_REPLY_CRC_ERROR,
};

/**
 * Get the 16-bit frame of a DShot command (0-47) or throttle (48-2047): the 11-bit value, the telemetry
 * request bit and the 4-bit checksum, which is inverted for bidirectional DShot.
 */
uint16_t dshot_frame(uint16_t value, bool telemetry, bool bidirectional);

/**
 * Decode the GCR reply of an ESC with bidirectional DShot.
 *
 * @param reply   the 20 bits of the reply that follow its start bit, as line levels (1 for high), the first
 *                one in bit 19
 * @param payload set to the 12-bit payload of a valid reply
 * @return DSHOT_REPLY_OK, DSHOT_REPLY_FRAME_ERROR if the last bit is low, which it never is in a valid reply,
 *         or DSHOT_REPLY_CRC_ERROR
 */
enum dshot_reply_status dshot_decode_reply(uint32_t reply, uint16_t *payload);

/**
 * Decode the payload of a reply: the eRPM / 100, or with extended DShot telemetry enabled, the value of
 * one of the DSHOT_EDT_* types.
 */
void dshot_decode_telemetry(uint16_t payload, bool edt_enabled, struct BDShotTelemetry *telemetry);

__END_DECLS
