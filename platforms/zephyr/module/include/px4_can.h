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
 * @file px4_can.h
 *
 * CAN buses of the px4,can-buses devicetree node, numbered from 0 in the
 * order of its child nodes, for the PX4 CAN drivers.
 *
 * Frames are sent in order of priority: a frame is only accepted while it has
 * a higher priority than all the frames being sent, as the controller may send
 * the frames of its mailboxes in any order. Received frames are queued with
 * the time of their reception. The event callback is called from interrupts
 * when a frame is received or sent.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_CAN_FD_MODE)
#define PX4_CAN_MAX_DLEN 64
#else
#define PX4_CAN_MAX_DLEN 8
#endif

/** 29-bit identifier */
#define PX4_CAN_FRAME_EXT (1U << 0)
/** CAN FD frame */
#define PX4_CAN_FRAME_FD (1U << 1)
/** CAN FD frame with its data sent at the data bitrate */
#define PX4_CAN_FRAME_BRS (1U << 2)

struct px4_can_frame {
	uint32_t id;
	uint8_t flags;
	/** number of data bytes */
	uint8_t len;
	uint8_t data[PX4_CAN_MAX_DLEN];
};

struct px4_can_rx_frame {
	struct px4_can_frame frame;
	/** time of the reception, see hrt_absolute_time() */
	uint64_t timestamp_us;
	/** a frame sent by this node with the loopback argument of px4_can_send() */
	bool loopback;
};

enum px4_can_state {
	PX4_CAN_ERROR_ACTIVE,
	PX4_CAN_ERROR_WARNING,
	PX4_CAN_ERROR_PASSIVE,
	PX4_CAN_BUS_OFF,
	PX4_CAN_STOPPED,
};

struct px4_can_status {
	enum px4_can_state state;
	uint8_t tx_error_counter;
	uint8_t rx_error_counter;
	/** frames that couldn't be sent */
	uint32_t tx_errors;
	/** received frames dropped because the receive queue was full */
	uint32_t rx_overruns;
	/** times the controller became error passive or went bus off */
	uint32_t bus_errors;
};

typedef void (*px4_can_event_t)(void *arg);

/**
 * Return the number of CAN buses.
 */
unsigned px4_can_bus_count(void);

/**
 * Start a CAN bus at a bitrate, or restart it, dropping the frames being
 * sent and received. The controller keeps its mode, for example the loopback
 * mode set by a test. event is called with arg when a frame is received or
 * sent, from interrupts.
 *
 * @return 0, or a negative errno
 */
int px4_can_open(unsigned bus, uint32_t bitrate, px4_can_event_t event, void *arg);

/**
 * Return true if px4_can_send() would accept the frame now.
 */
bool px4_can_tx_ready(unsigned bus, const struct px4_can_frame *frame);

/**
 * Send a frame. With loopback, the frame is also received once it is sent,
 * flagged as such.
 *
 * @return 1 if the frame is being sent, 0 if it can't be accepted now, or a
 *         negative errno
 */
int px4_can_send(unsigned bus, const struct px4_can_frame *frame, bool loopback);

/**
 * Take the oldest received frame.
 *
 * @return 1 if a frame was received, 0 otherwise
 */
int px4_can_receive(unsigned bus, struct px4_can_rx_frame *frame);

/**
 * Return true if a received frame is waiting.
 */
bool px4_can_rx_pending(unsigned bus);

/**
 * Get the state and the error counters of a CAN bus.
 *
 * @return 0, or a negative errno
 */
int px4_can_get_status(unsigned bus, struct px4_can_status *status);

#ifdef __cplusplus
}
#endif
