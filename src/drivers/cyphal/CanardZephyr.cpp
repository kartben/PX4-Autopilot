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

#include "CanardZephyr.hpp"
#include "Cyphal.hpp"

#include <parameters/param.h>
#include <px4_platform_common/log.h>

#include <errno.h>
#include <string.h>

/* wakes the node up when a frame is received or sent, from interrupts */
static void can_event(void *arg)
{
	CyphalNode *node = CyphalNode::instance();

	if (node != nullptr) {
		node->ScheduleNow();
	}
}

int CanardZephyr::init()
{
	int32_t bitrate = 1000000;
	param_get(param_find("CYPHAL_BAUD"), &bitrate);

	const int ret = px4_can_open(Bus, bitrate, can_event, nullptr);

	if (ret != 0) {
		PX4_ERR("CAN%u: start failed (%d)", Bus + 1, ret);
		return -1;
	}

	return 0;
}

int CanardZephyr::close()
{
	return px4_can_close(Bus);
}

int16_t CanardZephyr::transmit(const CanardTxQueueItem &txframe, int timeout_ms)
{
	if (txframe.frame.payload_size > PX4_CAN_MAX_DLEN) {
		return -EINVAL;
	}

	px4_can_frame frame{};
	frame.id = txframe.frame.extended_can_id;
	frame.flags = PX4_CAN_FRAME_EXT;
	frame.len = txframe.frame.payload_size;
	memcpy(frame.data, txframe.frame.payload, txframe.frame.payload_size);

	return px4_can_send(Bus, &frame, false);
}

int16_t CanardZephyr::receive(CanardRxFrame *rxf)
{
	if (px4_can_receive(Bus, &_rx_frame) != 1) {
		return 0;
	}

	rxf->timestamp_usec = _rx_frame.timestamp_us;
	rxf->frame.extended_can_id = _rx_frame.frame.id;
	rxf->frame.payload_size = _rx_frame.frame.len;
	rxf->frame.payload = _rx_frame.frame.data;
	return 1;
}
