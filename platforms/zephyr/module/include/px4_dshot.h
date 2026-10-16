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
 * @file px4_dshot.h
 *
 * Driver API of the DShot devices that the outputs of the px4,pwm-outputs devicetree node refer to with
 * their dshot property, on whose channels the platform layer of the PX4 dshot driver sends the frames of
 * the outputs, and gets the replies of the ESCs with bidirectional DShot.
 *
 * A device driver sends the bits of the frames at the bit rate, turns the line of a bidirectional channel
 * around for the reply of the ESC, and receives the reply at 5/4 of the bit rate, adapting the timing of
 * the reception to the ESC if needed. The frames and replies are those of lib/dshot/dshot_protocol.h.
 */

#pragma once

#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

__subsystem struct px4_dshot_driver_api {
	int (*configure)(const struct device *dev, uint32_t channels, uint32_t bidirectional_channels,
			 uint32_t bit_rate);
	int (*send)(const struct device *dev, uint32_t channels, const uint16_t *frames);
	int (*get_reply)(const struct device *dev, uint32_t channel, uint32_t *reply);
};

/**
 * Configure the channels of a DShot device, in place of its previous configuration, and leave the other
 * channels alone.
 *
 * @param dev                    DShot device
 * @param channels               mask of the channels to send frames on
 * @param bidirectional_channels mask of the channels, among channels, with bidirectional DShot: their frames
 *                               are inverted, and the ESC replies to each on the same line
 * @param bit_rate               bit rate of the frames: 150000, 300000 or 600000 bit/s
 *
 * @retval 0 on success
 * @retval -EINVAL if a channel or the bit rate is not supported
 */
static inline int px4_dshot_configure(const struct device *dev, uint32_t channels, uint32_t bidirectional_channels,
				      uint32_t bit_rate)
{
	const struct px4_dshot_driver_api *api = (const struct px4_dshot_driver_api *)dev->api;

	return api->configure(dev, channels, bidirectional_channels, bit_rate);
}

/**
 * Send a frame on configured channels, which all start at the same time.
 *
 * @param dev      DShot device
 * @param channels mask of the channels to send on
 * @param frames   frames of the channels, indexed by channel up to the highest one of channels
 *
 * @retval 0 on success
 * @retval -EINVAL if a channel is not configured
 * @retval -EBUSY if the reply to the last frame of a bidirectional channel can still arrive
 */
static inline int px4_dshot_send(const struct device *dev, uint32_t channels, const uint16_t *frames)
{
	const struct px4_dshot_driver_api *api = (const struct px4_dshot_driver_api *)dev->api;

	return api->send(dev, channels, frames);
}

/**
 * Get the reply of the ESC to the last frame sent on a bidirectional channel, which is returned once.
 *
 * @param dev     DShot device
 * @param channel bidirectional channel
 * @param reply   set to the 20 bits of the reply that follow its start bit, as line levels (1 for high), the
 *                first one in bit 19, see dshot_decode_reply()
 *
 * @retval 0 on success
 * @retval -EBUSY if the reply can still arrive: the channel can't send yet
 * @retval -ENODATA if no reply arrived, it was already returned, or no frame was sent
 */
static inline int px4_dshot_get_reply(const struct device *dev, uint32_t channel, uint32_t *reply)
{
	const struct px4_dshot_driver_api *api = (const struct px4_dshot_driver_api *)dev->api;

	return api->get_reply(dev, channel, reply);
}

#ifdef __cplusplus
}
#endif
