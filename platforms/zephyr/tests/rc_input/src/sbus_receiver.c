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

#include "sbus_receiver.h"

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/serial/uart_emul.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#define SBUS_FRAME_SIZE 25
#define SBUS_HEADER 0x0f
#define SBUS_CHANNEL_BITS 11
#define SBUS_FRAME_INTERVAL_MS 14

static const struct device *const uart = DEVICE_DT_GET(DT_NODELABEL(euart0));
static const struct device *const sbus = DEVICE_DT_GET(DT_NODELABEL(sbus));
static const struct device *const other_sbus = DEVICE_DT_GET(DT_NODELABEL(other_sbus));

static struct k_spinlock lock;
static uint16_t channel_values[SBUS_CHANNELS];

/* the channels are packed from the least significant bit of the second byte, the flags and the
 * footer are 0
 */
static void send_frame(struct k_timer *timer)
{
	uint8_t frame[SBUS_FRAME_SIZE] = {SBUS_HEADER};

	K_SPINLOCK(&lock) {
		for (size_t channel = 0; channel < SBUS_CHANNELS; channel++) {
			for (size_t bit = 0; bit < SBUS_CHANNEL_BITS; bit++) {
				if ((channel_values[channel] & BIT(bit)) != 0) {
					const size_t position = channel * SBUS_CHANNEL_BITS + bit;

					frame[1 + position / 8] |= BIT(position % 8);
				}
			}
		}
	}

	uart_emul_put_rx_data(uart, frame, sizeof(frame));
}

K_TIMER_DEFINE(frame_timer, send_frame, NULL);

void sbus_receiver_set_channels(const uint16_t channels[SBUS_CHANNELS])
{
	K_SPINLOCK(&lock) {
		memcpy(channel_values, channels, sizeof(channel_values));
	}
}

void sbus_receiver_start(void)
{
	k_timer_start(&frame_timer, K_NO_WAIT, K_MSEC(SBUS_FRAME_INTERVAL_MS));
}

void sbus_receiver_stop(void)
{
	k_timer_stop(&frame_timer);
}

void sbus_receiver_report_failsafe(bool failsafe)
{
	zassert_ok(input_report_key(sbus, INPUT_KEY_1, failsafe ? 1 : 0, false, K_FOREVER));
}

void sbus_receiver_report_sticks(uint16_t value)
{
	zassert_ok(input_report_abs(sbus, INPUT_ABS_RX, value, false, K_FOREVER));
	zassert_ok(input_report_abs(sbus, INPUT_ABS_RY, value, false, K_FOREVER));
	zassert_ok(input_report_abs(sbus, INPUT_ABS_Y, value, false, K_FOREVER));
	zassert_ok(input_report_abs(sbus, INPUT_ABS_X, value, true, K_FOREVER));
}

void sbus_receiver_press_other_switch(bool pressed)
{
	zassert_ok(input_report_key(other_sbus, INPUT_KEY_0, pressed ? 1 : 0, true, K_FOREVER));
}
