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

#include "px4_buzzer.h"

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/buzzer.h>
#include <zephyr/kernel.h>

static const struct device *const buzzer = DEVICE_DT_GET(DT_ALIAS(buzzer0));

static void stop_handler(struct k_work *work)
{
	buzzer_stop(buzzer);
}

/* the high-resolution timer ends the notes in an interrupt, where the buzzer API is not available */
static K_WORK_DEFINE(stop_work, stop_handler);

int px4_buzzer_init(void)
{
	if (!device_is_ready(buzzer)) {
		return -ENODEV;
	}

	/* a square wave for a PWM buzzer, as with NuttX */
	return buzzer_set_volume(buzzer, BUZZER_VOLUME_MAX);
}

int px4_buzzer_start(uint32_t frequency_hz)
{
	if (!device_is_ready(buzzer)) {
		return -ENODEV;
	}

	/* the end of the previous note, if it is still pending, would silence this one */
	struct k_work_sync sync;
	k_work_cancel_sync(&stop_work, &sync);

	return buzzer_tone(buzzer, frequency_hz, BUZZER_DURATION_FOREVER);
}

void px4_buzzer_stop(void)
{
	if (!device_is_ready(buzzer)) {
		return;
	}

	if (k_is_in_isr()) {
		k_work_submit(&stop_work);

	} else {
		struct k_work_sync sync;
		k_work_cancel_sync(&stop_work, &sync);
		buzzer_stop(buzzer);
	}
}
