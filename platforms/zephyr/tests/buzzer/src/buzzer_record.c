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

#include "buzzer_record.h"

#include <errno.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pwm/pwm_fake.h>
#include <zephyr/fff.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

DEFINE_FFF_GLOBALS;

static const struct pwm_dt_spec buzzer_pwm = PWM_DT_SPEC_GET(DT_ALIAS(buzzer0));

static struct k_spinlock lock;
static struct buzzer_event events[BUZZER_RECORD_MAX_EVENTS];
static size_t event_count;

static int record_set_cycles(const struct device *dev, uint32_t channel, uint32_t period_cycles,
			     uint32_t pulse_cycles, pwm_flags_t flags)
{
	if ((dev != buzzer_pwm.dev) || (channel != buzzer_pwm.channel)) {
		return -EINVAL;
	}

	const uint64_t now = k_ticks_to_us_floor64(k_uptime_ticks());

	K_SPINLOCK(&lock) {
		if (event_count < BUZZER_RECORD_MAX_EVENTS) {
			events[event_count].time_us = now;
			events[event_count].period_ns = period_cycles;
			events[event_count].pulse_ns = pulse_cycles;
		}

		event_count++;
	}

	return 0;
}

void buzzer_record_start(void)
{
	K_SPINLOCK(&lock) {
		event_count = 0;
	}

	/* reset by the fake PWM before each test */
	fake_pwm_set_cycles_fake.custom_fake = record_set_cycles;
}

size_t buzzer_record_get(struct buzzer_event *copy, size_t max_events)
{
	size_t count;

	K_SPINLOCK(&lock) {
		count = event_count;
		memcpy(copy, events, MIN(MIN(count, BUZZER_RECORD_MAX_EVENTS), max_events) * sizeof(events[0]));
	}

	return count;
}
