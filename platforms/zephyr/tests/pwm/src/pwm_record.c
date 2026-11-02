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

#include "pwm_record.h"

#include <errno.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pwm/pwm_fake.h>
#include <zephyr/fff.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

DEFINE_FFF_GLOBALS;

#define PWM_OUTPUT(node) PWM_DT_SPEC_GET(node),

static const struct pwm_dt_spec outputs[] = {
	DT_FOREACH_CHILD(DT_COMPAT_GET_ANY_STATUS_OKAY(px4_pwm_outputs), PWM_OUTPUT)
};

static struct k_spinlock lock;
static struct pwm_record records[ARRAY_SIZE(outputs)];
static int64_t last_write[ARRAY_SIZE(outputs)];

static int record_set_cycles(const struct device *dev, uint32_t channel, uint32_t period_cycles,
			     uint32_t pulse_cycles, pwm_flags_t flags)
{
	size_t output = 0;

	while ((output < ARRAY_SIZE(outputs)) &&
	       ((outputs[output].dev != dev) || (outputs[output].channel != channel))) {
		output++;
	}

	if (output == ARRAY_SIZE(outputs)) {
		return -EINVAL;
	}

	const int64_t now = k_uptime_ticks();

	K_SPINLOCK(&lock) {
		struct pwm_record *record = &records[output];

		if (record->writes > 0) {
			record->min_interval_us = MIN(record->min_interval_us,
						      k_ticks_to_us_floor32(now - last_write[output]));
		}

		record->period_us = period_cycles;
		record->pulse_us = pulse_cycles;
		record->writes++;
		last_write[output] = now;
	}

	return 0;
}

void pwm_record_start(void)
{
	K_SPINLOCK(&lock) {
		memset(records, 0, sizeof(records));

		for (size_t i = 0; i < ARRAY_SIZE(records); i++) {
			records[i].min_interval_us = UINT32_MAX;
		}
	}

	/* reset by the fake PWM before each test */
	fake_pwm_set_cycles_fake.custom_fake = record_set_cycles;
}

void pwm_record_get(size_t output, struct pwm_record *record)
{
	K_SPINLOCK(&lock) {
		*record = records[output];
	}
}
