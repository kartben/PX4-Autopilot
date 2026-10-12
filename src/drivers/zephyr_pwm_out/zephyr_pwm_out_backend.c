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

#include "zephyr_pwm_out_backend.h"

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>

#if DT_HAS_COMPAT_STATUS_OKAY(px4_pwm_outputs)

#define PWM_OUTPUTS_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(px4_pwm_outputs)

#define PWM_OUTPUT(node) PWM_DT_SPEC_GET(node),

static const struct pwm_dt_spec outputs[] = {DT_FOREACH_CHILD_STATUS_OKAY(PWM_OUTPUTS_NODE, PWM_OUTPUT)};

BUILD_ASSERT(ARRAY_SIZE(outputs) > 0, "px4,pwm-outputs has no output");
BUILD_ASSERT(ARRAY_SIZE(outputs) <= ZEPHYR_PWM_OUT_MAX_OUTPUTS,
	     "px4,pwm-outputs has more outputs than the PWM_MAIN parameters");

#define PWM_OUTPUT_PERIOD_CHECK(node)                                                              \
	BUILD_ASSERT(DT_PWMS_PERIOD(node) >= PWM_USEC(ZEPHYR_PWM_OUT_MAX_PULSE_US),                \
		     DT_NODE_PATH(node) ": the period is shorter than the pulses of the PWM_MAIN parameters");

DT_FOREACH_CHILD_STATUS_OKAY(PWM_OUTPUTS_NODE, PWM_OUTPUT_PERIOD_CHECK)

size_t zephyr_pwm_out_count(void)
{
	return ARRAY_SIZE(outputs);
}

int zephyr_pwm_out_get_period(size_t index, uint32_t *period_us)
{
	if (index >= ARRAY_SIZE(outputs)) {
		return -EINVAL;
	}

	if (!pwm_is_ready_dt(&outputs[index])) {
		return -ENODEV;
	}

	*period_us = outputs[index].period / NSEC_PER_USEC;
	return 0;
}

int zephyr_pwm_out_set_pulse(size_t index, uint32_t pulse_us)
{
	if (index >= ARRAY_SIZE(outputs)) {
		return -EINVAL;
	}

	return pwm_set_pulse_dt(&outputs[index], PWM_USEC(pulse_us));
}

bool zephyr_pwm_out_same_device(size_t index1, size_t index2)
{
	return (index1 < ARRAY_SIZE(outputs)) && (index2 < ARRAY_SIZE(outputs)) &&
	       (outputs[index1].dev == outputs[index2].dev);
}

#else

size_t zephyr_pwm_out_count(void)
{
	return 0;
}

int zephyr_pwm_out_get_period(size_t index, uint32_t *period_us)
{
	return -EINVAL;
}

int zephyr_pwm_out_set_pulse(size_t index, uint32_t pulse_us)
{
	return -EINVAL;
}

bool zephyr_pwm_out_same_device(size_t index1, size_t index2)
{
	return false;
}

#endif
