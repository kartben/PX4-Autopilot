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
 * @file zephyr_pwm_out_backend.h
 *
 * The PWM outputs listed in the px4,pwm-outputs devicetree node, in the order
 * of the child nodes.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* num_channels in module.yaml and module_dshot.yaml */
#define ZEPHYR_PWM_OUT_MAX_OUTPUTS 8

/* the maximum of the standard parameters in module.yaml */
#define ZEPHYR_PWM_OUT_MAX_PULSE_US 2200

size_t zephyr_pwm_out_count(void);

/**
 * Get the period of an output, from its pwms devicetree property.
 * @return 0, -EINVAL for an invalid output, or -ENODEV if the PWM device is not ready
 */
int zephyr_pwm_out_get_period(size_t index, uint32_t *period_us);

/**
 * Check if two outputs are on the same PWM device, which can have one period for all its channels.
 */
bool zephyr_pwm_out_same_device(size_t index1, size_t index2);

/**
 * Set the pulse of an output, or no pulses with 0.
 * @return 0 or a negative error code
 */
int zephyr_pwm_out_set_pulse(size_t index, uint32_t pulse_us);

#ifdef __cplusplus
}
#endif
