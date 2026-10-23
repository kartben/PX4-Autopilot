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
 * @file ZephyrPWMOut.hpp
 *
 * Drives the PWM outputs listed in the px4,pwm-outputs devicetree node.
 */

#pragma once

#include "zephyr_pwm_out_backend.h"

#include <drivers/drv_hrt.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

class ZephyrPWMOut : public ModuleBase, public OutputModuleInterface
{
public:
	static Descriptor desc;

	ZephyrPWMOut();
	~ZephyrPWMOut() override;

	static int task_spawn(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[]);

	static int print_usage(const char *reason = nullptr);

	int init();

	int print_status() override;

	bool updateOutputs(float outputs[MAX_ACTUATORS], unsigned num_outputs, unsigned num_control_groups_updated) override;

private:
	void Run() override;

	void set_pulse(size_t index, uint32_t pulse_us);

	bool dshot_selected(size_t index) const;

	// the values of PWM_MAIN_TIM<n> from BDShot150 to DShot600, for the outputs that the dshot driver drives
	static constexpr int32_t DSHOT_PROTOCOL_MIN{-8};
	static constexpr int32_t DSHOT_PROTOCOL_MAX{-3};

	MixingOutput _mixing_output{"PWM_MAIN", ZEPHYR_PWM_OUT_MAX_OUTPUTS, *this, MixingOutput::SchedulingPolicy::Auto, true};

	size_t _output_count{0};
	uint32_t _dshot_outputs{0};
	uint32_t _period_us[ZEPHYR_PWM_OUT_MAX_OUTPUTS] {};
	uint32_t _shortest_period_us{0};

	uint32_t _pulse_us[ZEPHYR_PWM_OUT_MAX_OUTPUTS] {};
	hrt_abstime _next_update[ZEPHYR_PWM_OUT_MAX_OUTPUTS] {};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
	perf_counter_t _error_perf{perf_alloc(PC_COUNT, MODULE_NAME": error")};
};
