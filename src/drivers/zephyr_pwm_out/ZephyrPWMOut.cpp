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

#include "ZephyrPWMOut.hpp"

#include <mathlib/mathlib.h>

ModuleBase::Descriptor ZephyrPWMOut::desc{task_spawn, custom_command, print_usage};

ZephyrPWMOut::ZephyrPWMOut() :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

ZephyrPWMOut::~ZephyrPWMOut()
{
	perf_free(_cycle_perf);
	perf_free(_interval_perf);
	perf_free(_error_perf);
}

int ZephyrPWMOut::init()
{
	_output_count = zephyr_pwm_out_count();

	for (size_t i = 0; i < _output_count; i++) {
		int ret = zephyr_pwm_out_get_period(i, &_period_us[i]);

		// no pulses until the output is set, which also checks that the PWM can make the period
		if (ret == 0) {
			ret = zephyr_pwm_out_set_pulse(i, 0);
		}

		if (ret != 0) {
			PX4_ERR("output %u: error %d", static_cast<unsigned>(i + 1), ret);
			return PX4_ERROR;
		}

		_shortest_period_us = (i == 0) ? _period_us[i] : math::min(_shortest_period_us, _period_us[i]);

		for (size_t j = 0; j < i; j++) {
			if ((_period_us[j] != _period_us[i]) && zephyr_pwm_out_same_device(i, j)) {
				PX4_WARN("outputs %u and %u: different periods on the same PWM device", static_cast<unsigned>(j + 1),
					 static_cast<unsigned>(i + 1));
			}
		}
	}

	_mixing_output.setMaxNumOutputs(_output_count);

	// the outputs are updated at the rate of the fastest one at most
	_mixing_output.setMaxTopicUpdateRate(_shortest_period_us);

	ScheduleNow();
	return PX4_OK;
}

void ZephyrPWMOut::set_pulse(size_t index, uint32_t pulse_us)
{
	// the PWM API refuses a pulse longer than the period, which parameters out of their range can give
	if (zephyr_pwm_out_set_pulse(index, math::min(pulse_us, _period_us[index])) != 0) {
		perf_count(_error_perf);
		return;
	}

	_pulse_us[index] = pulse_us;

	// a PWM driver can wait for the end of the period to take a new pulse: an output is set at most once per
	// period, but for a margin on the fastest outputs, which follow the mixer and its jitter
	const uint32_t margin_us = (_period_us[index] == _shortest_period_us) ? _shortest_period_us / 2 : 0;
	_next_update[index] = hrt_absolute_time() + _period_us[index] - margin_us;
}

bool ZephyrPWMOut::updateOutputs(float outputs[MAX_ACTUATORS], unsigned num_outputs, unsigned num_control_groups_updated)
{
	const hrt_abstime now = hrt_absolute_time();

	for (size_t i = 0; i < num_outputs; i++) {
		// no pulses on the outputs without a function, as with pwm_out
		const uint32_t pulse_us = _mixing_output.isFunctionSet(i) ? static_cast<uint32_t>(lroundf(outputs[i])) : 0;

		// 0 (no pulses) is set at once, as MixingOutput does not repeat the update that removes the last function
		if ((pulse_us != _pulse_us[i]) && ((pulse_us == 0) || (now >= _next_update[i]))) {
			set_pulse(i, pulse_us);
		}
	}

	return true;
}

void ZephyrPWMOut::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		// the outputs are left disarmed
		for (size_t i = 0; i < _output_count; i++) {
			set_pulse(i, _mixing_output.isFunctionSet(i) ? _mixing_output.disarmedValue(i) : 0);
		}

		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	_mixing_output.update();

	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update;
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
	}

	_mixing_output.updateSubscriptions(true);

	perf_end(_cycle_perf);
}

int ZephyrPWMOut::print_status()
{
	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);
	perf_print_counter(_error_perf);
	_mixing_output.printStatus();

	for (size_t i = 0; i < _output_count; i++) {
		PX4_INFO_RAW("Output %u: %u Hz, pulse %u us\n", static_cast<unsigned>(i + 1),
			     static_cast<unsigned>(1_s / _period_us[i]), static_cast<unsigned>(_pulse_us[i]));
	}

	return 0;
}

int ZephyrPWMOut::task_spawn(int argc, char *argv[])
{
	if (zephyr_pwm_out_count() == 0) {
		PX4_INFO("no PWM outputs in the devicetree");
		return PX4_OK;
	}

	ZephyrPWMOut *instance = new ZephyrPWMOut();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int ZephyrPWMOut::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ZephyrPWMOut::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Drives the PWM outputs listed in the px4,pwm-outputs devicetree node through the
Zephyr PWM API, as the main outputs: the first output is configured with
PWM_MAIN_FUNC1, PWM_MAIN_DIS1, PWM_MAIN_MIN1 and so on.

Each output has the period of its pwms devicetree property, at least 2200 us.
The outputs are updated at the rate of the fastest one at most, and the slower
ones at most once per period. The outputs without a function have no pulses,
and the outputs are left at their disarmed value when the driver stops.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("zephyr_pwm_out", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int zephyr_pwm_out_main(int argc, char *argv[])
{
	return ModuleBase::main(ZephyrPWMOut::desc, argc, argv);
}
