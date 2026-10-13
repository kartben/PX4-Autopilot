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

#include "ZephyrRcInput.hpp"

#include <lib/mathlib/mathlib.h>

#include <errno.h>

ModuleBase::Descriptor ZephyrRcInput::desc{task_spawn, custom_command, print_usage};

ZephyrRcInput::ZephyrRcInput(const zephyr_rc_input_info &info) :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_info(info)
{
	_input_rc.channel_count = info.channel_count;
	_input_rc.input_source = info.sbus ? input_rc_s::RC_INPUT_SOURCE_PX4FMU_SBUS
				 : info.crsf ? input_rc_s::RC_INPUT_SOURCE_PX4FMU_CRSF
				 : input_rc_s::RC_INPUT_SOURCE_UNKNOWN;
	_input_rc.rssi = RSSI_UNKNOWN;
	_input_rc.link_quality = -1;
	_input_rc.rssi_dbm = NAN;
	_input_rc.link_snr = -1;

	if (info.crsf) {
		_crsf_telemetry = new CrsfTelemetry();
	}
}

ZephyrRcInput::~ZephyrRcInput()
{
	delete _crsf_telemetry;
}

void ZephyrRcInput::update_link_stats()
{
	zephyr_rc_input_link_stats stats{};

	if (zephyr_rc_input_get_link_stats(&stats) != 0) {
		_input_rc.rssi = RSSI_UNKNOWN;
		return;
	}

	// the RSSI in % from the RSSI in dBm, as crsf_rc approximates it
	_input_rc.rssi_dbm = stats.rssi_dbm;
	_input_rc.rssi = math::constrain((1.f - _input_rc.rssi_dbm / -130.f) * input_rc_s::RSSI_MAX, 0.f,
					 (float)input_rc_s::RSSI_MAX);
	_input_rc.link_quality = stats.link_quality;
	_input_rc.link_snr = stats.snr;
	_input_rc.rc_frame_rate = stats.rf_mode;
}

void ZephyrRcInput::init()
{
	// only the frames received from now on are published
	zephyr_rc_input_frame frame{};

	if (zephyr_rc_input_get_frame(&frame) == 0) {
		_frame_count = frame.count;
	}

	zephyr_rc_input_set_event(frame_received, this);

	// frames are published when they are received, the interval checks for their loss
	ScheduleOnInterval(RC_LOST_TIMEOUT / 4);
}

void ZephyrRcInput::frame_received(void *arg)
{
	ZephyrRcInput *instance = static_cast<ZephyrRcInput *>(arg);
	const hrt_abstime now = hrt_absolute_time();

	// the frames that follow the last published one by less than the interval are skipped
	if (now >= instance->_frame_scheduled + PUBLISH_INTERVAL) {
		instance->_frame_scheduled = now;
		instance->ScheduleNow();
	}
}

void ZephyrRcInput::Run()
{
	if (should_exit()) {
		zephyr_rc_input_set_event(nullptr, nullptr);
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	zephyr_rc_input_frame frame{};

	if ((zephyr_rc_input_get_frame(&frame) == 0) && (frame.count != _frame_count)) {
		_frame_count = frame.count;

		_input_rc.timestamp_last_signal = frame.timestamp_us;
		_input_rc.rc_failsafe = frame.failsafe;

		if (_info.crsf) {
			update_link_stats();

		} else {
			_input_rc.rssi = RSSI_UNKNOWN;
		}

		_input_rc.rc_lost = false;

		for (unsigned i = 0; i < _info.channel_count; i++) {
			_input_rc.values[i] = frame.values[i];
		}

		_input_rc.timestamp = hrt_absolute_time();
		_input_rc_pub.publish(_input_rc);

		// telemetry is sent while the receiver sends frames
		if (_crsf_telemetry != nullptr) {
			_crsf_telemetry->update(_input_rc.timestamp);
		}

	} else if (!_input_rc.rc_lost && (_input_rc.timestamp_last_signal != 0)
		   && (hrt_elapsed_time(&_input_rc.timestamp_last_signal) > RC_LOST_TIMEOUT)) {
		// the last frame, without signal
		_input_rc.rssi = 0;
		_input_rc.rc_lost = true;

		if (_info.crsf) {
			// the link statistics of the receiver, without link
			_input_rc.rssi_dbm = NAN;
			_input_rc.link_quality = -1;
			_input_rc.link_snr = -1;
			_input_rc.rc_frame_rate = 0;
		}

		_input_rc.timestamp = hrt_absolute_time();
		_input_rc_pub.publish(_input_rc);
	}
}

int ZephyrRcInput::print_status()
{
	PX4_INFO("input device: %s, %u channels", _info.device, _info.channel_count);
	PX4_INFO("frames: %" PRIu32, _frame_count);

	if (_crsf_telemetry != nullptr) {
		PX4_INFO("telemetry frames: %" PRIu32, _crsf_telemetry->sent());
	}

	if (_input_rc.timestamp != 0) {
		print_message(ORB_ID(input_rc), _input_rc);
	}

	return 0;
}

int ZephyrRcInput::task_spawn(int argc, char *argv[])
{
	zephyr_rc_input_info info{};
	const int ret = zephyr_rc_input_get_info(&info);

	if (ret == -ENOENT) {
		PX4_INFO("no RC input in the devicetree");
		return PX4_OK;
	}

	if (ret != 0) {
		PX4_ERR("%s not ready (%d)", info.device, ret);
		return PX4_ERROR;
	}

	ZephyrRcInput *instance = new ZephyrRcInput(info);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;

	instance->init();

	return PX4_OK;
}

int ZephyrRcInput::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ZephyrRcInput::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Publishes the RC input of the Zephyr input device of the px4,rc-input
devicetree node, such as an SBUS or a CRSF receiver, to input_rc.

Each RC channel is an input event listed in the devicetree node. Its value is
scaled to a pulse width as PX4 scales the SBUS channels, and a key gives 1000 us
while it is released and 2000 us while it is pressed. The channels are
published after each frame of the input device, which ends with a sync event,
and RC is lost when no frame is received for 200 ms.

A CRSF receiver (tbs,crsf) also gives its link statistics, and is sent the
telemetry that crsf_rc sends: the battery, the GNSS receiver, the attitude,
the flight mode and the altitude.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("zephyr_rc_input", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("radio_control");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int zephyr_rc_input_main(int argc, char *argv[])
{
	return ModuleBase::main(ZephyrRcInput::desc, argc, argv);
}
