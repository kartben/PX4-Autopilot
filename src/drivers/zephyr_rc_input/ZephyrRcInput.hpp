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
 * @file ZephyrRcInput.hpp
 *
 * Publishes the RC input of the Zephyr input device of the px4,rc-input
 * devicetree node.
 */

#pragma once

#include "CrsfTelemetry.hpp"
#include "zephyr_rc_input_backend.h"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/input_rc.h>

using namespace time_literals;

static_assert(ZEPHYR_RC_INPUT_MAX_CHANNELS == input_rc_s::RC_INPUT_MAX_CHANNELS, "RC channel count mismatch");

class ZephyrRcInput : public ModuleBase, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	explicit ZephyrRcInput(const zephyr_rc_input_info &info);
	~ZephyrRcInput() override;

	static int task_spawn(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[]);

	static int print_usage(const char *reason = nullptr);

	void init();

	int print_status() override;

private:
	// RC is lost when no frame is received for this long, as with PX4IO
	static constexpr hrt_abstime RC_LOST_TIMEOUT{200_ms};

	// frames are published at most every 3 ms, at up to 333 Hz, instead of the 500 or 1000 Hz of
	// a CRSF link, as crsf_rc publishes them every 4 ms or more
	static constexpr hrt_abstime PUBLISH_INTERVAL{3_ms};

	// RSSI of a receiver that doesn't report it, as with rc_input
	static constexpr int32_t RSSI_UNKNOWN{255};

	void Run() override;

	static void frame_received(void *arg);

	void update_link_stats();

	const zephyr_rc_input_info _info;
	uint32_t _frame_count{0};

	// time of the last frame that was published, in the thread of the input driver
	hrt_abstime _frame_scheduled{0};

	// telemetry, for a CRSF receiver
	CrsfTelemetry *_crsf_telemetry{nullptr};

	input_rc_s _input_rc{};
	uORB::Publication<input_rc_s> _input_rc_pub{ORB_ID(input_rc)};
};
