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

#pragma once

#include <uavcan/driver/system_clock.hpp>
#include <drivers/uavcan/uavcan_drivers/hrt_clock.hpp>

namespace uavcan_zephyr
{

/*
 * Monotonic time is HRT. UTC is HRT plus a synced offset and rate (see
 * uavcan_hrt_clock::Clock).
 */
namespace clock
{
uavcan::MonotonicTime getMonotonic();

/**
 * Zero until set by setUtc() or the first adjustUtc().
 */
uavcan::UtcTime getUtc();

/**
 * UTC at an earlier monotonic time, for example when a frame was received.
 */
uavcan::UtcTime utcAt(uavcan::MonotonicTime time);

/**
 * UTC reads `time` from now on.
 */
void setUtc(uavcan::UtcTime time);

/**
 * Moves UTC by `adjustment`. While UTC is unset the adjustment is absolute:
 * UTC reads `adjustment` from now on.
 */
void adjustUtc(uavcan::UtcDuration adjustment);
}

/**
 * Adapter for uavcan::ISystemClock.
 */
class SystemClock : public uavcan::ISystemClock, uavcan::Noncopyable
{
	SystemClock() { }

	void adjustUtc(uavcan::UtcDuration adjustment) override { clock::adjustUtc(adjustment); }

public:
	uavcan::MonotonicTime getMonotonic() const override { return clock::getMonotonic(); }
	uavcan::UtcTime getUtc() const override { return clock::getUtc(); }

	static SystemClock &instance();
};

}
