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

#include <uavcan/driver/can.hpp>
#include <zephyr/kernel.h>

namespace uavcan_zephyr
{

/**
 * Wakes up select() and the node when a frame is received or sent.
 */
class BusEvent : uavcan::Noncopyable
{
public:
	using SignalCallbackHandler = void(*)();

	BusEvent() { k_sem_init(&_sem, 0, 1); }

	void registerSignalCallback(SignalCallbackHandler handler) { _signal_callback = handler; }

	/**
	 * Return true if an event happened before the timeout.
	 */
	bool wait(uavcan::MonotonicDuration duration);

	void signalFromInterrupt();

private:
	struct k_sem _sem;
	SignalCallbackHandler _signal_callback{nullptr};
};

/**
 * A CAN bus of the PX4 Zephyr platform, see px4_can.h.
 */
class CanIface : public uavcan::ICanIface, uavcan::Noncopyable
{
public:
	/**
	 * State and error counters of the controller. `errors` counts the frames
	 * that couldn't be sent and the times the controller became error passive
	 * or went bus off; `tec`/`rec` are the live CAN error counters.
	 */
	struct BusErrors {
		uint8_t  state{0};
		uint8_t  tec{0};
		uint8_t  rec{0};
		uint32_t errors{0};
		uint32_t rx_overruns{0};
	};

	void setBus(unsigned bus) { _bus = bus; }

	int open(uint32_t bitrate, BusEvent &event);

	uavcan::int16_t send(const uavcan::CanFrame &frame, uavcan::MonotonicTime tx_deadline,
			     uavcan::CanIOFlags flags) override;

	uavcan::int16_t receive(uavcan::CanFrame &out_frame, uavcan::MonotonicTime &out_ts_monotonic,
				uavcan::UtcTime &out_ts_utc, uavcan::CanIOFlags &out_flags) override;

	/* the controller receives all the frames with a 29-bit identifier */
	uavcan::int16_t configureFilters(const uavcan::CanFilterConfig *filter_configs,
					 uavcan::uint16_t num_configs) override { return 0; }

	uavcan::uint16_t getNumFilters() const override { return 0; }

	uavcan::uint64_t getErrorCount() const override;

	bool txReady(const uavcan::CanFrame &frame) const;

	bool rxPending() const;

	bool lastErrors(BusErrors &out) const;

	static const char *busStateName(uint8_t state);

private:
	unsigned _bus{0};
};

class CanDriver : public uavcan::ICanDriver, uavcan::Noncopyable
{
public:
	CanDriver();

	/**
	 * Start the CAN buses. Return 0 if at least one of them started, a negative errno otherwise.
	 */
	int init(uavcan::uint32_t bitrate);

	uavcan::int16_t select(uavcan::CanSelectMasks &inout_masks,
			       const uavcan::CanFrame * (&pending_tx)[uavcan::MaxCanIfaces],
			       uavcan::MonotonicTime blocking_deadline) override;

	uavcan::ICanIface *getIface(uavcan::uint8_t iface_index) override;

	uavcan::uint8_t getNumIfaces() const override { return _num_ifaces; }

	BusEvent &updateEvent() { return _update_event; }

private:
	BusEvent _update_event;
	CanIface _ifaces[UAVCAN_ZEPHYR_NUM_IFACES];
	uavcan::uint8_t _num_ifaces{0};
};

template <unsigned RxQueueCapacity = 128>
class CanInitHelper
{
public:
	CanDriver driver;

	/* the CAN buses are those of the px4,can-buses devicetree node */
	CanInitHelper(uint32_t unused = 0x7) {}

	int init(uavcan::uint32_t bitrate) { return driver.init(bitrate); }
};

}
