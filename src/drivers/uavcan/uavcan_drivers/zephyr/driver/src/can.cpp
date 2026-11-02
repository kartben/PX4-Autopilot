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

#include <uavcan_zephyr/can.hpp>
#include <uavcan_zephyr/clock.hpp>

#include <px4_can.h>
#include <px4_platform_common/log.h>

#include <cerrno>
#include <cstring>

namespace uavcan_zephyr
{

static void signal_bus_event(void *arg)
{
	static_cast<BusEvent *>(arg)->signalFromInterrupt();
}

static px4_can_frame to_px4_frame(const uavcan::CanFrame &frame)
{
	px4_can_frame px4_frame{};
	px4_frame.id = frame.id & uavcan::CanFrame::MaskExtID;
	px4_frame.flags = frame.isExtended() ? PX4_CAN_FRAME_EXT : 0;
	px4_frame.len = frame.dlc;
	memcpy(px4_frame.data, frame.data, frame.dlc);
	return px4_frame;
}

bool BusEvent::wait(uavcan::MonotonicDuration duration)
{
	if (!duration.isPositive()) {
		return false;
	}

	return k_sem_take(&_sem, K_USEC(duration.toUSec())) == 0;
}

void BusEvent::signalFromInterrupt()
{
	k_sem_give(&_sem);

	if (_signal_callback) {
		_signal_callback();
	}
}

int CanIface::open(uint32_t bitrate, BusEvent &event)
{
	return px4_can_open(_bus, bitrate, signal_bus_event, &event);
}

uavcan::int16_t CanIface::send(const uavcan::CanFrame &frame, uavcan::MonotonicTime tx_deadline,
			       uavcan::CanIOFlags flags)
{
	if (frame.isErrorFrame() || frame.isRemoteTransmissionRequest() || (frame.dlc > PX4_CAN_MAX_DLEN)) {
		return -EINVAL;
	}

	/* libuavcan drops the frame once its deadline has passed */
	if (!tx_deadline.isZero() && (clock::getMonotonic() >= tx_deadline)) {
		return 0;
	}

	const px4_can_frame px4_frame = to_px4_frame(frame);
	return px4_can_send(_bus, &px4_frame, (flags & uavcan::CanIOFlagLoopback) != 0);
}

uavcan::int16_t CanIface::receive(uavcan::CanFrame &out_frame, uavcan::MonotonicTime &out_ts_monotonic,
				  uavcan::UtcTime &out_ts_utc, uavcan::CanIOFlags &out_flags)
{
	px4_can_rx_frame rx;

	if (px4_can_receive(_bus, &rx) != 1) {
		return 0;
	}

	const uint32_t id = rx.frame.id | (((rx.frame.flags & PX4_CAN_FRAME_EXT) != 0) ? uavcan::CanFrame::FlagEFF : 0);
	out_frame = uavcan::CanFrame(id, rx.frame.data, rx.frame.len);
	out_ts_monotonic = uavcan::MonotonicTime::fromUSec(rx.timestamp_us);
	out_ts_utc = clock::utcAt(out_ts_monotonic);
	out_flags = rx.loopback ? uavcan::CanIOFlagLoopback : 0;
	return 1;
}

uavcan::uint64_t CanIface::getErrorCount() const
{
	px4_can_status status;

	if (px4_can_get_status(_bus, &status) != 0) {
		return 0;
	}

	return uavcan::uint64_t(status.tx_errors) + status.rx_overruns + status.bus_errors;
}

bool CanIface::txReady(const uavcan::CanFrame &frame) const
{
	const px4_can_frame px4_frame = to_px4_frame(frame);
	return px4_can_tx_ready(_bus, &px4_frame);
}

bool CanIface::rxPending() const
{
	return px4_can_rx_pending(_bus);
}

bool CanIface::lastErrors(BusErrors &out) const
{
	px4_can_status status;

	if (px4_can_get_status(_bus, &status) != 0) {
		return false;
	}

	out.state = status.state;
	out.tec = status.tx_error_counter;
	out.rec = status.rx_error_counter;
	out.errors = status.tx_errors + status.bus_errors;
	out.rx_overruns = status.rx_overruns;
	return true;
}

const char *CanIface::busStateName(uint8_t state)
{
	switch (state) {
	case PX4_CAN_ERROR_ACTIVE: return "error active";

	case PX4_CAN_ERROR_WARNING: return "error warning";

	case PX4_CAN_ERROR_PASSIVE: return "error passive";

	case PX4_CAN_BUS_OFF: return "bus off";

	default: return "stopped";
	}
}

CanDriver::CanDriver()
{
	const unsigned buses = px4_can_bus_count();
	_num_ifaces = (buses < UAVCAN_ZEPHYR_NUM_IFACES) ? buses : UAVCAN_ZEPHYR_NUM_IFACES;

	for (unsigned i = 0; i < UAVCAN_ZEPHYR_NUM_IFACES; i++) {
		_ifaces[i].setBus(i);
	}
}

int CanDriver::init(uavcan::uint32_t bitrate)
{
	int ret = -ENODEV;

	for (unsigned i = 0; i < _num_ifaces; i++) {
		const int iface_ret = _ifaces[i].open(bitrate, _update_event);

		if (iface_ret == 0) {
			ret = 0;

		} else {
			PX4_ERR("CAN%u: start failed (%d)", i + 1, iface_ret);
		}
	}

	return ret;
}

uavcan::int16_t CanDriver::select(uavcan::CanSelectMasks &inout_masks,
				  const uavcan::CanFrame * (&pending_tx)[uavcan::MaxCanIfaces],
				  uavcan::MonotonicTime blocking_deadline)
{
	const uavcan::CanSelectMasks wanted = inout_masks;

	for (;;) {
		inout_masks.read = 0;
		inout_masks.write = 0;

		for (unsigned i = 0; i < _num_ifaces; i++) {
			if (_ifaces[i].rxPending()) {
				inout_masks.read |= 1U << i;
			}

			if ((pending_tx[i] != nullptr) && _ifaces[i].txReady(*pending_tx[i])) {
				inout_masks.write |= 1U << i;
			}
		}

		if (((inout_masks.read & wanted.read) != 0) || ((inout_masks.write & wanted.write) != 0)) {
			return 0;
		}

		const uavcan::MonotonicTime now = clock::getMonotonic();

		if (now >= blocking_deadline) {
			return 0;
		}

		_update_event.wait(blocking_deadline - now);
	}
}

uavcan::ICanIface *CanDriver::getIface(uavcan::uint8_t iface_index)
{
	return (iface_index < _num_ifaces) ? &_ifaces[iface_index] : nullptr;
}

} // namespace uavcan_zephyr
