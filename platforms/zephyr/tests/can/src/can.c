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
 * Test the CAN buses of PX4 on the loopback controller, which receives the
 * frames it sends. The test sends the frames of other nodes with can_send().
 * The test thread is cooperative, so frames stay in transmission until it
 * sleeps.
 */

#include <px4_can.h>

#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/ztest.h>

#define BITRATE 1000000

static const struct device *const can = DEVICE_DT_GET(DT_NODELABEL(can_loopback0));

static atomic_t events;

static void event(void *arg)
{
	atomic_inc(&events);
}

static struct px4_can_frame frame(uint32_t id, uint8_t len)
{
	struct px4_can_frame f = {.id = id, .flags = PX4_CAN_FRAME_EXT, .len = len};

	for (int i = 0; i < len; i++) {
		f.data[i] = id + i;
	}

	return f;
}

/* send a frame of another node */
static void receive(uint32_t id, uint8_t len)
{
	struct can_frame f = {.id = id, .dlc = len, .flags = CAN_FRAME_IDE};

	for (int i = 0; i < len; i++) {
		f.data[i] = id + i;
	}

	zassert_ok(can_send(can, &f, K_FOREVER, NULL, NULL));

	/* the controller completes the transmission before it delivers the frame */
	k_msleep(1);
}

static void open_bus(can_mode_t mode)
{
	int ret = can_stop(can);

	zassert_true((ret == 0) || (ret == -EALREADY));
	zassert_ok(can_set_mode(can, mode));
	zassert_ok(px4_can_open(0, BITRATE, event, NULL));

	/* frames of an earlier test */
	struct px4_can_rx_frame rx;

	k_msleep(1);

	while (px4_can_receive(0, &rx) == 1) {
	}

	atomic_clear(&events);
}

static void before(void *fixture)
{
	open_bus(CAN_MODE_LOOPBACK);
}

ZTEST(px4_can, test_bus_count)
{
	zassert_equal(px4_can_bus_count(), 1);
}

ZTEST(px4_can, test_receive)
{
	const uint64_t start = k_ticks_to_us_floor64(k_uptime_ticks());

	receive(0x1234567, 8);

	struct px4_can_rx_frame rx;
	const struct px4_can_frame expected = frame(0x1234567, 8);

	zassert_true(px4_can_rx_pending(0));
	zassert_equal(px4_can_receive(0, &rx), 1);
	zassert_equal(rx.frame.id, expected.id);
	zassert_equal(rx.frame.flags, PX4_CAN_FRAME_EXT);
	zassert_equal(rx.frame.len, 8);
	zassert_mem_equal(rx.frame.data, expected.data, 8);
	zassert_false(rx.loopback);
	zassert_between_inclusive(rx.timestamp_us, start, k_ticks_to_us_floor64(k_uptime_ticks()));
	zassert_true(atomic_get(&events) > 0);

	zassert_equal(px4_can_receive(0, &rx), 0);
	zassert_false(px4_can_rx_pending(0));
}

ZTEST(px4_can, test_own_frames_dropped)
{
	const struct px4_can_frame f = frame(0x100, 3);
	struct px4_can_rx_frame rx;

	zassert_equal(px4_can_send(0, &f, false), 1);
	k_msleep(10);

	zassert_equal(px4_can_receive(0, &rx), 0);
	zassert_true(atomic_get(&events) > 0);

	/* a frame of another node with the same contents is still received */
	receive(0x100, 3);
	zassert_equal(px4_can_receive(0, &rx), 1);
}

ZTEST(px4_can, test_loopback)
{
	const struct px4_can_frame f = frame(0x200, 5);
	struct px4_can_rx_frame rx;

	zassert_equal(px4_can_send(0, &f, true), 1);
	k_msleep(10);

	zassert_equal(px4_can_receive(0, &rx), 1);
	zassert_true(rx.loopback);
	zassert_equal(rx.frame.id, 0x200);
	zassert_mem_equal(rx.frame.data, f.data, 5);

	zassert_equal(px4_can_receive(0, &rx), 0);
}

ZTEST(px4_can, test_priority_order)
{
	const struct px4_can_frame f = frame(0x1000, 8);
	const struct px4_can_frame same_id = frame(0x1000, 2);
	const struct px4_can_frame lower = frame(0x1001, 8);
	const struct px4_can_frame higher = frame(0x0fff, 8);

	zassert_equal(px4_can_send(0, &f, false), 1);

	/* a frame sent later must not leave first */
	zassert_false(px4_can_tx_ready(0, &same_id));
	zassert_equal(px4_can_send(0, &same_id, false), 0);
	zassert_false(px4_can_tx_ready(0, &lower));
	zassert_equal(px4_can_send(0, &lower, false), 0);

	zassert_true(px4_can_tx_ready(0, &higher));
	zassert_equal(px4_can_send(0, &higher, false), 1);

	k_msleep(10);

	zassert_true(px4_can_tx_ready(0, &same_id));
	zassert_equal(px4_can_send(0, &same_id, false), 1);
}

ZTEST(px4_can, test_slots)
{
	struct px4_can_frame f;

	for (int i = 0; i < CONFIG_PX4_CAN_TX_SLOTS; i++) {
		f = frame(0x1000 - i, 1);
		zassert_equal(px4_can_send(0, &f, false), 1);
	}

	f = frame(0x1000 - CONFIG_PX4_CAN_TX_SLOTS, 1);
	zassert_false(px4_can_tx_ready(0, &f));
	zassert_equal(px4_can_send(0, &f, false), 0);

	k_msleep(10);

	zassert_equal(px4_can_send(0, &f, false), 1);
}

ZTEST(px4_can, test_rx_overrun)
{
	struct px4_can_status status;
	struct px4_can_rx_frame rx;

	zassert_ok(px4_can_get_status(0, &status));
	const uint32_t overruns = status.rx_overruns;

	for (int i = 0; i < CONFIG_PX4_CAN_RX_QUEUE_SIZE + 3; i++) {
		receive(0x300 + i, 1);
	}

	zassert_ok(px4_can_get_status(0, &status));
	zassert_equal(status.rx_overruns, overruns + 3);

	for (int i = 0; i < CONFIG_PX4_CAN_RX_QUEUE_SIZE; i++) {
		zassert_equal(px4_can_receive(0, &rx), 1);
		zassert_equal(rx.frame.id, 0x300 + i);
	}

	zassert_equal(px4_can_receive(0, &rx), 0);
}

ZTEST(px4_can, test_reopen)
{
	const struct px4_can_frame f = frame(0x400, 4);
	struct px4_can_rx_frame rx;

	zassert_equal(px4_can_send(0, &f, false), 1);

	/* restarting frees the slot of the frame being sent, which is still dropped when received back */
	zassert_ok(px4_can_open(0, BITRATE, event, NULL));

	zassert_equal(px4_can_send(0, &f, false), 1);
	k_msleep(10);
	zassert_equal(px4_can_send(0, &f, false), 1);
	k_msleep(10);

	zassert_equal(px4_can_receive(0, &rx), 0);

	/* a single filter */
	receive(0x401, 1);
	zassert_equal(px4_can_receive(0, &rx), 1);
	zassert_equal(px4_can_receive(0, &rx), 0);
}

ZTEST(px4_can, test_stopped)
{
	const struct px4_can_frame f = frame(0x500, 1);
	struct px4_can_status status;

	zassert_ok(px4_can_get_status(0, &status));
	const uint32_t tx_errors = status.tx_errors;

	zassert_ok(can_stop(can));
	zassert_equal(px4_can_send(0, &f, false), -ENETDOWN);

	zassert_ok(px4_can_get_status(0, &status));
	zassert_equal(status.state, PX4_CAN_STOPPED);
	zassert_equal(status.tx_errors, tx_errors + 1);
}

ZTEST(px4_can, test_close)
{
	const struct px4_can_frame f = frame(0x700, 1);

	zassert_ok(px4_can_close(0));
	zassert_ok(px4_can_close(0));
	atomic_clear(&events);

	zassert_equal(px4_can_send(0, &f, false), -ENETDOWN);
	zassert_equal(atomic_get(&events), 0);
}

ZTEST(px4_can, test_normal_mode)
{
	const struct px4_can_frame f = frame(0x600, 1);
	struct px4_can_rx_frame rx;
	struct px4_can_status status;

	/* the loopback controller drops the frames it sends in normal mode */
	open_bus(CAN_MODE_NORMAL);

	zassert_equal(px4_can_send(0, &f, true), 1);
	k_msleep(10);

	zassert_equal(px4_can_receive(0, &rx), 1);
	zassert_true(rx.loopback);
	zassert_equal(px4_can_receive(0, &rx), 0);

	zassert_ok(px4_can_get_status(0, &status));
	zassert_equal(status.state, PX4_CAN_ERROR_ACTIVE);
}

ZTEST_SUITE(px4_can, NULL, NULL, before, NULL, NULL);
