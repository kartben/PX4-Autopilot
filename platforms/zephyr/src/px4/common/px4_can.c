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
 * @file px4_can.c
 *
 * CAN buses of the px4,can-buses devicetree node, on the Zephyr CAN API.
 *
 * A single filter puts the received frames with a 29-bit identifier, the
 * frames of DroneCAN and Cyphal, into a queue. Frames are sent from a few
 * slots: a frame is only accepted while it has a higher priority than the
 * frames in the slots, so that frames leave in order of priority whichever
 * mailbox the controller sends first. A controller in loopback mode, as in
 * tests, also receives the frames it sends: those are dropped.
 */

#include <px4_can.h>

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#define PX4_CAN_BUSES DT_COMPAT_GET_ANY_STATUS_OKAY(px4_can_buses)

/* frames sent that a controller in loopback mode receives back */
#define PX4_CAN_ECHOES (2 * CONFIG_PX4_CAN_TX_SLOTS)

/* user data of the transmit callback: the bus, the slot, and the generation of the slots */
#define PX4_CAN_TX_TAG(bus, slot, generation)                                                      \
	((void *)(uintptr_t)((((generation) & 0xffffU) << 16) | ((bus) << 8) | (slot)))

struct px4_can_slot {
	struct px4_can_frame frame;
	bool used;
	bool loopback;
};

struct px4_can_bus {
	const struct device *dev;
	struct k_spinlock lock;

	px4_can_event_t event;
	void *event_arg;
	bool loopback_mode;
	int filter_id;
	bool state_callback_added;
	struct can_state_change_callback state_callback;

	/* the slots of a previous px4_can_open() call are those of an older generation */
	uint32_t generation;
	struct px4_can_slot tx[CONFIG_PX4_CAN_TX_SLOTS];
	/* the controller has no free mailbox */
	bool tx_full;

	struct px4_can_slot echoes[PX4_CAN_ECHOES];
	unsigned int next_echo;

	struct px4_can_rx_frame rx[CONFIG_PX4_CAN_RX_QUEUE_SIZE];
	unsigned int rx_head;
	unsigned int rx_count;

	uint32_t tx_errors;
	uint32_t rx_overruns;
	uint32_t bus_errors;
};

#define PX4_CAN_BUS(node) {.dev = DEVICE_DT_GET(DT_PHANDLE(node, can)), .filter_id = -1},

static struct px4_can_bus px4_can_buses[] = {DT_FOREACH_CHILD_STATUS_OKAY(PX4_CAN_BUSES, PX4_CAN_BUS)};

BUILD_ASSERT(ARRAY_SIZE(px4_can_buses) > 0, "px4,can-buses has no CAN bus");
BUILD_ASSERT(ARRAY_SIZE(px4_can_buses) <= 0xff && CONFIG_PX4_CAN_TX_SLOTS <= 0xff);

static struct px4_can_bus *px4_can_get_bus(unsigned bus)
{
	return (bus < ARRAY_SIZE(px4_can_buses)) ? &px4_can_buses[bus] : NULL;
}

/* true if a wins the arbitration against b */
static bool px4_can_higher_priority(const struct px4_can_frame *a, const struct px4_can_frame *b)
{
	const bool a_ext = (a->flags & PX4_CAN_FRAME_EXT) != 0;
	const bool b_ext = (b->flags & PX4_CAN_FRAME_EXT) != 0;

	if (a_ext != b_ext) {
		/* a standard identifier is sent as the first 11 bits of an extended one */
		const uint32_t a_base = a_ext ? (a->id >> 18) : a->id;
		const uint32_t b_base = b_ext ? (b->id >> 18) : b->id;

		if (a_base != b_base) {
			return a_base < b_base;
		}

		return !a_ext;
	}

	return a->id < b->id;
}

static bool px4_can_same_frame(const struct px4_can_frame *a, const struct px4_can_frame *b)
{
	return (a->id == b->id) && (a->flags == b->flags) && (a->len == b->len) &&
	       (memcmp(a->data, b->data, a->len) == 0);
}

static void px4_can_signal(struct px4_can_bus *bus)
{
	px4_can_event_t event = bus->event;

	if (event != NULL) {
		event(bus->event_arg);
	}
}

/* called with the lock held */
static int px4_can_free_slot(struct px4_can_bus *bus, const struct px4_can_frame *frame)
{
	int free_slot = -1;

	if (bus->tx_full) {
		return -1;
	}

	for (int i = 0; i < CONFIG_PX4_CAN_TX_SLOTS; i++) {
		if (!bus->tx[i].used) {
			if (free_slot < 0) {
				free_slot = i;
			}

		} else if (!px4_can_higher_priority(frame, &bus->tx[i].frame)) {
			return -1;
		}
	}

	return free_slot;
}

/* called with the lock held */
static void px4_can_queue(struct px4_can_bus *bus, const struct px4_can_frame *frame, bool loopback)
{
	if (bus->rx_count == CONFIG_PX4_CAN_RX_QUEUE_SIZE) {
		bus->rx_overruns++;
		return;
	}

	struct px4_can_rx_frame *rx = &bus->rx[(bus->rx_head + bus->rx_count) % CONFIG_PX4_CAN_RX_QUEUE_SIZE];

	rx->frame = *frame;

	rx->timestamp_us = k_ticks_to_us_floor64(k_uptime_ticks());

	rx->loopback = loopback;

	bus->rx_count++;
}

/* called with the lock held: drop a frame of this node received back */
static bool px4_can_take_echo(struct px4_can_bus *bus, const struct px4_can_frame *frame)
{
	for (int i = 0; i < PX4_CAN_ECHOES; i++) {
		if (bus->echoes[i].used && px4_can_same_frame(&bus->echoes[i].frame, frame)) {
			bus->echoes[i].used = false;
			return true;
		}
	}

	return false;
}

static void px4_can_received(const struct device *dev, struct can_frame *can_frame, void *user_data)
{
	struct px4_can_bus *bus = user_data;
	struct px4_can_frame frame = {
		.id = can_frame->id,
		.flags = PX4_CAN_FRAME_EXT,
		.len = MIN(can_dlc_to_bytes(can_frame->dlc), PX4_CAN_MAX_DLEN),
	};

	if ((can_frame->flags & CAN_FRAME_FDF) != 0) {
		frame.flags |= PX4_CAN_FRAME_FD;
	}

	if ((can_frame->flags & CAN_FRAME_BRS) != 0) {
		frame.flags |= PX4_CAN_FRAME_BRS;
	}

	memcpy(frame.data, can_frame->data, frame.len);

	k_spinlock_key_t key = k_spin_lock(&bus->lock);

	if (!bus->loopback_mode || !px4_can_take_echo(bus, &frame)) {
		px4_can_queue(bus, &frame, false);
	}

	k_spin_unlock(&bus->lock, key);

	px4_can_signal(bus);
}

static void px4_can_sent(const struct device *dev, int error, void *user_data)
{
	const uintptr_t tag = (uintptr_t)user_data;
	struct px4_can_bus *bus = &px4_can_buses[(tag >> 8) & 0xffU];
	struct px4_can_slot *slot = &bus->tx[tag & 0xffU];

	k_spinlock_key_t key = k_spin_lock(&bus->lock);

	if (((bus->generation & 0xffffU) == (tag >> 16)) && slot->used) {
		slot->used = false;
		bus->tx_full = false;

		if (error != 0) {
			bus->tx_errors++;

		} else if (slot->loopback) {
			px4_can_queue(bus, &slot->frame, true);
		}
	}

	k_spin_unlock(&bus->lock, key);

	px4_can_signal(bus);
}

static void px4_can_state_changed(const struct device *dev, struct can_state_change_callback *callback,
				  enum can_state state, struct can_bus_err_cnt err_cnt)
{
	struct px4_can_bus *bus = CONTAINER_OF(callback, struct px4_can_bus, state_callback);

	if ((state == CAN_STATE_ERROR_PASSIVE) || (state == CAN_STATE_BUS_OFF)) {
		k_spinlock_key_t key = k_spin_lock(&bus->lock);
		bus->bus_errors++;
		k_spin_unlock(&bus->lock, key);
	}
}

unsigned px4_can_bus_count(void)
{
	return ARRAY_SIZE(px4_can_buses);
}

int px4_can_open(unsigned bus_index, uint32_t bitrate, px4_can_event_t event, void *arg)
{
	struct px4_can_bus *bus = px4_can_get_bus(bus_index);

	if ((bus == NULL) || !device_is_ready(bus->dev)) {
		return -ENODEV;
	}

	int ret = can_stop(bus->dev);

	if ((ret != 0) && (ret != -EALREADY)) {
		return ret;
	}

	k_spinlock_key_t key = k_spin_lock(&bus->lock);

	/* the echoes are kept: a frame being sent may still be received back */
	bus->generation++;
	memset(bus->tx, 0, sizeof(bus->tx));
	bus->tx_full = false;
	bus->rx_head = 0;
	bus->rx_count = 0;
	bus->event = event;
	bus->event_arg = arg;
	bus->loopback_mode = (can_get_mode(bus->dev) & CAN_MODE_LOOPBACK) != 0;

	k_spin_unlock(&bus->lock, key);

	ret = can_set_bitrate(bus->dev, bitrate);

	if (ret != 0) {
		return ret;
	}

	if (bus->filter_id < 0) {
		const struct can_filter filter = {.flags = CAN_FILTER_IDE};

		ret = can_add_rx_filter(bus->dev, px4_can_received, bus, &filter);

		if (ret < 0) {
			return ret;
		}

		bus->filter_id = ret;
	}

	if (!bus->state_callback_added) {
		can_init_state_change_callback(&bus->state_callback, px4_can_state_changed);
		bus->state_callback_added = can_add_state_change_callback(bus->dev, &bus->state_callback) == 0;
	}

	return can_start(bus->dev);
}

bool px4_can_tx_ready(unsigned bus_index, const struct px4_can_frame *frame)
{
	struct px4_can_bus *bus = px4_can_get_bus(bus_index);

	if (bus == NULL) {
		return false;
	}

	k_spinlock_key_t key = k_spin_lock(&bus->lock);
	const bool ready = px4_can_free_slot(bus, frame) >= 0;
	k_spin_unlock(&bus->lock, key);

	return ready;
}

int px4_can_send(unsigned bus_index, const struct px4_can_frame *frame, bool loopback)
{
	struct px4_can_bus *bus = px4_can_get_bus(bus_index);

	if (bus == NULL) {
		return -ENODEV;
	}

	struct can_frame can_frame = {
		.id = frame->id,
		.dlc = can_bytes_to_dlc(frame->len),
	};

	if ((frame->len > PX4_CAN_MAX_DLEN) || (can_dlc_to_bytes(can_frame.dlc) != frame->len)) {
		return -EINVAL;
	}

	if ((frame->flags & PX4_CAN_FRAME_EXT) != 0) {
		can_frame.flags |= CAN_FRAME_IDE;
	}

	if ((frame->flags & PX4_CAN_FRAME_FD) != 0) {
		can_frame.flags |= CAN_FRAME_FDF;
	}

	if ((frame->flags & PX4_CAN_FRAME_BRS) != 0) {
		can_frame.flags |= CAN_FRAME_BRS;
	}

	memcpy(can_frame.data, frame->data, frame->len);

	k_spinlock_key_t key = k_spin_lock(&bus->lock);

	const int slot = px4_can_free_slot(bus, frame);

	if (slot < 0) {
		k_spin_unlock(&bus->lock, key);
		return 0;
	}

	bus->tx[slot].frame = *frame;
	bus->tx[slot].used = true;
	bus->tx[slot].loopback = loopback;

	int echo = -1;

	if (bus->loopback_mode) {
		echo = bus->next_echo;
		bus->echoes[echo].frame = *frame;
		bus->echoes[echo].used = true;
		bus->next_echo = (bus->next_echo + 1) % PX4_CAN_ECHOES;
	}

	const uint32_t generation = bus->generation;

	k_spin_unlock(&bus->lock, key);

	/* the transmit callback can run before can_send() returns */
	int ret = can_send(bus->dev, &can_frame, K_NO_WAIT, px4_can_sent,
			   PX4_CAN_TX_TAG(bus_index, slot, generation));

	if (ret == 0) {
		return 1;
	}

	key = k_spin_lock(&bus->lock);

	if (bus->generation == generation) {
		bus->tx[slot].used = false;

		if (echo >= 0) {
			bus->echoes[echo].used = false;
		}

		if (ret == -EAGAIN) {
			/* until a frame in a mailbox is sent */
			for (int i = 0; i < CONFIG_PX4_CAN_TX_SLOTS; i++) {
				bus->tx_full |= bus->tx[i].used;
			}

		} else {
			bus->tx_errors++;
		}
	}

	k_spin_unlock(&bus->lock, key);

	return (ret == -EAGAIN) ? 0 : ret;
}

int px4_can_receive(unsigned bus_index, struct px4_can_rx_frame *frame)
{
	struct px4_can_bus *bus = px4_can_get_bus(bus_index);
	int received = 0;

	if (bus == NULL) {
		return 0;
	}

	k_spinlock_key_t key = k_spin_lock(&bus->lock);

	if (bus->rx_count > 0) {
		*frame = bus->rx[bus->rx_head];
		bus->rx_head = (bus->rx_head + 1) % CONFIG_PX4_CAN_RX_QUEUE_SIZE;
		bus->rx_count--;
		received = 1;
	}

	k_spin_unlock(&bus->lock, key);

	return received;
}

bool px4_can_rx_pending(unsigned bus_index)
{
	struct px4_can_bus *bus = px4_can_get_bus(bus_index);

	return (bus != NULL) && (bus->rx_count > 0);
}

int px4_can_get_status(unsigned bus_index, struct px4_can_status *status)
{
	struct px4_can_bus *bus = px4_can_get_bus(bus_index);

	if (bus == NULL) {
		return -ENODEV;
	}

	enum can_state state;
	struct can_bus_err_cnt err_cnt;
	int ret = can_get_state(bus->dev, &state, &err_cnt);

	if (ret != 0) {
		return ret;
	}

	switch (state) {
	case CAN_STATE_ERROR_ACTIVE:
		status->state = PX4_CAN_ERROR_ACTIVE;
		break;

	case CAN_STATE_ERROR_WARNING:
		status->state = PX4_CAN_ERROR_WARNING;
		break;

	case CAN_STATE_ERROR_PASSIVE:
		status->state = PX4_CAN_ERROR_PASSIVE;
		break;

	case CAN_STATE_BUS_OFF:
		status->state = PX4_CAN_BUS_OFF;
		break;

	default:
		status->state = PX4_CAN_STOPPED;
		break;
	}

	status->tx_error_counter = err_cnt.tx_err_cnt;
	status->rx_error_counter = err_cnt.rx_err_cnt;

	k_spinlock_key_t key = k_spin_lock(&bus->lock);
	status->tx_errors = bus->tx_errors;
	status->rx_overruns = bus->rx_overruns;
	status->bus_errors = bus->bus_errors;
	k_spin_unlock(&bus->lock, key);

	return 0;
}
