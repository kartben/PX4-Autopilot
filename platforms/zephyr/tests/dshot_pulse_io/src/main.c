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

/*
 * DShot device on pulse_io channels (px4,dshot-pulse-io), on pulse_io controllers of the test: the
 * configuration of the channels for each bit rate, the cells and symbols of the frames, the order of the
 * channels, and the errors.
 */

#include "fake_pulse_io.h"

#include <px4_dshot.h>

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/ztest.h>

static const struct device *const dshot = DEVICE_DT_GET(DT_NODELABEL(dshot0));
static const struct device *const cells_pio = DEVICE_DT_GET(DT_NODELABEL(cells_pio));
static const struct device *const symbols_pio = DEVICE_DT_GET(DT_NODELABEL(symbols_pio));
static const struct device *const levels_pio = DEVICE_DT_GET(DT_NODELABEL(levels_pio));

/* the pulse_io channels of the DShot channels, see app.overlay */
#define CH0 fake_pio_channel(cells_pio, 2)
#define CH1 fake_pio_channel(symbols_pio, 1)
#define CH2 fake_pio_channel(cells_pio, 0)
#define CH3 fake_pio_channel(levels_pio, 0)

/* ticks of a bit, high time of a 1 and of a 0 */
#define BIT_TICKS  40
#define ONE_TICKS  30
#define ZERO_TICKS 15

static void reset(void *fixture)
{
	ARG_UNUSED(fixture);

	zassert_ok(px4_dshot_configure(dshot, 0, 0, 600000));
	fake_pio_clear();
}

ZTEST_SUITE(dshot_pulse_io, NULL, NULL, reset, NULL, NULL);

static void check_channel_config(const struct fake_pio_channel *ch, enum pulse_io_mode mode, uint32_t bit_rate)
{
	zassert_true(ch->reserved);
	zassert_true(ch->configured);
	zassert_equal(ch->cfg.mode, mode);
	zassert_equal(ch->cfg.dir, PULSE_IO_DIR_TX);
	zassert_equal(ch->cfg.resolution_hz, bit_rate * BIT_TICKS);
	zassert_false(ch->cfg.idle_high);

	if (mode == PULSE_IO_MODE_CELL) {
		zassert_equal(ch->cfg.cell_period_ticks, BIT_TICKS);
	}
}

ZTEST(dshot_pulse_io, test_configure)
{
	zassert_ok(px4_dshot_configure(dshot, BIT(0) | BIT(1) | BIT(2), 0, 600000));

	check_channel_config(CH0, PULSE_IO_MODE_CELL, 600000);
	check_channel_config(CH1, PULSE_IO_MODE_SYMBOL, 600000);
	check_channel_config(CH2, PULSE_IO_MODE_CELL, 600000);

	/* the channels that aren't DShot channels, or aren't configured, are left alone */
	zassert_false(fake_pio_channel(cells_pio, 1)->reserved);
	zassert_false(fake_pio_channel(cells_pio, 3)->reserved);
	zassert_false(fake_pio_channel(symbols_pio, 0)->reserved);
	zassert_false(CH3->reserved);
}

ZTEST(dshot_pulse_io, test_bit_rates)
{
	zassert_ok(px4_dshot_configure(dshot, BIT(0) | BIT(1), 0, 150000));
	check_channel_config(CH0, PULSE_IO_MODE_CELL, 150000);
	check_channel_config(CH1, PULSE_IO_MODE_SYMBOL, 150000);

	zassert_ok(px4_dshot_configure(dshot, BIT(0) | BIT(1), 0, 300000));
	check_channel_config(CH0, PULSE_IO_MODE_CELL, 300000);
	check_channel_config(CH1, PULSE_IO_MODE_SYMBOL, 300000);
}

ZTEST(dshot_pulse_io, test_reconfigure)
{
	zassert_ok(px4_dshot_configure(dshot, BIT(0) | BIT(2), 0, 600000));

	/* in place of the previous configuration */
	zassert_ok(px4_dshot_configure(dshot, BIT(0), 0, 600000));
	check_channel_config(CH0, PULSE_IO_MODE_CELL, 600000);
	zassert_false(CH2->reserved);

	zassert_equal(px4_dshot_send(dshot, BIT(2), (const uint16_t[3]) {0}), -EINVAL);
	zassert_equal(CH2->transmits, 0);
}

ZTEST(dshot_pulse_io, test_configure_errors)
{
	zassert_ok(px4_dshot_configure(dshot, BIT(0), 0, 600000));

	/* no reply is received */
	zassert_equal(px4_dshot_configure(dshot, BIT(0), BIT(0), 600000), -EINVAL);

	zassert_equal(px4_dshot_configure(dshot, BIT(0), 0, 400000), -EINVAL);
	zassert_equal(px4_dshot_configure(dshot, BIT(4), 0, 600000), -EINVAL);

	/* cells without a duty cycle, which leaves no channel configured */
	zassert_ok(px4_dshot_configure(dshot, BIT(0), 0, 600000));
	zassert_equal(px4_dshot_configure(dshot, BIT(0) | BIT(3), 0, 600000), -EINVAL);
	zassert_false(CH0->reserved);
	zassert_false(CH3->reserved);
	zassert_equal(px4_dshot_send(dshot, BIT(0), (const uint16_t[1]) {0}), -EINVAL);
}

ZTEST(dshot_pulse_io, test_frames)
{
	/* bits 1010 0101 1100 0011, the most significant first */
	const uint16_t frame = 0xa5c3;
	const uint16_t frames[3] = {frame, frame, frame};

	zassert_ok(px4_dshot_configure(dshot, BIT(0) | BIT(1) | BIT(2), 0, 600000));
	zassert_ok(px4_dshot_send(dshot, BIT(0) | BIT(1) | BIT(2), frames));

	/* 16 cells and an idle cell */
	for (int c = 0; c < 2; c++) {
		const struct fake_pio_channel *ch = (c == 0) ? CH0 : CH2;

		zassert_equal(ch->transmits, 1);
		zassert_equal(ch->count, 17);

		for (int i = 0; i < 16; i++) {
			const bool one = (frame & BIT(15 - i)) != 0;

			zassert_equal(ch->cells[i].duty, one ? ONE_TICKS : ZERO_TICKS, "cell %d", i);
		}

		zassert_equal(ch->cells[16].duty, 0);
	}

	/* a high and a low symbol for each bit */
	zassert_equal(CH1->transmits, 1);
	zassert_equal(CH1->count, 32);

	for (int i = 0; i < 16; i++) {
		const bool one = (frame & BIT(15 - i)) != 0;

		zassert_equal(CH1->symbols[2 * i].level, 1);
		zassert_equal(CH1->symbols[2 * i].duration, one ? ONE_TICKS : ZERO_TICKS, "bit %d", i);
		zassert_equal(CH1->symbols[2 * i + 1].level, 0);
		zassert_equal(CH1->symbols[2 * i + 1].duration, one ? BIT_TICKS - ONE_TICKS : BIT_TICKS - ZERO_TICKS);
	}
}

ZTEST(dshot_pulse_io, test_channels)
{
	const uint16_t frames[3] = {0x0001, 0x0002, 0x0004};

	zassert_ok(px4_dshot_configure(dshot, BIT(0) | BIT(1) | BIT(2), 0, 600000));

	/* the given channels, each with its frame, one after the other in the order of the channels */
	zassert_ok(px4_dshot_send(dshot, BIT(0) | BIT(2), frames));
	zassert_equal(CH0->transmits, 1);
	zassert_equal(CH1->transmits, 0);
	zassert_equal(CH2->transmits, 1);
	zassert_equal(CH0->cells[15].duty, ONE_TICKS);
	zassert_equal(CH2->cells[13].duty, ONE_TICKS);
	zassert_equal(CH2->cells[15].duty, ZERO_TICKS);
	zassert_true(CH0->sequence < CH2->sequence);

	zassert_ok(px4_dshot_send(dshot, BIT(0) | BIT(1) | BIT(2), frames));
	zassert_true(CH0->sequence < CH1->sequence);
	zassert_true(CH1->sequence < CH2->sequence);
}

ZTEST(dshot_pulse_io, test_transmit_error)
{
	const uint16_t frames[3] = {0};

	zassert_ok(px4_dshot_configure(dshot, BIT(0) | BIT(1) | BIT(2), 0, 600000));

	/* the other channels still send */
	CH1->transmit_error = -EIO;
	zassert_equal(px4_dshot_send(dshot, BIT(0) | BIT(1) | BIT(2), frames), -EIO);
	zassert_equal(CH0->transmits, 1);
	zassert_equal(CH2->transmits, 1);
}

ZTEST(dshot_pulse_io, test_no_reply)
{
	uint32_t reply;

	zassert_ok(px4_dshot_configure(dshot, BIT(0), 0, 600000));
	zassert_ok(px4_dshot_send(dshot, BIT(0), (const uint16_t[1]) {0}));
	zassert_equal(px4_dshot_get_reply(dshot, 0, &reply), -ENODATA);
}
