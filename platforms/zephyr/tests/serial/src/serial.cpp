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
 * @file serial.cpp
 *
 * Tests of device::Serial on a serial port of the px4,serial-ports devicetree
 * node.
 */

#include "test_uart.h"

#include <px4_platform_common/Serial.hpp>

#include <zephyr/ztest.h>

static void before(void *fixture)
{
	ARG_UNUSED(fixture);

	test_uart_reset();
}

ZTEST_SUITE(px4_serial, NULL, NULL, before, NULL, NULL);

ZTEST(px4_serial, test_validate_port)
{
	zassert_true(device::Serial::validatePort(TEST_TTY));
	zassert_false(device::Serial::validatePort("/dev/ttyS9"));
}

ZTEST(px4_serial, test_configure)
{
	device::Serial serial(TEST_TTY, 115200, ByteSize::EightBits, Parity::Odd, StopBits::Two);

	zassert_true(serial.open());
	zassert_equal(test_uart_baudrate(), 115200);
	zassert_equal(test_uart_parity(), 'O');
	zassert_equal(test_uart_stop_bits(), 2);

	zassert_true(serial.setBaudrate(921600));
	zassert_equal(test_uart_baudrate(), 921600);

	zassert_false(serial.setFlowcontrol(FlowControl::Enabled), "the UART has no flow control");
}

ZTEST(px4_serial, test_write)
{
	device::Serial serial(TEST_TTY, 115200);
	uint8_t data[5];

	zassert_true(serial.open());
	zassert_equal(serial.write("hello", 5), 5);
	zassert_equal(test_uart_transmitted(data, sizeof(data)), 5);
	zassert_mem_equal(data, "hello", 5);
}

ZTEST(px4_serial, test_read_at_least)
{
	device::Serial serial(TEST_TTY, 115200);
	uint8_t data[16];

	zassert_true(serial.open());
	zassert_equal(serial.readAtLeast(data, sizeof(data), 6, 10), 0, "nothing received");

	test_uart_receive("abcdef", 6);
	zassert_equal(serial.readAtLeast(data, sizeof(data), 6, 100), 6);
	zassert_mem_equal(data, "abcdef", 6);
	zassert_equal(serial.bytesAvailable(), 0);
}
