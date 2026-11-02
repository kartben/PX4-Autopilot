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
 * @file tty.c
 *
 * Tests of the serial ports of the px4,serial-ports devicetree node, on an
 * emulated UART.
 */

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include "test_uart.h"

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#define TTY TEST_TTY

static int fd = -1;

static void before(void *fixture)
{
	ARG_UNUSED(fixture);

	test_uart_reset();

	fd = open(TTY, O_RDWR | O_NOCTTY);
	zassert_true(fd >= 0, "open failed: %d", errno);
}

static void after(void *fixture)
{
	ARG_UNUSED(fixture);

	close(fd);
	fd = -1;
}

ZTEST_SUITE(px4_tty, NULL, NULL, before, after, NULL);

ZTEST(px4_tty, test_open)
{
	zassert_equal(open(TTY, O_RDWR), -1, "opened twice");
	zassert_equal(errno, EBUSY);
	zassert_equal(open("/dev/ttyS9", O_RDWR), -1);

	zassert_ok(access(TTY, R_OK | W_OK));
	zassert_not_ok(access("/dev/ttyS9", R_OK));
}

ZTEST(px4_tty, test_write)
{
	uint8_t data[5];

	zassert_equal(write(fd, "hello", 5), 5);
	zassert_equal(test_uart_transmitted(data, sizeof(data)), 5);
	zassert_mem_equal(data, "hello", 5);
}

ZTEST(px4_tty, test_read)
{
	struct pollfd pfd = {.fd = fd, .events = POLLIN};
	uint8_t data[8];
	int available = 0;

	zassert_equal(poll(&pfd, 1, 20), 0, "nothing to read");

	test_uart_receive("abc", 3);
	zassert_equal(poll(&pfd, 1, 100), 1);
	zassert_true(pfd.revents & POLLIN);

	zassert_ok(ioctl(fd, FIONREAD, &available));
	zassert_equal(available, 3);
	zassert_equal(read(fd, data, sizeof(data)), 3);
	zassert_mem_equal(data, "abc", 3);
}

ZTEST(px4_tty, test_nonblocking_read)
{
	uint8_t data;

	zassert_ok(fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK));
	zassert_true(fcntl(fd, F_GETFL) & O_NONBLOCK);

	zassert_equal(read(fd, &data, 1), -1);
	zassert_equal(errno, EAGAIN);
}

ZTEST(px4_tty, test_overflow)
{
	static const char data[100];
	int available = 0;

	test_uart_receive(data, sizeof(data));
	k_msleep(20);

	zassert_ok(ioctl(fd, FIONREAD, &available));
	zassert_equal(available, 64, "the receive buffer holds 64 bytes");
}

ZTEST(px4_tty, test_flush)
{
	int available = -1;

	test_uart_receive("abc", 3);
	k_msleep(20);

	zassert_ok(tcflush(fd, TCIFLUSH));
	zassert_ok(ioctl(fd, FIONREAD, &available));
	zassert_equal(available, 0);

	zassert_equal(tcflush(fd, 42), -1);
	zassert_equal(errno, EINVAL);
}

ZTEST(px4_tty, test_termios)
{
	struct termios termios;

	zassert_ok(tcgetattr(fd, &termios));
	zassert_equal(cfgetospeed(&termios), B57600);

	zassert_ok(cfsetispeed(&termios, B921600));
	zassert_ok(cfsetospeed(&termios, B921600));
	termios.c_cflag |= PARENB | CSTOPB;
	zassert_ok(tcsetattr(fd, TCSANOW, &termios));

	zassert_equal(test_uart_baudrate(), 921600);
	zassert_equal(test_uart_parity(), 'E');
	zassert_equal(test_uart_stop_bits(), 2);

	termios.c_cflag |= CRTSCTS;
	zassert_equal(tcsetattr(fd, TCSANOW, &termios), -1, "the UART has no flow control");
	zassert_equal(errno, EINVAL);

	/* standard input */
	zassert_equal(tcgetattr(0, &termios), -1);
	zassert_equal(errno, ENOTTY);
}

K_THREAD_STACK_DEFINE(reader_stack, 2048);
static struct k_thread reader_thread;

static void reader(void *data, void *len, void *p3)
{
	ARG_UNUSED(p3);

	*(ssize_t *)len = read(fd, data, 8);
}

ZTEST(px4_tty, test_write_while_reading)
{
	uint8_t data[8];
	ssize_t len = 0;

	k_thread_create(&reader_thread, reader_stack, K_THREAD_STACK_SIZEOF(reader_stack), reader,
			data, &len, NULL, K_PRIO_PREEMPT(1), 0, K_NO_WAIT);
	k_msleep(10);

	const int64_t start = k_uptime_get();

	zassert_equal(write(fd, "out", 3), 3);
	zassert_true(k_uptime_get() - start < 5, "the write waited for the read");

	test_uart_receive("in", 2);
	zassert_ok(k_thread_join(&reader_thread, K_MSEC(100)));
	zassert_equal(len, 2);
	zassert_mem_equal(data, "in", 2);
}
