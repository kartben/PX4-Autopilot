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


#include "test_uart.h"

#include <zephyr/drivers/serial/uart_emul.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

static const struct device *const uart = DEVICE_DT_GET(DT_NODELABEL(euart0));

static struct uart_config get_config(void)
{
	struct uart_config config;

	zassert_ok(uart_config_get(uart, &config));
	return config;
}

void test_uart_reset(void)
{
	const struct uart_config config = {
		.baudrate = 57600,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
	};

	uart_emul_flush_rx_data(uart);
	uart_emul_flush_tx_data(uart);
	zassert_ok(uart_configure(uart, &config));
}

void test_uart_receive(const void *data, size_t size)
{
	zassert_equal(uart_emul_put_rx_data(uart, data, size), size);
}

size_t test_uart_transmitted(void *data, size_t size)
{
	size_t len = 0;

	/* the emulated UART transmits from a work queue */
	for (int i = 0; (i < 100) && (len < size); i++) {
		k_msleep(1);
		len += uart_emul_get_tx_data(uart, (uint8_t *)data + len, size - len);
	}

	return len;
}

uint32_t test_uart_baudrate(void)
{
	return get_config().baudrate;
}

char test_uart_parity(void)
{
	switch (get_config().parity) {
	case UART_CFG_PARITY_ODD:
		return 'O';

	case UART_CFG_PARITY_EVEN:
		return 'E';

	default:
		return 'N';
	}
}

int test_uart_stop_bits(void)
{
	return (get_config().stop_bits == UART_CFG_STOP_BITS_2) ? 2 : 1;
}
