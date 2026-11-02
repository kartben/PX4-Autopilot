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

#include "nmea_receiver.h"

#include <zephyr/device.h>
#include <zephyr/drivers/serial/uart_emul.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/ztest.h>

#define FIX_INTERVAL_MS 100

static const struct device *const uart = DEVICE_DT_GET(DT_NODELABEL(euart0));

/* tenths of a second since noon, the UTC time of the fixes, which must increase */
static uint32_t fix_time;

static void send_sentence(const char *body)
{
	char sentence[96];
	uint8_t checksum = 0;

	for (size_t i = 0; body[i] != '\0'; i++) {
		checksum ^= (uint8_t)body[i];
	}

	const int len = snprintk(sentence, sizeof(sentence), "$%s*%02X\r\n", body, checksum);

	uart_emul_put_rx_data(uart, (const uint8_t *)sentence, len);
}

/* a 3D fix with 8 satellites at 47.2852331667 N, 8.565265 E and 499.6 m above MSL, moving east
 * at 10 knots
 */
static void send_fix(struct k_timer *timer)
{
	const uint32_t seconds = 12U * 3600U + fix_time / 10U;
	char time[16];
	char body[80];

	snprintk(time, sizeof(time), "%02u%02u%02u.%u0", seconds / 3600U, (seconds / 60U) % 60U,
		 seconds % 60U, fix_time % 10U);
	fix_time++;

	snprintk(body, sizeof(body), "GPGGA,%s,4717.11399,N,00833.91590,E,1,08,1.18,499.6,M,48.0,M,,",
		 time);
	send_sentence(body);
	send_sentence("GPGSA,A,3,04,05,09,12,24,25,29,31,,,,,1.94,1.18,1.54");
	snprintk(body, sizeof(body), "GPGST,%s,0.006,0.023,0.020,273.6,0.023,0.020,0.031", time);
	send_sentence(body);
	snprintk(body, sizeof(body), "GPRMC,%s,A,4717.11399,N,00833.91590,E,10.0,90.0,121026,,,A",
		 time);
	send_sentence(body);
}

K_TIMER_DEFINE(fix_timer, send_fix, NULL);

void nmea_receiver_start(void)
{
	k_timer_start(&fix_timer, K_NO_WAIT, K_MSEC(FIX_INTERVAL_MS));
}

void nmea_receiver_stop(void)
{
	k_timer_stop(&fix_timer);
}

uint32_t nmea_receiver_baudrate(void)
{
	struct uart_config config;

	zassert_ok(uart_config_get(uart, &config));
	return config.baudrate;
}
