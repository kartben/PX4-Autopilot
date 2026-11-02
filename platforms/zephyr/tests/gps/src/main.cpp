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

#include <px4_zephyr.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_gnss.h>

#include <string.h>

#include <zephyr/ztest.h>

static constexpr int FIX_TIMEOUT_MS{3000};

static bool wait_for_fix(sensor_gnss_s &sensor_gnss)
{
	uORB::Subscription subscription{ORB_ID(sensor_gnss)};

	for (int i = 0; i < FIX_TIMEOUT_MS; i++) {
		if (subscription.update(&sensor_gnss) && (sensor_gnss.fix_type == sensor_gnss_s::FIX_TYPE_3D)) {
			return true;
		}

		k_sleep(K_MSEC(1));
	}

	return false;
}

static int run(const char *command)
{
	char buffer[64];
	strncpy(buffer, command, sizeof(buffer) - 1);
	buffer[sizeof(buffer) - 1] = '\0';

	char *argv[8] {};
	int argc = 0;
	char *saveptr = nullptr;

	for (char *token = strtok_r(buffer, " ", &saveptr); token != nullptr && argc < 8;
	     token = strtok_r(nullptr, " ", &saveptr)) {
		argv[argc++] = token;
	}

	return px4_zephyr_run_command(argc, argv);
}

static void *setup(void)
{
	zassert_ok(px4_zephyr_init());
	nmea_receiver_start();
	zassert_ok(run("gps start -d " NMEA_RECEIVER_TTY " -p nmea -b 115200"));

	return nullptr;
}

static void teardown(void *fixture)
{
	run("gps stop");
	nmea_receiver_stop();
}

ZTEST(px4_gps, test_fix)
{
	sensor_gnss_s sensor_gnss{};
	zassert_true(wait_for_fix(sensor_gnss));

	zassert_within(sensor_gnss.latitude, 47.2852331667, 1e-9);
	zassert_within(sensor_gnss.longitude, 8.565265, 1e-9);
	zassert_within(sensor_gnss.altitude_msl, 499.6, 1e-3);
	zassert_within(sensor_gnss.altitude_ellipsoid, 547.6, 1e-3);
	zassert_equal(sensor_gnss.satellites_used, 8);

	// the GST standard deviations of the latitude, longitude and altitude
	zassert_within(sensor_gnss.eph, 0.0305f, 1e-4f);
	zassert_within(sensor_gnss.epv, 0.031f, 1e-4f);
	zassert_within(sensor_gnss.hdop, 1.18f, 1e-4f);
	zassert_within(sensor_gnss.vdop, 1.54f, 1e-4f);

	// 10 knots to the east
	zassert_true(sensor_gnss.vel_ned_valid);
	zassert_within(sensor_gnss.vel_north, 0.f, 1e-3f);
	zassert_within(sensor_gnss.vel_east, 5.1444f, 1e-3f);
}

ZTEST(px4_gps, test_baudrate)
{
	sensor_gnss_s sensor_gnss{};
	zassert_true(wait_for_fix(sensor_gnss));

	zassert_equal(nmea_receiver_baudrate(), 115200);
}

ZTEST_SUITE(px4_gps, nullptr, setup, nullptr, nullptr, teardown);
