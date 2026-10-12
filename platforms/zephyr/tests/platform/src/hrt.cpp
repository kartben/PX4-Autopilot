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

#include "px4_test.h"

#include <drivers/drv_hrt.h>

#include <zephyr/ztest.h>

struct callout_data {
	px4_sem_t sem;
	hrt_abstime time;
	bool in_isr;
	int count;
};

static void callout(void *arg)
{
	callout_data *data = static_cast<callout_data *>(arg);
	data->time = hrt_absolute_time();
	data->in_isr = k_is_in_isr();
	data->count++;
	px4_sem_post(&data->sem);
}

ZTEST(px4_hrt, test_absolute_time)
{
	const hrt_abstime start = hrt_absolute_time();
	k_sleep(K_MSEC(10));
	const hrt_abstime elapsed = hrt_elapsed_time(&start);

	zassert_true(elapsed >= 10000, "elapsed %llu us", elapsed);
	zassert_true(elapsed < 20000, "elapsed %llu us", elapsed);

	// hrt shares the kernel time base
	zassert_within(hrt_absolute_time() / 1000, k_uptime_get(), 1);
}

ZTEST(px4_hrt, test_usleep)
{
	hrt_abstime start = hrt_absolute_time();
	zassert_ok(px4_usleep(2000));
	zassert_true(hrt_elapsed_time(&start) >= 2000);

	// more than the second that usleep() takes
	start = hrt_absolute_time();
	zassert_ok(px4_usleep(1200000));
	zassert_true(hrt_elapsed_time(&start) >= 1200000, "slept %llu us", hrt_elapsed_time(&start));
}

ZTEST(px4_hrt, test_call_after)
{
	callout_data data{};
	px4_sem_init(&data.sem, 0, 0);
	struct hrt_call call {};

	const hrt_abstime start = hrt_absolute_time();
	hrt_call_after(&call, 5000, callout, &data);

	zassert_ok(px4_test_sem_wait(&data.sem, 100));
	zassert_true(data.in_isr);
	zassert_true(data.time - start >= 5000, "fired after %llu us", data.time - start);
	zassert_true(hrt_called(&call));

	px4_sem_destroy(&data.sem);
}

ZTEST(px4_hrt, test_call_every)
{
	callout_data data{};
	px4_sem_init(&data.sem, 0, 0);
	struct hrt_call call {};

	hrt_call_every(&call, 1000, 2000, callout, &data);
	k_sleep(K_MSEC(21));
	hrt_cancel(&call);
	const int count = data.count;

	zassert_within(count, 10, 1);

	k_sleep(K_MSEC(5));
	zassert_equal(data.count, count, "called after cancel");

	px4_sem_destroy(&data.sem);
}

ZTEST(px4_hrt, test_sem_timedwait)
{
	px4_sem_t sem;
	px4_sem_init(&sem, 0, 0);

	const hrt_abstime start = hrt_absolute_time();
	zassert_equal(px4_test_sem_wait(&sem, 10), -1);
	zassert_equal(errno, ETIMEDOUT);
	zassert_true(hrt_elapsed_time(&start) >= 10000);

	px4_sem_post(&sem);
	zassert_ok(px4_test_sem_wait(&sem, 10));

	px4_sem_destroy(&sem);
}

ZTEST_SUITE(px4_hrt, NULL, px4_test_setup, NULL, NULL, NULL);
