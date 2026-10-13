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

#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include <px4_platform_common/tasks.h>
#include <px4_sched.h>
#include <px4_zephyr.h>

#include <climits>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/ztest.h>

static struct {
	int argc;
	char name[16];
	int value;
	int priority;
	px4_task_t pid;
} result;

static int task_main(int argc, char *argv[])
{
	result.argc = argc;
	strncpy(result.name, argv[0], sizeof(result.name) - 1);
	result.value = (argc > 1) ? atoi(argv[1]) : -1;
	result.priority = k_thread_priority_get(k_current_get());
	result.pid = px4_getpid();
	return 0;
}

static int run_task(int priority, char *const argv[])
{
	const px4_task_t task = px4_task_spawn_cmd("test_task", SCHED_DEFAULT, priority, 2048, task_main, argv);
	zassert_true(task >= 0, "spawn failed (%d)", task);
	zassert_ok(px4_task_join(task));
	return task;
}

ZTEST(px4_tasks, test_spawn_join)
{
	char value[] = "42";
	char *const argv[] = {value, nullptr};

	memset(&result, 0, sizeof(result));
	const px4_task_t task = run_task(SCHED_PRIORITY_DEFAULT, argv);

	zassert_equal(result.argc, 2);
	zassert_str_equal(result.name, "test_task");
	zassert_equal(result.value, 42);
	zassert_equal(result.pid, task);
	zassert_false(px4_task_is_running("test_task"));
}

ZTEST(px4_tasks, test_priority)
{
	run_task(SCHED_PRIORITY_MAX, nullptr);
	const int high = result.priority;

	run_task(SCHED_PRIORITY_MIN, nullptr);
	const int low = result.priority;

	// preemptible threads, a lower Zephyr priority value runs first
	zassert_true(high >= 0);
	zassert_true(high < low, "high %d, low %d", high, low);
	zassert_true(low < K_IDLE_PRIO);
}

ZTEST(px4_tasks, test_priority_order)
{
	// PX4 priorities keep their order, and the work queues one Zephyr priority each
	for (int priority = SCHED_PRIORITY_MIN; priority < SCHED_PRIORITY_MAX; priority++) {
		const int posix = px4_zephyr_posix_priority(priority);
		const int next = px4_zephyr_posix_priority(priority + 1);

		zassert_true(posix <= next, "priority %d", priority);

		if (priority >= SCHED_PRIORITY_MAX - PX4_ZEPHYR_PRIORITY_BAND) {
			zassert_true(posix < next, "priority %d", priority);
			zassert_equal(px4_zephyr_px4_priority(next), priority + 1);
		}
	}

	zassert_equal(px4_zephyr_posix_priority(SCHED_PRIORITY_MIN), 0);
	zassert_equal(px4_zephyr_posix_priority(SCHED_PRIORITY_MAX), CONFIG_NUM_PREEMPT_PRIORITIES - 1);

	// the commander, at the default priority + 40, below the controllers and the estimator, as on NuttX
	const int commander = px4_zephyr_posix_priority(SCHED_PRIORITY_DEFAULT + 40);
	zassert_true(commander < px4_zephyr_posix_priority(SCHED_PRIORITY_MAX
			+ px4::wq_configurations::nav_and_controllers.relative_priority));
	zassert_true(commander < px4_zephyr_posix_priority(SCHED_PRIORITY_MAX
			+ px4::wq_configurations::INS0.relative_priority));
	zassert_true(commander < px4_zephyr_posix_priority(SCHED_PRIORITY_MAX
			+ px4::wq_configurations::lp_default.relative_priority));
}

static void *thread_main(void *arg)
{
	*(int *)arg = k_thread_priority_get(k_current_get());
	return nullptr;
}

static int pthread_priority(int policy, int priority)
{
	pthread_attr_t attr;
	zassert_ok(pthread_attr_init(&attr));
	zassert_ok(pthread_attr_setstacksize(&attr, 2048));
	zassert_ok(pthread_attr_setschedpolicy(&attr, policy));

	sched_param param{};
	param.sched_priority = priority;
	zassert_ok(pthread_attr_setschedparam(&attr, &param));

	sched_param read{};
	zassert_ok(pthread_attr_getschedparam(&attr, &read));
	zassert_equal(read.sched_priority, priority);

	int zephyr_priority = INT_MIN;
	pthread_t thread;
	zassert_ok(pthread_create(&thread, &attr, thread_main, &zephyr_priority));
	zassert_ok(pthread_join(thread, nullptr));
	pthread_attr_destroy(&attr);

	return zephyr_priority;
}

ZTEST(px4_tasks, test_pthread_priority)
{
	// threads that set a priority run at it, not at the priority of the thread that creates them
	const int priority = SCHED_PRIORITY_MAX - 10;
	const int expected = CONFIG_NUM_PREEMPT_PRIORITIES - 1 - px4_zephyr_posix_priority(priority);

	zassert_equal(pthread_priority(SCHED_RR, priority), expected);

	// SCHED_FIFO threads are preemptible, as on NuttX
	zassert_equal(pthread_priority(SCHED_FIFO, priority), expected);
}

ZTEST(px4_tasks, test_resources_released)
{
	for (int i = 0; i < 2 * CONFIG_POSIX_THREAD_THREADS_MAX; i++) {
		run_task(SCHED_PRIORITY_DEFAULT, nullptr);
	}

	// exited tasks that are not joined are released by the next spawn
	for (int i = 0; i < 2 * CONFIG_POSIX_THREAD_THREADS_MAX; i++) {
		const px4_task_t task = px4_task_spawn_cmd("test_task", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 2048, task_main,
					nullptr);
		zassert_true(task >= 0, "spawn %d failed (%d)", i, task);
		k_sleep(K_MSEC(1));
	}
}

ZTEST(px4_tasks, test_run_command)
{
	char ver[] = "ver";
	char unknown[] = "unknown";
	char *ver_argv[] = {ver};
	char *unknown_argv[] = {unknown};

	zassert_ok(px4_zephyr_run_command(1, ver_argv));
	zassert_equal(px4_zephyr_run_command(1, unknown_argv), -ENOENT);
}

ZTEST_SUITE(px4_tasks, NULL, px4_test_setup, NULL, NULL, NULL);
