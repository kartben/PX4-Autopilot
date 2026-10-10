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
#include <px4_platform_common/tasks.h>
#include <px4_zephyr.h>

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
