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

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/printload.h>
#include <px4_platform_common/tasks.h>
#include <px4_zephyr.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/task_stack_info.h>

#include <stdio.h>
#include <string.h>

#include <zephyr/ztest.h>

static constexpr int BUSY_STACK_SIZE = 2048;
static constexpr int MAX_LINES = CONFIG_FS_PROCFS_MAX_TASKS + 8;
static constexpr size_t LINE_LENGTH = 140;

static px4::atomic_bool busy{false};
static px4_task_t busy_task{-1};

// lines printed by print_load_buffer()
static struct {
	char buffer[LINE_LENGTH];
	char lines[MAX_LINES][LINE_LENGTH];
	int count;
} output;

struct thread_line_s {
	char name[32];
	unsigned cpu_ms;
	float cpu_percent;
	unsigned stack_used;
	unsigned stack_size;
	int priority;
	char state[8];
};

static int busy_main(int argc, char *argv[])
{
	while (busy.load()) {
	}

	return 0;
}

// a thread that takes all the CPU that other threads leave
static void start_busy_task()
{
	busy.store(true);
	busy_task = px4_task_spawn_cmd("busy", SCHED_DEFAULT, SCHED_PRIORITY_MIN, BUSY_STACK_SIZE, busy_main, nullptr);
	zassert_true(busy_task >= 0);
}

static void stop_busy_task()
{
	busy.store(false);
	zassert_ok(px4_task_join(busy_task));
}

static void add_line(void *user)
{
	if (output.count < MAX_LINES) {
		memcpy(output.lines[output.count], output.buffer, LINE_LENGTH);
		output.lines[output.count][LINE_LENGTH - 1] = '\0';
		output.count++;
	}
}

static void print(print_load_s *load)
{
	output.count = 0;
	print_load_buffer(output.buffer, sizeof(output.buffer), add_line, nullptr, load);
}

static bool find_thread(const char *name, thread_line_s *thread)
{
	for (int i = 0; i < output.count; i++) {
		if ((sscanf(output.lines[i], "%31s %u %f %u/%u %d %7s", thread->name, &thread->cpu_ms, &thread->cpu_percent,
			    &thread->stack_used, &thread->stack_size, &thread->priority, thread->state) == 7)
		    && (strcmp(thread->name, name) == 0)) {
			return true;
		}
	}

	return false;
}

static void cpu_usage(float *tasks, float *idle)
{
	for (int i = 0; i < output.count; i++) {
		if (sscanf(output.lines[i], "CPU usage: %f%% tasks, %f%% idle", tasks, idle) == 2) {
			return;
		}
	}

	ztest_test_fail();
}

ZTEST(px4_cpuload, test_print_load)
{
	print_load_s load{};
	thread_line_s thread{};
	float tasks = 0.f;
	float idle = 0.f;

	init_print_load(&load);
	start_busy_task();
	k_msleep(500);
	print(&load);
	stop_busy_task();

	zassert_true(find_thread("busy", &thread));
	zassert_true(thread.cpu_percent > 80.f, "busy thread at %.1f %%", (double)thread.cpu_percent);
	zassert_equal(thread.priority, K_LOWEST_APPLICATION_THREAD_PRIO);
	zassert_str_equal(thread.state, "READY");
	zassert_true(thread.stack_size <= BUSY_STACK_SIZE, "stack of %u bytes", thread.stack_size);
	zassert_true((thread.stack_used > 0) && (thread.stack_used < thread.stack_size), "%u bytes used", thread.stack_used);

	zassert_true(find_thread("idle", &thread));
	zassert_equal(thread.priority, K_IDLE_PRIO);
	zassert_true(thread.cpu_percent < 20.f, "idle at %.1f %%", (double)thread.cpu_percent);

	cpu_usage(&tasks, &idle);
	zassert_true(tasks > 80.f, "tasks at %.1f %%", (double)tasks);
	zassert_true(idle < 20.f, "idle at %.1f %%", (double)idle);

	k_msleep(500);
	print(&load);

	zassert_false(find_thread("busy", &thread));
	cpu_usage(&tasks, &idle);
	zassert_true(tasks < 20.f, "tasks at %.1f %%", (double)tasks);
	zassert_true(idle > 80.f, "idle at %.1f %%", (double)idle);
}

ZTEST(px4_cpuload, test_load_mon)
{
	char load_mon[] = "load_mon";
	char start[] = "start";
	char *start_argv[] = {load_mon, start};

	uORB::Subscription cpuload_sub{ORB_ID(cpuload)};
	uORB::Subscription task_stack_info_sub{ORB_ID(task_stack_info)};
	cpuload_s cpuload{};
	task_stack_info_s task_stack_info{};

	zassert_ok(px4_zephyr_run_command(2, start_argv));

	start_busy_task();
	k_msleep(1500);
	const bool busy_published = cpuload_sub.update(&cpuload);
	stop_busy_task();

	zassert_true(busy_published);
	zassert_true(cpuload.load > 0.8f, "load %.2f", (double)cpuload.load);
	zassert_true((cpuload.ram_usage > 0.f) && (cpuload.ram_usage < 1.f), "RAM usage %.2f", (double)cpuload.ram_usage);

	k_msleep(1500);
	zassert_true(cpuload_sub.update(&cpuload));
	zassert_true(cpuload.load < 0.2f, "load %.2f", (double)cpuload.load);

	zassert_true(task_stack_info_sub.update(&task_stack_info));
	zassert_true(task_stack_info.task_name[0] != '\0');
	zassert_true(task_stack_info.stack_free > 0);
}

// stop what a test started, also when it failed
static void cpuload_after(void *fixture)
{
	char load_mon[] = "load_mon";
	char stop[] = "stop";
	char *stop_argv[] = {load_mon, stop};

	if (busy.load()) {
		busy.store(false);
		px4_task_join(busy_task);
	}

	px4_zephyr_run_command(2, stop_argv);
}

ZTEST_SUITE(px4_cpuload, NULL, px4_test_setup, NULL, cpuload_after, NULL);
