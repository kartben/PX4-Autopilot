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
 * @file print_load.cpp
 *
 * CPU load, priority, state and stack usage of the threads, from the thread
 * runtime statistics of Zephyr.
 */

#include <px4_platform/cpuload.h>
#include <px4_platform_common/printload.h>
#include <drivers/drv_hrt.h>

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <zephyr/kernel.h>

#define CL "\033[K" // clear line

static constexpr int MAX_THREADS = CONFIG_FS_PROCFS_MAX_TASKS;
static constexpr int NAME_WIDTH = 24;

struct runtimes_s {
	const void **threads;
	uint64_t *times;
	int count;
};

struct find_thread_s {
	const void *thread;
	bool found;
};

struct thread_info_s {
	char name[CONFIG_THREAD_MAX_NAME_LEN];
	int priority;
	const char *state;
	bool running;
	size_t stack_size;
	size_t stack_used;
};

static uint64_t runtime_cycles(const struct k_thread *thread)
{
#if defined(CONFIG_SCHED_THREAD_USAGE)
	k_thread_runtime_stats_t stats;
	k_thread_runtime_stats_get((k_tid_t)thread, &stats);
	return stats.execution_cycles;
#else
	return 0;
#endif
}

static void add_runtime(const struct k_thread *thread, void *user_data)
{
	runtimes_s *runtimes = (runtimes_s *)user_data;

	if (runtimes->count < MAX_THREADS) {
		runtimes->threads[runtimes->count] = thread;
		runtimes->times[runtimes->count] = runtime_cycles(thread);
	}

	runtimes->count++;
}

// Runtime in us of all the threads at the same time, newest thread first. Returns the number of threads.
static int get_runtimes(const void **threads, uint64_t *times, hrt_abstime *now)
{
	runtimes_s runtimes{threads, times, 0};

	// no thread is created or exits while the scheduler is locked
	k_sched_lock();
	*now = hrt_absolute_time();
	k_thread_foreach_unlocked(add_runtime, &runtimes);
	k_sched_unlock();

	for (int i = 0; (i < runtimes.count) && (i < MAX_THREADS); i++) {
		times[i] = k_cyc_to_us_floor64(times[i]);
	}

	for (int i = runtimes.count; i < MAX_THREADS; i++) {
		threads[i] = nullptr;
		times[i] = 0;
	}

	return runtimes.count;
}

static void find_thread(const struct k_thread *thread, void *user_data)
{
	find_thread_s *find = (find_thread_s *)user_data;

	if (thread == find->thread) {
		find->found = true;
	}
}

static const char *state_name(const struct k_thread *thread, bool *running)
{
	const uint8_t state = thread->base.thread_state;

	*running = false;

	if (state & _THREAD_SUSPENDED) {
		return "SUSP";

	} else if (state & _THREAD_PENDING) {
		return "WAIT";

	} else if (state & _THREAD_SLEEPING) {
		return "SLEEP";
	}

	*running = true;
	return (thread == k_current_get()) ? "RUN" : "READY";
}

// false if the thread has exited
static bool get_info(const void *thread_id, thread_info_s *info)
{
	find_thread_s find{thread_id, false};

	k_sched_lock();
	k_thread_foreach_unlocked(find_thread, &find);

	if (find.found) {
		const struct k_thread *thread = (const struct k_thread *)thread_id;
		const char *name = k_thread_name_get((k_tid_t)thread);

		if (name[0] != '\0') {
			strncpy(info->name, name, sizeof(info->name) - 1);
			info->name[sizeof(info->name) - 1] = '\0';

		} else {
			snprintf(info->name, sizeof(info->name), "%p", thread_id);
		}

		info->priority = k_thread_priority_get((k_tid_t)thread);
		info->state = state_name(thread, &info->running);
		info->stack_size = thread->stack_info.size;
		info->stack_used = 0;

#if defined(CONFIG_INIT_STACKS)
		size_t unused = 0;

		if (k_thread_stack_space_get(thread, &unused) == 0) {
			info->stack_used = info->stack_size - unused;
		}

#endif
	}

	k_sched_unlock();

	return find.found;
}

void init_print_load(struct print_load_s *s)
{
	cpuload_monitor_start();

	s->total_user_time = 0;

	s->running_count = 0;
	s->blocked_count = 0;

	get_runtimes(s->last_threads, s->last_times, &s->new_time);
	s->interval_start_time = s->new_time;

	s->interval_time_us = 0.f;
}

void print_load_buffer(char *buffer, int buffer_length, print_load_callback_f cb, void *user,
		       struct print_load_s *print_state)
{
	const void **threads = print_state->threads;
	uint64_t *times = print_state->times;
	const int thread_count = get_runtimes(threads, times, &print_state->new_time);

	if (print_state->new_time > print_state->interval_start_time) {
		print_state->interval_time_us = print_state->new_time - print_state->interval_start_time;

		snprintf(buffer, buffer_length, "%-*s %8s %6s %11s %4s %s",
			 NAME_WIDTH, "COMMAND",
			 "CPU(ms)",
			 "CPU(%)",
			 "USED/STACK",
			 "PRIO",
			 "STATE");
		cb(user);
	}

	print_state->running_count = 0;
	print_state->blocked_count = 0;
	print_state->total_user_time = 0;

	float idle_load = 0.f;
	uint64_t idle_time = 0;

	// oldest thread first
	for (int i = MAX_THREADS - 1; i >= 0; i--) {
		thread_info_s info;

		if ((threads[i] == nullptr) || !get_info(threads[i], &info)) {
			continue;
		}

		if (info.running) {
			print_state->running_count++;

		} else {
			print_state->blocked_count++;
		}

		uint64_t last_time = 0;

		for (int j = 0; j < MAX_THREADS; j++) {
			if (print_state->last_threads[j] == threads[i]) {
				last_time = print_state->last_times[j];
				break;
			}
		}

		// the runtime of a thread created in the k_thread of one that exited starts from 0
		if (last_time > times[i]) {
			last_time = 0;
		}

		const bool idle = (info.priority == K_IDLE_PRIO);
		float current_load = 0.f;

		if (times[i] > last_time) {
			const uint64_t interval_runtime = times[i] - last_time;

			current_load = interval_runtime / print_state->interval_time_us;

			if (idle) {
				idle_load = current_load;

			} else {
				print_state->total_user_time += interval_runtime;
			}
		}

		if (idle) {
			idle_time = times[i];
		}

		if (print_state->new_time <= print_state->interval_start_time) {
			continue; // not enough data yet
		}

		snprintf(buffer, buffer_length, "%-*s %8u %6.3f %5u/%5u %4d %s",
			 NAME_WIDTH, info.name,
			 (unsigned)(times[i] / 1000), // us -> ms
			 (double)(current_load * 100.f),
			 (unsigned)info.stack_used,
			 (unsigned)info.stack_size,
			 info.priority,
			 info.state);
		cb(user);
	}

	memcpy(print_state->last_threads, print_state->threads, sizeof(print_state->last_threads));
	memcpy(print_state->last_times, print_state->times, sizeof(print_state->last_times));

	if (print_state->new_time <= print_state->interval_start_time) {
		// first run, not enough data yet
		return;
	}

	// Print footer
	buffer[0] = 0;
	cb(user);

	const float task_load = (float)print_state->total_user_time / print_state->interval_time_us;

	snprintf(buffer, buffer_length, "Threads: %d total, %d running, %d sleeping",
		 thread_count,
		 print_state->running_count,
		 print_state->blocked_count);
	cb(user);
	snprintf(buffer, buffer_length, "CPU usage: %.2f%% tasks, %.2f%% idle",
		 (double)(task_load * 100.f),
		 (double)(idle_load * 100.f));
	cb(user);
	snprintf(buffer, buffer_length, "Uptime: %.3fs total, %.3fs idle",
		 (double)print_state->new_time / 1e6, (double)idle_time / 1e6);
	cb(user);

	print_state->interval_start_time = print_state->new_time;
}

struct print_load_callback_data_s {
	int fd;
	char buffer[140];
};

static void print_load_callback(void *user)
{
	char clear_line[] {CL};
	struct print_load_callback_data_s *data = (struct print_load_callback_data_s *)user;

	if (data->fd != STDOUT_FILENO) {
		clear_line[0] = '\0';
	}

	dprintf(data->fd, "%s%s\n", clear_line, data->buffer);
}

void print_load(int fd, struct print_load_s *print_state)
{
	// print system information
	if (fd == STDOUT_FILENO) {
		// move cursor home and clear screen
		dprintf(fd, "\033[H");
	}

	print_load_callback_data_s data{};
	data.fd = fd;

	print_load_buffer(data.buffer, sizeof(data.buffer), print_load_callback, &data, print_state);
}
