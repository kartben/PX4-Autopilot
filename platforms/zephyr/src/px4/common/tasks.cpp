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
 * @file tasks.cpp
 * Implementation of the task API for Zephyr
 */

#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>

#include <errno.h>
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define PX4_MAX_TASKS CONFIG_POSIX_THREAD_THREADS_MAX

// Zephyr thread attributes own the thread stack, so they are only destroyed after the thread has been joined
enum class TaskState : uint8_t {
	Unused,
	Running,
	Exited,
	Joining,
};

struct task_entry {
	pthread_t pid;
	pthread_attr_t attr;
	char name[16];
	TaskState state;
};

static task_entry taskmap[PX4_MAX_TASKS] {};
static pthread_mutex_t task_mutex = PTHREAD_MUTEX_INITIALIZER;

typedef struct {
	px4_main_t entry;
	px4_task_t id;
	int argc;
	char *argv[];
	// strings are allocated after the struct data
} pthdata_t;

static void *entry_adapter(void *ptr)
{
	pthdata_t *data = (pthdata_t *)ptr;

	// px4_task_delete() is expected to stop the task immediately
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, nullptr);
	pthread_setname_np(pthread_self(), taskmap[data->id].name);

	data->entry(data->argc, data->argv);
	free(ptr);
	px4_task_exit(0);

	return nullptr;
}

static int task_release(task_entry &task)
{
	const int rv = pthread_join(task.pid, nullptr);
	pthread_attr_destroy(&task.attr);

	pthread_mutex_lock(&task_mutex);
	task.state = TaskState::Unused;
	pthread_mutex_unlock(&task_mutex);

	return rv;
}

static void task_release_exited()
{
	for (task_entry &task : taskmap) {
		pthread_mutex_lock(&task_mutex);
		const bool exited = (task.state == TaskState::Exited);

		if (exited) {
			task.state = TaskState::Joining;
		}

		pthread_mutex_unlock(&task_mutex);

		if (exited) {
			task_release(task);
		}
	}
}

px4_task_t px4_task_spawn_cmd(const char *name, int scheduler, int priority, int stack_size, px4_main_t entry,
			      char *const argv[])
{
	task_release_exited();

	int argc = 0;
	size_t len = strlen(name) + 1;

	while (argv != nullptr && argv[argc] != nullptr) {
		len += strlen(argv[argc]) + 1;
		argc++;
	}

	const size_t structsize = sizeof(pthdata_t) + (argc + 2) * sizeof(char *);

	pthdata_t *taskdata = (pthdata_t *)calloc(1, structsize + len);

	if (taskdata == nullptr) {
		return -ENOMEM;
	}

	taskdata->entry = entry;
	taskdata->argc = argc + 1;

	// argv[0] is the task name, as on NuttX
	char *offset = (char *)taskdata + structsize;
	taskdata->argv[0] = offset;
	strcpy(offset, name);
	offset += strlen(name) + 1;

	for (int i = 0; i < argc; i++) {
		taskdata->argv[i + 1] = offset;
		strcpy(offset, argv[i]);
		offset += strlen(argv[i]) + 1;
	}

	pthread_mutex_lock(&task_mutex);

	px4_task_t id = -1;

	for (int i = 0; i < PX4_MAX_TASKS; i++) {
		if (taskmap[i].state == TaskState::Unused) {
			id = i;
			break;
		}
	}

	if (id < 0) {
		pthread_mutex_unlock(&task_mutex);
		free(taskdata);
		return -ENOSPC;
	}

	task_entry &task = taskmap[id];
	taskdata->id = id;
	strncpy(task.name, name, sizeof(task.name) - 1);
	task.name[sizeof(task.name) - 1] = '\0';

	if (stack_size < (int)PTHREAD_STACK_MIN) {
		stack_size = PTHREAD_STACK_MIN;
	}

	sched_param param{};
	param.sched_priority = priority;

	int rv = pthread_attr_init(&task.attr);

	if (rv == 0) {
		rv = pthread_attr_setstacksize(&task.attr, stack_size);

		if (rv == 0) {
			rv = pthread_attr_setinheritsched(&task.attr, PTHREAD_EXPLICIT_SCHED);
		}

		if (rv == 0) {
			rv = pthread_attr_setschedpolicy(&task.attr, scheduler);
		}

		if (rv == 0) {
			rv = pthread_attr_setschedparam(&task.attr, &param);
		}

		if (rv == 0) {
			task.state = TaskState::Running;
			rv = pthread_create(&task.pid, &task.attr, &entry_adapter, taskdata);
		}

		if (rv != 0) {
			pthread_attr_destroy(&task.attr);
		}
	}

	if (rv != 0) {
		PX4_ERR("failed to create thread for %s (%i)", name, rv);
		task.state = TaskState::Unused;
		pthread_mutex_unlock(&task_mutex);
		free(taskdata);
		return -rv;
	}

	pthread_mutex_unlock(&task_mutex);

	return id;
}

int px4_task_join(px4_task_t id)
{
	if (id < 0 || id >= PX4_MAX_TASKS) {
		return -EINVAL;
	}

	pthread_mutex_lock(&task_mutex);
	const bool joinable = (taskmap[id].state == TaskState::Running) || (taskmap[id].state == TaskState::Exited);

	if (joinable) {
		taskmap[id].state = TaskState::Joining;
	}

	pthread_mutex_unlock(&task_mutex);

	if (!joinable) {
		return -EINVAL;
	}

	return task_release(taskmap[id]);
}

int px4_task_delete(px4_task_t id)
{
	if (id < 0 || id >= PX4_MAX_TASKS) {
		return -EINVAL;
	}

	pthread_mutex_lock(&task_mutex);

	if (taskmap[id].state != TaskState::Running) {
		pthread_mutex_unlock(&task_mutex);
		return -EINVAL;
	}

	const pthread_t pid = taskmap[id].pid;

	if (pthread_equal(pid, pthread_self())) {
		pthread_mutex_unlock(&task_mutex);
		px4_task_exit(0);
	}

	const int rv = pthread_cancel(pid);

	if (rv == 0) {
		taskmap[id].state = TaskState::Exited;
	}

	pthread_mutex_unlock(&task_mutex);

	return -rv;
}

void px4_task_exit(int ret)
{
	const pthread_t pid = pthread_self();

	pthread_mutex_lock(&task_mutex);

	for (task_entry &task : taskmap) {
		if ((task.state == TaskState::Running) && pthread_equal(task.pid, pid)) {
			task.state = TaskState::Exited;
			break;
		}
	}

	pthread_mutex_unlock(&task_mutex);

	pthread_exit((void *)(intptr_t)ret);
}

int px4_task_kill(px4_task_t id, int sig)
{
	return -ENOTSUP;
}

void px4_show_tasks()
{
	int count = 0;

	PX4_INFO("Active Tasks:");

	pthread_mutex_lock(&task_mutex);

	for (const task_entry &task : taskmap) {
		if (task.state == TaskState::Running) {
			PX4_INFO("   %-10s %lu", task.name, (unsigned long)task.pid);
			count++;
		}
	}

	pthread_mutex_unlock(&task_mutex);

	if (count == 0) {
		PX4_INFO("   No running tasks");
	}
}

bool px4_task_is_running(const char *taskname)
{
	bool running = false;

	pthread_mutex_lock(&task_mutex);

	for (const task_entry &task : taskmap) {
		if ((task.state == TaskState::Running) && (strcmp(task.name, taskname) == 0)) {
			running = true;
			break;
		}
	}

	pthread_mutex_unlock(&task_mutex);

	return running;
}

px4_task_t px4_getpid()
{
	const pthread_t pid = pthread_self();
	px4_task_t ret = -1;

	pthread_mutex_lock(&task_mutex);

	for (int i = 0; i < PX4_MAX_TASKS; i++) {
		if ((taskmap[i].state != TaskState::Unused) && pthread_equal(taskmap[i].pid, pid)) {
			ret = i;
			break;
		}
	}

	pthread_mutex_unlock(&task_mutex);

	return ret;
}

const char *px4_get_taskname()
{
	const px4_task_t id = px4_getpid();
	return (id >= 0) ? taskmap[id].name : "UnknownApp";
}

int px4_prctl(int option, const char *arg2, px4_task_t pid)
{
	if (option == PR_SET_NAME) {
		return pthread_setname_np(pthread_self(), arg2);
	}

	PX4_WARN("FAILED SETTING TASK NAME");
	return -1;
}
