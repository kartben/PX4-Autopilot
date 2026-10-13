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
 * @file px4_sched.c
 *
 * PX4 sets thread priorities for NuttX, from 1 to 255, with offsets from
 * SCHED_PRIORITY_MAX and SCHED_PRIORITY_DEFAULT. Scaled down to the
 * preemptible priorities of Zephyr, the offsets would change the order of
 * the threads, so the POSIX scheduling functions are wrapped at link time to
 * map PX4 priorities in order.
 *
 * The wrappers also give new thread attributes explicit scheduling, as PX4
 * expects: the POSIX threads of Zephyr otherwise inherit the priority of the
 * thread that creates them, whatever priority the attributes set. SCHED_FIFO
 * becomes SCHED_RR, as SCHED_FIFO threads are cooperative on Zephyr.
 */

#include <px4_sched.h>

#include <pthread.h>
#include <sched.h>

#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>

#include <px4_platform_common/tasks.h>

#define POSIX_PRIORITY_MAX (CONFIG_NUM_PREEMPT_PRIORITIES - 1)

// the PX4 priorities below the band
#define PX4_COMPRESSED_MAX (SCHED_PRIORITY_MAX - PX4_ZEPHYR_PRIORITY_BAND)
#define POSIX_COMPRESSED_MAX (POSIX_PRIORITY_MAX - PX4_ZEPHYR_PRIORITY_BAND)

BUILD_ASSERT(POSIX_COMPRESSED_MAX > 0, "too few preemptible priorities for the PX4 priorities");

int px4_zephyr_posix_priority(int priority)
{
	priority = CLAMP(priority, SCHED_PRIORITY_MIN, SCHED_PRIORITY_MAX);

	if (priority > PX4_COMPRESSED_MAX) {
		return POSIX_PRIORITY_MAX - (SCHED_PRIORITY_MAX - priority);
	}

	return (priority - SCHED_PRIORITY_MIN) * POSIX_COMPRESSED_MAX / (PX4_COMPRESSED_MAX - SCHED_PRIORITY_MIN);
}

int px4_zephyr_px4_priority(int posix_priority)
{
	posix_priority = CLAMP(posix_priority, 0, POSIX_PRIORITY_MAX);

	if (posix_priority > POSIX_COMPRESSED_MAX) {
		return SCHED_PRIORITY_MAX - (POSIX_PRIORITY_MAX - posix_priority);
	}

	return SCHED_PRIORITY_MIN + posix_priority * (PX4_COMPRESSED_MAX - SCHED_PRIORITY_MIN) / POSIX_COMPRESSED_MAX;
}

static int zephyr_policy(int policy)
{
	return (policy == SCHED_FIFO) ? SCHED_RR : policy;
}

int __real_pthread_attr_init(pthread_attr_t *attr);
int __real_pthread_attr_setschedpolicy(pthread_attr_t *attr, int policy);
int __real_pthread_attr_setschedparam(pthread_attr_t *attr, const struct sched_param *param);
int __real_pthread_attr_getschedparam(const pthread_attr_t *attr, struct sched_param *param);
int __real_pthread_setschedparam(pthread_t thread, int policy, const struct sched_param *param);
int __real_pthread_getschedparam(pthread_t thread, int *policy, struct sched_param *param);

int __wrap_pthread_attr_init(pthread_attr_t *attr)
{
	int ret = __real_pthread_attr_init(attr);

	if (ret == 0) {
		const struct sched_param param = {
			.sched_priority = px4_zephyr_posix_priority(SCHED_PRIORITY_DEFAULT),
		};

		(void)pthread_attr_setinheritsched(attr, PTHREAD_EXPLICIT_SCHED);
		(void)__real_pthread_attr_setschedpolicy(attr, SCHED_RR);
		(void)__real_pthread_attr_setschedparam(attr, &param);
	}

	return ret;
}

int __wrap_pthread_attr_setschedpolicy(pthread_attr_t *attr, int policy)
{
	return __real_pthread_attr_setschedpolicy(attr, zephyr_policy(policy));
}

int __wrap_pthread_attr_setschedparam(pthread_attr_t *attr, const struct sched_param *param)
{
	const struct sched_param zephyr_param = {
		.sched_priority = px4_zephyr_posix_priority(param->sched_priority),
	};

	return __real_pthread_attr_setschedparam(attr, &zephyr_param);
}

int __wrap_pthread_attr_getschedparam(const pthread_attr_t *attr, struct sched_param *param)
{
	int ret = __real_pthread_attr_getschedparam(attr, param);

	if (ret == 0) {
		param->sched_priority = px4_zephyr_px4_priority(param->sched_priority);
	}

	return ret;
}

int __wrap_pthread_setschedparam(pthread_t thread, int policy, const struct sched_param *param)
{
	const struct sched_param zephyr_param = {
		.sched_priority = px4_zephyr_posix_priority(param->sched_priority),
	};

	return __real_pthread_setschedparam(thread, zephyr_policy(policy), &zephyr_param);
}

int __wrap_pthread_getschedparam(pthread_t thread, int *policy, struct sched_param *param)
{
	int ret = __real_pthread_getschedparam(thread, policy, param);

	if (ret == 0) {
		param->sched_priority = px4_zephyr_px4_priority(param->sched_priority);
	}

	return ret;
}
