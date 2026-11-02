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
 * @file storage.cpp
 *
 * Tests of the parameters and the file functions of PX4 on a FAT file system,
 * mounted at PX4_STORAGEDIR from a RAM disk.
 */

#include <parameters/param.h>
#include <px4_platform_common/defines.h>
#include <px4_zephyr.h>

#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/statfs.h>
#include <unistd.h>

#include <zephyr/ztest.h>

#define TEST_FILE PX4_STORAGEDIR "/test.bin"

static void *setup(void)
{
	zassert_ok(px4_zephyr_init());
	return nullptr;
}

static void after(void *fixture)
{
	ARG_UNUSED(fixture);

	unlink(TEST_FILE);
}

ZTEST_SUITE(px4_storage, NULL, setup, NULL, after, NULL);

ZTEST(px4_storage, test_statfs)
{
	struct statfs storage;

	zassert_ok(statfs(PX4_STORAGEDIR, &storage));
	zassert_true(storage.f_blocks > 0);
	zassert_true(storage.f_bavail > 0);
	zassert_true(storage.f_bavail <= storage.f_blocks);
}

ZTEST(px4_storage, test_parameters)
{
	const param_t param = param_find("SYS_AUTOSTART");
	int32_t value = 4001;
	struct stat st;

	zassert_ok(param_set_no_notification(param, &value));
	zassert_ok(param_save_default(true));
	zassert_ok(stat(PX4_STORAGEDIR "/parameters.bson", &st));

	param_reset_no_notification(param);
	zassert_ok(param_get(param, &value));
	zassert_equal(value, 0);

	zassert_ok(param_load_default());
	zassert_ok(param_get(param, &value));
	zassert_equal(value, 4001);

	param_reset_no_notification(param);
	zassert_ok(param_save_default(true));
}

ZTEST(px4_storage, test_files)
{
	struct stat st;
	const int fd = open(TEST_FILE, O_WRONLY | O_CREAT, 0666);

	zassert_true(fd >= 0, "open failed: %d", errno);
	zassert_equal(write(fd, "px4", 3), 3);
	zassert_ok(close(fd));

	zassert_ok(access(TEST_FILE, F_OK));
	zassert_ok(lstat(TEST_FILE, &st));
	zassert_equal(st.st_size, 3);

	// FAT can't seek past the end of a file, but it can grow it
	zassert_ok(truncate(TEST_FILE, 1024));
	zassert_ok(lstat(TEST_FILE, &st));
	zassert_equal(st.st_size, 1024);

	zassert_ok(unlink(TEST_FILE));
	zassert_equal(access(TEST_FILE, F_OK), -1);
}
