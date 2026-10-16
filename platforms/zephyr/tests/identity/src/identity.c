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
 * Test the board identity of the generic board: from the device ID of the hwinfo
 * driver of the SoC, or of the test in QEMU, or from BOARD_OVERRIDE_UUID without
 * a hwinfo driver, as in QEMU.
 */

#include <px4_platform_common/px4_config.h>

#include <stdio.h>
#include <string.h>

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#if defined(CONFIG_PX4_TEST_HWINFO)
static const uint8_t device_id[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef};

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	length = MIN(length, sizeof(device_id));
	memcpy(buffer, device_id, length);
	return length;
}
#endif

static void hex(char *string, const uint8_t *bytes, size_t length)
{
	for (size_t i = 0; i < length; i++) {
		sprintf(&string[2 * i], "%02x", bytes[i]);
	}
}

ZTEST(px4_identity, test_px4_guid)
{
	px4_guid_t px4_guid;
	mfguid_t mfguid;

	zassert_equal(board_get_px4_guid(px4_guid), PX4_GUID_BYTE_LENGTH);
	zassert_equal(board_get_mfguid(mfguid), PX4_CPU_MFGUID_BYTE_LENGTH);

	zassert_equal(px4_guid[0], (PX4_SOC_ARCH_ID >> 8) & 0xff);
	zassert_equal(px4_guid[1], PX4_SOC_ARCH_ID & 0xff);
	zassert_mem_equal(&px4_guid[PX4_GUID_BYTE_LENGTH - PX4_CPU_MFGUID_BYTE_LENGTH], mfguid,
			  PX4_CPU_MFGUID_BYTE_LENGTH);
}

ZTEST(px4_identity, test_formatted)
{
	px4_guid_t px4_guid;
	mfguid_t mfguid;
	char expected[PX4_GUID_FORMAT_SIZE];
	char formatted[PX4_GUID_FORMAT_SIZE];

	board_get_px4_guid(px4_guid);
	hex(expected, px4_guid, PX4_GUID_BYTE_LENGTH);
	board_get_px4_guid_formated(formatted, sizeof(formatted));
	zassert_str_equal(formatted, expected);

	board_get_mfguid(mfguid);
	hex(expected, mfguid, PX4_CPU_MFGUID_BYTE_LENGTH);
	board_get_mfguid_formated(formatted, sizeof(formatted));
	zassert_str_equal(formatted, expected);
}

ZTEST(px4_identity, test_hwinfo)
{
	Z_TEST_SKIP_IFNDEF(CONFIG_HWINFO_HAS_DRIVER);

	uint8_t id[PX4_CPU_MFGUID_BYTE_LENGTH];
	uint8_t expected[PX4_CPU_MFGUID_BYTE_LENGTH] = {0};
	mfguid_t mfguid;

	ssize_t length = hwinfo_get_device_id(id, sizeof(id));
	zassert_true(length > 0);
	memcpy(&expected[sizeof(expected) - length], id, length);

	board_get_mfguid(mfguid);
	zassert_mem_equal(mfguid, expected, sizeof(expected));

#if defined(CONFIG_PX4_TEST_HWINFO)
	uuid_uint32_t uuid;
	board_get_uuid32(uuid);
	zassert_equal(uuid[PX4_CPU_UUID_WORD32_UNIQUE_M], 0x01234567);
	zassert_equal(uuid[PX4_CPU_UUID_WORD32_UNIQUE_H], 0x89abcdef);
#endif

#if defined(BOARD_OVERRIDE_UUID) || defined(BOARD_HAS_NO_UUID)
	ztest_test_fail();
#endif
}

ZTEST(px4_identity, test_override)
{
	Z_TEST_SKIP_IFDEF(CONFIG_HWINFO_HAS_DRIVER);

#if !defined(BOARD_OVERRIDE_UUID) || !defined(BOARD_HAS_NO_UUID)
	ztest_test_fail();
#endif
}

ZTEST_SUITE(px4_identity, NULL, NULL, NULL, NULL, NULL);
