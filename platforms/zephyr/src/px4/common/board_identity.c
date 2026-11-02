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
 * @file board_identity.c
 *
 * Board identity from the device ID of the hwinfo driver, for the boards that
 * do not define BOARD_OVERRIDE_UUID. The device ID is right-aligned in the
 * PX4_CPU_UUID_BYTE_LENGTH bytes of the UUID, most significant byte first.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/byteorder.h>

#if !defined(BOARD_OVERRIDE_UUID)

BUILD_ASSERT(IS_ENABLED(CONFIG_HWINFO_HAS_DRIVER), "boards without a hwinfo driver define BOARD_OVERRIDE_UUID");

static const uint16_t soc_arch_id = PX4_SOC_ARCH_ID;

static void get_device_id(uint8_t id[PX4_CPU_UUID_BYTE_LENGTH])
{
	static bool reported;
	uint8_t device_id[PX4_CPU_UUID_BYTE_LENGTH];
	ssize_t length = hwinfo_get_device_id(device_id, sizeof(device_id));

	memset(id, 0, PX4_CPU_UUID_BYTE_LENGTH);

	if (length > 0) {
		memcpy(&id[PX4_CPU_UUID_BYTE_LENGTH - length], device_id, length);

	} else if (!reported) {
		PX4_ERR("no device ID (%d)", (int)length);
		reported = true;
	}
}

void board_get_uuid32(uuid_uint32_t uuid_words)
{
	uint8_t id[PX4_CPU_UUID_BYTE_LENGTH];
	get_device_id(id);

	for (unsigned i = 0; i < PX4_CPU_UUID_WORD32_LENGTH; i++) {
		uuid_words[i] = sys_get_be32(&id[4 * i]);
	}
}

int board_get_uuid32_formated(char *format_buffer, int size,
			      const char *format,
			      const char *seperator)
{
	uuid_uint32_t uuid;
	board_get_uuid32(uuid);
	int offset = 0;
	int sep_size = seperator ? strlen(seperator) : 0;

	for (unsigned i = 0; (offset < size - 1) && (i < PX4_CPU_UUID_WORD32_LENGTH); i++) {
		offset += snprintf(&format_buffer[offset], size - offset, format, uuid[i]);

		if (sep_size && (offset < size - sep_size - 1) && (i < PX4_CPU_UUID_WORD32_LENGTH - 1)) {
			strncat(&format_buffer[offset], seperator, size - offset);
			offset += sep_size;
		}
	}

	return 0;
}

int board_get_mfguid(mfguid_t mfgid)
{
	get_device_id(mfgid);
	return PX4_CPU_MFGUID_BYTE_LENGTH;
}

int board_get_mfguid_formated(char *format_buffer, int size)
{
	mfguid_t mfguid;
	board_get_mfguid(mfguid);
	int offset = 0;

	for (unsigned i = 0; (offset + 2 < size) && (i < PX4_CPU_MFGUID_BYTE_LENGTH); i++) {
		offset += snprintf(&format_buffer[offset], size - offset, "%02x", mfguid[i]);
	}

	return offset;
}

int board_get_px4_guid(px4_guid_t px4_guid)
{
	px4_guid[0] = (soc_arch_id >> 8) & 0xff;
	px4_guid[1] = soc_arch_id & 0xff;
	memset(&px4_guid[2], 0, PX4_GUID_BYTE_LENGTH - 2 - PX4_CPU_MFGUID_BYTE_LENGTH);
	board_get_mfguid(&px4_guid[PX4_GUID_BYTE_LENGTH - PX4_CPU_MFGUID_BYTE_LENGTH]);
	return PX4_GUID_BYTE_LENGTH;
}

int board_get_px4_guid_formated(char *format_buffer, int size)
{
	px4_guid_t px4_guid;
	board_get_px4_guid(px4_guid);
	int offset = 0;

	for (unsigned i = 0; (offset + 2 < size) && (i < PX4_GUID_BYTE_LENGTH); i++) {
		offset += snprintf(&format_buffer[offset], size - offset, "%02x", px4_guid[i]);
	}

	return offset;
}
#endif
