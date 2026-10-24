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


/*
 * Emulator of the NCP5623C of a GPS module. Each command is a byte, followed by a byte that the chip
 * ignores, as the Zephyr, NuttX and Linux drivers write them: its bits 7..5 select a register, and bits
 * 4..0 are its value.
 */

#include "led_record.h"

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

#define REGISTER_SHIFT 5
#define VALUE_MASK 0x1f
#define REGISTER_COUNT 8

static struct k_spinlock lock;
static uint8_t registers[REGISTER_COUNT];

static int ncp5623_emul_transfer(const struct emul *target, struct i2c_msg *msgs, int num_msgs, int addr)
{
	size_t position = 0;

	for (int i = 0; i < num_msgs; i++) {
		/* the NCP5623C can't be read */
		if ((msgs[i].flags & I2C_MSG_READ) != 0) {
			return -EIO;
		}

		for (uint32_t j = 0; j < msgs[i].len; j++, position++) {
			if ((position % 2) == 0) {
				K_SPINLOCK(&lock) {
					registers[msgs[i].buf[j] >> REGISTER_SHIFT] = msgs[i].buf[j] & VALUE_MASK;
				}
			}
		}
	}

	return 0;
}

uint8_t test_gps_led_register(unsigned reg)
{
	uint8_t value;

	K_SPINLOCK(&lock) {
		value = registers[reg];
	}

	return value;
}

static const struct i2c_emul_api ncp5623_emul_api = {
	.transfer = ncp5623_emul_transfer,
};

static int ncp5623_emul_init(const struct emul *target, const struct device *parent)
{
	return 0;
}

EMUL_DT_DEFINE(DT_NODELABEL(test_gps_ncp5623c), ncp5623_emul_init, NULL, NULL, &ncp5623_emul_api, NULL);
