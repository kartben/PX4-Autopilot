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
 * @file px4_mavlink_shell.h
 *
 * Zephyr shell of the MAVLink shell (SERIAL_CONTROL), which MavlinkShell
 * opens, writes the received characters to and reads the output from.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Attach the shell to MAVLink. Only one MAVLink instance at a time has the
 * shell.
 *
 * @return 0, or -EBUSY if another MAVLink instance has the shell.
 */
int px4_mavlink_shell_open(void);

/**
 * Detach the shell from MAVLink. The output that follows is dropped.
 */
void px4_mavlink_shell_close(void);

/**
 * Give the characters received from MAVLink to the shell.
 *
 * @return the number of characters taken, fewer if the input buffer is full.
 */
size_t px4_mavlink_shell_write(const uint8_t *buffer, size_t len);

/**
 * Take the output of the shell and of the commands it runs, without waiting.
 *
 * @return the number of characters read.
 */
size_t px4_mavlink_shell_read(uint8_t *buffer, size_t len);

/**
 * @return the number of characters of output that px4_mavlink_shell_read() can take.
 */
size_t px4_mavlink_shell_available(void);

#ifdef __cplusplus
}
#endif
