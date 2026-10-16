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
 * @file px4_mavlink_shell.c
 *
 * The MAVLink shell: a Zephyr shell instance whose transport is the
 * SERIAL_CONTROL messages of a MAVLink instance, as nsh on NuttX.
 *
 * The PX4 commands print to the standard output, not to the shell that runs
 * them. The shell sets itself as the custom data of its thread when it runs a
 * PX4 command (see px4_shell.c), and the PX4 tasks inherit the custom data of
 * the thread that starts them (see tasks.cpp): the standard output of the
 * threads whose custom data is this shell goes to MAVLink, the one of the
 * other threads to the console.
 */

#include <errno.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/libc-hooks.h>
#include <zephyr/sys/printk-hooks.h>
#include <zephyr/sys/ring_buffer.h>

#include <px4_mavlink_shell.h>

// how long the standard output waits for MAVLink to take some output, before it is dropped
#define OUTPUT_TIMEOUT K_MSEC(100)

// bytes of the log messages queued for the shell, which is not a log backend
#define LOG_QUEUE_SIZE 128

struct transport_context {
	shell_transport_handler_t handler;
	void *context;
};

static int transport_init(const struct shell_transport *transport, const void *config,
			  shell_transport_handler_t evt_handler, void *context);
static int transport_uninit(const struct shell_transport *transport);
static int transport_enable(const struct shell_transport *transport, bool blocking);
static int transport_write(const struct shell_transport *transport, const void *data, size_t length, size_t *cnt);
static int transport_read(const struct shell_transport *transport, void *data, size_t length, size_t *cnt);

static const struct shell_transport_api transport_api = {
	.init = transport_init,
	.uninit = transport_uninit,
	.enable = transport_enable,
	.write = transport_write,
	.read = transport_read,
};

static struct transport_context transport_context;

static struct shell_transport transport = {
	.api = &transport_api,
	.ctx = &transport_context,
};

SHELL_DEFINE(px4_mavlink_shell, CONFIG_PX4_MAVLINK_SHELL_PROMPT, &transport, LOG_QUEUE_SIZE, 0,
	     SHELL_FLAG_OLF_CRLF);

RING_BUF_DECLARE(input, CONFIG_PX4_MAVLINK_SHELL_INPUT_BUFFER_SIZE);
RING_BUF_DECLARE(output, CONFIG_PX4_MAVLINK_SHELL_OUTPUT_BUFFER_SIZE);

// protects the buffers, attached and output_stalled
static struct k_spinlock lock;

// a MAVLink instance has the shell
static bool attached;

// MAVLink took no output for OUTPUT_TIMEOUT: the standard output is dropped until it takes some again
static bool output_stalled;

// given when MAVLink takes output
static K_SEM_DEFINE(output_taken, 0, 1);

// the console, to which the standard output of the other threads goes
static printk_hook_fn_t console_out;

static void transport_event(enum shell_transport_evt event)
{
	if (transport_context.handler != NULL) {
		transport_context.handler(event, transport_context.context);
	}
}

static int transport_init(const struct shell_transport *transport, const void *config,
			  shell_transport_handler_t evt_handler, void *context)
{
	struct transport_context *ctx = transport->ctx;

	ctx->handler = evt_handler;
	ctx->context = context;

	return 0;
}

static int transport_uninit(const struct shell_transport *transport)
{
	return 0;
}

static int transport_enable(const struct shell_transport *transport, bool blocking)
{
	return 0;
}

static int transport_write(const struct shell_transport *transport, const void *data, size_t length, size_t *cnt)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	if (attached) {
		// 0 when the buffer is full: the shell waits for the TX ready event of px4_mavlink_shell_read()
		*cnt = ring_buf_put(&output, data, length);

	} else {
		// no ground station: the prompt and the echo are dropped
		*cnt = length;
	}

	k_spin_unlock(&lock, key);

	return 0;
}

static int transport_read(const struct shell_transport *transport, void *data, size_t length, size_t *cnt)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	*cnt = ring_buf_get(&input, data, length);
	k_spin_unlock(&lock, key);

	return 0;
}

static int stdout_out(int c)
{
	if (!k_is_in_isr() && (k_thread_custom_data_get() == &px4_mavlink_shell)) {
		const uint8_t byte = c;

		for (;;) {
			k_spinlock_key_t key = k_spin_lock(&lock);
			const bool to_mavlink = attached;
			const bool put = to_mavlink && (ring_buf_put(&output, &byte, 1) == 1);
			const bool stalled = output_stalled;
			k_spin_unlock(&lock, key);

			if (!to_mavlink) {
				// the ground station closed the shell: the tasks that the shell started print to the console
				break;
			}

			if (put || stalled) {
				return c;
			}

			if (k_sem_take(&output_taken, OUTPUT_TIMEOUT) != 0) {
				key = k_spin_lock(&lock);
				output_stalled = true;
				k_spin_unlock(&lock, key);
			}
		}
	}

	return (console_out != NULL) ? console_out(c) : c;
}

int px4_mavlink_shell_open(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	if (attached) {
		k_spin_unlock(&lock, key);
		return -EBUSY;
	}

	ring_buf_reset(&input);
	ring_buf_reset(&output);
	output_stalled = false;
	attached = true;
	k_spin_unlock(&lock, key);

	return 0;
}

void px4_mavlink_shell_close(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	attached = false;
	ring_buf_reset(&input);
	ring_buf_reset(&output);
	k_spin_unlock(&lock, key);

	// wake up the shell and the tasks that wait to write their output, which is now dropped
	k_sem_give(&output_taken);
	transport_event(SHELL_TRANSPORT_EVT_TX_RDY);
}

size_t px4_mavlink_shell_write(const uint8_t *buffer, size_t len)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	const size_t written = attached ? ring_buf_put(&input, buffer, len) : 0;
	k_spin_unlock(&lock, key);

	if (written > 0) {
		transport_event(SHELL_TRANSPORT_EVT_RX_RDY);
	}

	return written;
}

size_t px4_mavlink_shell_read(uint8_t *buffer, size_t len)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	const size_t read = ring_buf_get(&output, buffer, len);

	if (read > 0) {
		output_stalled = false;
	}

	k_spin_unlock(&lock, key);

	if (read > 0) {
		k_sem_give(&output_taken);
		transport_event(SHELL_TRANSPORT_EVT_TX_RDY);
	}

	return read;
}

size_t px4_mavlink_shell_available(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	const size_t available = ring_buf_size_get(&output);
	k_spin_unlock(&lock, key);

	return available;
}

static int px4_mavlink_shell_init(void)
{
	// as nsh: no colors and no VT100 commands, which the ground stations and the scripts would have to filter
	const struct shell_backend_config_flags flags = {
		.insert_mode = 0,
		.echo = 1,
		.obscure = 0,
		.mode_delete = 1,
		.use_colors = 0,
		.use_vt100 = 0,
	};

	// the console installs the same hook for printk and for the standard output
	console_out = __printk_get_hook();
	__stdout_hook_install(stdout_out);

	return shell_init(&px4_mavlink_shell, NULL, flags, false, 0);
}

// after the console
SYS_INIT(px4_mavlink_shell_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
