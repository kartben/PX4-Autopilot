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

#include <px4_zephyr.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#if defined(CONFIG_SYS_HEAP_RUNTIME_STATS)
#include <zephyr/sys/sys_heap.h>
#include <sys_malloc.h>
#endif

int px4_shell_run_command(const struct shell *sh, size_t argc, char **argv)
{
#if defined(CONFIG_PX4_MAVLINK_SHELL)
	// the standard output of the command, and of the tasks that it starts, goes to this shell if it is
	// the MAVLink shell, see px4_mavlink_shell.c
	k_thread_custom_data_set((void *)sh);
#endif

	return px4_zephyr_run_command(argc, argv);
}

static void px4_builtin_get(size_t idx, struct shell_static_entry *entry)
{
	if (idx < px4_zephyr_builtins_count) {
		entry->syntax = px4_zephyr_builtins[idx].name;
		entry->handler = px4_shell_run_command;
		entry->help = NULL;
		entry->subcmd = NULL;
		entry->args.mandatory = 1;
		entry->args.optional = SHELL_OPT_ARG_MAX;

	} else {
		entry->syntax = NULL;
	}
}

SHELL_DYNAMIC_CMD_CREATE(px4_builtins, px4_builtin_get);

SHELL_CMD_REGISTER(px4, &px4_builtins, "PX4 commands", NULL);

// the commands of nsh that the scripts of the ground stations and of the bench tests use

static int cmd_echo(const struct shell *sh, size_t argc, char **argv)
{
	for (size_t i = 1; i < argc; i++) {
		shell_fprintf_normal(sh, "%s%s", (i > 1) ? " " : "", argv[i]);
	}

	shell_fprintf_normal(sh, "\n");

	return 0;
}

SHELL_CMD_ARG_REGISTER(echo, NULL, "Print the arguments", cmd_echo, 1, SHELL_OPT_ARG_MAX);

#if defined(CONFIG_SYS_HEAP_RUNTIME_STATS)
extern struct k_heap _system_heap;

static void print_heap(const struct shell *sh, const char *name, const struct sys_memory_stats *stats)
{
	shell_print(sh, "%s %10zu %10zu %10zu %10zu", name, stats->allocated_bytes + stats->free_bytes,
		    stats->allocated_bytes, stats->free_bytes, stats->max_allocated_bytes);
}

static int cmd_free(const struct shell *sh, size_t argc, char **argv)
{
	struct sys_memory_stats stats;

	// the columns of nsh, with the heap of the C library, of the PX4 objects, as the user heap of NuttX
	shell_print(sh, "      %10s %10s %10s %10s", "total", "used", "free", "maxused");

	if (malloc_runtime_stats_get(&stats) == 0) {
		print_heap(sh, "Umem:", &stats);
	}

#if K_HEAP_MEM_POOL_SIZE > 0

	// the kernel heap, of the thread stacks
	if (sys_heap_runtime_stats_get(&_system_heap.heap, &stats) == 0) {
		print_heap(sh, "Kmem:", &stats);
	}

#endif

	return 0;
}

SHELL_CMD_REGISTER(free, NULL, "Print the usage of the heaps", cmd_free);
#endif
