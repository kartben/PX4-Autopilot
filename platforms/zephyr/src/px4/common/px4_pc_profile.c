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
 * @file px4_pc_profile.c
 *
 * Samples the program counter without a debug probe, for Tools/itcm_profile.py.
 * Comparator 0 of the DWT matches the cycle counter, which raises the
 * DebugMonitor exception: its handler records the return address of the
 * interrupted context and sets the next match. The exception has the highest
 * priority, so that it samples interrupt handlers and code with interrupts
 * locked too. The cycle counter stops while the CPU sleeps, so that the
 * samples are spread over the time the CPU runs.
 */

#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include <cmsis_core.h>

static uint32_t pc_samples[CONFIG_PX4_PC_PROFILE_SAMPLES] __noinit;
static volatile uint32_t pc_count;
static uint32_t pc_period;

// frame: the exception frame of the interrupted context, whose return address is its 7th word
void px4_pc_profile_sample(const uint32_t *frame)
{
	DWT->COMP0 = DWT->CYCCNT + pc_period;
	(void)DWT->FUNCTION0; // clears MATCHED
	SCB->DFSR = SCB_DFSR_DWTTRAP_Msk;

	if (pc_count < ARRAY_SIZE(pc_samples)) {
		pc_samples[pc_count++] = frame[6];

	} else {
		DWT->FUNCTION0 = 0;
	}
}

// replaces the weak fault handler of Zephyr: the frame is on the stack that the interrupted context used
__attribute__((naked)) void z_arm_debug_monitor(void)
{
	__asm volatile(
		"tst lr, #4\n"
		"ite eq\n"
		"mrseq r0, msp\n"
		"mrsne r0, psp\n"
		"b px4_pc_profile_sample\n");
}

static int cmd_start(const struct shell *sh, size_t argc, char **argv)
{
	// a period that is no multiple of the control loops, at about 2 kHz at 480 MHz
	pc_period = (argc > 1) ? strtoul(argv[1], NULL, 0) : 240007;
	pc_count = 0;

	DWT->FUNCTION0 = 0;
	DCB->DEMCR |= DCB_DEMCR_TRCENA_Msk | DCB_DEMCR_MON_EN_Msk;
#if defined(CONFIG_CPU_CORTEX_M7)
	DWT->LAR = 0xC5ACCE55;
#endif
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	NVIC_SetPriority(DebugMonitor_IRQn, 0);

	DWT->MASK0 = 0;
	DWT->COMP0 = DWT->CYCCNT + pc_period;
	DWT->FUNCTION0 = DWT_FUNCTION_CYCMATCH_Msk | (4 << DWT_FUNCTION_FUNCTION_Pos);

	shell_print(sh, "sampling every %u cycles", pc_period);
	return 0;
}

static int cmd_stop(const struct shell *sh, size_t argc, char **argv)
{
	DWT->FUNCTION0 = 0;
	shell_print(sh, "%u samples", pc_count);
	return 0;
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(sh, "%u of %u samples, every %u cycles", pc_count, (uint32_t)ARRAY_SIZE(pc_samples), pc_period);
	return 0;
}

// dump [first [count]]: the samples in hexadecimal, 8 per line
static int cmd_dump(const struct shell *sh, size_t argc, char **argv)
{
	const uint32_t n = pc_count;
	uint32_t first = (argc > 1) ? strtoul(argv[1], NULL, 0) : 0;
	uint32_t end = (argc > 2) ? first + strtoul(argv[2], NULL, 0) : n;

	if (end > n) {
		end = n;
	}

	for (uint32_t i = first; i < end; i += 8) {
		char line[8 * 9 + 1];
		size_t len = 0;

		for (uint32_t j = i; j < end && j < i + 8; j++) {
			len += snprintk(&line[len], sizeof(line) - len, "%08x ", pc_samples[j]);
		}

		shell_print(sh, "%s", line);
	}

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(pc_profile_cmds,
			       SHELL_CMD_ARG(start, NULL, "Start sampling [period in cycles]", cmd_start, 1, 1),
			       SHELL_CMD(stop, NULL, "Stop sampling", cmd_stop),
			       SHELL_CMD(status, NULL, "Print the number of samples", cmd_status),
			       SHELL_CMD_ARG(dump, NULL, "Print the samples [first [count]]", cmd_dump, 1, 2),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(pc_profile, &pc_profile_cmds, "Program counter sampling", NULL);
