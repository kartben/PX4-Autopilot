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
 * @file px4_flexspi.c
 *
 * The boot ROM of the i.MX RT1170 leaves the AHB read prefetch of the FlexSPI
 * on, which Zephyr keeps when the firmware runs in place from the flash. A
 * fetch from the flash then waits for the prefetch of other data to finish,
 * and the CPU stalls for 50 to 60 us, also with interrupts locked. The
 * prefetch is turned off before the kernel starts, from RAM, as no
 * instruction may be fetched from the flash meanwhile.
 */

#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/linker/section_tags.h>

#include <fsl_device_registers.h>

static __ramfunc int px4_flexspi_prefetch_off(void)
{
	FLEXSPI_Type *flexspi = (FLEXSPI_Type *)DT_REG_ADDR_BY_IDX(DT_NODELABEL(flexspi), 0);
	const unsigned int key = irq_lock();

	__DSB();
	__ISB();

	// no AHB read in progress
	while ((flexspi->STS0 & (FLEXSPI_STS0_ARBIDLE_MASK | FLEXSPI_STS0_SEQIDLE_MASK)) !=
	       (FLEXSPI_STS0_ARBIDLE_MASK | FLEXSPI_STS0_SEQIDLE_MASK)) {
	}

	flexspi->AHBCR = (flexspi->AHBCR & ~FLEXSPI_AHBCR_PREFETCHEN_MASK) | FLEXSPI_AHBCR_CLRAHBRXBUF_MASK;

	__DSB();
	__ISB();
	irq_unlock(key);

	return 0;
}

SYS_INIT(px4_flexspi_prefetch_off, PRE_KERNEL_1, 0);
