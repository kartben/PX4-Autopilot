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
 * @file system_config.h
 *
 * Definitions used by all Zephyr targets
 */

#pragma once

#define PX4_CPU_UUID_BYTE_LENGTH 16
#define PX4_CPU_UUID_WORD32_LENGTH 4

#define PX4_CPU_MFGUID_BYTE_LENGTH              PX4_CPU_UUID_BYTE_LENGTH
#define PX4_CPU_UUID_WORD32_UNIQUE_H            3 /* Least significant digits change the most */
#define PX4_CPU_UUID_WORD32_UNIQUE_M            2 /* Most significant digits change the least */
#define PX4_CPU_UUID_WORD32_FORMAT_SIZE         (PX4_CPU_UUID_WORD32_LENGTH-1+(2*PX4_CPU_UUID_BYTE_LENGTH)+1)
#define PX4_CPU_MFGUID_FORMAT_SIZE              ((2*PX4_CPU_MFGUID_BYTE_LENGTH)+1)

#define BOARD_OVERRIDE_CPU_VERSION (-1)
#define board_mcu_version(rev, revstr, errata) BOARD_OVERRIDE_CPU_VERSION

/* the other boards read the device ID of their hwinfo driver */
#if defined(BOARD_OVERRIDE_UUID)
#define BOARD_HAS_NO_UUID
#endif

/* the board ADC publishes the 5 V rail to system_power, without detecting USB */
#if defined(ADC_SCALED_V5_CHANNEL) && !defined(BOARD_ADC_USB_CONNECTED)
#define BOARD_ADC_USB_CONNECTED 0
#endif

/*
 * the analog batteries of the board ADC: two, always valid, if channels are named battery1-* or
 * battery2-*, and one otherwise, whose missing channels are -1 for BATn_V_CHANNEL to select one
 */
#if defined(ADC_BATTERY1_VOLTAGE_CHANNEL) || defined(ADC_BATTERY1_CURRENT_CHANNEL) || \
    defined(ADC_BATTERY2_VOLTAGE_CHANNEL) || defined(ADC_BATTERY2_CURRENT_CHANNEL)
#  if !defined(BOARD_NUMBER_BRICKS)
#    define BOARD_NUMBER_BRICKS 2
#    define BOARD_ADC_BRICK1_VALID (1)
#    define BOARD_ADC_BRICK2_VALID (1)
#  endif
#  if !defined(ADC_BATTERY1_VOLTAGE_CHANNEL)
#    define ADC_BATTERY1_VOLTAGE_CHANNEL (-1)
#  endif
#  if !defined(ADC_BATTERY1_CURRENT_CHANNEL)
#    define ADC_BATTERY1_CURRENT_CHANNEL (-1)
#  endif
#  if !defined(ADC_BATTERY2_VOLTAGE_CHANNEL)
#    define ADC_BATTERY2_VOLTAGE_CHANNEL (-1)
#  endif
#  if !defined(ADC_BATTERY2_CURRENT_CHANNEL)
#    define ADC_BATTERY2_CURRENT_CHANNEL (-1)
#  endif
#else
#  if !defined(ADC_BATTERY_VOLTAGE_CHANNEL)
#    define ADC_BATTERY_VOLTAGE_CHANNEL (-1)
#  endif
#  if !defined(ADC_BATTERY_CURRENT_CHANNEL)
#    define ADC_BATTERY_CURRENT_CHANNEL (-1)
#  endif
#endif

#define CONFIG_NFILE_STREAMS 1
#define CONFIG_SCHED_WORKQUEUE 1
#define CONFIG_SCHED_HPWORK 1
#define CONFIG_SCHED_LPWORK 1

/** time in ms between checks for work in work queues **/
#define CONFIG_SCHED_WORKPERIOD 50000

#define CONFIG_SCHED_INSTRUMENTATION 1
