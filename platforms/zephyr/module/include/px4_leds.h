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
 * @file px4_leds.h
 *
 * LEDs of the px4,leds devicetree node.
 *
 * The board LEDs are the LEDs of PX4 (LED_BLUE, LED_RED, LED_SAFETY and
 * LED_GREEN of drivers/drv_board_led.h) that the node names, which PX4 turns
 * on and off.
 */

#pragma once

#include <stdbool.h>

#include <zephyr/devicetree.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PX4_LEDS_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(px4_leds)

/**
 * struct led_dt_spec initializer of an LED node of px4,leds: the LED of a
 * leds-group-multicolor, or an LED of the LED controller of its parent.
 */
#define PX4_LED_DT_SPEC_GET(node) \
	COND_CODE_1(DT_NODE_HAS_COMPAT(node, leds_group_multicolor), \
	({.dev = DEVICE_DT_GET(node), .index = 0}), (LED_DT_SPEC_GET(node)))

/**
 * Turn the board LEDs off.
 * @return 0, or -ENODEV if one of them is not ready
 */
int px4_board_leds_init(void);

/**
 * Turn a board LED on or off. A LED that the board doesn't have is ignored.
 * @param led LED_BLUE, LED_RED, LED_SAFETY or LED_GREEN
 */
void px4_board_led_set(int led, bool on);

/**
 * Turn a board LED on if it is off, and off if it is on.
 * @param led LED_BLUE, LED_RED, LED_SAFETY or LED_GREEN
 */
void px4_board_led_toggle(int led);

#ifdef __cplusplus
}
#endif
