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


#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LED_RECORD_MAX_EVENTS 256

/* a color set on the fake RGB LED */
struct led_event {
	uint64_t time_us;
	uint8_t red;
	uint8_t green;
	uint8_t blue;
	uint8_t white;
};

/* the LEDs of the gpio-leds node of the test */
enum test_gpio_led {
	TEST_LED_BLUE,
	TEST_LED_RED,
	TEST_LED_GREEN,
	TEST_LED_RGB_RED,
	TEST_LED_RGB_GREEN,
	TEST_LED_RGB_BLUE,
};

/* record the colors set on the fake RGB LED, from now on */
void led_record_start(void);

/* copy the events recorded so far, up to max_events, and return their number */
size_t led_record_get(struct led_event *events, size_t max_events);

/* the level of the GPIO pin of an LED */
int test_led_level(enum test_gpio_led led);

/* the LED is lit, from the level of its pin and its polarity */
bool test_led_lit(enum test_gpio_led led);

/* light a GPIO LED, or turn it off, through its pin */
void test_led_set(enum test_gpio_led led, bool lit);

/* the NCP5623C of the GPS module is ready */
bool test_gps_led_ready(void);

/* the NCP5623C that doesn't respond is ready */
bool test_absent_led_ready(void);

/* the registers of the NCP5623C */
#define NCP5623_CURRENT 1
#define NCP5623_PWM1 2
#define NCP5623_PWM2 3
#define NCP5623_PWM3 4

/* the value of a register of the NCP5623C of the GPS module, from 0 to 31 */
uint8_t test_gps_led_register(unsigned reg);

#ifdef __cplusplus
}
#endif
