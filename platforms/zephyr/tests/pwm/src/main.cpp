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

#include "pwm_record.h"

#include <math.h>

#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <px4_zephyr.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/actuator_test.h>

#include <zephyr/ztest.h>

// MAIN1 drives motor 1 at 400 Hz, MAIN2 nothing, and MAIN3 servo 1 at 333 Hz, a digital servo (app.overlay)
static constexpr uint32_t MOTOR{0};
static constexpr uint32_t UNUSED{1};
static constexpr uint32_t SERVO{2};

static constexpr uint32_t MOTOR_PERIOD_US{2500};
static constexpr uint32_t SERVO_PERIOD_US{3003};

// output_functions.yaml
static constexpr int32_t SERVO1_FUNCTION{201};

static constexpr int32_t MOTOR_DISARMED{900};
static constexpr int32_t MOTOR_MIN{1100};
static constexpr int32_t MOTOR_MAX{1900};
static constexpr int32_t MOTOR_FAILSAFE{950};
static constexpr int32_t SERVO_DISARMED{1400};

// the default range of the servo, whose default failsafe value is its center
static constexpr int32_t SERVO_MIN{1000};
static constexpr int32_t SERVO_MAX{2000};
static constexpr int32_t SERVO_CENTER{1500};

// the range of the ESC calibration, whatever the minimum and maximum of the motors
static constexpr int32_t ESC_CALIBRATION_MIN{1000};
static constexpr int32_t ESC_CALIBRATION_MAX{2000};

// longer than the 500 ms ramp of the outputs after arming
static constexpr int PULSE_TIMEOUT_MS{1000};

// the driver reads the parameters once per second
static constexpr int PARAM_UPDATE_MS{1100};

static uORB::Publication<actuator_armed_s> armed_pub{ORB_ID(actuator_armed)};
static uORB::Publication<actuator_motors_s> motors_pub{ORB_ID(actuator_motors)};
static uORB::Publication<actuator_servos_s> servos_pub{ORB_ID(actuator_servos)};
static uORB::Publication<actuator_test_s> test_pub{ORB_ID(actuator_test)};

static int run(const char *command)
{
	char buffer[64];
	strncpy(buffer, command, sizeof(buffer) - 1);
	buffer[sizeof(buffer) - 1] = '\0';

	char *argv[4] {};
	int argc = 0;
	char *saveptr = nullptr;

	for (char *token = strtok_r(buffer, " ", &saveptr); token != nullptr && argc < 4;
	     token = strtok_r(nullptr, " ", &saveptr)) {
		argv[argc++] = token;
	}

	return px4_zephyr_run_command(argc, argv);
}

static void set_param(const char *name, int32_t value)
{
	zassert_ok(param_set(param_find(name), &value), "%s", name);
}

static void publish_armed(bool armed, bool termination = false, bool kill = false)
{
	actuator_armed_s actuator_armed{};
	actuator_armed.armed = armed;
	actuator_armed.termination = termination;
	actuator_armed.kill = kill;
	actuator_armed.timestamp = hrt_absolute_time();
	armed_pub.publish(actuator_armed);
}

// the servos first, as the motors trigger the update of the outputs
static void publish_controls(float motor, float servo)
{
	actuator_servos_s servos{};

	for (float &control : servos.control) {
		control = NAN;
	}

	servos.control[0] = servo;
	servos.timestamp = hrt_absolute_time();
	servos_pub.publish(servos);

	actuator_motors_s motors{};

	for (float &control : motors.control) {
		control = NAN;
	}

	motors.control[0] = motor;
	motors.timestamp = hrt_absolute_time();
	motors.timestamp_sample = motors.timestamp;
	motors_pub.publish(motors);
}

static void publish_motor_test(float value)
{
	actuator_test_s actuator_test{};
	actuator_test.action = actuator_test_s::ACTION_DO_CONTROL;
	actuator_test.function = actuator_test_s::FUNCTION_MOTOR1;
	actuator_test.value = value;
	actuator_test.timestamp = hrt_absolute_time();
	test_pub.publish(actuator_test);
}

static void check_pulse(uint32_t output, int32_t pulse_us, int timeout_ms = PULSE_TIMEOUT_MS)
{
	pwm_record record{};

	for (int i = 0; i < timeout_ms; i++) {
		pwm_record_get(output, &record);

		if ((record.writes > 0) && (record.pulse_us == static_cast<uint32_t>(pulse_us))) {
			return;
		}

		k_sleep(K_MSEC(1));
	}

	zassert_unreachable("MAIN%u: %u us instead of %d us", output + 1, record.pulse_us, pulse_us);
}

static void *setup(void)
{
	zassert_ok(px4_zephyr_init());
	return nullptr;
}

static void before(void *fixture)
{
	set_param("PWM_MAIN_FUNC1", actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1);
	set_param("PWM_MAIN_DIS1", MOTOR_DISARMED);
	set_param("PWM_MAIN_MIN1", MOTOR_MIN);
	set_param("PWM_MAIN_MAX1", MOTOR_MAX);
	set_param("PWM_MAIN_FAIL1", MOTOR_FAILSAFE);
	set_param("PWM_MAIN_FUNC3", SERVO1_FUNCTION);
	set_param("PWM_MAIN_DIS3", SERVO_DISARMED);

	pwm_record_start();
	publish_armed(false);
	zassert_ok(run("zephyr_pwm_out start"));

	// the output functions are only set while disarmed
	check_pulse(MOTOR, MOTOR_DISARMED);
	check_pulse(SERVO, SERVO_DISARMED);
}

static void after(void *fixture)
{
	run("zephyr_pwm_out stop");
}

ZTEST(px4_pwm, test_disarmed)
{
	publish_controls(0.5f, 0.5f);
	k_sleep(K_MSEC(PULSE_TIMEOUT_MS));

	pwm_record record{};
	pwm_record_get(MOTOR, &record);
	zassert_equal(record.pulse_us, MOTOR_DISARMED);
	pwm_record_get(SERVO, &record);
	zassert_equal(record.pulse_us, SERVO_DISARMED);

	// no pulses on an output without a function
	check_pulse(UNUSED, 0);

	// the periods of the devicetree
	pwm_record_get(MOTOR, &record);
	zassert_equal(record.period_us, MOTOR_PERIOD_US);
	pwm_record_get(UNUSED, &record);
	zassert_equal(record.period_us, MOTOR_PERIOD_US);
	pwm_record_get(SERVO, &record);
	zassert_equal(record.period_us, SERVO_PERIOD_US);
}

ZTEST(px4_pwm, test_armed)
{
	publish_armed(true);

	publish_controls(0.f, -1.f);
	check_pulse(MOTOR, MOTOR_MIN);
	check_pulse(SERVO, SERVO_MIN);

	publish_controls(1.f, 1.f);
	check_pulse(MOTOR, MOTOR_MAX);
	check_pulse(SERVO, SERVO_MAX);

	publish_controls(0.5f, 0.5f);
	check_pulse(MOTOR, (MOTOR_MIN + MOTOR_MAX) / 2);
	check_pulse(SERVO, (SERVO_CENTER + SERVO_MAX) / 2);

	// a stopped motor is disarmed
	publish_controls(NAN, 0.5f);
	check_pulse(MOTOR, MOTOR_DISARMED);

	check_pulse(UNUSED, 0);
}

ZTEST(px4_pwm, test_failsafe)
{
	publish_armed(true, true);
	publish_controls(0.5f, 0.5f);
	check_pulse(MOTOR, MOTOR_FAILSAFE);
	check_pulse(SERVO, SERVO_CENTER);

	// the kill switch disarms the outputs
	publish_armed(true, false, true);
	publish_controls(0.5f, 0.5f);
	check_pulse(MOTOR, MOTOR_DISARMED);
	check_pulse(SERVO, SERVO_DISARMED);
}

ZTEST(px4_pwm, test_long_pulse)
{
	// a pulse longer than the period, out of the range of the parameter, is limited to the period
	set_param("PWM_MAIN_FAIL1", MOTOR_PERIOD_US + 100);
	publish_armed(true, true);
	check_pulse(MOTOR, MOTOR_PERIOD_US, PARAM_UPDATE_MS + PULSE_TIMEOUT_MS);
}

ZTEST(px4_pwm, test_esc_calibration)
{
	actuator_armed_s actuator_armed{};
	actuator_armed.in_esc_calibration_mode = true;
	actuator_armed.timestamp = hrt_absolute_time();
	armed_pub.publish(actuator_armed);

	publish_motor_test(1.f);
	check_pulse(MOTOR, ESC_CALIBRATION_MAX);

	publish_motor_test(0.f);
	check_pulse(MOTOR, ESC_CALIBRATION_MIN);
}

ZTEST(px4_pwm, test_functions_removed)
{
	publish_armed(true);
	publish_controls(0.5f, 0.5f);
	check_pulse(MOTOR, (MOTOR_MIN + MOTOR_MAX) / 2);
	check_pulse(SERVO, (SERVO_CENTER + SERVO_MAX) / 2);

	// the functions are only removed when disarmed, right after the disarmed pulses
	set_param("PWM_MAIN_FUNC1", 0);
	set_param("PWM_MAIN_FUNC3", 0);
	k_sleep(K_MSEC(PARAM_UPDATE_MS));
	publish_armed(false);

	check_pulse(MOTOR, 0);
	check_pulse(SERVO, 0);
}

ZTEST(px4_pwm, test_stop)
{
	publish_armed(true);
	publish_controls(0.5f, 0.5f);
	check_pulse(MOTOR, (MOTOR_MIN + MOTOR_MAX) / 2);

	// the outputs are left disarmed
	zassert_ok(run("zephyr_pwm_out stop"));

	pwm_record record{};
	pwm_record_get(MOTOR, &record);
	zassert_equal(record.pulse_us, MOTOR_DISARMED);
	pwm_record_get(SERVO, &record);
	zassert_equal(record.pulse_us, SERVO_DISARMED);
	pwm_record_get(UNUSED, &record);
	zassert_equal(record.pulse_us, 0);
}

ZTEST(px4_pwm, test_update_rate)
{
	// controls faster than the outputs
	static constexpr int UPDATES{250};
	static constexpr int UPDATE_INTERVAL_US{1600};

	publish_armed(true);
	publish_controls(0.f, 0.f);
	check_pulse(MOTOR, MOTOR_MIN);
	check_pulse(SERVO, SERVO_CENTER);

	// the motor output follows the mixer at its period, the slower servo output is set at most once per period
	pwm_record_start();
	const hrt_abstime start = hrt_absolute_time();

	for (int i = 1; i <= UPDATES; i++) {
		publish_controls((i % 100) * 0.01f, (i % 100) * 0.01f);
		k_sleep(K_USEC(UPDATE_INTERVAL_US));
	}

	pwm_record motor{};
	pwm_record servo{};
	pwm_record_get(MOTOR, &motor);
	pwm_record_get(SERVO, &servo);
	const uint32_t duration_us = hrt_elapsed_time(&start);

	zassert_true(motor.writes > duration_us / MOTOR_PERIOD_US * 3 / 4, "%u motor writes in %u us", motor.writes,
		     duration_us);
	zassert_true(servo.writes <= duration_us / SERVO_PERIOD_US + 1, "%u servo writes in %u us", servo.writes,
		     duration_us);

	zassert_true(servo.min_interval_us >= SERVO_PERIOD_US, "%u us between servo writes", servo.min_interval_us);
}

ZTEST_SUITE(px4_pwm, NULL, setup, before, after, NULL);
