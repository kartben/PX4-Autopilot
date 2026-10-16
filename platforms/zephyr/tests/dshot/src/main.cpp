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

#include "fake_dshot.h"
#include "pwm_record.h"

#include <initializer_list>
#include <math.h>

#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <px4_zephyr.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

#include <zephyr/ztest.h>

using namespace time_literals;

// app.overlay: MAIN1 and MAIN2 drive motors 1 and 2 with DShot600 on channels 3 and 1 of fake_dshot1, MAIN3
// and MAIN4 motors 3 and 4 with bidirectional DShot600 on channel 0 of fake_dshot2 and of fake_dshot1, MAIN5
// and MAIN6 servos 1 and 2 with PWM, MAIN6 though it has a DShot channel, and MAIN7 motor 5 with
// bidirectional DShot600, though it has no DShot channel
static constexpr fake_dshot dshot1{FAKE_DSHOT1};
static constexpr fake_dshot dshot2{FAKE_DSHOT2};

static constexpr uint32_t MAIN1_CHANNEL{3};
static constexpr uint32_t MAIN2_CHANNEL{1};
static constexpr uint32_t MAIN3_CHANNEL{0};
static constexpr uint32_t MAIN4_CHANNEL{0};
static constexpr uint32_t DSHOT1_CHANNELS{BIT(MAIN1_CHANNEL) | BIT(MAIN2_CHANNEL) | BIT(MAIN4_CHANNEL)};
static constexpr uint32_t DSHOT2_CHANNELS{BIT(MAIN3_CHANNEL)};

// the outputs of the fake PWM, from 0
static constexpr size_t DSHOT_OUTPUTS[] {0, 1, 2, 3, 6};
static constexpr size_t SERVO1_OUTPUT{4};
static constexpr size_t SERVO2_OUTPUT{5};

// esc_status, in motor order
static constexpr int MOTOR3{2};
static constexpr int MOTOR4{3};
static constexpr int MOTOR5{4};

// PWM_MAIN_TIM<n> (module_dshot.yaml of zephyr_pwm_out)
static constexpr int32_t PROTOCOL_PWM{0};
static constexpr int32_t PROTOCOL_DSHOT600{-3};
static constexpr int32_t PROTOCOL_BDSHOT600{-6};
static constexpr uint32_t DSHOT600_BIT_RATE{600000};

// output_functions.yaml
static constexpr int32_t MOTOR1_FUNCTION{101};
static constexpr int32_t SERVO1_FUNCTION{201};

static constexpr int32_t SERVO_DISARMED{1400};
static constexpr int32_t SERVO_MAX{2000};

// DShot values: the commands, then the throttle from 48
static constexpr uint16_t DSHOT_MOTOR_STOP{0};
static constexpr uint16_t DSHOT_SPIN_DIRECTION_2{8};
static constexpr uint16_t DSHOT_SAVE_SETTINGS{12};
static constexpr uint16_t DSHOT_EXTENDED_TELEMETRY_ENABLE{13};
static constexpr uint16_t DSHOT_THROTTLE_MIN{48};
static constexpr uint16_t DSHOT_THROTTLE_MAX{2047};

// the throttle of the default DSHOT_MIN, a fraction of the 1999 throttle steps, and the middle of the range from it
static constexpr float DSHOT_MIN{0.055f};
static constexpr uint16_t THROTTLE_IDLE{DSHOT_THROTTLE_MIN + static_cast<uint16_t>(DSHOT_MIN * 1999)};
static constexpr uint16_t THROTTLE_HALF{THROTTLE_IDLE + (DSHOT_THROTTLE_MAX - THROTTLE_IDLE) / 2};

// commands sent while disarmed, and the save of the settings after one that changes them (DShot.cpp)
static constexpr int COMMAND_REPETITIONS{10};

// extended DShot telemetry
static constexpr uint8_t EDT_TEMPERATURE{0x02};
static constexpr uint8_t EDT_VOLTAGE{0x04};
static constexpr uint8_t EDT_CURRENT{0x06};

// replies of the ESCs, valid or not in a row, after which they are online or offline
static constexpr int ONLINE_REPLIES{200};

// the dshot driver ignores the telemetry for 5 s after boot, for the ESCs to start
static constexpr hrt_abstime TELEMETRY_START{5_s};

static constexpr int SEND_TIMEOUT_MS{1000};

static uORB::Publication<actuator_armed_s> armed_pub{ORB_ID(actuator_armed)};
static uORB::Publication<actuator_motors_s> motors_pub{ORB_ID(actuator_motors)};
static uORB::Publication<actuator_servos_s> servos_pub{ORB_ID(actuator_servos)};
static uORB::Publication<vehicle_command_s> command_pub{ORB_ID(vehicle_command)};

// a DShot frame: the 11-bit value, the telemetry request bit, and the XOR of their nibbles, inverted for
// bidirectional DShot
static uint16_t frame(uint16_t value, bool telemetry, bool bidirectional)
{
	const uint16_t data = (value << 1) | (telemetry ? 1 : 0);
	const uint16_t crc = (data ^ (data >> 4) ^ (data >> 8)) & 0xF;

	return (data << 4) | (bidirectional ? (~crc & 0xF) : crc);
}

// the reply of an ESC: the 12-bit payload and its checksum, the inverted XOR of its nibbles, GCR encoded, each
// 1 a transition of the line, which is low after the start bit; the 20 line levels after the start bit, the
// first one in bit 19
static uint32_t reply(uint16_t payload, uint16_t crc_error = 0)
{
	static constexpr uint8_t gcr[16] {
		0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17, 0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F
	};

	const uint16_t crc = (~(payload ^ (payload >> 4) ^ (payload >> 8)) ^ crc_error) & 0xF;
	const uint16_t word = (payload << 4) | crc;
	uint32_t code = 0;

	for (int nibble = 3; nibble >= 0; nibble--) {
		code = (code << 5) | gcr[(word >> (4 * nibble)) & 0xF];
	}

	uint32_t levels = 0;
	bool high = false;

	for (int bit = 19; bit >= 0; bit--) {
		if (code & BIT(bit)) {
			high = !high;
		}

		levels |= (high ? 1u : 0u) << bit;
	}

	return levels;
}

// the period of an electrical revolution in us: a 9-bit mantissa shifted by a 3-bit exponent
static constexpr uint16_t erpm_payload(uint16_t mantissa, uint16_t exponent)
{
	return (exponent << 9) | mantissa;
}

static constexpr uint16_t edt_payload(uint8_t type, uint8_t value)
{
	return (type << 8) | value;
}

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

static void publish_armed(bool armed)
{
	actuator_armed_s actuator_armed{};
	actuator_armed.armed = armed;
	actuator_armed.timestamp = hrt_absolute_time();
	armed_pub.publish(actuator_armed);
}

static void publish_servos(float servo)
{
	actuator_servos_s servos{};

	for (float &control : servos.control) {
		control = servo;
	}

	servos.timestamp = hrt_absolute_time();
	servos_pub.publish(servos);
}

// publish the motors, and wait for the frames of the update on both DShot devices
static void update(float motor = NAN)
{
	const uint32_t sends1 = fake_dshot_send_count(dshot1);
	const uint32_t sends2 = fake_dshot_send_count(dshot2);

	actuator_motors_s motors{};

	for (float &control : motors.control) {
		control = motor;
	}

	motors.timestamp = hrt_absolute_time();
	motors.timestamp_sample = motors.timestamp;
	motors_pub.publish(motors);

	for (int i = 0; i < SEND_TIMEOUT_MS * 10; i++) {
		if ((fake_dshot_send_count(dshot1) > sends1) && (fake_dshot_send_count(dshot2) > sends2)) {
			return;
		}

		k_sleep(K_USEC(100));
	}

	zassert_unreachable("no frames sent");
}

static fake_dshot_send last_send(fake_dshot dshot)
{
	fake_dshot_send send{};
	zassert_true(fake_dshot_get_send(dshot, fake_dshot_send_count(dshot) - 1, &send));
	return send;
}

// update the motors until the last frames are the ones of a value on all the DShot outputs
static void check_frames(float motor, uint16_t value, int max_updates = 10)
{
	fake_dshot_send send1{};
	fake_dshot_send send2{};

	for (int i = 0; i < max_updates; i++) {
		update(motor);
		send1 = last_send(dshot1);
		send2 = last_send(dshot2);

		if ((send1.frames[MAIN1_CHANNEL] == frame(value, false, false))
		    && (send1.frames[MAIN2_CHANNEL] == frame(value, false, false))
		    && (send1.frames[MAIN4_CHANNEL] == frame(value, false, true))
		    && (send2.frames[MAIN3_CHANNEL] == frame(value, false, true))) {
			return;
		}
	}

	zassert_unreachable("value %u: frames 0x%04x 0x%04x 0x%04x 0x%04x", value, send1.frames[MAIN1_CHANNEL],
			    send1.frames[MAIN2_CHANNEL], send2.frames[MAIN3_CHANNEL], send1.frames[MAIN4_CHANNEL]);
}

static void set_replies(fake_dshot dshot, uint32_t channel, std::initializer_list<uint32_t> replies,
			uint32_t *replied = nullptr, uint32_t *unreplied = nullptr)
{
	fake_dshot_set_replies(dshot, channel, replies.begin(), replies.size(), replied, unreplied);
}

static void wait_telemetry_start()
{
	while (hrt_absolute_time() < TELEMETRY_START) {
		k_sleep(K_MSEC(10));
	}
}

// update the motors until esc_status meets a condition
template<typename Condition>
static esc_status_s wait_esc_status(Condition condition, int max_updates = 4 * ONLINE_REPLIES)
{
	uORB::Subscription esc_status_sub{ORB_ID(esc_status)};
	esc_status_s esc_status{};
	const hrt_abstime start = hrt_absolute_time();

	for (int i = 0; i < max_updates; i++) {
		update();

		// not the last status of the previous test
		if (esc_status_sub.update(&esc_status) && (esc_status.timestamp > start) && condition(esc_status)) {
			return esc_status;
		}
	}

	zassert_unreachable("esc_status: online 0x%x, errors %u %u", esc_status.esc_online_flags,
			    esc_status.esc[MOTOR3].esc_errorcount, esc_status.esc[MOTOR4].esc_errorcount);
	return esc_status;
}

static bool online(const esc_status_s &esc_status, int motor)
{
	return esc_status.esc_online_flags & BIT(motor);
}

static void start_dshot()
{
	zassert_ok(run("dshot start"));

	// the outputs are configured on the first update of the outputs, and sent from the next one
	update();
}

static void *setup(void)
{
	zassert_ok(px4_zephyr_init());
	return nullptr;
}

static void before(void *fixture)
{
	fake_dshot_reset(dshot1);
	fake_dshot_reset(dshot2);

	for (int i = 0; i < 4; i++) {
		char name[17];
		snprintf(name, sizeof(name), "PWM_MAIN_FUNC%d", i + 1);
		set_param(name, MOTOR1_FUNCTION + i);
	}

	set_param("PWM_MAIN_FUNC5", SERVO1_FUNCTION);
	set_param("PWM_MAIN_FUNC6", SERVO1_FUNCTION + 1);
	set_param("PWM_MAIN_FUNC7", MOTOR1_FUNCTION + 4);
	set_param("PWM_MAIN_DIS5", SERVO_DISARMED);
	set_param("PWM_MAIN_DIS6", SERVO_DISARMED);

	set_param("PWM_MAIN_TIM0", PROTOCOL_DSHOT600);
	set_param("PWM_MAIN_TIM1", PROTOCOL_DSHOT600);
	set_param("PWM_MAIN_TIM2", PROTOCOL_BDSHOT600);
	set_param("PWM_MAIN_TIM3", PROTOCOL_BDSHOT600);
	set_param("PWM_MAIN_TIM4", PROTOCOL_PWM);
	set_param("PWM_MAIN_TIM5", PROTOCOL_PWM);
	set_param("PWM_MAIN_TIM6", PROTOCOL_BDSHOT600);

	set_param("DSHOT_MOT_POL3", 12);
	set_param("DSHOT_BIDIR_EDT", 0);

	pwm_record_start();
	publish_armed(false);
	publish_servos(NAN);

	zassert_ok(run("zephyr_pwm_out start"));
	start_dshot();
}

static void after(void *fixture)
{
	run("dshot stop");
	run("zephyr_pwm_out stop");

	// both devices send on each update
	zassert_equal(fake_dshot_send_count(dshot1), fake_dshot_send_count(dshot2));

	fake_dshot_config config{};
	fake_dshot_get_config(dshot1, &config);
	zassert_equal(config.invalid_calls, 0);
	fake_dshot_get_config(dshot2, &config);
	zassert_equal(config.invalid_calls, 0);
}

ZTEST(px4_dshot, test_configuration)
{
	fake_dshot_config config{};

	// the motors with DShot on the channels of their dshot property, bidirectional or not, but not MAIN6, a
	// servo, nor MAIN7, without DShot channel
	fake_dshot_get_config(dshot1, &config);
	zassert_equal(config.channels, DSHOT1_CHANNELS);
	zassert_equal(config.bidirectional_channels, BIT(MAIN4_CHANNEL));
	zassert_equal(config.bit_rate, DSHOT600_BIT_RATE);
	zassert_equal(config.configure_calls, 1);

	fake_dshot_get_config(dshot2, &config);
	zassert_equal(config.channels, DSHOT2_CHANNELS);
	zassert_equal(config.bidirectional_channels, BIT(MAIN3_CHANNEL));
	zassert_equal(config.bit_rate, DSHOT600_BIT_RATE);
	zassert_equal(config.configure_calls, 1);
}

ZTEST(px4_dshot, test_disarmed)
{
	const uint32_t first_send = fake_dshot_send_count(dshot1);

	for (int i = 0; i < 20; i++) {
		update(0.5f);
	}

	// the motor stop command, with the inverted checksum on the bidirectional outputs, on all the channels of
	// a device at once
	for (uint32_t number = first_send; number < fake_dshot_send_count(dshot1); number++) {
		fake_dshot_send send{};
		zassert_true(fake_dshot_get_send(dshot1, number, &send));
		zassert_equal(send.channels, DSHOT1_CHANNELS);
		zassert_equal(send.frames[MAIN1_CHANNEL], frame(DSHOT_MOTOR_STOP, false, false));
		zassert_equal(send.frames[MAIN2_CHANNEL], frame(DSHOT_MOTOR_STOP, false, false));
		zassert_equal(send.frames[MAIN4_CHANNEL], frame(DSHOT_MOTOR_STOP, false, true));
	}

	fake_dshot_send send = last_send(dshot2);
	zassert_equal(send.channels, DSHOT2_CHANNELS);
	zassert_equal(send.frames[MAIN3_CHANNEL], frame(DSHOT_MOTOR_STOP, false, true));
}

ZTEST(px4_dshot, test_armed)
{
	publish_armed(true);

	check_frames(0.f, THROTTLE_IDLE);
	check_frames(1.f, DSHOT_THROTTLE_MAX);
	check_frames(0.5f, THROTTLE_HALF);

	// a stopped motor
	check_frames(NAN, DSHOT_MOTOR_STOP);

	publish_armed(false);
}

ZTEST(px4_dshot, test_pwm_outputs)
{
	pwm_record record{};

	// the servos of the PWM outputs, MAIN6 though it has a DShot channel
	for (int i = 0; i < 1000; i++) {
		pwm_record_get(SERVO2_OUTPUT, &record);

		if (record.pulse_us == SERVO_DISARMED) {
			break;
		}

		k_sleep(K_MSEC(1));
	}

	zassert_equal(record.pulse_us, SERVO_DISARMED);
	pwm_record_get(SERVO1_OUTPUT, &record);
	zassert_equal(record.pulse_us, SERVO_DISARMED);

	publish_armed(true);

	for (int i = 0; i < 1000; i++) {
		publish_servos(1.f);
		k_sleep(K_MSEC(1));
		pwm_record_get(SERVO1_OUTPUT, &record);

		if (record.pulse_us == SERVO_MAX) {
			break;
		}
	}

	zassert_equal(record.pulse_us, SERVO_MAX, "%u us", record.pulse_us);

	// the servos are left disarmed
	zassert_ok(run("zephyr_pwm_out stop"));
	pwm_record_get(SERVO1_OUTPUT, &record);
	zassert_equal(record.pulse_us, SERVO_DISARMED);
	pwm_record_get(SERVO2_OUTPUT, &record);
	zassert_equal(record.pulse_us, SERVO_DISARMED);

	publish_armed(false);

	// never a pulse on the DShot outputs, and the DShot frames still sent
	for (size_t output : DSHOT_OUTPUTS) {
		pwm_record_get(output, &record);
		zassert_equal(record.writes, 0, "MAIN%u: %u writes", static_cast<unsigned>(output + 1), record.writes);
	}

	update();
}

ZTEST(px4_dshot, test_command)
{
	uORB::Subscription ack_sub{ORB_ID(vehicle_command_ack)};

	vehicle_command_s command{};
	command.command = vehicle_command_s::VEHICLE_CMD_CONFIGURE_ACTUATOR;
	command.param1 = vehicle_command_s::ACTUATOR_CONFIGURATION_SPIN_DIRECTION2;
	// MAVLink ACTUATOR_OUTPUT_FUNCTION_MOTOR1
	command.param5 = 1.f;
	command.timestamp = hrt_absolute_time();
	command_pub.publish(command);

	const uint32_t first_send = fake_dshot_send_count(dshot1);

	for (int i = 0; i < 3 * COMMAND_REPETITIONS; i++) {
		update();
	}

	// the command on motor 1, with the telemetry request bit for the ESCs that need it, then the save of the
	// settings, and the motor stop on the other motors
	int spin_direction = 0;
	int save_settings = 0;
	int motor_stop = 0;

	for (uint32_t number = first_send; number < fake_dshot_send_count(dshot1); number++) {
		fake_dshot_send send{};
		zassert_true(fake_dshot_get_send(dshot1, number, &send));

		const uint16_t main1 = send.frames[MAIN1_CHANNEL];

		if (main1 == frame(DSHOT_SPIN_DIRECTION_2, true, false)) {
			zassert_equal(save_settings, 0);
			spin_direction++;

		} else if (main1 == frame(DSHOT_SAVE_SETTINGS, true, false)) {
			zassert_equal(spin_direction, COMMAND_REPETITIONS);
			save_settings++;

		} else {
			zassert_equal(main1, frame(DSHOT_MOTOR_STOP, false, false));
			zassert_true((spin_direction == 0) || (save_settings == COMMAND_REPETITIONS));
			motor_stop++;
		}

		zassert_equal(send.frames[MAIN2_CHANNEL], frame(DSHOT_MOTOR_STOP, false, false));
		zassert_equal(send.frames[MAIN4_CHANNEL], frame(DSHOT_MOTOR_STOP, false, true));
	}

	zassert_equal(spin_direction, COMMAND_REPETITIONS);
	zassert_equal(save_settings, COMMAND_REPETITIONS);
	zassert_true(motor_stop > 0);

	vehicle_command_ack_s ack{};
	zassert_true(ack_sub.update(&ack));
	zassert_equal(ack.command, vehicle_command_s::VEHICLE_CMD_CONFIGURE_ACTUATOR);
	zassert_equal(ack.result, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
}

ZTEST(px4_dshot, test_erpm)
{
	wait_telemetry_start();

	// a period of 500 us on motor 3, of 12 poles, and of 1000 us on motor 4, of 14
	uint32_t unreplied3 = 0;
	uint32_t unreplied4 = 0;
	set_replies(dshot2, MAIN3_CHANNEL, {reply(erpm_payload(500, 0))}, nullptr, &unreplied3);
	set_replies(dshot1, MAIN4_CHANNEL, {reply(erpm_payload(500, 1))}, nullptr, &unreplied4);

	const esc_status_s esc_status = wait_esc_status([](const esc_status_s & status) {
		return online(status, MOTOR3) && online(status, MOTOR4);
	});

	zassert_equal(esc_status.esc_count, 5);
	zassert_equal(esc_status.esc_connectiontype, esc_status_s::ESC_CONNECTION_TYPE_DSHOT);
	zassert_equal(esc_status.esc[MOTOR3].esc_rpm, 60000000 / 500 / 6);
	zassert_equal(esc_status.esc[MOTOR4].esc_rpm, 60000000 / 1000 / 7);

	// the frames sent before the replies were set got none
	zassert_equal(esc_status.esc[MOTOR3].esc_errorcount, unreplied3);
	zassert_equal(esc_status.esc[MOTOR4].esc_errorcount, unreplied4);

	// no DShot channel for motor 5
	zassert_false(online(esc_status, MOTOR5));
}

ZTEST(px4_dshot, test_extended_telemetry)
{
	run("dshot stop");
	set_param("DSHOT_BIDIR_EDT", 1);
	start_dshot();

	wait_telemetry_start();

	// eRPM replies alternate with the temperature, voltage (0.25 V) and current of the ESC
	const uint32_t erpm3 = reply(erpm_payload(500, 0));
	const uint32_t erpm4 = reply(erpm_payload(500, 1));
	set_replies(dshot2, MAIN3_CHANNEL, {erpm3, reply(edt_payload(EDT_TEMPERATURE, 45)), erpm3,
					    reply(edt_payload(EDT_VOLTAGE, 64)), erpm3, reply(edt_payload(EDT_CURRENT, 12))
					   });
	set_replies(dshot1, MAIN4_CHANNEL, {erpm4, reply(edt_payload(EDT_TEMPERATURE, 50)), erpm4,
					    reply(edt_payload(EDT_VOLTAGE, 50)), erpm4, reply(edt_payload(EDT_CURRENT, 3))
					   });

	const esc_status_s esc_status = wait_esc_status([](const esc_status_s & status) {
		return online(status, MOTOR3) && online(status, MOTOR4) && (status.esc[MOTOR3].esc_current > 0.f)
		       && (status.esc[MOTOR4].esc_current > 0.f);
	});

	zassert_equal(esc_status.esc[MOTOR3].esc_rpm, 60000000 / 500 / 6);
	zassert_equal(esc_status.esc[MOTOR3].esc_temperature, 45.f);
	zassert_equal(esc_status.esc[MOTOR3].esc_voltage, 16.f);
	zassert_equal(esc_status.esc[MOTOR3].esc_current, 12.f);
	zassert_equal(esc_status.esc[MOTOR4].esc_rpm, 60000000 / 1000 / 7);
	zassert_equal(esc_status.esc[MOTOR4].esc_temperature, 50.f);
	zassert_equal(esc_status.esc[MOTOR4].esc_voltage, 12.5f);
	zassert_equal(esc_status.esc[MOTOR4].esc_current, 3.f);

	// the request of the extended telemetry, once per ESC as both reply with it, a second after they are online
	int requests3 = 0;
	int requests4 = 0;
	uint32_t number1 = fake_dshot_send_count(dshot1);
	uint32_t number2 = fake_dshot_send_count(dshot2);

	for (int i = 0; (i < 4000) && ((requests3 < COMMAND_REPETITIONS) || (requests4 < COMMAND_REPETITIONS)); i++) {
		update();

		for (fake_dshot_send send{}; fake_dshot_get_send(dshot1, number1, &send); number1++) {
			requests4 += send.frames[MAIN4_CHANNEL] == frame(DSHOT_EXTENDED_TELEMETRY_ENABLE, true, true);
		}

		for (fake_dshot_send send{}; fake_dshot_get_send(dshot2, number2, &send); number2++) {
			requests3 += send.frames[MAIN3_CHANNEL] == frame(DSHOT_EXTENDED_TELEMETRY_ENABLE, true, true);
		}
	}

	zassert_equal(requests3, COMMAND_REPETITIONS);
	zassert_equal(requests4, COMMAND_REPETITIONS);

	set_param("DSHOT_BIDIR_EDT", 0);
}

ZTEST(px4_dshot, test_reply_errors)
{
	static constexpr int BAD_REPLIES{20};

	wait_telemetry_start();

	const uint32_t valid = reply(erpm_payload(500, 0));
	uint32_t unreplied = 0;
	set_replies(dshot2, MAIN3_CHANNEL, {valid}, nullptr, &unreplied);
	set_replies(dshot1, MAIN4_CHANNEL, {reply(erpm_payload(500, 1))});

	wait_esc_status([](const esc_status_s & status) {
		return online(status, MOTOR3) && online(status, MOTOR4);
	});

	// a wrong checksum, then a reply whose last level is low, as if the receiver lost a transition
	uint32_t crc_errors = 0;
	uint32_t frame_errors = 0;
	set_replies(dshot2, MAIN3_CHANNEL, {reply(erpm_payload(500, 0), 0x8)});

	for (int i = 0; i < BAD_REPLIES; i++) {
		update();
	}

	set_replies(dshot2, MAIN3_CHANNEL, {valid & ~1u}, &crc_errors);

	for (int i = 0; i < BAD_REPLIES; i++) {
		update();
	}

	set_replies(dshot2, MAIN3_CHANNEL, {valid}, &frame_errors);
	zassert_true(crc_errors >= BAD_REPLIES);
	zassert_true(frame_errors >= BAD_REPLIES);

	const uint32_t errors = unreplied + crc_errors + frame_errors;
	const esc_status_s esc_status = wait_esc_status([errors](const esc_status_s & status) {
		return status.esc[MOTOR3].esc_errorcount == errors;
	}, BAD_REPLIES);

	// too few to take the ESC offline
	zassert_true(online(esc_status, MOTOR3));
	zassert_equal(esc_status.esc[MOTOR3].esc_rpm, 60000000 / 500 / 6);

	zassert_ok(run("dshot status"));
}

ZTEST(px4_dshot, test_offline)
{
	wait_telemetry_start();

	set_replies(dshot2, MAIN3_CHANNEL, {reply(erpm_payload(500, 0))});
	set_replies(dshot1, MAIN4_CHANNEL, {reply(erpm_payload(500, 1))});

	wait_esc_status([](const esc_status_s & status) {
		return online(status, MOTOR3) && online(status, MOTOR4);
	});

	// the ESC of motor 3 stops replying
	set_replies(dshot2, MAIN3_CHANNEL, {});

	const esc_status_s esc_status = wait_esc_status([](const esc_status_s & status) {
		return !online(status, MOTOR3);
	});

	zassert_true(online(esc_status, MOTOR4));

	// and replies again
	set_replies(dshot2, MAIN3_CHANNEL, {reply(erpm_payload(500, 0))});

	wait_esc_status([](const esc_status_s & status) {
		return online(status, MOTOR3);
	});
}

ZTEST(px4_dshot, test_busy)
{
	// while the reply to its last frame can still arrive, a channel waits for the next update
	fake_dshot_set_busy(dshot1, MAIN4_CHANNEL, true);
	update();
	update();
	zassert_equal(last_send(dshot1).channels, DSHOT1_CHANNELS & ~BIT(MAIN4_CHANNEL));
	zassert_equal(last_send(dshot2).channels, DSHOT2_CHANNELS);

	fake_dshot_set_busy(dshot1, MAIN4_CHANNEL, false);
	update();
	zassert_equal(last_send(dshot1).channels, DSHOT1_CHANNELS);
}

ZTEST_SUITE(px4_dshot, NULL, setup, before, after, NULL);
