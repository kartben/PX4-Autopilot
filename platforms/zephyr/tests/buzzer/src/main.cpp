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

#include "buzzer_record.h"

#include <drivers/drv_hrt.h>
#include <lib/circuit_breaker/circuit_breaker.h>
#include <lib/tunes/tunes.h>
#include <parameters/param.h>
#include <px4_zephyr.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/tune_control.h>

#include <zephyr/ztest.h>

// the end of a note and the start of the next one are late by the latency of the work queues
static constexpr uint32_t LATENCY_US{2000};

// time for the tone alarm to start a tune or to stop
static constexpr uint32_t RESPONSE_US{50000};

static constexpr uint16_t TONE_FREQUENCY{2000};
static constexpr uint32_t TONE_DURATION_US{100000};

struct note {
	unsigned frequency;
	unsigned duration_us;
	unsigned silence_us;
};

static uORB::Publication<tune_control_s> tune_control_pub{ORB_ID(tune_control)};

static int run(const char *command)
{
	char buffer[64];
	strncpy(buffer, command, sizeof(buffer) - 1);
	buffer[sizeof(buffer) - 1] = '\0';

	char *argv[6] {};
	int argc = 0;
	char *saveptr = nullptr;

	for (char *token = strtok_r(buffer, " ", &saveptr); token != nullptr && argc < 6;
	     token = strtok_r(nullptr, " ", &saveptr)) {
		argv[argc++] = token;
	}

	return px4_zephyr_run_command(argc, argv);
}

static void publish(tune_control_s &tune_control)
{
	tune_control.volume = tune_control_s::VOLUME_LEVEL_DEFAULT;
	tune_control.timestamp = hrt_absolute_time();
	tune_control_pub.publish(tune_control);
}

static void publish_tune(uint8_t tune_id)
{
	tune_control_s tune_control{};
	tune_control.tune_id = tune_id;
	publish(tune_control);
}

// a single tone, as the tune_control command publishes the notes of a melody
static void publish_tone(uint16_t frequency, uint32_t duration_us, bool tune_override = false)
{
	tune_control_s tune_control{};
	tune_control.tune_id = static_cast<uint8_t>(TuneID::CUSTOM);
	tune_control.tune_override = tune_override;
	tune_control.frequency = frequency;
	tune_control.duration = duration_us;
	publish(tune_control);
}

// the notes of a tune, as the tunes library plays them
static size_t tune_notes(uint8_t tune_id, note *notes, size_t max_notes)
{
	Tunes tunes;
	tune_control_s tune_control{};
	tune_control.tune_id = tune_id;
	zassert_equal(tunes.set_control(tune_control), Tunes::ControlResult::Success);

	size_t count = 0;
	note next{};

	while (tunes.get_next_note(next.frequency, next.duration_us, next.silence_us) == Tunes::Status::Continue) {
		zassert_true(count < max_notes);
		notes[count++] = next;
	}

	return count;
}

static uint32_t tune_duration_us(const note *notes, size_t count)
{
	uint32_t duration_us = 0;

	for (size_t i = 0; i < count; i++) {
		duration_us += notes[i].duration_us + notes[i].silence_us + LATENCY_US;
	}

	return duration_us;
}

static size_t get_events(buzzer_event *events)
{
	const size_t count = buzzer_record_get(events, BUZZER_RECORD_MAX_EVENTS);
	zassert_true(count <= BUZZER_RECORD_MAX_EVENTS, "%zu events", count);
	return count;
}

static void check_tone(const buzzer_event &event, unsigned frequency)
{
	const uint32_t period_ns = NSEC_PER_SEC / frequency;
	zassert_equal(event.period_ns, period_ns, "%u ns instead of %u ns (%u Hz)", event.period_ns, period_ns, frequency);

	// the loudest volume, a square wave
	zassert_equal(event.pulse_ns, period_ns / 2, "%u ns pulse", event.pulse_ns);
}

// check that the buzzer has played the notes and is silent
static void check_notes(const note *notes, size_t count)
{
	buzzer_event events[BUZZER_RECORD_MAX_EVENTS];
	const size_t event_count = get_events(events);

	size_t event = 0;

	// from the start of the previous tone to the start of the next one
	uint64_t previous_start_us = 0;
	uint32_t interval_us = 0;
	uint32_t interval_latency_us = 0;

	for (size_t i = 0; i < count; i++) {
		if (notes[i].frequency == 0) {
			// a rest, which the tone alarm schedules after its start
			interval_us += notes[i].duration_us + notes[i].silence_us;
			interval_latency_us += LATENCY_US;
			continue;
		}

		// the stops of the previous note and of the rests
		while ((event < event_count) && (events[event].pulse_ns == 0)) {
			event++;
		}

		zassert_true(event + 1 < event_count, "note %zu not played and ended", i);

		const buzzer_event &tone = events[event];
		check_tone(tone, notes[i].frequency);

		const uint32_t duration_us = events[event + 1].time_us - tone.time_us;
		zassert_between_inclusive(duration_us, notes[i].duration_us, notes[i].duration_us + LATENCY_US,
					  "note %zu: %u us instead of %u us", i, duration_us, notes[i].duration_us);

		if (previous_start_us != 0) {
			const uint32_t start_interval_us = tone.time_us - previous_start_us;
			zassert_between_inclusive(start_interval_us, interval_us, interval_us + interval_latency_us,
						  "note %zu: %u us after the previous one instead of %u us", i, start_interval_us, interval_us);
		}

		previous_start_us = tone.time_us;
		interval_us = notes[i].duration_us + notes[i].silence_us;
		interval_latency_us = LATENCY_US;
		event++;
	}

	while ((event < event_count) && (events[event].pulse_ns == 0)) {
		event++;
	}

	zassert_equal(event, event_count, "tone after the last note");
	zassert_true((event_count > 0) && (events[event_count - 1].pulse_ns == 0), "buzzer not silent");
}

static void *setup(void)
{
	zassert_ok(px4_zephyr_init());
	return nullptr;
}

static void before(void *fixture)
{
	// the tone alarm plays the last tune_control message when it starts: a stop
	tune_control_s stop{};
	stop.tune_override = true;
	publish(stop);

	zassert_ok(run("tone_alarm start"));
	k_sleep(K_USEC(RESPONSE_US));

	buzzer_record_start();
}

static void after(void *fixture)
{
	run("tone_alarm stop");
	param_reset(param_find("CBRK_BUZZER"));
}

ZTEST(px4_buzzer, test_tone)
{
	publish_tone(TONE_FREQUENCY, TONE_DURATION_US);

	const note tone{TONE_FREQUENCY, TONE_DURATION_US, 0};
	k_sleep(K_USEC(tune_duration_us(&tone, 1) + RESPONSE_US));
	check_notes(&tone, 1);
}

ZTEST(px4_buzzer, test_tune)
{
	publish_tune(tune_control_s::TUNE_ID_NOTIFY_POSITIVE);

	// MFT200e8a8a: at 200 beats per minute, the eighth notes E and A last 150 ms and the quarter note A 300 ms, the
	// last 1/8 of which is silent
	const note notes[] {
		{1318, 131250, 18750},
		{1760, 131250, 18750},
		{1760, 262500, 37500},
	};

	k_sleep(K_USEC(tune_duration_us(notes, ARRAY_SIZE(notes)) + RESPONSE_US));
	check_notes(notes, ARRAY_SIZE(notes));
}

ZTEST(px4_buzzer, test_startup_tune)
{
	note notes[BUZZER_RECORD_MAX_EVENTS / 2];
	const size_t count = tune_notes(tune_control_s::TUNE_ID_STARTUP, notes, ARRAY_SIZE(notes));

	// as in the startup script of a board
	zassert_ok(run("tune_control play -t 1"));

	k_sleep(K_USEC(tune_duration_us(notes, count) + RESPONSE_US));
	check_notes(notes, count);
}

ZTEST(px4_buzzer, test_override)
{
	// a note of 2.8 s, which only a tune that overrides it can interrupt
	note arming_warning;
	zassert_equal(tune_notes(tune_control_s::TUNE_ID_ARMING_WARNING, &arming_warning, 1), 1);
	publish_tune(tune_control_s::TUNE_ID_ARMING_WARNING);
	k_sleep(K_MSEC(100));

	const hrt_abstime override_time = hrt_absolute_time();
	publish_tone(TONE_FREQUENCY, TONE_DURATION_US, true);
	k_sleep(K_USEC(tune_duration_us(&arming_warning, 1)));

	buzzer_event events[BUZZER_RECORD_MAX_EVENTS];
	const size_t event_count = get_events(events);

	zassert_true(event_count >= 4, "%zu events", event_count);
	check_tone(events[0], arming_warning.frequency);

	// the arming warning stops, and the tone starts right away
	zassert_equal(events[1].pulse_ns, 0);
	zassert_between_inclusive(events[1].time_us, override_time, override_time + LATENCY_US);

	size_t event = 2;

	while ((event < event_count) && (events[event].pulse_ns == 0)) {
		event++;
	}

	zassert_true(event + 1 < event_count);
	check_tone(events[event], TONE_FREQUENCY);
	zassert_between_inclusive(events[event].time_us, override_time, override_time + LATENCY_US);

	const uint32_t duration_us = events[event + 1].time_us - events[event].time_us;
	zassert_between_inclusive(duration_us, TONE_DURATION_US, TONE_DURATION_US + LATENCY_US, "%u us", duration_us);

	// the arming warning doesn't resume
	for (event++; event < event_count; event++) {
		zassert_equal(events[event].pulse_ns, 0, "tone after the override");
	}
}

ZTEST(px4_buzzer, test_stop)
{
	note arming_warning;
	zassert_equal(tune_notes(tune_control_s::TUNE_ID_ARMING_WARNING, &arming_warning, 1), 1);
	publish_tune(tune_control_s::TUNE_ID_ARMING_WARNING);
	k_sleep(K_MSEC(100));

	// the driver stops in the middle of the note, and silences the buzzer
	const hrt_abstime stop_time = hrt_absolute_time();
	zassert_ok(run("tone_alarm stop"));
	const hrt_abstime stopped_time = hrt_absolute_time();
	zassert_true(stopped_time - stop_time < RESPONSE_US, "stopped in %u us", (uint32_t)(stopped_time - stop_time));

	k_sleep(K_USEC(tune_duration_us(&arming_warning, 1)));

	buzzer_event events[BUZZER_RECORD_MAX_EVENTS];
	const size_t event_count = get_events(events);

	zassert_true(event_count >= 2, "%zu events", event_count);
	check_tone(events[0], arming_warning.frequency);

	// nothing more at the end of the note
	for (size_t event = 1; event < event_count; event++) {
		zassert_equal(events[event].pulse_ns, 0);
		zassert_between_inclusive(events[event].time_us, stop_time, stopped_time, "event %u us after the stop",
					  (uint32_t)(events[event].time_us - stop_time));
	}
}

ZTEST(px4_buzzer, test_circuit_breaker)
{
	// the tone alarm stops when it starts
	zassert_ok(run("tone_alarm stop"));
	const int32_t circuit_breaker = CBRK_BUZZER_KEY;
	zassert_ok(param_set(param_find("CBRK_BUZZER"), &circuit_breaker));
	zassert_ok(run("tone_alarm start"));

	publish_tune(tune_control_s::TUNE_ID_NOTIFY_POSITIVE);
	k_sleep(K_USEC(RESPONSE_US));

	buzzer_event events[BUZZER_RECORD_MAX_EVENTS];
	const size_t event_count = get_events(events);

	for (size_t event = 0; event < event_count; event++) {
		zassert_equal(events[event].pulse_ns, 0, "tone played");
	}
}

ZTEST_SUITE(px4_buzzer, NULL, setup, before, after, NULL);
