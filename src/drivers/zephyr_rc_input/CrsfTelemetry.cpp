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

#include "CrsfTelemetry.hpp"
#include "zephyr_rc_input_backend.h"

#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>

#include <math.h>
#include <string.h>

// CRSF frame types
static constexpr uint8_t CRSF_GPS{0x02};
static constexpr uint8_t CRSF_BATTERY{0x08};
static constexpr uint8_t CRSF_BARO_ALTITUDE{0x09};
static constexpr uint8_t CRSF_ATTITUDE{0x1E};
static constexpr uint8_t CRSF_FLIGHT_MODE{0x21};

// big-endian payloads
class Payload
{
public:
	void put(uint32_t value, unsigned bytes)
	{
		for (unsigned i = bytes; i > 0; i--) {
			_buffer[_length++] = (value >> (8 * (i - 1))) & 0xff;
		}
	}

	const uint8_t *data() const { return _buffer; }

	size_t length() const { return _length; }

private:
	uint8_t _buffer[16] {};
	size_t _length{0};
};

void CrsfTelemetry::update(hrt_abstime now)
{
	if (now < _last + INTERVAL) {
		return;
	}

	_last = now;

	// the next frame that has data, in turn
	for (unsigned i = 0; i < 5; i++) {
		const unsigned type = _next;
		_next = (_next + 1) % 5;

		bool sent = false;

		switch (type) {
		case 0: sent = send_battery(); break;

		case 1: sent = send_gps(); break;

		case 2: sent = send_attitude(); break;

		case 3: sent = send_flight_mode(); break;

		case 4: sent = send_altitude(); break;
		}

		if (sent) {
			return;
		}
	}
}

bool CrsfTelemetry::send(uint8_t type, const uint8_t *payload, size_t length)
{
	if (zephyr_rc_input_send_telemetry(type, payload, length) == 0) {
		_sent++;
		return true;
	}

	return false;
}

bool CrsfTelemetry::send_battery()
{
	battery_status_s battery;

	if (!_battery_status_sub.update(&battery)) {
		return false;
	}

	Payload payload;
	payload.put(lroundf(math::constrain(battery.voltage_v * 10.f, 0.f, (float)UINT16_MAX)), 2);
	payload.put(lroundf(math::constrain(battery.current_a * 10.f, 0.f, (float)UINT16_MAX)), 2);
	payload.put(lroundf(math::constrain(battery.discharged_mah, 0.f, (float)0xFFFFFF)), 3);
	payload.put(lroundf(math::constrain(battery.remaining * 100.f, 0.f, 100.f)), 1);
	return send(CRSF_BATTERY, payload.data(), payload.length());
}

bool CrsfTelemetry::send_gps()
{
	vehicle_gnss_s gnss;

	if (!_vehicle_gnss_sub.update(&gnss)) {
		return false;
	}

	Payload payload;
	payload.put(static_cast<int32_t>(round(gnss.receiver.latitude * 1e7)), 4);
	payload.put(static_cast<int32_t>(round(gnss.receiver.longitude * 1e7)), 4);
	payload.put(lroundf(math::constrain(gnss.receiver.ground_speed * 3.6f * 10.f, 0.f, (float)UINT16_MAX)), 2); // 0.1 km/h
	payload.put(lroundf(math::degrees(matrix::wrap_2pi(gnss.receiver.course)) * 100.f), 2);
	payload.put(static_cast<int16_t>(gnss.receiver.altitude_msl) + 1000, 2); // m, + 1000 m
	payload.put(gnss.receiver.satellites_used, 1);
	return send(CRSF_GPS, payload.data(), payload.length());
}

bool CrsfTelemetry::send_attitude()
{
	vehicle_attitude_s attitude;

	if (!_vehicle_attitude_sub.update(&attitude)) {
		return false;
	}

	const matrix::Eulerf euler{matrix::Quatf(attitude.q)};

	Payload payload;
	payload.put(static_cast<int16_t>(euler.theta() * 1e4f), 2);
	payload.put(static_cast<int16_t>(euler.phi() * 1e4f), 2);
	payload.put(static_cast<int16_t>(euler.psi() * 1e4f), 2);
	return send(CRSF_ATTITUDE, payload.data(), payload.length());
}

static const char *flight_mode(const vehicle_status_s &status)
{
	switch (status.nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL: return "Manual";

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL: return "Altitude";

	case vehicle_status_s::NAVIGATION_STATE_ALTITUDE_CRUISE: return "Altitude Cruise";

	case vehicle_status_s::NAVIGATION_STATE_MANUAL_PARKING: return "Manual Parking";

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		return (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) ? "Cruise" : "Position";

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL: return "Return";

	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION: return "Mission";

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
		return "Auto";

	case vehicle_status_s::NAVIGATION_STATE_ACRO: return "Acro";

	case vehicle_status_s::NAVIGATION_STATE_TERMINATION: return "Terminate";

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD: return "Offboard";

	case vehicle_status_s::NAVIGATION_STATE_STAB: return "Stabilized";

	default: return "Unknown";
	}
}

bool CrsfTelemetry::send_flight_mode()
{
	vehicle_status_s status;

	if (!_vehicle_status_sub.update(&status)) {
		return false;
	}

	// with its terminating null character
	const char *mode = flight_mode(status);
	return send(CRSF_FLIGHT_MODE, reinterpret_cast<const uint8_t *>(mode), strlen(mode) + 1);
}

bool CrsfTelemetry::send_altitude()
{
	vehicle_local_position_s local_position;

	if (!_vehicle_local_position_sub.update(&local_position) || !local_position.z_valid) {
		return false;
	}

	// the altitude above the origin of the estimator, in dm + 1000 m, or in m with the MSB set
	// above the decimeter range, as crsf_rc sends it
	const float altitude_m = -local_position.z;
	const int32_t altitude_dm = lroundf(altitude_m * 10.f) + 10000;
	uint16_t altitude = 0;

	if (altitude_dm >= 0x8000) {
		altitude = math::min(lroundf(altitude_m), 0x7FFFL) | 0x8000;

	} else if (altitude_dm > 0) {
		altitude = altitude_dm;
	}

	int16_t vertical_speed = 0; // cm/s

	if (local_position.v_z_valid) {
		vertical_speed = lroundf(math::constrain(-local_position.vz * 100.f, (float)INT16_MIN, (float)INT16_MAX));
	}

	Payload payload;
	payload.put(altitude, 2);
	payload.put(static_cast<uint16_t>(vertical_speed), 2);
	return send(CRSF_BARO_ALTITUDE, payload.data(), payload.length());
}
