############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################


"""
Connect to PX4 with MAVLink over a serial port, which QEMU connects to a TCP
port of the host.
"""

import time

import pytest
from pymavlink import mavutil
from twister_harness import Shell

PARAM = 'MPC_Z_VEL_MAX_UP'


def px4(shell: Shell, command: str) -> list[str]:
    try:
        return shell.exec_command(f'px4 {command}')
    except AssertionError:
        # a PX4 message was printed over the echo of the command
        return []


def wait_until(condition, timeout: float, what: str):
    end = time.time() + timeout

    while time.time() < end:
        if condition():
            return

        time.sleep(1)

    raise AssertionError(f'timeout waiting for {what}')


@pytest.fixture(scope='session')
def mav(shell: Shell):
    # the startup script starts MAVLink on UDP last
    wait_until(lambda: any('instance #0' in line for line in px4(shell, 'mavlink status')), 300,
               'the startup script')

    px4(shell, 'mavlink start -d /dev/ttyS1 -b 115200')

    connection = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255)
    assert connection.wait_heartbeat(timeout=60), 'no heartbeat'

    yield connection

    connection.close()


def test_heartbeat(mav):
    heartbeat = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=30)

    assert heartbeat is not None
    assert heartbeat.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4


def test_param_set(shell: Shell, mav):
    mav.mav.param_set_send(mav.target_system, mav.target_component, PARAM.encode(), 2.5,
                           mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    value = mav.recv_match(type='PARAM_VALUE', condition=f'PARAM_VALUE.param_id == "{PARAM}"', blocking=True,
                           timeout=30)

    assert value is not None
    assert value.param_value == pytest.approx(2.5)
    wait_until(lambda: any('2.5' in line for line in px4(shell, f'param show {PARAM}')), 30, 'the parameter value')


def test_autopilot_version(mav):
    mav.mav.command_long_send(mav.target_system, mav.target_component, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                              mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0)

    version = mav.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=30)

    assert version is not None
    assert version.capabilities & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_MAVLINK2


def link_status(shell: Shell, device: str) -> str:
    output = '\n'.join(px4(shell, 'mavlink status'))

    for instance in output.split('instance #')[1:]:
        if f'serial ({device} ' in instance:
            return instance

    return ''


def test_no_loss(shell: Shell, mav):
    errors = mav.mav.total_receive_errors
    received = mav.mav_count
    end = time.time() + 120
    next_heartbeat = 0.0

    # heartbeats at 50 Hz, well below what the serial port carries
    while (mav.mav_count - received < 200) and (time.time() < end):
        if time.time() >= next_heartbeat:
            mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            next_heartbeat = time.time() + 0.02

        mav.recv_match(blocking=True, timeout=0.01)

    # PX4 drops the messages that don't fit in the transmit buffer, but sends
    # the others whole
    assert mav.mav_count - received >= 200, 'too few messages'
    assert mav.mav.total_receive_errors == errors, 'corrupted messages'
    wait_until(lambda: 'rx loss: 0.0%' in link_status(shell, '/dev/ttyS1'), 30, 'no message lost')
