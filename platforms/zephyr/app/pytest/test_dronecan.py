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
DroneCAN on the loopback CAN controller of QEMU, which receives the frames
that PX4 sends. The test sends the frames of another node with the CAN shell.
"""

import re
import struct
import time

import pytest
from twister_harness import Shell

PEER = 10


def run(shell: Shell, command: str) -> list[str]:
    try:
        return shell.exec_command(command)
    except AssertionError:
        # a PX4 message was printed over the echo of the command
        return []


def px4(shell: Shell, command: str) -> str:
    return '\n'.join(run(shell, f'px4 {command}'))


def wait_until(condition, timeout: float, what: str):
    end = time.time() + timeout

    while time.time() < end:
        if condition():
            return

        time.sleep(1)

    raise AssertionError(f'timeout waiting for {what}')


def broadcast(shell: Shell, data_type_id: int, payload: bytes, transfer_id: int):
    """Send a single-frame DroneCAN message of the peer node, at the default priority."""
    can_id = (16 << 24) | (data_type_id << 8) | PEER
    tail = 0xC0 | (transfer_id & 0x1F)  # start and end of the transfer
    data = ' '.join(f'{byte:02x}' for byte in payload + bytes([tail]))
    run(shell, f'can send can_loopback0 -e {can_id:x} {data}')


@pytest.fixture(scope='session')
def dronecan(shell: Shell):
    run(shell, 'can mode can_loopback0 loopback')
    px4(shell, 'uavcan start')


def test_transmit(shell: Shell, dronecan):
    def transmitting():
        match = re.search(r'TX frames: (\d+)', px4(shell, 'uavcan status'))
        return match is not None and int(match.group(1)) > 0

    wait_until(transmitting, 30, 'frames sent by PX4')


def test_peer_node(shell: Shell, dronecan):
    uptime = 0

    def online():
        nonlocal uptime
        uptime += 1
        # uavcan.protocol.NodeStatus: uptime, health OK and mode OPERATIONAL, vendor status
        broadcast(shell, 341, struct.pack('<IBH', uptime, 0, 0), uptime)
        return re.search(rf'^\s+{PEER} OK\s+OPERAT', px4(shell, 'uavcan status'), re.MULTILINE) is not None

    wait_until(online, 30, f'node {PEER}')

    # PX4 doesn't take the frames it sends for those of another node
    assert re.search(r'^\s+1 ', px4(shell, 'uavcan status'), re.MULTILINE) is None


def test_static_pressure(shell: Shell, dronecan):
    transfer_id = 0

    def received():
        nonlocal transfer_id
        # uavcan.equipment.air_data.StaticPressure: pressure, variance as a float16
        broadcast(shell, 1028, struct.pack('<fH', 101325.0, 0), transfer_id)
        transfer_id += 1
        return re.search(r'pressure: 101325\.0', px4(shell, 'listener sensor_baro')) is not None

    wait_until(received, 30, 'the static pressure')
