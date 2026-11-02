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
Cyphal on the loopback CAN controller of QEMU, which receives the frames that
PX4 sends. The CAN shell sends the frames of another node, and prints those
of PX4.
"""

import re

import pytest
from twister_harness import DeviceAdapter, Shell

HEARTBEAT = 0x107D5501  # uavcan.node.Heartbeat from node 1
GETINFO_REQUEST = 0x136B808A  # uavcan.node.GetInfo request from node 10 to node 1
GETINFO_RESPONSE = 0x126B8501  # its response from node 1 to node 10

FRAME = re.compile(r'can_loopback0\s+([0-9a-f]+)\s+\[\s*(\d+)\]\s+((?:[0-9a-f]{2}\s*)+)$')


def run(shell: Shell, command: str) -> list[str]:
    try:
        return shell.exec_command(command)
    except AssertionError:
        # a PX4 message was printed over the echo of the command
        return []


def add_filter(shell: Shell, can_id: int) -> int:
    for line in run(shell, f'can filter add can_loopback0 -e {can_id:x}'):
        match = re.search(r'filter ID: (\d+)', line)

        if match:
            return int(match.group(1))

    raise AssertionError(f'no filter for {can_id:x}')


def frames(lines: list[str], can_id: int) -> list[bytes]:
    found = []

    for line in lines:
        match = FRAME.search(line)

        if match and int(match.group(1), 16) == can_id:
            found.append(bytes.fromhex(match.group(3)))

    return found


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE of the Cyphal multi-frame transfers."""
    crc = 0xFFFF

    for byte in data:
        crc ^= byte << 8

        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF

    return crc


@pytest.fixture(scope='session')
def cyphal(shell: Shell):
    run(shell, 'can mode can_loopback0 loopback')
    run(shell, 'px4 cyphal start')


def test_heartbeat(dut: DeviceAdapter, shell: Shell, cyphal):
    filter_id = add_filter(shell, HEARTBEAT)

    try:
        lines = dut.readlines_until(regex=f'{HEARTBEAT:x}', timeout=30)
        lines += dut.readlines_until(regex=f'{HEARTBEAT:x}', timeout=30)

    finally:
        run(shell, f'can filter remove can_loopback0 {filter_id}')

    heartbeats = frames(lines, HEARTBEAT)
    assert len(heartbeats) >= 2

    # single-frame transfers, with the uptime in seconds first
    uptimes = [int.from_bytes(frame[:4], 'little') for frame in heartbeats]
    assert all((frame[-1] & 0xE0) == 0xE0 for frame in heartbeats)
    assert uptimes == sorted(uptimes) and uptimes[-1] > uptimes[0]


def test_get_info(dut: DeviceAdapter, shell: Shell, cyphal):
    filter_id = add_filter(shell, GETINFO_RESPONSE)

    try:
        dut.clear_buffer()
        # an empty request, in a single frame with transfer ID 0
        dut.write(f'can send can_loopback0 -e {GETINFO_REQUEST:x} e0\n'.encode())
        # until the frame with the end of the transfer
        lines = dut.readlines_until(regex=rf'{GETINFO_RESPONSE:x}\s+\[\s*\d+\]\s+([0-9a-f]{{2}}\s+)*[4-7][0-9a-f]\s*$',
                                    timeout=30)

    finally:
        run(shell, f'can filter remove can_loopback0 {filter_id}')

    assert any('GetInfo request' in line for line in lines)

    response = frames(lines, GETINFO_RESPONSE)
    tails = [frame[-1] for frame in response]
    assert len(response) > 1
    assert tails[0] & 0x80 and tails[-1] & 0x40
    # the toggle bit alternates, starting at 1
    assert [(tail >> 5) & 1 for tail in tails] == [(i + 1) % 2 for i in range(len(tails))]

    payload = b''.join(frame[:-1] for frame in response)
    assert crc16(payload) == 0, 'transfer CRC'
    assert b'ZEPHYR_GENERIC' in payload
