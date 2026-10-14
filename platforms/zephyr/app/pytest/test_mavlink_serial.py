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
port of the host. Download the component metadata with MAVLink FTP, as
QGroundControl does, and check that the logger writes the events metadata to
the log.
"""

import json
import lzma
import os
import struct
import time
import zlib

import pytest
from pymavlink import mavftp, mavutil
from twister_harness import Shell

PARAM = 'MPC_Z_VEL_MAX_UP'

# not in the common dialect of pymavlink
MAVLINK_MSG_ID_COMPONENT_METADATA = 397
COMP_METADATA_TYPE_PARAMETER = 1
COMP_METADATA_TYPE_EVENTS = 4
COMP_METADATA_TYPE_ACTUATORS = 5

METADATA_FILES = {'component_general.json.xz', 'parameters.json.xz', 'all_events.json.xz', 'actuators.json.xz'}


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

    px4(shell, 'mavlink start -d /dev/ttyS1 -b 115200 -x')

    # MAVLink 2, as QGroundControl, for the messages with an ID above 255
    os.environ['MAVLINK20'] = '1'
    connection = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255, dialect='common')
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


@pytest.fixture(scope='session')
def ftp(mav):
    return mavftp.MAVFTP(mav, mav.target_system, mav.target_component)


def mavlink_crc32(data: bytes) -> int:
    # the CRC-32 of MAVLink has no final XOR
    return zlib.crc32(data, 0xFFFFFFFF) ^ 0xFFFFFFFF


def ftp_path(uri: str) -> str:
    # QGroundControl requests mftp://etc/extras/file as /etc/extras/file
    assert uri.startswith('mftp://'), uri
    return uri[len('mftp:/'):]


def download(ftp: mavftp.MAVFTP, path: str) -> bytes:
    data = []

    def received(file):
        # None when the download fails
        if file is not None:
            data.append(file.read())

    ftp.cmd_get([path], callback=received)
    result = ftp.process_ftp_reply('OpenFileRO', timeout=120)

    assert result.error_code == mavftp.FtpError.Success, f'{path}: {result.error_code.name}'
    assert len(data) == 1, f'{path}: no data'
    return data[0]


def list_dir(ftp: mavftp.MAVFTP, path: str) -> dict[str, mavftp.DirectoryEntry]:
    result = ftp.cmd_list([path])

    assert result.error_code == mavftp.FtpError.Success, f'{path}: {result.error_code.name}'
    return {entry.name: entry for entry in result.directory_listing}


def component_metadata(mav) -> tuple[int, str]:
    mav.mav.command_long_send(mav.target_system, mav.target_component, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                              MAVLINK_MSG_ID_COMPONENT_METADATA, 0, 0, 0, 0, 0, 0)

    message = mav.recv_match(type=f'UNKNOWN_{MAVLINK_MSG_ID_COMPONENT_METADATA}', blocking=True, timeout=30)
    assert message is not None, 'no COMPONENT_METADATA'

    # the payload follows the 10 bytes of the MAVLink 2 header, without its trailing zeros
    payload = bytes(message.data[10:10 + message.data[1]]).ljust(108, b'\0')
    _, file_crc, uri = struct.unpack('<II100s', payload)
    return file_crc, uri.rstrip(b'\0').decode()


def test_component_metadata(mav, ftp):
    general_crc, general_uri = component_metadata(mav)
    assert general_uri == 'mftp://etc/extras/component_general.json.xz'

    general = lzma.decompress(download(ftp, ftp_path(general_uri)))
    assert mavlink_crc32(general) == general_crc

    types = {item['type']: item for item in json.loads(general)['metadataTypes']}
    assert set(types) == {COMP_METADATA_TYPE_PARAMETER, COMP_METADATA_TYPE_EVENTS, COMP_METADATA_TYPE_ACTUATORS}

    metadata = {}

    for metadata_type, item in types.items():
        assert item['uri'].startswith('mftp://etc/extras/'), item['uri']

        data = download(ftp, ftp_path(item['uri']))
        assert mavlink_crc32(data) == item['fileCrc'], item['uri']
        metadata[metadata_type] = json.loads(lzma.decompress(data))

    parameters = {parameter['name'] for parameter in metadata[COMP_METADATA_TYPE_PARAMETER]['parameters']}
    assert PARAM in parameters
    assert 'components' in metadata[COMP_METADATA_TYPE_EVENTS]

    # the Actuators setup of QGroundControl: the geometries of the control allocator, and the outputs of
    # zephyr_pwm_out with their parameters
    actuators = metadata[COMP_METADATA_TYPE_ACTUATORS]
    assert any(config['option'] == 'CA_AIRFRAME==0' for config in actuators['mixer_v1']['config'])

    outputs = {output['label']: output for output in actuators['outputs_v1']}
    assert 'PWM MAIN' in outputs

    for subgroup in outputs['PWM MAIN']['subgroups']:
        assert len(subgroup['channels']) == 8

        for parameter in subgroup['per-channel-parameters']:
            assert parameter['name'].replace('${i}', '1') in parameters, parameter['name']


def test_ftp_list(ftp):
    # MAVLink FTP serves the root of the file systems, as on NuttX
    root = list_dir(ftp, '/')
    assert root['SD:'].is_dir
    assert root['etc'].is_dir

    extras = list_dir(ftp, '/etc/extras')
    assert set(extras) == METADATA_FILES

    # a path relative to the root, as MAVSDK requests mftp://etc/extras/file
    name = 'component_general.json.xz'
    assert len(download(ftp, f'etc/extras/{name}')) == extras[name].size_b


def test_log_events(shell: Shell, ftp):
    px4(shell, 'logger on')
    time.sleep(2)
    px4(shell, 'logger off')

    session = sorted(name for name, entry in list_dir(ftp, '/SD:/log').items() if entry.is_dir)[-1]
    log = sorted(name for name in list_dir(ftp, f'/SD:/log/{session}') if name.endswith('.ulg'))[-1]

    # the logger writes the events metadata of /etc/extras after the parameters
    data = b''

    while b'metadata_events_sha256' not in data:
        chunk = ftp.read(f'/SD:/log/{session}/{log}', 16384, len(data))
        assert chunk, 'no events metadata in the log'
        data += chunk


class MavlinkShell:
    """The MAVLink shell, as the MAVLink console of QGroundControl."""

    def __init__(self, mav):
        self.mav = mav
        self.sentinel = 0

    def write(self, text: str):
        data = text.encode()

        for offset in range(0, len(data), 70):
            chunk = list(data[offset:offset + 70])
            self.mav.mav.serial_control_send(
                mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL,
                mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE | mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                0, 0, len(chunk), chunk + [0] * (70 - len(chunk)))

    def run(self, command: str, timeout: float = 30) -> list[str]:
        # the output ends with the output of an echo
        self.sentinel += 1
        sentinel = f'DONE{self.sentinel}'
        self.write(f'{command}\necho {sentinel}\n')

        output = ''
        end = time.time() + timeout

        while time.time() < end:
            message = self.mav.recv_match(type='SERIAL_CONTROL', blocking=True, timeout=1)

            if message is not None and message.count > 0:
                output += bytes(message.data[:message.count]).decode(errors='replace')
                lines = [line.strip() for line in output.splitlines()]

                if sentinel in lines:
                    # without the prompts and the echo of the commands
                    return [line.removeprefix('px4>').strip() for line in lines[:lines.index(sentinel)]
                            if line.removeprefix('px4>').strip() not in ('', command, f'echo {sentinel}')]

        raise AssertionError(f'no output of {command}: {output!r}')

    def close(self):
        self.mav.mav.serial_control_send(mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL, 0, 0, 0, 0, [0] * 70)


@pytest.fixture
def mavlink_shell(mav):
    mavlink_shell = MavlinkShell(mav)
    yield mavlink_shell
    mavlink_shell.close()
    time.sleep(1)


def test_mavlink_shell_px4_command(mavlink_shell):
    # the standard output of the PX4 command, which runs in a task of its own
    output = mavlink_shell.run('ver all')

    assert 'HW arch: ZEPHYR_GENERIC' in output
    assert 'OS: Zephyr' in output


def test_mavlink_shell_zephyr_command(mavlink_shell):
    assert any(line.startswith('Uptime:') for line in mavlink_shell.run('kernel uptime'))


def test_mavlink_shell_nsh_commands(mavlink_shell):
    assert mavlink_shell.run('echo a  b') == ['a b']

    header, umem = mavlink_shell.run('free')[:2]
    assert header.split() == ['total', 'used', 'free', 'maxused']
    assert umem.startswith('Umem:')


def test_mavlink_shell_reopen(shell: Shell, mav):
    # the shell of a ground station that closed it and opens it again
    for _ in range(2):
        mavlink_shell = MavlinkShell(mav)
        assert 'OS: Zephyr' in mavlink_shell.run('ver all')
        mavlink_shell.close()
        time.sleep(1)

    # the PX4 commands of the console still print to the console
    assert any('OS: Zephyr' in line for line in px4(shell, 'ver all'))
