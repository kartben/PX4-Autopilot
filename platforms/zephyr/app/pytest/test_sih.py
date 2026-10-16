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
Fly the SIH quadcopter from the Zephyr shell: take off, hover and land. Then
check the flight log, the CPU load, and that a parameter is kept across a
reboot.
"""

import re
import time

from twister_harness import DeviceAdapter, Shell


PARAM = 'MPC_Z_VEL_MAX_UP'


def run(shell: Shell, command: str) -> list[str]:
    try:
        return shell.exec_command(command)
    except AssertionError:
        # a PX4 message was printed over the echo of the command
        return []


def px4(shell: Shell, command: str) -> list[str]:
    return run(shell, f'px4 {command}')


def list_dir(shell: Shell, path: str) -> dict[str, int]:
    entries = {}

    for line in run(shell, f'fs ls {path}'):
        match = re.match(r'\s*(\d+) (\S+)$', line)

        if match:
            entries[match.group(2)] = int(match.group(1))

    return entries


def listener_value(shell: Shell, topic: str, field: str) -> float | None:
    for line in px4(shell, f'listener {topic}'):
        match = re.match(rf'\s+{field}: (\S+)', line)

        if match:
            return float(match.group(1))

    return None


def wait_until(condition, timeout: float, what: str):
    end = time.time() + timeout

    while time.time() < end:
        if condition():
            return

        time.sleep(1)

    raise AssertionError(f'timeout waiting for {what}')


def commander_output(shell: Shell, command: str) -> str:
    return '\n'.join(px4(shell, f'commander {command}'))


def param_value(shell: Shell, name: str) -> float | None:
    for line in px4(shell, f'param show {name}'):
        match = re.search(rf'{name} \[\d+,\d+\] : (\S+)', line)

        if match:
            return float(match.group(1))

    return None


def test_takeoff_and_land(shell: Shell):
    wait_until(lambda: 'Preflight check: OK' in commander_output(shell, 'check'), 300, 'the preflight checks')

    commander_output(shell, 'takeoff')

    def climbed():
        z = listener_value(shell, 'vehicle_local_position', 'z')
        return z is not None and z < -2.0

    wait_until(climbed, 120, 'the takeoff')

    commander_output(shell, 'land')
    wait_until(lambda: 'INFO  [commander] Disarmed' in commander_output(shell, 'status'), 120, 'the landing')


def test_log(shell: Shell):
    def logs() -> dict[str, int]:
        dirs = [name for name in list_dir(shell, '/SD:/log') if name.endswith('/')]

        if not dirs:
            return {}

        return {name: size for name, size in list_dir(shell, f'/SD:/log/{dirs[-1]}').items()
                if name.endswith('.ulg')}

    wait_until(lambda: any(size > 0 for size in logs().values()), 30, 'the flight log')


def top(shell: Shell, command: str) -> list[str]:
    lines = []

    def printed() -> bool:
        nonlocal lines
        lines = [re.sub(r'\x1b\[[0-9;]*[A-Za-z]', '', line) for line in px4(shell, command)]
        return any(re.match(r'CPU usage: [\d.]+% tasks, [\d.]+% idle', line) for line in lines)

    wait_until(printed, 30, f'the output of {command}')
    return lines


def test_cpu_load(shell: Shell):
    # top stops after one print, as it can't read a key from the Zephyr shell
    top(shell, 'top')

    stack_left = {}

    for line in top(shell, 'top once'):
        match = re.match(r'(\S+) +\d+ +[\d.]+ +(\d+)/ *(\d+) +-?\d+ +[A-Z]+', line)

        if match:
            stack_left[match.group(1)] = int(match.group(3)) - int(match.group(2))

    for thread in ('idle', 'wq:rate_ctrl', 'sih', 'logger'):
        assert thread in stack_left, f'{thread} not in top'

    # load_mon warns below 300 bytes; the logger printed the load on its stack when the flight started and ended
    for thread, left in stack_left.items():
        assert thread == 'idle' or left >= 300, f'{thread} has {left} bytes of stack left'

    load = listener_value(shell, 'cpuload', 'load')
    assert load is not None and 0 < load <= 1, f'CPU load {load}'


def test_parameter_kept_after_reboot(dut: DeviceAdapter, shell: Shell):
    px4(shell, f'param set {PARAM} 2.5')

    # the parameters are saved in the background
    time.sleep(5)

    dut.write(b'px4 reboot\n')
    dut.readlines_until(regex='Booting Zephyr', timeout=60)
    wait_until(lambda: param_value(shell, PARAM) == 2.5, 120, f'{PARAM} after the reboot')
