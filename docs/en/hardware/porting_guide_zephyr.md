# Zephyr Board Porting Guide

<Badge type="warning" text="Experimental" />

::: warning
The Zephyr port is experimental.
PX4 has only flown in simulation on Zephyr (see [Limitations](#limitations)).
:::

PX4 can run on the [Zephyr RTOS](https://zephyrproject.org/), using the Zephyr drivers for the board and its sensors.
A board that Zephyr supports needs no PX4 drivers: its sensors are described in the devicetree, and PX4 publishes them to uORB.

PX4 is built with Zephyr's tools: [west](https://docs.zephyrproject.org/latest/develop/west/index.html) builds the firmware and [twister](https://docs.zephyrproject.org/latest/develop/test/twister.html) runs the tests.

## Architecture

The PX4-Autopilot repository is a [Zephyr module](https://docs.zephyrproject.org/latest/develop/modules.html) ([zephyr/module.yml](https://github.com/PX4/PX4-Autopilot/blob/main/zephyr/module.yml)).
A Zephyr application that sets `CONFIG_PX4=y` gets PX4 built into it:

- The PX4 modules are selected by a [PX4 board configuration](porting_guide_config.md) (`.px4board` file), as on the other platforms.
  The configuration in `boards/zephyr/<board>/` is used if it exists for the Zephyr board, and [boards/zephyr/generic](https://github.com/PX4/PX4-Autopilot/tree/main/boards/zephyr/generic) otherwise.
- The operating system is configured with Zephyr Kconfig (`prj.conf`) and devicetree, as for any Zephyr application.
- The platform layer in [platforms/zephyr](https://github.com/PX4/PX4-Autopilot/tree/main/platforms/zephyr) runs PX4 on Zephyr's POSIX API.
  PX4 tasks and work queues are preemptible Zephyr threads, and the high-resolution timer counts the cycles of the system timer, with its callouts on the kernel tick (1 MHz by default).
  PX4 is built as on NuttX, with `-Os`, and its estimators and controllers with `-O2`.
  A board with flash to spare can build it for speed with `CONFIG_SPEED_OPTIMIZATIONS`.
- The application calls `px4_zephyr_start()`, which initialises PX4 and runs the commands of the board's `init/rc.px4` startup script, one per line.
  A board configuration with a [label](porting_guide_config.md#px4-kconfig-label-inheritance) runs `init/rc.<label>.px4` instead, if it exists.
- The `px4` command of the Zephyr shell runs PX4 commands, for example `px4 uorb status`.

The PX4 options of the Zephyr configuration are:

| Option                      | Description                                                                                         |
| --------------------------- | --------------------------------------------------------------------------------------------------- |
| `CONFIG_PX4`                | Build PX4 into the application.                                                                     |
| `CONFIG_PX4_BOARD_CONFIG`   | PX4 board configuration to build instead of the default one, for example `zephyr_generic_sih`.      |
| `CONFIG_PX4_SHELL`          | Add the `px4` shell command (enabled with the shell).                                               |
| `CONFIG_PX4_SENSORS`        | Enable the Zephyr sensor API for the sensors in the devicetree (enabled by the `px4,sensors` node). |
| `CONFIG_PX4_SERIAL`         | Open the UARTs of the devicetree as device files (enabled by the `px4,serial-ports` node).          |
| `CONFIG_PX4_PWM_OUTPUTS`    | Enable the Zephyr PWM API for the actuator outputs (enabled by the `px4,pwm-outputs` node).         |
| `CONFIG_PX4_RC_INPUT`       | Enable the input subsystem for the RC input (enabled by the `px4,rc-input` node).                   |
| `CONFIG_PX4_ADC`            | Enable the Zephyr ADC API for the board ADC (enabled by the `px4,adc` node).                        |
| `CONFIG_PX4_BUZZER`         | Enable the buzzer driver for the tone alarm (enabled by the `buzzer0` devicetree alias).            |
| `CONFIG_PX4_LEDS`           | Enable the LED drivers for the board and RGB LEDs (enabled by the `px4,leds` node).                 |
| `CONFIG_PX4_SAFETY_BUTTON`  | Enable the Zephyr GPIO API for the safety button (enabled by the `px4,safety-button` node).         |
| `CONFIG_PX4_STACK_OVERHEAD` | Bytes added to the stack of each PX4 task and work queue.                                           |
| `CONFIG_PX4_ITCM`           | Run the most used code from the ITCM instead of the flash (enabled on the MIMXRT1170-EVK).          |

## Setting Up a Workspace

PX4-Autopilot is the manifest repository of the west workspace, which fetches the version of Zephyr that PX4 is tested with:

```sh
mkdir px4-zephyr && cd px4-zephyr
git clone https://github.com/PX4/PX4-Autopilot.git
python3 -m venv .venv
source .venv/bin/activate
pip install west
west init -l --mf platforms/zephyr/west.yml PX4-Autopilot
west update --narrow
west packages pip --install
west sdk install
```

The PX4 build only fetches the Git submodules it uses.

## Building and Running in QEMU

The reference application is in [platforms/zephyr/app](https://github.com/PX4/PX4-Autopilot/tree/main/platforms/zephyr/app).
Build it and run it in QEMU, emulating an Arm Cortex-M4F board (`mps2/an386`):

```sh
west build -b mps2/an386 PX4-Autopilot/platforms/zephyr/app -t run
```

or, from the **PX4-Autopilot** directory:

```sh
make zephyr_qemu
```

PX4 starts and the Zephyr shell runs PX4 commands:

```sh
uart:~$ px4 work_queue status

Work Queue: 3  threads                          RATE        INTERVAL
|__ 1) wq:rate_ctrl
|   \__ 1) vehicle_angular_velocity          97.4 Hz        10263 us
|__ 2) wq:nav_and_controllers
|   |__ 1) sensors                           94.1 Hz        10624 us
...
```

Exit QEMU with **Ctrl+A**, **X**.

## Flying in Simulation (SIH)

PX4 flies a quadcopter in QEMU with the [SIH](../sim_sih/index.md) simulator, which runs in PX4 itself, as on a flight controller (airframe 1100):

```sh
west build -b mps2/an386 PX4-Autopilot/platforms/zephyr/app -t run -- -DFILE_SUFFIX=sih
```

or `make zephyr_sih`.

The `sih` [file suffix](https://docs.zephyrproject.org/latest/develop/application/index.html#application-file-suffixes) selects `prj_sih.conf`, which builds the `zephyr_generic_sih` PX4 board configuration ([sih.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/zephyr/generic/sih.px4board) and [rc.sih.px4](https://github.com/PX4/PX4-Autopilot/blob/main/boards/zephyr/generic/init/rc.sih.px4)), and `boards/mps2_an386_sih.conf`.
The emulated CPU runs at 250 MHz, the speed of a flight controller.

Fly from the Zephyr shell:

```sh
uart:~$ px4 commander takeoff
uart:~$ px4 listener vehicle_local_position
uart:~$ px4 commander land
```

or with _QGroundControl_ on the host, which connects automatically: MAVLink is sent over UDP to port 14550 of the host, with [QEMU user networking](https://docs.zephyrproject.org/latest/connectivity/networking/qemu_user_setup.html).

MAVLink also runs over a serial port, as with a telemetry radio.
`boards/mps2_an386_serial.overlay` makes the second UART of the QEMU board `/dev/ttyS1`, which QEMU connects to TCP port 5760 of the host:

```sh
west build -b mps2/an386 PX4-Autopilot/platforms/zephyr/app -t run -- -DFILE_SUFFIX=sih \
  -DEXTRA_CONF_FILE=boards/mps2_an386_serial.conf -DEXTRA_DTC_OVERLAY_FILE=boards/mps2_an386_serial.overlay
uart:~$ px4 mavlink start -d /dev/ttyS1 -b 115200 -x
```

Then add a TCP link to `127.0.0.1:5760` in _QGroundControl_.

The QEMU board has a RAM disk in the 16 MiB PSRAM of the emulated board (`boards/mps2_an386.overlay`), mounted at `/SD:`.
PX4 keeps its parameters there, and the SIH configuration also its missions and a log of each flight, in `/SD:/log`.
QEMU keeps the files when the board reboots (`px4 reboot`), but not when QEMU restarts.
List them with `fs ls /SD:`.

DroneCAN runs in QEMU on the Zephyr loopback CAN controller, which receives the frames sent on it (`boards/mps2_an386_can.overlay`), with the `zephyr_generic_can` PX4 board configuration.
Put the controller in loopback mode before starting DroneCAN, and send the frames of other nodes with the [CAN shell](https://docs.zephyrproject.org/latest/hardware/peripherals/can/shell.html), here the static pressure of node 10:

```sh
west build -b mps2/an386 PX4-Autopilot/platforms/zephyr/app -t run -- \
  -DEXTRA_CONF_FILE=boards/mps2_an386_can.conf -DEXTRA_DTC_OVERLAY_FILE=boards/mps2_an386_can.overlay
uart:~$ can mode can_loopback0 loopback
uart:~$ px4 uavcan start
uart:~$ can send can_loopback0 -e 1004040a 80 e6 c5 47 00 00 c0
uart:~$ px4 listener sensor_baro
```

Cyphal runs the same way with the `zephyr_generic_cyphal` board configuration (`-DEXTRA_CONF_FILE=boards/mps2_an386_cyphal.conf`), started with `px4 cyphal start`.

## Building for Other Boards

To build the application for another Zephyr board:

```sh
make zephyr ZEPHYR_BOARD=vmu_rt1170/mimxrt1176/cm7
```

## Running on the NXP MIMXRT1170-EVK

The [MIMXRT1170-EVK](https://www.nxp.com/design/design-center/development-boards-and-designs/MIMXRT1170-EVKB) evaluation kit has the i.MX RT1176 of the NXP VMU RT1170, without flight sensors.
PX4 flies the SIH quadcopter on it, with the [mimxrt1170_evk](https://github.com/PX4/PX4-Autopilot/tree/main/boards/zephyr/mimxrt1170_evk) PX4 board configuration.

Set SW1 to boot from the flash (switch 3 on, the others off), connect the debug USB port (J86 on the EVKB, J11 on the EVK), and build and flash with the LinkServer runner (or `-r jlink` or `-r pyocd`):

```sh
west build -b mimxrt1170_evk/mimxrt1176/cm7 PX4-Autopilot/platforms/zephyr/app -- -DFILE_SUFFIX=sih
west flash
```

`mimxrt1170_evk/mimxrt1176/cm7` is the EVKB, and `mimxrt1170_evk@A/mimxrt1176/cm7` the EVK.

The Zephyr shell is on the serial port of the debug USB port, at 115200 baud, and flies the quadcopter as in QEMU:

```sh
uart:~$ px4 commander takeoff
uart:~$ px4 listener vehicle_local_position
uart:~$ px4 commander land
```

The USB OTG1 port is `/dev/ttyACM0`, a USB serial port that runs MAVLink while a ground station has it open.
_QGroundControl_ doesn't recognise its USB IDs: add a serial link to its port (`/dev/cu.usbmodem*` on macOS) in **Application Settings > Comm Links**, and fly missions from there.

- The RAM is the on-chip OCRAM from 0x20200000 (1 MiB: the OCRAM of the M4, OCRAM1 and half of OCRAM2), instead of the SDRAM that the board chooses, with which the flight stack takes about 1.5 points more of the CPU.
- The parameters and the logs are kept on the microSD card (FAT32), and the missions in RAM, as an EVK may have no card.
  The card runs at 50 MHz, without 1.8 V signalling, with which it often failed to initialise after a warm reset, such as the one of `west flash`.
- DroneCAN runs on the CAN3 connector, CAN1 of PX4: start it with `px4 uavcan start`.
- The rate controller and the simulator run at 400 Hz instead of 800 Hz.

An ExpressLRS or TBS Crossfire receiver connects to the UART of the Arduino header (LPUART2), which the Zephyr CRSF driver (`tbs,crsf`) reads for `zephyr_rc_input`:

| Receiver | MIMXRT1170-EVK |
| -------- | -------------- |
| TX       | D0 (J9)        |
| RX       | D1 (J9)        |
| 5V       | 5V (J25)       |
| GND      | GND (J25)      |

The M.2 slot must be empty, as its Bluetooth module shares the UART.
The SIH configuration disables the sticks: set [COM_RC_IN_MODE](../advanced_config/parameter_reference.md#COM_RC_IN_MODE) to 0 to fly with the transmitter, after the radio calibration of _QGroundControl_.
The driver receives with DMA, and sends the telemetry of `crsf_rc`.

The EVK runs the firmware from its quad SPI flash, which stalls the CPU for hundreds of cycles when an instruction isn't in the cache.
With the 32 KiB instruction cache of the Cortex-M7, the flight stack spends most of its time waiting for the flash.
`CONFIG_PX4_ITCM` runs the most used code from the ITCM, the instruction memory of the Cortex-M7, instead: the kernel, uORB, the work queues, the sensors, the rate and attitude controllers, the control allocation, the simulator and the trigonometric functions of the C library, and the functions of [itcm_functions.ld](https://github.com/PX4/PX4-Autopilot/blob/main/platforms/zephyr/module/itcm_functions.ld), which a profile of SIH on the EVK found running most from the flash (250 KiB in all).
The flight stack then uses about 55% of the CPU at 400 Hz, instead of 94% with only the code that runs at the rate of the IMU.

[Tools/itcm_profile.py](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/itcm_profile.py) makes the list again, for example after changes to the flight stack, from the program counter of a board running SIH, sampled with pyOCD, within the free space of the ITCM:

```sh
PX4-Autopilot/Tools/itcm_profile.py --target mimxrt1170_cm7 --max-size 94000 build/zephyr/zephyr.elf build/zephyr/zephyr.map --output PX4-Autopilot/platforms/zephyr/module/itcm_functions.ld
```

## Running the Tests

The tests run in QEMU with twister:

```sh
west twister -T PX4-Autopilot/platforms/zephyr -p mps2/an386
```

or `make zephyr_tests`, which also builds the application for the boards that are only build-tested.

- [tests/platform](https://github.com/PX4/PX4-Autopilot/tree/main/platforms/zephyr/tests/platform) tests the platform layer: time, callouts from interrupts, tasks, work queues, uORB, parameters, and the CPU load that `top` and `load_mon` show.
- [tests/sensors](https://github.com/PX4/PX4-Autopilot/tree/main/platforms/zephyr/tests/sensors) tests the sensor driver with Zephyr's sensor emulators, on emulated SPI and I2C buses, with a sensor that doesn't respond, a power monitor and an emulated SBS fuel gauge.
  The FIFO of an ICM-42688-P is streamed at 8 kHz, on a 250 MHz emulated CPU, from an emulator of the test, as the Zephyr emulator has no FIFO, and a second ICM-42688-P is polled, as its interrupt doesn't fire.
- [tests/serial](https://github.com/PX4/PX4-Autopilot/tree/main/platforms/zephyr/tests/serial) tests the serial ports, termios and `device::Serial` on an emulated UART.
- [tests/storage](https://github.com/PX4/PX4-Autopilot/tree/main/platforms/zephyr/tests/storage) tests the parameter file and the file functions of PX4 on a RAM disk.
- [tests/can](https://github.com/PX4/PX4-Autopilot/tree/main/platforms/zephyr/tests/can) tests the CAN buses on the Zephyr loopback CAN controller.
- [tests/pwm](https://github.com/PX4/PX4-Autopilot/tree/main/platforms/zephyr/tests/pwm) tests the PWM outputs on the Zephyr fake PWM driver: the disarmed, armed, failsafe and ESC calibration values, a pulse longer than the period, the outputs left without pulses when their functions are removed and disarmed when the driver stops, and the update rate of outputs with different periods.
- [tests/rc_input](https://github.com/PX4/PX4-Autopilot/tree/main/platforms/zephyr/tests/rc_input) tests the RC input with the Zephyr SBUS driver, which receives SBUS frames on an emulated UART: the channel values, keys, the failsafe, the loss of RC, and the events of another input device.
- [tests/adc](https://github.com/PX4/PX4-Autopilot/tree/main/platforms/zephyr/tests/adc) tests the board ADC on the Zephyr ADC emulator: the channels of the ADC report, the 5 V and 3.3 V rails of the system power, a channel that can't be read, and two analog batteries with their voltage dividers, a disconnected battery and `BAT1_V_CHANNEL`.
- [tests/buzzer](https://github.com/PX4/PX4-Autopilot/tree/main/platforms/zephyr/tests/buzzer) tests the tone alarm on a PWM buzzer of the Zephyr fake PWM driver: the frequency and duration of the notes of a tone and of tunes, a tone that overrides a tune, a stop in the middle of a note, and `CBRK_BUZZER`.
- [tests/leds](https://github.com/PX4/PX4-Autopilot/tree/main/platforms/zephyr/tests/leds) tests the board LEDs on GPIO LEDs of an emulated GPIO controller, through `/dev/led0` and as Commander drives them, and the RGB LED driver on a fake LED, a group of GPIO LEDs and an emulated NCP5623C: the colors, blinks and breathing of `led_control`, `SYS_RGB_MAXBRT`, and an NCP5623C that doesn't respond.
- [tests/safety_button](https://github.com/PX4/PX4-Autopilot/tree/main/platforms/zephyr/tests/safety_button) tests the safety button and its LED on an emulated GPIO controller, with the button also read by the `gpio-keys` input driver: the safety button event after the button has been held for 1 second, and none while it is held or for presses interrupted by releases, the pairing request of 3 presses within 2 seconds, and the LED pattern of each safety and armed state.
  A second test has no safety LED nor `gpio-keys` input driver.
- [tests/gps](https://github.com/PX4/PX4-Autopilot/tree/main/platforms/zephyr/tests/gps) tests the GPS driver with an NMEA receiver on an emulated UART: the position, accuracy, dilutions of precision and velocity of the fix, and the baud rate.
- [tests/identity](https://github.com/PX4/PX4-Autopilot/tree/main/platforms/zephyr/tests/identity) tests the board identity, from the device ID of a hwinfo driver and from `BOARD_OVERRIDE_UUID`.
- The application tests check that PX4 starts, fly the SIH quadcopter (take off, hover and land), check the flight log, `top` and the CPU load, and that a parameter is kept across a reboot, connect to PX4 with MAVLink over a serial port and download the component metadata with MAVLink FTP, and run DroneCAN and Cyphal with another node on a loopback CAN bus.

The tests also run on the MIMXRT1170-EVK, which twister flashes and reads on the serial port of its debug USB port:

```sh
west twister -T PX4-Autopilot/platforms/zephyr/tests -p mimxrt1170_evk@B/mimxrt1176/cm7 --device-testing --device-serial /dev/ttyACM0
```

## CPU Load

`top` shows the CPU load, stack usage, priority and state of the threads, from the [runtime statistics](https://docs.zephyrproject.org/latest/kernel/services/threads/index.html#runtime-statistics) of the Zephyr threads, here in the SIH configuration:

```sh
uart:~$ px4 top once
COMMAND                   CPU(ms) CPU(%)  USED/STACK PRIO STATE
idle                        14619 40.654    72/  320  100 READY
sysworkq                        0  0.000   272/ 1024   -1 WAIT
...
wq:rate_ctrl                 2407  7.876  2432/ 4176    0 WAIT
sih                          6009 20.726  1384/ 2584    0 WAIT
...
logger                        493  1.951  3112/ 4728   25 READY
log_writer_file                49  0.212   576/ 2200   25 WAIT
top                             0  0.003  2428/ 5120   18 RUN

Threads: 23 total, 4 running, 19 sleeping
CPU usage: 58.99% tasks, 40.65% idle
Uptime: 32.817s total, 14.620s idle
```

The oldest threads come first.
`PRIO` is the Zephyr priority, which runs first when it is lower: the PX4 priorities 0 to 99 are the Zephyr priorities 99 to 0, and the cooperative threads of Zephyr have negative priorities.
`STATE` is `RUN`, `READY`, `WAIT` (for a kernel object), `SLEEP` or `SUSP` (suspended).
Unlike NuttX, Zephyr threads have no process ID nor file descriptors of their own, `top` has no line for the interrupt stack, and the CPU usage has no `sched` share.
The time of an interrupt counts for the thread that it interrupted, and at most 64 threads are listed, as on NuttX.
As PX4 commands don't get the input of the Zephyr shell, `top` stops after its first print, of the load over 0.2 s, while `top once` prints the load over 1.2 s.

The [load_mon](../modules/modules_system.md#load-mon) module publishes the CPU load and the RAM usage to [Cpuload](../msg_docs/Cpuload.md).
The RAM usage is the share of the heaps that is allocated: the C library heap of the PX4 objects and the kernel heap of the thread stacks.
As on NuttX, `load_mon` checks the free stack of a thread every 0.5 s if [SYS_STCK_EN](../advanced_config/parameter_reference.md#SYS_STCK_EN) is set, publishes it to [TaskStackInfo](../msg_docs/TaskStackInfo.md), and warns when less than 300 bytes are left.
The logger writes the output of `top` to the log when it starts and stops logging.
The generic and NXP VMU RT1170 board configurations start `load_mon`.

PX4 enables the Zephyr options that they need: the runtime statistics of the threads (`CONFIG_THREAD_RUNTIME_STATS`), which the POSIX API enables too, the pattern written to the stack of a thread when it is created, which shows how much of the stack is used (`CONFIG_INIT_STACKS`), and the statistics of the heaps (`CONFIG_SYS_HEAP_RUNTIME_STATS`).
With `top` and `load_mon`, the CPU load adds 4 KiB of flash and almost no static RAM to the SIH configuration of the NXP VMU RT1170.

## Adding a Board

A board supported by Zephyr runs PX4 without any PX4 board files.
Porting it means describing its sensors in a devicetree overlay, for example in `platforms/zephyr/app/boards/<board>.overlay`.
See the overlay for the [NXP VMU RT1170](https://github.com/PX4/PX4-Autopilot/blob/main/platforms/zephyr/app/boards/vmu_rt1170_mimxrt1176_cm7.overlay).

A board that needs its own PX4 modules or startup commands gets a PX4 board configuration in `boards/zephyr/<board>/`, named after the Zephyr board without its qualifiers (for example `vmu_rt1170`).
Copy [boards/zephyr/generic](https://github.com/PX4/PX4-Autopilot/tree/main/boards/zephyr/generic) as a starting point.

### Board Identity

The unique ID of the board, which PX4 sends in MAVLink, DroneCAN, Cyphal and Remote ID messages and writes in the flight logs, is the device ID of the Zephyr [hwinfo](https://docs.zephyrproject.org/latest/hardware/peripherals/hwinfo.html) driver of the SoC.
On the i.MX RT, this is the unique ID in the fuses of the chip.
It can differ from the one NuttX reports on the same board, as Zephyr may read other fuses or order them differently.
The PX4 GUID starts with the `PX4_SOC_ARCH_ID` of the board, which `src/board_config.h` of its PX4 board configuration defines.

A board without a hwinfo driver, such as `mps2/an386` in QEMU, has the fixed `BOARD_OVERRIDE_UUID` of the generic board instead, as SITL does.

### Sensors

The `zephyr_sensors` driver publishes the sensors listed in the `px4,sensors` devicetree node to uORB:

```dts
#include <dt-bindings/px4/rotation.h>

/ {
	px4-sensors {
		compatible = "px4,sensors";

		imu0 {
			sensor = <&icm42688>;
			type = "imu";
			rotation = <PX4_ROTATION_ROLL_180>;
		};

		mag0 {
			sensor = <&ist8310>;
			type = "mag";
			external;
		};

		baro0 {
			sensor = <&bmp388>;
			type = "baro";
		};
	};
};
```

| Property   | Description                                                                                                                                               |
| ---------- | --------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `sensor`   | The Zephyr sensor device.                                                                                                                                 |
| `type`     | `imu` (publishes [SensorAccel] and [SensorGyro]), `accel`, `gyro`, `mag` ([SensorMag]), `baro` ([SensorBaro]), `power` or `fuel-gauge` ([BatteryStatus]). |
| `rotation` | Rotation from the sensor frame to the board frame, one of the `PX4_ROTATION_*` values. Default: none.                                                     |
| `external` | The sensor is not on the flight controller board.                                                                                                         |
| `battery`  | Battery measured by a power monitor, which selects its `BATn_*` parameters, or a fuel gauge. Default: 1.                                                  |

[SensorAccel]: ../msg_docs/SensorAccel.md
[SensorGyro]: ../msg_docs/SensorGyro.md
[SensorMag]: ../msg_docs/SensorMag.md
[SensorBaro]: ../msg_docs/SensorBaro.md
[BatteryStatus]: ../msg_docs/BatteryStatus.md

The driver reads the sensors with the Zephyr sensor API, which works with any Zephyr sensor driver: the IMUs at the rate of the rate controller, [IMU_GYRO_RATEMAX](../advanced_config/parameter_reference.md#IMU_GYRO_RATEMAX), unless their FIFO is streamed (below), the magnetometers at 100 Hz, the barometers at 50 Hz and the power monitors at 10 Hz.
The sensors of each bus are read in turn on the work queue of the bus, as with the PX4 drivers, so that a slow sensor, such as a barometer that converts, doesn't delay the IMUs of the other buses.
Most Zephyr sensor drivers read on the RTIO work queue, which has 8 threads with PX4 so that 8 buses can be read at the same time: a board with more sensor buses raises `CONFIG_RTIO_WORKQ_THREADS_POOL` and `CONFIG_RTIO_WORKQ_POOL_ITEMS`.
The device IDs are made from the bus, the address and the sensor model, as for the PX4 drivers.

An IMU whose interrupt pin is in the devicetree, as `int-gpios`, `irq-gpios`, `int1-gpios` or `int2-gpios` of its device, has its FIFO streamed with the Zephyr [sensor streaming API](https://docs.zephyrproject.org/latest/hardware/peripherals/sensor/read_and_decode.html#streaming) if its Zephyr driver supports it.
The ICM-4268x driver supports it when `CONFIG_SPI_RTIO` is enabled, and other drivers, such as the BMI08x, BMI270 and ICM-45686 drivers, with Kconfig options, such as `CONFIG_BMI08X_ACCEL_STREAM`.
The FIFO is read each time it reaches its watermark, and all its samples are published to [SensorAccelFifo] and [SensorGyroFifo], as the PX4 IMU drivers do with NuttX, so that the rate controller filters the IMU at its output data rate.
The decoder of the Zephyr driver gives the time of the samples, from the time of the interrupt, and their temperature, if the FIFO has it.

- Drivers that take a batch duration (`SENSOR_ATTR_BATCH_DURATION`), such as the ICM-4268x driver, get a FIFO watermark of at most 1 / IMU_GYRO_RATEMAX and 32 samples, the size of the FIFO messages, for example 20 samples at 8 kHz for 400 Hz.
- The FIFO watermark of the other drivers is a property of their device, such as the `fifo-watermark` of the BMI08x, which must read the FIFO at least at IMU_GYRO_RATEMAX, for example 5 samples at 2000 Hz for 400 Hz.
- An IMU is polled instead if its driver can't stream, or if its FIFO isn't read for 100 ms, for example if its interrupt pin isn't connected.

Each streamed IMU buffers its FIFO reads in `CONFIG_PX4_SENSORS_FIFO_BUFFER_SIZE` bytes, 2 KiB by default, which must hold at least two of them.
The NXP VMU RT1170 streams the FIFOs of its IMUs, at the output data rates and full scales of the NuttX drivers of FMU-v6XRT: 8 kHz for the ICM-42686-P and ICM-42688-P, 1600 Hz for the accelerometer of the BMI088 and 2000 Hz for its gyroscope, with its 532 Hz filter.
The anti-aliasing and UI filters of the ICM-4268x keep their reset values, as the Zephyr driver doesn't configure them.

[SensorAccelFifo]: ../msg_docs/SensorAccelFifo.md
[SensorGyroFifo]: ../msg_docs/SensorGyroFifo.md

A sensor that may not be connected, such as the compass of a GPS module, has the `zephyr,deferred-init` property on its device, so that Zephyr doesn't initialize it at boot.
`zephyr_sensors` initializes it when it starts, and leaves it out if it doesn't respond, as PX4 probes the external buses with NuttX.
For example, the overlay of the NXP VMU RT1170 lists the IST8310, RM3100 and LIS3MDL compasses on the GPS1 bus:

```dts
&lpi2c1 {
	rm3100: rm3100@20 {
		compatible = "pni,rm3100";
		reg = <0x20>;
		zephyr,deferred-init;
	};
};
```

These sensors need different addresses on a bus, and Zephyr drivers that check the identity of the chip before they configure it.

A power monitor, such as the TI INA226 or INA228 of a digital power module, publishes the voltage and current of a battery, which is disconnected when the power monitor can't be read for 2 seconds.
Its shunt resistor and current resolution are properties of its Zephyr device, instead of the `INA226_*` and `INA228_*` parameters, and battery 2 is published if [BAT2_SOURCE](../advanced_config/parameter_reference.md#BAT2_SOURCE) selects a power module, as with NuttX.
The NXP VMU RT1170 probes INA226 and INA228 power modules on POWER1 and POWER2.

A smart battery has a `fuel-gauge` entry, whose device implements the Zephyr fuel gauge API instead of the sensor API, such as the generic SBS fuel gauge driver, and needs `CONFIG_FUEL_GAUGE`.
It publishes the voltage, current, state of charge, capacity, cycle count and remaining time measured by its fuel gauge, as the `batt_smbus` driver, with the warnings of [BAT_LOW_THR](../advanced_config/parameter_reference.md#BAT_LOW_THR), [BAT_CRIT_THR](../advanced_config/parameter_reference.md#BAT_CRIT_THR) and [BAT_EMERGEN_THR](../advanced_config/parameter_reference.md#BAT_EMERGEN_THR).
An optional fuel gauge is used if it can be read when `zephyr_sensors` starts, as fuel gauge drivers don't check the gauge when they are initialized.
The NXP VMU RT1170 probes SBS smart batteries on POWER1 and POWER2.

The generic board configuration starts `zephyr_sensors` and the [sensors](../modules/modules_system.md#sensors) module.

### Board ADC

The [board_adc](../modules/modules_driver_adc.md#adc) driver samples the ADC channels listed in the `px4,adc` devicetree node with the Zephyr [ADC API](https://docs.zephyrproject.org/latest/hardware/peripherals/adc.html), at 100 Hz, and publishes them to [AdcReport], as on NuttX.
PX4 numbers the channels from 0, in the order of `io-channels`, and the name of a channel gives the macro that `board_config.h` defines for the channel on NuttX, for example `ADC_SCALED_V5_CHANNEL` for `scaled-v5`:

```dts
/ {
	px4-adc {
		compatible = "px4,adc";
		io-channels = <&adc1 0>, <&adc1 1>, <&adc1 2>, <&adc1 3>;
		io-channel-names = "scaled-v5", "battery-voltage", "battery-current", "adc-3v3";
	};
};
```

| Channel name                                                                        | Use                                                                                                                                                    |
| ----------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `scaled-v5`                                                                         | 5 V rail, through a voltage divider of 2, published to [SystemPower].                                                                                  |
| `scaled-vdd-3v3-sensors`, or `scaled-vdd-3v3-sensors1` to `scaled-vdd-3v3-sensors4` | 3.3 V rails of the sensors, through voltage dividers of 2, published to [SystemPower].                                                                 |
| `battery-voltage`, `battery-current`                                                | Voltage and current of an analog battery, which the [battery_status](../modules/modules_system.md#battery-status) module publishes to [BatteryStatus]. |
| `battery1-voltage`, `battery1-current`, `battery2-voltage`, `battery2-current`      | Voltages and currents of two analog batteries.                                                                                                         |

[AdcReport]: ../msg_docs/AdcReport.md
[SystemPower]: ../msg_docs/SystemPower.md

The other channels, such as the inputs of an ADC connector, are only published to [AdcReport].
Each channel is configured by its channel node on its ADC, with the `zephyr,gain`, `zephyr,reference`, `zephyr,vref-mv`, `zephyr,acquisition-time` and `zephyr,resolution` properties of the Zephyr ADC API, and all the channels need the same resolution and full scale, as [AdcReport] has one reference voltage.
A board whose voltage dividers differ from those of the Pixhawk boards defines `ADC_V5_V_FULL_SCALE` (the 5 V rail at the full scale of 3.3 V) or `ADC_3V3_SCALE` in the `src/board_config.h` of its PX4 board configuration, as on NuttX.
USB isn't detected: the `usb_connected` field of [SystemPower] is 0, so the commander checks the 5 V rail also when the board is powered over USB, and the power modules are always valid.

`battery_status` scales the voltage and current of an analog battery with [BAT1_V_DIV](../advanced_config/parameter_reference.md#BAT1_V_DIV) and [BAT1_A_PER_V](../advanced_config/parameter_reference.md#BAT1_A_PER_V), and those of a second one with `BAT2_V_DIV` and `BAT2_A_PER_V`, as calibrated in [Battery Estimation Tuning](../config/battery.md#voltage-divider).
A battery without a current channel has no current, and [BAT1_V_CHANNEL](../advanced_config/parameter_reference.md#BAT1_V_CHANNEL) selects another channel, by its number, for the voltage of battery 1.

Add `CONFIG_DRIVERS_ADC_BOARD_ADC` to the PX4 board configuration, and `CONFIG_MODULES_BATTERY_STATUS` for analog batteries, and start them in the startup script with `board_adc start` and `battery_status start`.
`CONFIG_PX4_ADC` enables the ADC API when the devicetree has the node, and the build fails without it if the PX4 board configuration has `board_adc`.
A PX4 board configuration without `board_adc`, such as the SIH configuration, sets `CONFIG_PX4_ADC=n` in the Zephyr configuration.

The NXP VMU RT1170 has the ADC channels of FMU-v6XRT on LPADC1, and its board configuration starts `board_adc`: the 3.3 V rails of the sensors, the 5 V rail and the 3.3 V and 6.6 V inputs of the ADC connector, numbered 0 to 6 instead of 4 to 11 with NuttX.
Its batteries are measured by digital power modules.
The LPADC scales its inputs by 30/64, for a full scale of 3.89 V with its 1.825 V reference, which the Zephyr ADC API takes as a gain of 1/2: the overlay gives the reference as 1947 mV, so that the voltages are those of NuttX.
This adds 6.2 KiB of flash and 0.4 KiB of static RAM.

### Serial Ports

PX4 opens the UARTs listed in the `px4,serial-ports` devicetree node as device files, for MAVLink, GPS receivers and the other drivers of serial devices:

```dts
/ {
	px4-serial-ports {
		compatible = "px4,serial-ports";

		telem1 {
			uart = <&lpuart4>;
			path = "/dev/ttyS2";
		};

		usb {
			uart = <&cdc_acm_uart0>;
			path = "/dev/ttyACM0";
		};
	};
};
```

| Property         | Description                                                 |
| ---------------- | ----------------------------------------------------------- |
| `uart`           | The UART, which must support the interrupt-driven UART API. |
| `path`           | Path of the device file.                                    |
| `rx-buffer-size` | Size of the receive buffer, in bytes. Default: 512.         |
| `tx-buffer-size` | Size of the transmit buffer, in bytes. Default: 1024.       |

Use the paths of the NuttX build for the same hardware, if there is one, so that the startup commands and the serial port parameters stay the same.
The Zephyr console UART can't be a PX4 serial port.

The termios functions set the speed, data bits, parity and stop bits of the UART with the Zephyr UART configuration API, and RTS/CTS flow control if the UART has `hw-flow-control` in the devicetree.
A UART that can't be configured at runtime keeps its devicetree settings.
The `px4_tty` shell command prints the receive statistics of the serial ports: the bytes received, the bytes dropped while the receive buffer was full, and the overrun, framing, noise and parity errors of the UARTs.

A USB CDC ACM UART (`zephyr,cdc-acm-uart`) is a USB serial port.
Data written to it is dropped while no host has opened it, and MAVLink detects a USB port from its `/dev/ttyACM` path.
`cdcacm_autostart` takes a host that opens a USB port for VBUS, which Zephyr has no generic way to detect: it starts MAVLink when the host opens the port, as set by `SYS_USB_AUTO` and `USB_MAV_MODE`, and stops it when the host closes it.
MAVLink would otherwise build its streams for nobody, which takes 23% of the CPU of the MIMXRT1170-EVK at 400 Hz.

The NXP VMU RT1170 has the serial ports of FMU-v6XRT and a USB port.
Its PX4 board configuration ([boards/zephyr/vmu_rt1170](https://github.com/PX4/PX4-Autopilot/tree/main/boards/zephyr/vmu_rt1170)) starts the GPS driver on `/dev/ttyS1` (GPS1), and MAVLink on `/dev/ttyS2` (TELEM1) and USB.
MAVLink only runs over UDP if the Zephyr configuration has the network stack (`CONFIG_NET_SOCKETS`).

### Storage

PX4 keeps its parameters, missions and logs on the FAT file system mounted at the root path of the PX4 board configuration (`CONFIG_BOARD_ROOT_PATH`), which is `/SD:` for the Zephyr boards.
Mount it with a [devicetree fstab](https://docs.zephyrproject.org/latest/services/storage/file_system/index.html) entry:

```dts
/ {
	fstab {
		compatible = "zephyr,fstab";

		sd {
			compatible = "zephyr,fstab,fatfs";
			mount-point = "/SD:";
			automount;
			disk-access;
		};
	};
};
```

The FAT mount point is the name of the Zephyr disk: here the `SD` disk of an SD card (`zephyr,sdmmc-disk`) or of a RAM disk (`zephyr,ram-disk`).
The `PX4_STORAGE` Kconfig option then enables the FAT file system with long file names, and enough file descriptors for PX4.
A RAM disk is formatted the first time it is mounted, but an SD card never is: format it with a FAT file system on a computer.

At startup, PX4 loads the parameters from `/SD:/parameters.bson`, and saves them there when they change.
Without storage, the parameters are kept in RAM only.

The NXP VMU RT1170 mounts its microSD card.

### Component Metadata

As on NuttX, the firmware has the [metadata](../advanced/px4_metadata.md) of its parameters, events and actuators in `/etc/extras`, from where _QGroundControl_ downloads it with MAVLink FTP, for example for its [Actuators](../config/actuators.md) setup, and the logger writes the events metadata to the logs.
`/etc` is a read-only file system of the files built into the firmware, so the metadata needs no SD card.
MAVLink FTP, which runs on USB ports and on the links started with `-x`, serves the root of the Zephyr file systems, as it serves the root of NuttX (`CONFIG_BOARD_FS_ROOT_PATH` is empty): the storage is in `/SD:`, next to `/etc`.

The metadata is built into the firmware when the PX4 board configuration has MAVLink, unless the `PX4_COMPONENT_METADATA` Kconfig option is disabled.
It takes 38 KiB of flash on the NXP VMU RT1170, and 74 KiB with the modules of the SIH configuration, mostly for the parameter metadata.

### CAN Buses

PX4 numbers the CAN buses of the `px4,can-buses` devicetree node CAN1, CAN2 and so on, in the order of its child nodes:

```dts
/ {
	px4-can-buses {
		compatible = "px4,can-buses";

		can1 {
			can = <&flexcan1>;
		};

		can2 {
			can = <&flexcan2>;
		};
	};
};
```

The [DroneCAN](../dronecan/index.md) driver runs on them with the Zephyr CAN API: add `CONFIG_DRIVERS_UAVCAN` and `CONFIG_BOARD_UAVCAN_INTERFACES` to the PX4 board configuration.
Frames are sent in order of priority, and received with the time of their reception.

[Cyphal](../can/index.md) runs on CAN1, with classic CAN frames at `CYPHAL_BAUD`: add `CONFIG_DRIVERS_CYPHAL` to the PX4 board configuration.
It is supported as a flight controller (`CONFIG_CYPHAL_FMU`), not as a CAN node, and doesn't share a CAN bus with DroneCAN.

The NXP VMU RT1170 has DroneCAN on its three CAN buses.
Its startup script doesn't start it, as Zephyr startup scripts can't check `UAVCAN_ENABLE`: start it with `px4 uavcan start`.

### PWM Outputs

The `zephyr_pwm_out` driver drives the PWM outputs listed in the `px4,pwm-outputs` devicetree node as the main outputs, up to 8.
The first child node is the first output, configured with [PWM_MAIN_FUNC1](../advanced_config/parameter_reference.md#PWM_MAIN_FUNC1), [PWM_MAIN_DIS1](../advanced_config/parameter_reference.md#PWM_MAIN_DIS1), `PWM_MAIN_MIN1`, `PWM_MAIN_MAX1` and `PWM_MAIN_FAIL1`, the second one with `PWM_MAIN_FUNC2` and so on:

```dts
#include <zephyr/dt-bindings/pwm/pwm.h>

/ {
	px4-pwm-outputs {
		compatible = "px4,pwm-outputs";

		main1 {
			pwms = <&flexpwm1_pwm0 0 PWM_HZ(400) PWM_POLARITY_NORMAL>;
		};

		main2 {
			pwms = <&flexpwm1_pwm1 0 PWM_HZ(50) PWM_POLARITY_NORMAL>;
		};
	};
};
```

The outputs have the same parameters as with the `pwm_out` driver of NuttX on a board without an I/O processor, and are configured in the [Actuators](../config/actuators.md) setup of _QGroundControl_ (see [Component Metadata](#component-metadata)).
The driver supports ESC calibration, and the generic board configuration starts it.

Each output has the period of its `pwms` property, for example 400 Hz for ESCs or 50 Hz for analog servos, instead of the timer parameters of NuttX, such as [PWM_MAIN_TIM0](../advanced_config/parameter_reference.md#PWM_MAIN_TIM0).
The build checks that the period is at least 2200 us, the longest pulse of the parameters.
Many PWM controllers have one period for all their channels, such as an STM32 timer or the channels A and B of an NXP FlexPWM submodule: outputs on the same PWM device then need the same period, and the driver warns when they differ.

The driver updates the outputs at the rate of the fastest one at most, and sets the slower outputs at most once per period, as some Zephyr PWM drivers wait for the end of the period to take a new pulse: the NXP FlexPWM driver busy-waits until the previous pulse is loaded.
The fastest outputs follow the mixer with a margin for its jitter, which limits this wait to half their period.
The outputs without a function have no pulses, and the driver leaves the outputs at their disarmed value when it stops.
It is tested on the Zephyr fake PWM driver, in QEMU and on the MIMXRT1170-EVK.

The NXP VMU RT1170 has the outputs `FMU_CH1` to `FMU_CH8` of FMU-v6XRT, at 400 Hz, and its board configuration starts `zephyr_pwm_out`.
The prescaler of its PWM limits the period to 17 ms: a 50 Hz output needs a larger `nxp,prescaler` on its FlexPWM submodule.
With NuttX, FMU-v6XRT drives these outputs as its AUX outputs, `PWM_AUX_FUNC1` to `PWM_AUX_FUNC8`, as its firmware also supports an I/O processor for the main outputs: the `PWM_AUX_*` parameters of its parameter files are the `PWM_MAIN_*` parameters on Zephyr.

### RC Input

The `zephyr_rc_input` driver publishes the RC input of a Zephyr [input device](https://docs.zephyrproject.org/latest/services/input/index.html), such as an SBUS receiver read by the Zephyr SBUS driver (`futaba,sbus`) or a CRSF receiver read by the Zephyr CRSF driver (`tbs,crsf`), to [InputRc](../msg_docs/InputRc.md).
The `px4,rc-input` devicetree node selects the input device, and lists the input event of each RC channel, as its type and code, in the order of the RC channels:

```dts
#include <zephyr/dt-bindings/input/input-event-codes.h>

/ {
	px4-rc-input {
		compatible = "px4,rc-input";
		input = <&sbus>;
		channels = <INPUT_EV_ABS INPUT_ABS_RX>,
			   <INPUT_EV_ABS INPUT_ABS_RY>,
			   <INPUT_EV_ABS INPUT_ABS_Y>,
			   <INPUT_EV_ABS INPUT_ABS_X>,
			   <INPUT_EV_MSC 5>;
	};
};
```

| Property    | Description                                                                             |
| ----------- | --------------------------------------------------------------------------------------- |
| `input`     | The input device, which sends a sync event after each frame.                            |
| `channels`  | Type and code of the input event of each RC channel, up to 18 channels.                 |
| `input-min` | Raw value of a channel at 1000 us. Default: 200, the SBUS value at -100%.               |
| `input-max` | Raw value of a channel at 2000 us. Default: 1800, the SBUS value at +100%.              |
| `failsafe`  | Type and code of an input event whose value is not 0 while the receiver is in failsafe. |

The value of an `INPUT_EV_ABS` or `INPUT_EV_MSC` event is scaled from `input-min`..`input-max` to 1000..2000 us, as PX4 scales the SBUS channels with NuttX, so that an SBUS receiver gives the same channel values with both.
A key (`INPUT_EV_KEY`) gives 2000 us while it is pressed, and 1000 us while it is released, which it is until its first event.
The Zephyr SBUS driver only reports a key when its channel goes above `CONFIG_INPUT_SBUS_CHANNEL_VALUE_ONE` (1800) or below `CONFIG_INPUT_SBUS_CHANNEL_VALUE_ZERO` (1200): map the switches of an SBUS receiver as `INPUT_EV_MSC` events instead, with the SBUS channel as code for example, to keep the middle position of 3-position switches and the channel values of NuttX.
The channels are published after each frame of the input device, once each of them has been reported, and RC is lost when no frame is received for 200 ms, as with PX4IO.
`CONFIG_PX4_RC_INPUT` has the input events handled in the thread of the input driver (`CONFIG_INPUT_MODE_SYNCHRONOUS`), instead of the input thread of Zephyr, which has the lowest priority, and enables the interrupt-driven UART API that the SBUS driver needs, or the asynchronous one of the CRSF driver.
The thread of the CRSF driver runs at the priority of the serial RC drivers of PX4 (the `wq:ttyS1` work queue), instead of the top priority.

A CRSF receiver also gives its link statistics, the RSSI, link quality, SNR and RF mode of `crsf_rc`, and is sent its telemetry: the battery, the GNSS receiver, the attitude, the flight mode and the altitude, one frame every 100 ms.
The Zephyr CRSF driver gives the channels in microseconds: set `input-min` to 1000 and `input-max` to 2000.

::: warning
The Zephyr SBUS driver doesn't report the failsafe flag of the receiver, so PX4 doesn't detect the failsafe of a receiver that keeps sending frames, as in its hold or custom failsafe modes.
Before flying with an SBUS receiver, as on the NXP VMU RT1170, set it to stop sending frames in failsafe (no pulses), so that RC is lost, or set [RC_FAILS_THR](../advanced_config/parameter_reference.md#RC_FAILS_THR), as described in [RC Loss Detection](../config/radio.md#rc-loss-detection).
:::

The generic board configuration starts `zephyr_rc_input`.
The NXP VMU RT1170 has the SBUS receiver of its Zephyr board on its RC input, a single-wire inverted UART.
Its overlay maps the 16 proportional SBUS channels, with the values of NuttX: the sticks of the board devicetree on channels 1 to 4, and channels 5 to 16 as `INPUT_EV_MSC` events, instead of the kill switch key of the board on channel 5.
The MIMXRT1170-EVK maps the 16 channels of its CRSF receiver as `INPUT_EV_MSC` events, with the CRSF channel as code.

### Buzzer

The [tone_alarm](../modules/modules_driver.md#tone-alarm) driver plays the tunes of PX4, such as the startup tune and the arming warning, on the buzzer of the `buzzer0` devicetree alias, with the Zephyr [buzzer API](https://docs.zephyrproject.org/latest/hardware/peripherals/buzzer.html).
The Zephyr boards with a buzzer have this alias, here for a passive buzzer on a PWM output:

```dts
#include <zephyr/dt-bindings/pwm/pwm.h>

/ {
	aliases {
		buzzer0 = &buzzer;
	};

	buzzer: buzzer {
		compatible = "pwm-buzzer";
		pwms = <&qtmr1 0 PWM_HZ(2700) PWM_POLARITY_NORMAL>;
	};
};
```

`CONFIG_PX4_BUZZER` enables the buzzer driver when the devicetree has the alias.
Add `CONFIG_DRIVERS_TONE_ALARM` to the PX4 board configuration, and start the driver in the startup script with `tone_alarm start`: the build fails if the devicetree has no buzzer.
`tune_control play -t 1` plays the startup tune, with `CONFIG_SYSTEMCMDS_TUNE_CONTROL`.
A PX4 board configuration without `tone_alarm`, such as the SIH configuration, sets `CONFIG_PX4_BUZZER=n` in the Zephyr configuration, so that the buzzer driver isn't built.

The tone alarm times the notes with the PX4 high-resolution timer, as with NuttX, and plays them at the loudest volume of the buzzer, a square wave on a PWM buzzer.
The timer ends the notes in an interrupt, and the buzzer is then silenced from the Zephyr system work queue, as the buzzer API isn't available in interrupts.
An active buzzer (`gpio-buzzer`) plays its own tone whatever the note.
Set [CBRK_BUZZER](../advanced_config/parameter_reference.md#CBRK_BUZZER) to 782097 to disable the tone alarm, as on NuttX.
782090, which only skips the startup tune on NuttX, has no effect, as Zephyr startup scripts can't check it: remove `tune_control play -t 1` from the startup script instead.

The NXP VMU RT1170 has the buzzer of FMU-v6XRT in its Zephyr board devicetree, on a channel of QTMR1.
Its board configuration starts `tone_alarm`, and plays the startup tune at the end of its startup script.

### LEDs

As on NuttX, PX4 turns its board LEDs on and off through `/dev/led0`, for example to show the arming state and the CPU overload with [Commander](../modules/modules_system.md#commander), and shows the system status on its RGB LEDs, the [UI LED](../getting_started/led_meanings.md#ui_led), such as the LED of a GPS module.
The `px4,leds` devicetree node selects their Zephyr LEDs:

```dts
#include <zephyr/dt-bindings/led/led.h>

/ {
	px4-leds {
		compatible = "px4,leds";
		blue = <&blue_led>;
		red = <&red_led>;
		green = <&green_led>;
		rgb = <&gps_led>;
	};
};

&lpi2c1 {
	ncp5623c@39 {
		compatible = "onnn,ncp5623";
		reg = <0x39>;
		zephyr,deferred-init;

		gps_led: led_0 {
			label = "GPS LED";
			index = <0>;
			color-mapping = <LED_COLOR_ID_RED>, <LED_COLOR_ID_GREEN>, <LED_COLOR_ID_BLUE>;
		};
	};
};
```

| Property               | Description                                                              |
| ---------------------- | ------------------------------------------------------------------------ |
| `blue`, `red`, `green` | The board LEDs `LED_BLUE`, `LED_RED` (also `LED_AMBER`) and `LED_GREEN`. |
| `rgb`                  | RGB LEDs, which all show the system status.                              |

Each LED is an LED of an LED controller, such as `gpio-leds` or `pwm-leds`, or a `leds-group-multicolor`, which PX4 drives with the Zephyr [LED API](https://docs.zephyrproject.org/latest/hardware/peripherals/led.html).
The node names the board LEDs after their color, which the `led0`, `led1`... aliases of the Zephyr boards don't give.
`CONFIG_PX4_LEDS` enables the LED drivers when the devicetree has the node.

The board LEDs are off when PX4 starts.
`BOARD_ARMED_STATE_LED` and `BOARD_OVERLOAD_LED` in `src/board_config.h` of the PX4 board configuration select the LEDs on which Commander blinks the arming state and the overload, as on NuttX: the blue and the red LED in the generic board configuration, as on most flight controllers.

The `zephyr_rgbled` driver shows the system status of the `led_control` topic on the RGB LEDs, as the RGB LED drivers of NuttX, such as `rgbled_ncp5623c`: add `CONFIG_DRIVERS_LIGHTS_ZEPHYR_RGBLED` to the PX4 board configuration, and start it in the startup script with `zephyr_rgbled start`.
An RGB LED is a multi-color LED with red, green or blue channels in its `color-mapping`, such as the LED of an NCP5623C, LP5562 or IS31FL319x, or single-color LEDs grouped by a `leds-group-multicolor`.
The driver sets their color every 100 ms, or every 25 ms while they breathe, and [SYS_RGB_MAXBRT](../advanced_config/parameter_reference.md#SYS_RGB_MAXBRT) limits their brightness.
An LED whose device has the `zephyr,deferred-init` property, such as the LED of a GPS module that may not be connected, is initialized when the driver starts, and left out if it doesn't respond.
The [led_control](../modules/modules_command.md#led-control) command (`CONFIG_SYSTEMCMDS_LED_CONTROL`) sets their color, for example `px4 led_control blink -c green`.

The NXP VMU RT1170 has the board LEDs of FMU-v6XRT, with their roles, and shows the system status on the LED of a GPS module on GPS1, an NCP5623C.
Its overlay makes the green LED active low, and gives the NCP5623C the compatible of the Zephyr NCP5623 driver, as its Zephyr board has them wrong.
Its board configuration has no Commander: the board LEDs show the arming state in the SIH configuration.
The MIMXRT1170-EVK blinks the arming state on its green user LED, and the overload on its red one.

### Safety Button

The [safety_button](../modules/modules_driver.md#safety-button) driver reads the [safety switch](../getting_started/px4_basic_concepts.md#safety-switch), such as the one of a GPS module, as on NuttX.
Once the button has been held for 1 second, it publishes a safety button event, with which [Commander](../modules/modules_system.md#commander) turns safety off, depending on [COM_SAFETY_MODE](../advanced_config/parameter_reference.md#COM_SAFETY_MODE), and three presses within 2 seconds request a pairing with the ground station.
It blinks the safety and armed states on the safety LED.
The `px4,safety-button` devicetree node selects the button, and the `safety` property of the [px4,leds](#leds) node the LED:

```dts
#include <zephyr/dt-bindings/gpio/gpio.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>

/ {
	gpio-keys {
		compatible = "gpio-keys";

		safety_switch: safety-switch {
			gpios = <&gpio1 24 (GPIO_PULL_DOWN | GPIO_ACTIVE_HIGH)>;
			zephyr,code = <INPUT_KEY_0>;
		};
	};

	gpio-leds {
		compatible = "gpio-leds";

		safety_led: safety-led {
			gpios = <&gpio1 3 GPIO_ACTIVE_LOW>;
		};
	};

	px4-safety-button {
		compatible = "px4,safety-button";
		button = <&safety_switch>;
	};

	px4-leds {
		compatible = "px4,leds";
		safety = <&safety_led>;
	};
};
```

The button is a node with a `gpios` property, pressed while its GPIO is active, such as a button of `gpio-keys`, with which the Zephyr boards describe their buttons.
The driver reads the GPIO every 33 ms and debounces it, as on NuttX, rather than the input events of the `gpio-keys` input driver, which come after a debounce interval of their own.
PX4 configures the GPIO when it starts, unless the `gpio-keys` input driver has configured it, which then still reports the button.
The safety LED is `LED_SAFETY` of the board LEDs, which PX4 drives with the Zephyr LED API, with the polarity of the devicetree, and through `/dev/led0`.
The driver reads the button with `BOARD_SAFETY_BUTTON()` and sets the LED with `BOARD_SAFETY_LED()`, which the Zephyr platform defines from the devicetree, instead of the `GPIO_BTN_SAFETY` and `GPIO_LED_SAFETY` GPIOs of the NuttX boards.

The LED blinks a pattern of 1.5 seconds, as the safety LED of PX4IO, once the armed state is published, by Commander, and stays off before:

| Pattern      | State                 |
| ------------ | --------------------- |
| Slow blink   | Safety on, disarmed.  |
| Double blink | Safety off, disarmed. |
| Quad blink   | Safety on, armed.     |
| Solid        | Safety off, armed.    |

A board without a safety LED has no `BOARD_SAFETY_LED()`, and the driver blinks no LED, as on NuttX.

Add `CONFIG_DRIVERS_SAFETY_BUTTON` to the PX4 board configuration, and start the driver in the startup script with `safety_button start`.
`CONFIG_PX4_SAFETY_BUTTON` enables the Zephyr GPIO API when the devicetree has the node, and the build fails without it if the PX4 board configuration has `safety_button`.
A PX4 board configuration without `safety_button`, such as the SIH configuration, sets `CONFIG_PX4_SAFETY_BUTTON=n` in the Zephyr configuration.

The NXP VMU RT1170 has the safety switch of FMU-v6XRT, on GPS1: the arming button of its Zephyr board, on GPIO1_IO24, and the LED on GPIO1_IO03, which its overlay adds.
The Zephyr board has a pull-down on the button, with which a disconnected switch is released, where NuttX enables the pull-up of the pin.
Its board configuration starts `safety_button`, but has no Commander, so that the button doesn't turn safety off, and the LED stays off.
This adds 1.3 KiB of flash and 20 bytes of static RAM.

## Footprint

[Tools/footprint.py](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/footprint.py) breaks down the flash and static RAM of PX4 firmware by component, for NuttX and Zephyr builds alike.
Each section of the ELF file counts as flash, RAM or both, as in the PX4 flash analysis, and the linker map file gives the library or object file that each byte comes from.

For example, to compare the two operating systems on the NXP i.MX RT1176, with the NuttX build of FMU-v6XRT and the SIH configuration on the VMU RT1170, as is and matched to the NuttX build ([footprint.conf](https://github.com/PX4/PX4-Autopilot/blob/main/platforms/zephyr/app/footprint.conf)):

```sh
make -C PX4-Autopilot px4_fmu-v6xrt_default
west build -b vmu_rt1170/mimxrt1176/cm7 -d build-sih PX4-Autopilot/platforms/zephyr/app -- -DFILE_SUFFIX=sih -DCONFIG_SIZE_OPTIMIZATIONS=y
west build -b vmu_rt1170/mimxrt1176/cm7 -d build-matched PX4-Autopilot/platforms/zephyr/app -- -DFILE_SUFFIX=sih -DEXTRA_CONF_FILE=footprint.conf -DCONFIG_SIZE_OPTIMIZATIONS=y
PX4-Autopilot/Tools/footprint.py PX4-Autopilot/build/px4_fmu-v6xrt_default/px4_fmu-v6xrt_default.elf build-sih/zephyr/zephyr.elf build-matched/zephyr/zephyr.elf
```

At the time of writing, with a commit of Zephyr main after 4.5.0-rc2, this gives (in KiB):

| Component                      | NuttX flash | NuttX RAM | Zephyr flash | Zephyr RAM | Matched flash | Matched RAM |
| ------------------------------ | ----------: | --------: | -----------: | ---------: | ------------: | ----------: |
| Kernel and POSIX API           |          35 |        59 |           42 |         29 |            41 |          29 |
| Drivers and HAL                |          67 |        60 |          133 |         57 |           114 |          57 |
| Shell and logging              |          48 |         1 |           19 |          8 |            20 |          15 |
| File systems                   |          35 |         1 |           16 |          2 |            14 |           2 |
| Networking                     |          34 |         9 |           29 |         17 |            95 |          33 |
| C library and compiler runtime |          51 |         6 |           43 |          0 |            42 |           0 |
| Operating system               |         271 |       136 |          282 |        113 |           326 |         135 |
| PX4 platform layer             |          33 |        17 |           13 |         13 |            13 |          13 |
| PX4 startup scripts (ROMFS)    |         177 |         0 |            0 |          0 |             0 |           0 |
| PX4 modules and libraries      |        1924 |       224 |          918 |         16 |           918 |          16 |
| Other and alignment            |           8 |         1 |            8 |          9 |             8 |           9 |
| Total                          |        2414 |       378 |         1220 |        151 |          1264 |         173 |

Only the operating system rows compare the two systems, as the builds differ in other ways:

- The NuttX build has all the modules and drivers of FMU-v6XRT, and the Zephyr builds the multicopter flight stack, with SIH, MAVLink and the logger.
  The sensor drivers are PX4 modules with NuttX, and Zephyr drivers with Zephyr, where they take 28 KiB of flash, or 21 KiB without their log messages.
- NuttX is configured with the FAT, compressed ROM and proc file systems, the SD card, USB, CAN sockets, Ethernet with TCP, UDP, DHCP and DNS, and its shell, nsh, also over telnet.
  The Zephyr build has the FAT file system, the SD card, USB, the three CAN buses, Ethernet with UDP, and the Zephyr shell and logging.
  The matched build adds TCP, DHCP, DNS, IGMP, the shell over telnet and the network commands of the shell, which count as networking (29 KiB of flash).
  Like NuttX, whose subsystems have no debug output, it also leaves out the log messages of the Zephyr subsystems, which take 27 KiB of flash.
- The other NuttX features aren't matched: the startup scripts are built into the firmware instead of a compressed ROM file system, PX4 uses the Zephyr CAN API instead of CAN sockets, and there is no proc file system.
- The Zephyr builds were measured without `top` and `load_mon`, which add 4 KiB of flash.
- The Zephyr builds were measured without the [component metadata](#component-metadata), which adds 74 KiB of flash, mostly in the ROMFS row, where NuttX has it.
- The buffers of the 7 serial ports of the Zephyr builds (10.5 KiB) are in the PX4 platform layer.
- NuttX copies the most used PX4 code (226 KiB) to the ITCM, the fast instruction memory of the Cortex-M7, so that this code also counts as RAM.
- The RAM is the static RAM.
  NuttX uses the rest of the memory as its heap.
  Zephyr reserves a kernel heap for the thread stacks at build time (256 KiB, which is not in the table), and the C library heap takes the rest.
- NuttX is built with GCC 13.2, and Zephyr with GCC 14.3 from the Zephyr SDK.
  The Zephyr builds are built with `-Os` here, as NuttX builds itself, instead of for speed as the board configuration of the VMU RT1170 does.

## Limitations

- PX4 has only flown in simulation: the PWM outputs, the RC input, the GPS driver, the buzzer and the safety button are only tested in QEMU.
- PWM outputs have the period of the devicetree, without OneShot or DShot.
- RC input is read from Zephyr input devices, such as SBUS and CRSF receivers, and from CRSF receivers with `crsf_rc` on a serial port: the other RC protocols that PX4 decodes on a serial port with NuttX, such as DSM and GHST, aren't supported.
  The Zephyr CRSF driver doesn't bind the receiver, as `crsf_rc` does on a command of the ground station.
  The Zephyr SBUS driver doesn't report the failsafe flag of the receiver, nor the digital channels 17 and 18.
- The LEDs are only tested in QEMU, and an RGB LED needs a Zephyr LED driver: the NCP5623C, LP5562 and IS31FL319x have one, but not the TCA62724, IS31FL3195 and AW2023 of the NuttX RGB LED drivers.
- Storage is on FAT file systems only.
- The board ADC is only tested in QEMU, doesn't detect USB, and doesn't read the valid signals of the power modules nor the hardware version.
- Serial ports use the interrupt-driven UART API, without DMA, and have no single-wire, inverted or swapped modes.
- DroneCAN isn't started at boot, and a frame that a CAN controller has started to send is sent even once its deadline has passed. Cyphal uses classic CAN frames only.
- Dynamic modules (`DYNAMIC` in `px4_add_module()`) are not supported.
- `native_sim` is not supported, as its POSIX API uses the host C library.
- The tests run in QEMU and on the MIMXRT1170-EVK. The NXP VMU RT1170 is only built.

## Roadmap

- RC input over a serial port, with the other protocols of the PX4 RC drivers (DSM, GHST).
- Device files and termios for UARTs in Zephyr itself, instead of the PX4 platform layer.
- Parameters in flash, with ZMS, on boards without an SD card.
- Dynamic modules as Zephyr loadable extensions (below).

### Loadable Modules

On Linux, PX4 builds `DYNAMIC` modules as shared libraries, which the `dyn` command loads.
On Zephyr, the plan is to build them as [LLEXT](https://docs.zephyrproject.org/latest/services/llext/index.html) extensions:

- `px4_add_module(... DYNAMIC)` creates the extension with `add_llext_target()`.
- `dyn` loads the extension from a file system with `llext_load()`, runs its constructors with `llext_bringup()`, finds its `px4_module_main` entry point with `llext_find_sym()`, and runs it as a PX4 task.
- The firmware exports the PX4 API that modules use (uORB, parameters, work queues, performance counters and logging), from a list generated at build time.
- The extensions can be tested in QEMU, as Zephyr supports LLEXT on Arm Cortex-M.
