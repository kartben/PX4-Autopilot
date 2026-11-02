# Zephyr Board Porting Guide

<Badge type="warning" text="Experimental" />

::: warning
The Zephyr port is experimental.
PX4 only flies in simulation on Zephyr, as there are no actuator outputs or RC input yet (see [Limitations](#limitations)).
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
  PX4 tasks and work queues are preemptible Zephyr threads, and the high-resolution timer counts the cycles of the system timer, with its callouts on the kernel tick (100 kHz by default).
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
| `CONFIG_PX4_STACK_OVERHEAD` | Bytes added to the stack of each PX4 task and work queue.                                           |

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
uart:~$ px4 mavlink start -d /dev/ttyS1 -b 115200
```

Then add a TCP link to `127.0.0.1:5760` in _QGroundControl_.

## Building for Other Boards

To build the application for another Zephyr board:

```sh
make zephyr ZEPHYR_BOARD=vmu_rt1170/mimxrt1176/cm7
```

## Running the Tests

The tests run in QEMU with twister:

```sh
west twister -T PX4-Autopilot/platforms/zephyr -p mps2/an386
```

or `make zephyr_tests`, which also builds the application for the boards that are only build-tested.

- [tests/platform](https://github.com/PX4/PX4-Autopilot/tree/main/platforms/zephyr/tests/platform) tests the platform layer: time, callouts from interrupts, tasks, work queues, uORB and parameters.
- [tests/sensors](https://github.com/PX4/PX4-Autopilot/tree/main/platforms/zephyr/tests/sensors) tests the sensor driver with Zephyr's sensor emulators, on emulated SPI and I2C buses.
- [tests/serial](https://github.com/PX4/PX4-Autopilot/tree/main/platforms/zephyr/tests/serial) tests the serial ports, termios and `device::Serial` on an emulated UART.
- The application tests check that PX4 starts, fly the SIH quadcopter (take off, hover and land), and connect to PX4 with MAVLink over a serial port.

## Adding a Board

A board supported by Zephyr runs PX4 without any PX4 board files.
Porting it means describing its sensors in a devicetree overlay, for example in `platforms/zephyr/app/boards/<board>.overlay`.
See the overlay for the [NXP VMU RT1170](https://github.com/PX4/PX4-Autopilot/blob/main/platforms/zephyr/app/boards/vmu_rt1170_mimxrt1176_cm7.overlay).

A board that needs its own PX4 modules or startup commands gets a PX4 board configuration in `boards/zephyr/<board>/`, named after the Zephyr board without its qualifiers (for example `vmu_rt1170`).
Copy [boards/zephyr/generic](https://github.com/PX4/PX4-Autopilot/tree/main/boards/zephyr/generic) as a starting point.

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

| Property   | Description                                                                                                      |
| ---------- | ---------------------------------------------------------------------------------------------------------------- |
| `sensor`   | The Zephyr sensor device.                                                                                        |
| `type`     | `imu` (publishes [SensorAccel] and [SensorGyro]), `accel`, `gyro`, `mag` ([SensorMag]) or `baro` ([SensorBaro]). |
| `rotation` | Rotation from the sensor frame to the board frame, one of the `PX4_ROTATION_*` values. Default: none.            |
| `external` | The sensor is not on the flight controller board.                                                                |

[SensorAccel]: ../msg_docs/SensorAccel.md
[SensorGyro]: ../msg_docs/SensorGyro.md
[SensorMag]: ../msg_docs/SensorMag.md
[SensorBaro]: ../msg_docs/SensorBaro.md

The driver reads the sensors with the Zephyr sensor API, which works with any Zephyr sensor driver, at 1 kHz for IMUs, 100 Hz for magnetometers and 50 Hz for barometers.
The device IDs are made from the bus, the address and the sensor model, as for the PX4 drivers.

The generic board configuration starts `zephyr_sensors` and the [sensors](../modules/modules_system.md#sensors) module.

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

The NXP VMU RT1170 has the serial ports of FMU-v6XRT and a USB port.
Its PX4 board configuration ([boards/zephyr/vmu_rt1170](https://github.com/PX4/PX4-Autopilot/tree/main/boards/zephyr/vmu_rt1170)) starts MAVLink on `/dev/ttyS2` (TELEM1) and USB.
MAVLink only runs over UDP if the Zephyr configuration has the network stack (`CONFIG_NET_SOCKETS`).

## Footprint

[Tools/footprint.py](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/footprint.py) breaks down the flash and static RAM of PX4 firmware by component, for NuttX and Zephyr builds alike.
Each section of the ELF file counts as flash, RAM or both, as in the PX4 flash analysis, and the linker map file gives the library or object file that each byte comes from.

For example, to compare the two operating systems on the NXP i.MX RT1176, with the NuttX build of FMU-v6XRT and the SIH configuration on the VMU RT1170:

```sh
make -C PX4-Autopilot px4_fmu-v6xrt_default
west build -b vmu_rt1170/mimxrt1176/cm7 -d build-sih PX4-Autopilot/platforms/zephyr/app -- -DFILE_SUFFIX=sih -DCONFIG_SIZE_OPTIMIZATIONS=y
PX4-Autopilot/Tools/footprint.py PX4-Autopilot/build/px4_fmu-v6xrt_default/px4_fmu-v6xrt_default.elf build-sih/zephyr/zephyr.elf
```

At the time of writing, with a commit of Zephyr main after 4.5.0-rc2, this gives (in KiB):

| Component                      | NuttX flash | NuttX RAM | Zephyr flash | Zephyr RAM |
| ------------------------------ | ----------: | --------: | -----------: | ---------: |
| Kernel and POSIX API           |          35 |        59 |           46 |         32 |
| Drivers and HAL                |          67 |        60 |          107 |         48 |
| Shell and logging              |          48 |         1 |           20 |          6 |
| File systems                   |          35 |         1 |            3 |          0 |
| Networking                     |          34 |         9 |           29 |         17 |
| C library and compiler runtime |          51 |         6 |           38 |          0 |
| Operating system               |         271 |       136 |          243 |        104 |
| PX4 platform layer             |          34 |        17 |           11 |         13 |
| PX4 startup scripts (ROMFS)    |         177 |         0 |            0 |          0 |
| PX4 modules and libraries      |        1924 |       224 |          860 |         15 |
| Other and alignment            |           8 |         1 |           15 |          9 |
| Total                          |        2414 |       378 |         1128 |        141 |

Only the operating system rows compare the two systems, as the builds differ in other ways:

- The NuttX build has all the modules and drivers of FMU-v6XRT, and the Zephyr build the multicopter flight stack, with SIH and MAVLink.
  The sensor drivers are PX4 modules with NuttX, and Zephyr drivers with Zephyr, where they take 28 KiB of flash.
- NuttX is configured with the FAT, compressed ROM and proc file systems, the SD card, USB, CAN, TCP and its shell, nsh.
  The Zephyr build has USB, Ethernet with UDP, and the Zephyr shell and logging.
  The buffers of its 7 serial ports (10.5 KiB) are in the PX4 platform layer.
- NuttX copies the most used PX4 code (226 KiB) to the ITCM, the fast instruction memory of the Cortex-M7, so that this code also counts as RAM.
- The RAM is the static RAM.
  NuttX uses the rest of the memory as its heap.
  Zephyr reserves a kernel heap for the thread stacks at build time (256 KiB, which is not in the table), and the C library heap takes the rest.
- NuttX is built with GCC 13.2, and Zephyr with GCC 14.3 from the Zephyr SDK.
  The Zephyr builds are built with `-Os` here, as NuttX builds itself, instead of for speed as the board configuration of the VMU RT1170 does.

## Limitations

- PX4 only flies in simulation: there are no actuator outputs, RC input or logging.
- Parameters are kept in RAM only.
- Sensors are polled: the IMU FIFOs are not used.
- Serial ports use the interrupt-driven UART API, without DMA, and have no single-wire, inverted or swapped modes.
- The GPS driver does not build, as its protocol parsers define `MIN` and `MAX`, which Zephyr defines too.
- Dynamic modules (`DYNAMIC` in `px4_add_module()`) are not supported.
- `native_sim` is not supported, as its POSIX API uses the host C library.
- Only the QEMU board is tested at runtime. The NXP VMU RT1170 is only built.

## Roadmap

- Read the IMU FIFOs with the Zephyr sensor streaming API.
- Actuator outputs with the Zephyr PWM API.
- Device files and termios for UARTs in Zephyr itself, instead of the PX4 platform layer.
- Parameter and log storage on the Zephyr file systems or ZMS.
- Dynamic modules as Zephyr loadable extensions (below).

### Loadable Modules

On Linux, PX4 builds `DYNAMIC` modules as shared libraries, which the `dyn` command loads.
On Zephyr, the plan is to build them as [LLEXT](https://docs.zephyrproject.org/latest/services/llext/index.html) extensions:

- `px4_add_module(... DYNAMIC)` creates the extension with `add_llext_target()`.
- `dyn` loads the extension from a file system with `llext_load()`, runs its constructors with `llext_bringup()`, finds its `px4_module_main` entry point with `llext_find_sym()`, and runs it as a PX4 task.
- The firmware exports the PX4 API that modules use (uORB, parameters, work queues, performance counters and logging), from a list generated at build time.
- The extensions can be tested in QEMU, as Zephyr supports LLEXT on Arm Cortex-M.
