#!/usr/bin/env python3
"""Break down the flash and static RAM of PX4 firmware by component.

Each allocated section of the ELF counts as flash, RAM or both, as in
Tools/ci/firmware_size.py. The linker map file next to the ELF attributes the
bytes of every section to the library or object file they come from, which
gives the component. NuttX and Zephyr builds are both supported, so that the
cost of each operating system can be compared:

    Tools/footprint.py build/px4_fmu-v6xrt_default/px4_fmu-v6xrt_default.elf \\
        build/zephyr_vmu_rt1170_mimxrt1176_cm7/zephyr/zephyr.elf
"""

import argparse
from collections import defaultdict
from dataclasses import dataclass
import os
from pathlib import Path
import re
import subprocess

OS = "Operating system"
PX4_PLATFORM = "PX4 platform layer"
PX4_ROMFS = "PX4 startup scripts (ROMFS)"
PX4_CODE = "PX4 modules and libraries"
HEAP = "Heap reserved at build time"
OTHER = "Other and alignment"

# The first matching rule gives the component of an operating system input
OS_RULES = [
    ("C++ runtime", r"libs/libxx/|/libstdc\+\+\.a|/libsupc\+\+\.a|zephyr/lib/cpp/"),
    ("C library and compiler runtime",
     r"libs/libc/|libs/libm/|/libc\.a|/libm\.a|/libgcc\.a|/libc_nano\.a|zephyr/lib/libc/"
     r"|libzephyr\.a\((cbprintf|printk)"),
    ("Networking", r"nuttx/net/|zephyr/subsys/net/|zephyr/lib/net_buf/"),
    ("File systems", r"nuttx/fs/|zephyr/subsys/fs/|^modules/fatfs/"),
    ("Shell and logging", r"NuttX/apps/|zephyr/subsys/(shell|logging)/|libzephyr\.a\((log_|shell)"),
    ("Drivers and HAL",
     r"nuttx/drivers/|libarch\.a\((?!arm_)|zephyr/drivers/|zephyr/subsys/(usb|sd)/|zephyr/soc/|zephyr/boards/|^modules/hal"),
    ("Kernel and POSIX API", r"NuttX/nuttx/|^zephyr/"),
]

OS_COMPONENTS = [rule[0] for rule in reversed(OS_RULES)]
COMPONENTS = OS_COMPONENTS + [OS, PX4_PLATFORM, PX4_ROMFS, PX4_CODE, HEAP, OTHER]

# Mergeable strings, whose duplicates the linker removes, by element size and alignment
MERGED = re.compile(r"\.str(\d+\.\d+)$")
OUTPUT_SECTION = re.compile(r"^(\S+)(?:\s+0x([0-9a-f]+)\s+0x([0-9a-f]+).*)?$")
INPUT_SECTION = re.compile(r"^ (\S+)(?:\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)(?:\s+(\S.*))?)?$")
WRAPPED = re.compile(r"^\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)\s*(\S.*)?$")


@dataclass
class Input:
    """An input section of the map, at its place in an output section."""
    section: str
    address: int
    size: float
    path: str


def section_memory(elf: Path) -> tuple[dict[str, tuple[int, bool, bool]], int]:
    """Return {section: (size, in flash, in RAM)} for the allocated sections of an ELF, and the gaps in its image."""
    headers = subprocess.check_output(
        ["arm-none-eabi-objdump", "--section-headers", "--wide", str(elf)],
        text=True,
        env={**os.environ, "LC_ALL": "C"},
    )
    sections = {}
    image = []

    for line in headers.splitlines():
        fields = line.split(maxsplit=7)
        if len(fields) < 8 or not fields[0].isdigit():
            continue

        size, vma, lma = (int(value, 16) for value in fields[2:5])
        flags = {flag.strip() for flag in fields[7].split(",")}
        if "ALLOC" not in flags:
            continue

        # Code and constants execute in place, initialized data and RAM
        # functions are copied out of the image, NOLOAD sections only take RAM
        in_image = {"LOAD", "CONTENTS"} <= flags
        sections[fields[1]] = (size, in_image, not (in_image and vma == lma))

        if in_image and size:
            image.append((lma, size))

    # the image spans its sections and the gaps between them, as objcopy -O binary
    span = max(lma + size for lma, size in image) - min(lma for lma, _ in image)
    return sections, span - sum(size for _, size in image)


def map_inputs(map_file: Path) -> dict[str, list[Input]]:
    """Return the inputs of each output section of a GNU ld map file."""
    lines = map_file.read_text(errors="replace").splitlines()
    start = lines.index("Linker script and memory map")
    sections: dict[str, list[Input]] = defaultdict(list)
    output = None
    pending = None

    for line in lines[start + 1:]:
        if pending is not None:
            wrapped = WRAPPED.match(line)
            name, pending = pending, None

            if wrapped and line.startswith("  "):
                if output and wrapped.group(3):
                    sections[output].append(Input(name, int(wrapped.group(1), 16), int(wrapped.group(2), 16),
                                                  wrapped.group(3)))

                continue

        if not line.strip():
            continue

        if not line[0].isspace():
            match = OUTPUT_SECTION.match(line)
            output = match.group(1) if match else None
            continue

        match = INPUT_SECTION.match(line)

        if not match or match.group(1).startswith("*("):
            continue

        if match.group(2) is None:
            pending = match.group(1)

        elif output and (match.group(4) or match.group(1) == "*fill*"):
            sections[output].append(Input(match.group(1), int(match.group(2), 16), int(match.group(3), 16),
                                          match.group(4) or ""))

    return sections


def share_merged_strings(inputs: list[Input], end: int) -> None:
    """Share the merged strings of a section among the inputs they come from.

    The map gives all the merged strings to the first input, and lists the other
    inputs with their size before merging, without advancing the location
    counter: the next input starts before their end. Their share of the merged
    strings is in proportion to their size.
    """
    inputs = [i for i in inputs if i.size]
    groups: dict[str, tuple[list[Input], list[Input]]] = defaultdict(lambda: ([], []))

    for index, entry in enumerate(inputs):
        merged = MERGED.search(entry.section)

        if not merged:
            continue

        following = inputs[index + 1].address if index + 1 < len(inputs) else end
        groups[merged.group(1)][following < entry.address + entry.size].append(entry)

    for kept, merged_away in groups.values():
        if not merged_away:
            continue

        size = sum(entry.size for entry in kept)
        weights = [max(0, entry.size - sum(e.size for e in merged_away)) for entry in kept]
        weights += [entry.size for entry in merged_away]

        for entry, weight in zip(kept + merged_away, weights):
            entry.size = size * weight / sum(weights)


def component(section: str, path: str, zephyr: bool) -> str:
    """Return the component of an input section."""
    if "kheap_buf" in section:
        return HEAP

    if zephyr:
        px4 = re.match(r"modules/px4/px4/(.*)", path)
        px4_path = px4.group(1) if px4 else None

        # the application, and the Zephyr libraries of the PX4 module
        if not px4 and path.startswith(("app/", "modules/px4/")):
            return PX4_PLATFORM

    else:
        px4_path = None if not path or path.startswith(("/", "NuttX/")) else path

    if px4_path is not None:
        if px4_path.startswith(("platforms/nuttx/", "platforms/zephyr/", "boards/")):
            return PX4_PLATFORM

        if px4_path.startswith("ROMFS/"):
            return PX4_ROMFS

        return PX4_CODE

    for name, pattern in OS_RULES:
        if re.search(pattern, path):
            return name

    return OTHER


def footprint(elf: Path) -> dict[str, list[float]]:
    """Return {component: [flash, RAM]} for a firmware ELF."""
    memory, gaps = section_memory(elf)
    sections = map_inputs(elf.with_suffix(".map"))
    zephyr = elf.name == "zephyr.elf"
    totals: dict[str, list[float]] = defaultdict(lambda: [0, 0])
    totals[OTHER][0] = gaps

    for output, (size, in_flash, in_ram) in memory.items():
        inputs = sections.get(output, [])

        if inputs:
            share_merged_strings(inputs, min(i.address for i in inputs) + size)

        # padding that the map does not list
        inputs.append(Input("", 0, max(0, size - sum(i.size for i in inputs)), ""))

        for entry in inputs:
            name = component(entry.section, entry.path, zephyr)
            totals[name][0] += entry.size if in_flash else 0
            totals[name][1] += entry.size if in_ram else 0

    totals[OS] = [sum(totals[name][i] for name in OS_COMPONENTS) for i in (0, 1)]
    return totals


def kib(size: float) -> str:
    return f"{size / 1024:.1f}"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("elf", type=Path, nargs="+", help="firmware ELF, with its map file next to it")
    args = parser.parse_args()

    results = [footprint(elf) for elf in args.elf]

    print("| Component | " + " | ".join(f"{elf} flash (KiB) | RAM (KiB)" for elf in args.elf) + " |")
    print("| --- |" + " ---: | ---: |" * len(results))

    for name in COMPONENTS:
        if any(totals[name] != [0, 0] for totals in results):
            cells = [f"{kib(totals[name][0])} | {kib(totals[name][1])}" for totals in results]
            print(f"| {name} | " + " | ".join(cells) + " |")

    totals = [[sum(t[name][i] for name in COMPONENTS if name not in (OS, HEAP)) for i in (0, 1)] for t in results]
    print("| Total, without the reserved heap | " +
          " | ".join(f"{kib(flash)} | {kib(ram)}" for flash, ram in totals) + " |")


if __name__ == "__main__":
    main()
