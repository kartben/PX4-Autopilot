#!/usr/bin/env python3
"""
Profile a running firmware and rank the code to move from the flash to the ITCM.

The program counter of the target is sampled with pyOCD, or read from the output of the
pc_profile shell command of the firmware (CONFIG_PX4_PC_PROFILE), which samples it without a
debug probe. The functions that run from the flash are ranked by samples per byte, and
written in that order as linker script lines, each with the size of its input section:

    *(.text._ZN3Ekf17predictCovarianceERKN9estimator9imuSampleE) /* 1264 */

The build takes the lines in order while they fit in the ITCM of the board, see
platforms/zephyr/module/itcm_fit.py. A function without its own input section, as those
of the compiler runtime, is listed as the archive member that holds it.

The functions of the list given as --output count wherever they run, so that the list can be
made again with a firmware built with it. memcpy() and memset() are left out, as the
start-up code loads the ITCM with them, and so are the functions that run before the ITCM is
loaded.

Examples, with SIH flying on a MIMXRT1170-EVK with a probe, and on a board without one:

    Tools/itcm_profile.py --target mimxrt1170_cm7 build/zephyr/zephyr.elf build/zephyr/zephyr.map \\
        --output platforms/zephyr/module/itcm_functions.ld

    Tools/itcm_profile.py --samples-file samples_*.txt build/zephyr/zephyr.elf build/zephyr/zephyr.map \\
        --output platforms/zephyr/module/itcm_functions.ld
"""

import argparse
import bisect
import collections
import os
import re
import time

from elftools.elf.constants import SH_FLAGS
from elftools.elf.elffile import ELFFile

EXCLUDED = {'memcpy', 'memset', 'z_arm_reset', 'z_prep_c', 'z_data_copy', 'z_bss_zero', 'arch_cpu_idle',
            'z_arm_debug_monitor', 'px4_pc_profile_sample'}


def load_elf(path):
    elf = ELFFile(open(path, 'rb'))
    functions = sorted((s['st_value'] & ~1, s['st_size'], s.name)
                       for s in elf.get_section_by_name('.symtab').iter_symbols()
                       if s['st_info']['type'] == 'STT_FUNC' and s['st_size'])
    itcm = [(s['sh_addr'], s['sh_addr'] + s['sh_size']) for s in elf.iter_sections()
            if s['sh_flags'] & SH_FLAGS.SHF_EXECINSTR and 'itcm' in s.name.lower()]
    return functions, itcm


def load_map(path):
    """The code input sections of the link, by address: (address, size, section name, object file)."""
    text = open(path, errors='replace').read()
    inputs = [(int(addr, 16), int(size, 16), name, obj) for name, addr, size, obj in
              re.findall(r'^ (\.text\S*)(?:\n\s+|\s+)0x([0-9a-f]+)\s+0x([0-9a-f]+)\s+(\S+)', text, re.M)
              if int(size, 16)]
    return sorted(inputs)


def sample_with_probe(target_name, samples):
    from pyocd.core.helpers import ConnectHelper

    pcs = []
    with ConnectHelper.session_with_chosen_probe(target_override=target_name, connect_mode='attach',
                                                 options={'resume_on_disconnect': True}) as session:
        target = session.target
        for _ in range(samples):
            target.halt()
            pcs.append(target.read_core_register('pc'))
            target.resume()
            time.sleep(0.002)

    return pcs


def read_samples(paths):
    pcs = []
    for path in paths:
        pcs += [int(word, 16) for word in re.findall(r'\b[0-9a-f]{8}\b', open(path).read())]
    return pcs


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('elf', help='firmware ELF file')
    parser.add_argument('map', help='linker map file, for the input sections of the functions')
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--target', help='pyOCD target, for example mimxrt1170_cm7')
    source.add_argument('--samples-file', nargs='+', help='output of "pc_profile dump" on the target')
    parser.add_argument('--samples', type=int, default=6000, help='number of samples with a probe (default: 6000)')
    parser.add_argument('--coverage', type=float, default=0.99,
                        help='share of the samples in the flash to cover (default: 0.99)')
    parser.add_argument('--output', help='linker script to write the ranked list to')
    args = parser.parse_args()

    functions, itcm = load_elf(args.elf)
    starts = [f[0] for f in functions]
    inputs = load_map(args.map)
    input_starts = [i[0] for i in inputs]
    listed = set()

    if args.output and os.path.exists(args.output):
        listed = set(re.findall(r'^\*\(\.text\.(\S+)\)', open(args.output).read(), re.M))

    def lookup(table, table_starts, address):
        i = bisect.bisect_right(table_starts, address) - 1
        if i >= 0 and address < table[i][0] + table[i][1]:
            return table[i]
        return None

    if args.target:
        pcs = sample_with_probe(args.target, args.samples)
    else:
        pcs = read_samples(args.samples_file)

    in_itcm = 0
    in_flash = collections.Counter()
    for pc in pcs:
        f = lookup(functions, starts, pc)

        if any(start <= pc < end for start, end in itcm) and not (f and f[2] in listed):
            in_itcm += 1
        elif f:
            in_flash[f] += 1

    flash_samples = sum(in_flash.values())
    print(f'{len(pcs)} samples, {in_itcm} in the ITCM, {flash_samples} in the flash or in the listed functions')

    # rank the input sections that hold the sampled functions, by samples per byte
    sections = collections.Counter()
    for (address, _, name), count in in_flash.items():
        section = lookup(inputs, input_starts, address)
        if name in EXCLUDED or not section:
            continue

        _, size, section_name, obj = section
        if section_name == f'.text.{name}':
            line = f'*({section_name})'
        else:
            member = re.match(r'.*/(lib[^/(]+\.a)\(([^)]+)\)$', obj)
            if not member or section_name != '.text':
                continue
            line = f'*{member.group(1)}:{member.group(2)}(.text .text.*)'

        sections[(line, size)] += count

    ranked = sorted(sections.items(), key=lambda item: item[1] / item[0][1], reverse=True)
    covered = 0
    size = 0
    lines = []

    for (line, length), count in ranked:
        if covered >= args.coverage * flash_samples:
            break

        covered += count
        size += length
        lines.append(f'{line} /* {length} */')

    print(f'{len(lines)} input sections, {size} bytes, for {covered * 100 / max(flash_samples, 1):.0f}% '
          'of the samples in the flash')

    if args.output:
        with open(args.output, 'w') as output:
            output.write('/*\n'
                         ' * Code that runs most from the flash, ranked by samples per byte, with the size of each\n'
                         ' * input section, from Tools/itcm_profile.py. The build takes the lines in order while\n'
                         ' * they fit in the ITCM, see itcm_fit.py.\n'
                         ' */\n\n')
            output.write('\n'.join(lines) + '\n')


if __name__ == '__main__':
    main()
