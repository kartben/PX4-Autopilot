#!/usr/bin/env python3
"""
Profile a running firmware and list the functions to move from the flash to the ITCM.

The program counter of the target is sampled with pyOCD, and the functions that run
from the flash are ranked by samples per byte. The most sampled ones, up to the
requested share of the samples in the flash and within the size given, are written as
linker script lines, in the format of the ITCM function lists:

    *(.text._ZN3Ekf17predictCovarianceERKN9estimator9imuSampleE)

The functions of the list given as --output count wherever they run, so that the list can be
made again with a firmware built with it. Only functions with their own input section, from
-ffunction-sections, can be listed.
memcpy() and memset() are left out, as the start-up code loads the ITCM with them, and so
are the functions that run before the ITCM is loaded.

Example, on a MIMXRT1170-EVK running SIH:

    Tools/itcm_profile.py --target mimxrt1170_cm7 build/zephyr/zephyr.elf build/zephyr/zephyr.map \\
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

EXCLUDED = {'memcpy', 'memset', 'z_arm_reset', 'z_prep_c', 'z_data_copy', 'z_bss_zero', 'arch_cpu_idle'}


def load_elf(path):
    elf = ELFFile(open(path, 'rb'))
    functions = sorted((s['st_value'] & ~1, s['st_size'], s.name)
                       for s in elf.get_section_by_name('.symtab').iter_symbols()
                       if s['st_info']['type'] == 'STT_FUNC' and s['st_size'])
    itcm = [(s['sh_addr'], s['sh_addr'] + s['sh_size']) for s in elf.iter_sections()
            if s['sh_flags'] & SH_FLAGS.SHF_EXECINSTR and 'itcm' in s.name.lower()]
    return functions, itcm


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('elf', help='firmware ELF file')
    parser.add_argument('map', help='linker map file, for the input sections of the functions')
    parser.add_argument('--target', required=True, help='pyOCD target, for example mimxrt1170_cm7')
    parser.add_argument('--samples', type=int, default=6000, help='number of samples (default: 6000)')
    parser.add_argument('--coverage', type=float, default=0.9,
                        help='share of the samples in the flash to cover (default: 0.9)')
    parser.add_argument('--max-size', type=int, help='maximum size of the listed functions, in bytes')
    parser.add_argument('--output', help='linker script to write the function list to')
    args = parser.parse_args()

    from pyocd.core.helpers import ConnectHelper

    functions, itcm = load_elf(args.elf)
    starts = [f[0] for f in functions]
    sections = set(re.findall(r'^ \.text\.(\S+)', open(args.map, errors='replace').read(), re.M))
    listed = set()

    if args.output and os.path.exists(args.output):
        listed = set(re.findall(r'^\*\(\.text\.(\S+)\)$', open(args.output).read(), re.M))

    def function(address):
        i = bisect.bisect_right(starts, address) - 1
        if i >= 0 and address < functions[i][0] + functions[i][1]:
            return functions[i]
        return None

    in_itcm = 0
    in_flash = collections.Counter()
    with ConnectHelper.session_with_chosen_probe(target_override=args.target, connect_mode='attach',
                                                 options={'resume_on_disconnect': True}) as session:
        target = session.target
        for _ in range(args.samples):
            target.halt()
            pc = target.read_core_register('pc')
            target.resume()

            f = function(pc)

            if any(start <= pc < end for start, end in itcm) and not (f and f[2] in listed):
                in_itcm += 1
            elif f:
                in_flash[f] += 1

            time.sleep(0.002)

    flash_samples = sum(in_flash.values())
    print(f'{in_itcm} samples in the ITCM, {flash_samples} in the flash or in the listed functions')

    ranked = sorted(in_flash.items(), key=lambda item: item[1] / item[0][1], reverse=True)
    covered = 0
    size = 0
    lines = []

    for (_, length, name), count in ranked:
        if covered >= args.coverage * flash_samples:
            break

        if args.max_size and size + length > args.max_size:
            continue

        covered += count

        if name not in EXCLUDED and name in sections:
            lines.append(f'*(.text.{name})')
            size += length

    print(f'{len(lines)} functions, {size} bytes, for {covered * 100 / max(flash_samples, 1):.0f}% '
          'of the samples in the flash')

    if args.output:
        with open(args.output, 'w') as output:
            output.write('/* Functions that run most from the flash, from Tools/itcm_profile.py */\n\n')
            output.write('\n'.join(lines) + '\n')


if __name__ == '__main__':
    main()
