"""Check ARM object code/relocations and ISR math after the two firmware builds.

This is a static isolation check, not a substitute for hardware WCET measurement.
"""
import argparse
from pathlib import Path
import re
import struct


def elf_sections(path):
    data = path.read_bytes()
    assert data[:6] == b'\x7fELF\x01\x01', 'Expected little-endian ELF32'
    offset = struct.unpack_from('<I', data, 32)[0]
    size, count, names = struct.unpack_from('<HHH', data, 46)
    headers = [struct.unpack_from('<10I', data, offset + i * size) for i in range(count)]
    strings = data[headers[names][4]:headers[names][4] + headers[names][5]]
    def name(table, start):
        return table[start:table.index(b'\0', start)].decode()
    result = {}
    for idx, h in enumerate(headers):
        payload = data[h[4]:h[4] + h[5]]
        relocations = []
        for rel in headers:
            if rel[1] != 9 or rel[7] != idx:
                continue
            sym = headers[rel[6]]
            st = headers[sym[6]]
            symnames = data[st[4]:st[4] + st[5]]
            for pos in range(rel[4], rel[4] + rel[5], 8):
                address, info = struct.unpack_from('<II', data, pos)
                symname = struct.unpack_from('<I', data, sym[4] + (info >> 8) * sym[9])[0]
                relocations.append((address, info & 255, name(symnames, symname)))
        result[name(strings, h[0])] = (payload, relocations)
    return result


p = argparse.ArgumentParser()
p.add_argument('--default-build', type=Path, required=True)
p.add_argument('--enabled-build', type=Path, required=True)
a = p.parse_args()
normal = elf_sections(a.default_build / 'foc.o')
diagnostic = elf_sections(a.enabled_build / 'foc.o')
for function in ('foc_apply_voltage', 'foc_current', 'foc_control_run', 'foc_modulate_voltage_sincos'):
    section = 'i.' + function
    assert normal[section] == diagnostic[section], function + ' differs between builds'
    print('PASS identical code and relocations:', function, len(normal[section][0]), 'bytes')
for section, (_, relocations) in normal.items():
    assert not any('sweep' in symbol for _, _, symbol in relocations), section
print('PASS default FOC object has no sweep references')

disasm = (a.enabled_build / 'sweep_engine.disasm.txt').read_text()
sections = re.split(r"\*\* Section #\d+ '([^']+)'[^\n]*\n", disasm)
for section in ('i.sweep_engine_step', 'i.reset_phase'):
    body = sections[sections.index(section) + 1]
    assert not re.search(r'VDIV|__aeabi_|__hardfp_|\b(?:sinf|cosf|powf|sqrtf)\b', body), section
print('PASS engine ISR step/phase reset: no division, software double, or libm calls')
