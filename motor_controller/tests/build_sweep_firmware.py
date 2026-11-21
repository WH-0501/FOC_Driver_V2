"""Compile/link the actual Keil target into a separate directory; never flash.

Uses the project's includes, defines, optimization level and SRAM scatter file.
Requires ARM Compiler 5.06. --disable-sweep verifies complete feature removal.
"""
from pathlib import Path
import argparse
import subprocess
import xml.etree.ElementTree as ET

p = argparse.ArgumentParser()
p.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
p.add_argument('--build-dir', type=Path, required=True)
p.add_argument('--tool-dir', type=Path, default=Path('C:/Keil_v5/ARM/ARM_Compiler_5.06u7/bin'))
features = p.add_mutually_exclusive_group()
features.add_argument('--disable-sweep', action='store_true')
features.add_argument('--enable-sweep', action='store_true')
a = p.parse_args()
mdk = a.root.resolve() / 'project/MDK_V5'
out = a.build_dir.resolve()
out.mkdir(parents=True, exist_ok=True)
target = ET.parse(mdk / 'motor_controller.uvprojx').find('.//Target')
cads = target.find('.//Cads')
includes = [(mdk / s).resolve() for s in cads.findtext('VariousControls/IncludePath').split(';') if s]
defines = [s.strip() for s in cads.findtext('VariousControls/Define').split(',') if s.strip()]
if a.disable_sweep or a.enable_sweep:
    defines = [d for d in defines if not d.startswith('FOC_SWEEP_ENABLED=')]
    defines.append('FOC_SWEEP_ENABLED=' + ('1' if a.enable_sweep else '0'))
logs, objects = [], []

def run(cmd):
    result = subprocess.run([str(x) for x in cmd], cwd=out, capture_output=True,
                            text=True, encoding='utf-8', errors='replace')
    logs.extend([subprocess.list2cmdline([str(x) for x in cmd]), result.stdout, result.stderr])
    (out / 'build.log').write_text('\n'.join(logs), encoding='utf-8')
    if result.returncode:
        print(result.stdout, result.stderr)
        raise SystemExit(result.returncode)

run([a.tool_dir/'armcc.exe', '--vsn'])
for entry in target.findall('.//Groups/Group/Files/File'):
    if entry.findtext('.//IncludeInBuild') == '0':
        continue
    src = (mdk / entry.findtext('FilePath')).resolve()
    if src.suffix.lower() not in ('.c', '.s'):
        continue
    obj = out / (src.stem + '.o')  # scatter selectors use exact object names
    assert obj not in objects, 'Duplicate object basename: ' + src.name
    if src.suffix.lower() == '.c':
        cmd = [a.tool_dir/'armcc.exe', '--cpu=Cortex-M4.fp.sp', '--c99', '--gnu',
               '--apcs=interwork', '--split_sections', '-O'+str(int(cads.findtext('Optim'))-1)]
        if cads.findtext('oTime') == '1':
            cmd.append('-Otime')
        for inc in includes:
            cmd.extend(['-I', inc])
        for define in defines:
            cmd.extend(['-D', define])
        cmd.extend(['-c', src, '-o', obj])
    else:
        cmd = [a.tool_dir/'armasm.exe', '--cpu=Cortex-M4.fp.sp', '--apcs=interwork', src, '-o', obj]
    run(cmd)
    objects.append(obj)
run([a.tool_dir/'armlink.exe', '--cpu=Cortex-M4.fp.sp', '--strict', '--scatter',
     (mdk / target.findtext('.//LDads/ScatterFile')).resolve(), '--map', '--info=sizes,totals',
     '--list', out/'motor_controller.map', '-o', out/'motor_controller.axf', *objects])
for name in ('foc', 'sweep_engine', 'foc_sweep'):
    if (out/(name+'.o')).exists():
        run([a.tool_dir/'fromelf.exe', '--text', '-c', '--output', out/(name+'.disasm.txt'), out/(name+'.o')])
for option, suffix in (('--i32combined', 'hex'), ('--bin', 'bin')):
    run([a.tool_dir/'fromelf.exe', option, '--output', out/('motor_controller.'+suffix), out/'motor_controller.axf'])
print(f'PASS: {len(objects)} ARM objects freshly compiled and linked; sweep_override={"enabled" if a.enable_sweep else "disabled" if a.disable_sweep else "project default"}')
for line in (out/'motor_controller.map').read_text(encoding='utf-8').splitlines():
    if 'Total RO  Size' in line or 'Total RW  Size' in line or 'Total ROM Size' in line or 'Execution Region' in line:
        print(line.strip())
