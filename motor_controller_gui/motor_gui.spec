# -*- mode: python ; coding: utf-8 -*-
import sys
from pathlib import Path

can_backends = ['can.interfaces.virtual', 'can.interfaces.pcan', 'can.interfaces.slcan']
if sys.platform.startswith('linux'):
    can_backends.append('can.interfaces.socketcan')


a = Analysis(
    ['motor_gui.py'],
    pathex=[],
    binaries=[],
    datas=[('library', 'library')] if Path('library').is_dir() else [],
    hiddenimports=can_backends,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='motor_gui',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
