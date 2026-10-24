"""Host checks for S01. Requires a C99 GCC-compatible compiler, not an MCU.

Run: python3 motor_controller/tests/run_sampling_window_tests.py
These checks prove configured timing inequalities and modulation invariants;
they cannot qualify the analog settling time or the actual timer waveforms.
"""
from pathlib import Path
import os
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
project = root / 'project'
cc = os.environ.get('CC', 'gcc')
flags = ['-std=c99', '-O2', '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter', '-pedantic',
         '-I' + str(project / 'Board'), '-I' + str(project / 'APP'), '-I' + str(project / 'APP/FOC')]
with tempfile.TemporaryDirectory(prefix='foc_s01_') as temporary:
    binary = Path(temporary) / 'test_sampling_window'
    command = [cc, *flags, str(root / 'tests/test_sampling_window.c'), '-lm', '-o', str(binary)]
    subprocess.run(command, check=True)
    subprocess.run([str(binary)], check=True)
    # Independent negative configuration: refuse impossible timing at compile time.
    probe = Path(temporary) / 'invalid_window.c'
    probe.write_text('#include "at32_foc_timing.h"\nint main(void) { return 0; }\n')
    bad = subprocess.run([cc, *flags, '-DAT32_FOC_CURRENT_SETTLING_NS=26000u',
                          '-fsyntax-only', str(probe)], capture_output=True, text=True)
    assert bad.returncode != 0 and 'no usable PWM duty range' in bad.stderr

    vendor = root / 'libraries'
    if vendor.is_dir():
        include_paths = [project / 'APP', project / 'APP/FOC', project / 'APP/Driver',
                         project / 'APP/config', project / 'Board', project / 'inc',
                         vendor / 'drivers/inc', vendor / 'cmsis/cm4/core_support',
                         vendor / 'cmsis/cm4/device_support']
        target_flags = ['-std=c99', '-O2', '-DAT32M416KBU7_4', '-DUSE_STDPERIPH_DRIVER',
                        '-DPLATFORM_AT32', '-Wno-int-to-pointer-cast', '-Wno-pointer-to-int-cast']
        target_flags += ['-I' + str(p) for p in include_paths]
        integration = Path(temporary) / 'test_foc_voltage'
        subprocess.run([cc, *target_flags, '-ffunction-sections', '-fdata-sections',
                        str(project / 'APP/FOC/foc.c'), str(root / 'tests/test_foc_voltage_integration.c'),
                        '-Wl,--gc-sections', '-lm', '-o', str(integration)], check=True)
        subprocess.run([str(integration)], check=True)
        board_test = Path(temporary) / 'test_sampling_board'
        subprocess.run([cc, *target_flags, '-ffunction-sections', '-fdata-sections',
                        str(root / 'tests/test_sampling_board.c'), '-Wl,--gc-sections',
                        '-lm', '-o', str(board_test)], check=True)
        subprocess.run([str(board_test)], check=True)
        for name in ['APP/FOC/foc.c', 'Board/board_at32.c', 'src/wk_tmr.c', 'src/wk_adc.c']:
            subprocess.run([cc, *target_flags, '-fsyntax-only', str(project / name)], check=True)
        print('PASS: AT32 modified translation units pass host syntax checks (not an ARM firmware build)')
    else:
        print('SKIP: real foc_voltage integration and vendor-header syntax checks need libraries/')

adc = (project / 'src/wk_adc.c').read_text()
board = (project / 'Board/board_at32.c').read_text()
timer = (project / 'src/wk_tmr.c').read_text()
assert 'adc_common_struct.div = ADC_HCLK_DIV_6;' in adc
sequence = re.findall(r'adc_preempt_channel_set\(ADC2, ADC_CHANNEL_(\d+), (\d+), ADC_SAMPLETIME_13_5\)', adc)
assert sequence == [('7', '1'), ('8', '2'), ('10', '3'), ('5', '4')], sequence
for phase, channel in enumerate([1, 2, 3]):
    assert f's->phase_current.adc_raw[{phase}] = adc_preempt_conversion_data_get(ADC2, ADC_PREEMPT_CHANNEL_{channel});' in board
assert 's->motor_temp.adc_raw = adc_preempt_conversion_data_get(ADC2, ADC_PREEMPT_CHANNEL_4);' in board
assert 'tmr_base_init(TMR1, AT32_FOC_TIMER_ARR, AT32_FOC_TIMER_DIV_REG);' in timer
assert 'TMR_SELECT_CHANNEL_4, (uint16_t)AT32_FOC_ADC_TRIGGER_CCR' in timer
print('PASS: impossible timing rejected; ADC rank/readback and timer contract aligned')

