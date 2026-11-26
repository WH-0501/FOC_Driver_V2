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
         '-I' + str(root / 'tests/c06_support'), '-I' + str(project / 'Board'), '-I' + str(project / 'APP'), '-I' + str(project / 'APP/FOC')]
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

    irq_source=(project/'src/at32m412_416_int.c').read_text(encoding='utf-8')
    begin=irq_source.index('void ADC1_2_IRQHandler(void)')
    end=irq_source.index('\n}',begin)+2
    (Path(temporary)/'adc_sequence_irq_impl.inc').write_text(irq_source[begin:end],encoding='utf-8')
    irq_test=Path(temporary)/('test_adc_sequence_irq.exe' if os.name == 'nt' else 'test_adc_sequence_irq')
    if os.name == 'nt':
        # Use the same native Windows toolchain as the offset/encoder tests.
        vcvars=Path('C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Auxiliary/Build/vcvars64.bat')
        cmd=['cl','/nologo','/O2','/TC','/std:c11','/utf-8','/I'+temporary,
             str(root/'tests/test_adc_sequence_irq.c'),'/Fe:'+str(irq_test)]
        batch=Path(temporary)/'build_irq.cmd'
        batch.write_text('@echo off\ncall "'+str(vcvars)+'" >nul\nif errorlevel 1 exit /b 1\n'+subprocess.list2cmdline(cmd)+'\nexit /b %errorlevel%\n')
        subprocess.run(['cmd','/d','/c',str(batch)],cwd=temporary,check=True)
    else:
        subprocess.run([cc,*flags,'-I'+temporary,str(root/'tests/test_adc_sequence_irq.c'),'-o',str(irq_test)],check=True)
    subprocess.run([str(irq_test)],check=True)

    vendor = root / 'libraries'
    if vendor.is_dir():
        include_paths = [project / 'APP', project / 'APP/FOC', project / 'APP/Driver',
                         project / 'APP/config', project / 'Board', project / 'inc',
                         vendor / 'drivers/inc', vendor / 'cmsis/cm4/core_support',
                         vendor / 'cmsis/cm4/device_support']
        target_flags = ['-std=c99', '-O2', '-DAT32M416KBU7_4', '-DUSE_STDPERIPH_DRIVER',
                        '-DPLATFORM_AT32', '-Wno-int-to-pointer-cast', '-Wno-pointer-to-int-cast']
        if os.name == 'nt':
            # MinGW COFF keeps references to unrelated ARM entry points without LTO.
            target_flags.extend(['-flto', '-fwhole-program'])
        target_flags += ['-I' + str(p) for p in include_paths]
        for reuse in (0, 1):
            integration = Path(temporary) / f'test_foc_voltage_{reuse}'
            subprocess.run([cc, *target_flags, f'-DFOC_REUSE_PARK_SINCOS={reuse}', '-ffunction-sections', '-fdata-sections',
                            str(project / 'APP/FOC/foc_pid.c'), str(root / 'tests/test_foc_voltage_integration.c'),
                            '-Wl,--gc-sections', '-lm', '-o', str(integration)], check=True)
            subprocess.run([str(integration)], check=True)
        for direct_ccr in (0, 1):
            board_source = (project/'Board/board_at32.c').read_text(encoding='utf-8')
            for channel, value in ((3,'u'),(2,'v'),(1,'w')):
                board_source = board_source.replace(f'PWM_TIM_HANDLE->c{channel}dt = {value};',
                                                    f'test_direct_write({channel-1}u, {value});')
            (Path(temporary)/'sampling_board_impl.inc').write_text(board_source,encoding='utf-8')
            board_test = Path(temporary) / f'test_sampling_board_{direct_ccr}'
            subprocess.run([cc, *target_flags, '-I'+temporary, f'-DAT32_PWM_INLINE_STAGE_CCR={direct_ccr}', '-ffunction-sections', '-fdata-sections',
                            str(root / 'tests/test_sampling_board.c'), '-Wl,--gc-sections',
                            '-lm', '-o', str(board_test)], check=True)
            subprocess.run([str(board_test)], check=True)
        current_test = Path(temporary)/'test_current_signal'
        subprocess.run([cc,*target_flags,'-ffunction-sections','-fdata-sections',
                        str(root/'tests/test_current_signal.c'),str(project/'APP/FOC/current_sense.c'),
                        '-Wl,--gc-sections','-lm','-o',str(current_test)],check=True)
        subprocess.run([str(current_test)],check=True)
        for name in ['APP/FOC/foc.c', 'Board/board_at32.c', 'src/wk_tmr.c', 'src/wk_adc.c']:
            subprocess.run([cc, *target_flags, '-fsyntax-only', str(project / name)], check=True)
        print('PASS: AT32 modified translation units pass host syntax checks (not an ARM firmware build)')
    else:
        print('SKIP: real foc_voltage integration and vendor-header syntax checks need libraries/')

adc = (project / 'src/wk_adc.c').read_text(encoding='utf-8')
board = (project / 'Board/board_at32.c').read_text(encoding='utf-8')
timer = (project / 'src/wk_tmr.c').read_text(encoding='utf-8')
assert 'adc_common_struct.div = ADC_HCLK_DIV_6;' in adc
sequence = re.findall(r'adc_preempt_channel_set\(ADC2, ADC_CHANNEL_(\d+), (\d+), ADC_SAMPLETIME_13_5\)', adc)
assert sequence == [('7', '1'), ('8', '2'), ('10', '3'), ('5', '4')], sequence
for phase in range(3):
    active_board = re.sub(r'/\*.*?\*/|//[^\n]*','',board,flags=re.S)
    assert f's->phase_current.adc_raw[{phase}] = (uint16_t)ADC2->pdt{phase + 1};' in active_board
assert 'board_get_vbus_and_temp' in board and 'ADC_PREEMPT_CHANNEL_4' in board
assert 'motor_temp.adc_raw = adc_preempt_conversion_data_get(ADC2, ADC_PREEMPT_CHANNEL_4)' in board
assert 'FOC_TEMP_ADC_TO_CELSIUS' in board
assert 'tmr_base_init(TMR1, AT32_FOC_TIMER_ARR, AT32_FOC_TIMER_DIV_REG);' in timer
assert 'TMR_SELECT_CHANNEL_4, (uint16_t)AT32_FOC_ADC_TRIGGER_CCR' in timer
assert 'tmr_primary_mode_select(TMR1, TMR_PRIMARY_SEL_C4ORAW);' in timer
assert 'adc_preempt_conversion_trigger_set(ADC2, ADC_PREEMPT_TRIG_TMR1TRGOUT, ADC_PREEMPT_TRIG_EDGE_RISING);' in adc
irq=(project/'src/at32m412_416_int.c').read_text(encoding='utf-8')
handler=irq[irq.index('void ADC1_2_IRQHandler'):]
assert handler.index('adc_flag_clear(ADC2, ADC_PCCE_FLAG);') < handler.index('CURRENT_LOOP_IRQ_HANDLER(ADC2);')
print('PASS: impossible timing rejected; ADC rank/readback and timer contract aligned')

