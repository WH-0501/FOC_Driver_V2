"""Compare a preserved pre-optimization engine with the current implementation.

Runs identical deterministic plant/noise cases; no hardware. Baseline source is
supplied explicitly so obsolete production code need not live in this repository.
Requires Windows MSVC and numpy. Writes a numerical comparison summary as JSON.
"""
from pathlib import Path
import argparse
import ctypes as ct
import json
import subprocess
import numpy as np

p = argparse.ArgumentParser()
p.add_argument('--baseline-root', type=Path, required=True, help='Previous motor_controller directory')
p.add_argument('--build-dir', type=Path, required=True)
a = p.parse_args()
current = Path(__file__).resolve().parents[1]
work = a.build_dir.resolve()
work.mkdir(parents=True, exist_ok=True)
wrapper = work/'compare_wrapper.c'
wrapper.write_text(r'''
#include "sweep_engine.h"
#include <math.h>
#include <stdint.h>
__declspec(dllexport) unsigned engine_bytes(void) { return sizeof(sweep_engine_t); }
__declspec(dllexport) int simulate(const sweep_config_t *cfg, unsigned seed, float noise,
                                  float *trace, sweep_result_t *results) {
  sweep_engine_t e = {0};
  unsigned n=0, i;
  float y=0, a=expf(-6.2831853071795864769f*800.0f/cfg->sample_hz);
  int error=sweep_engine_prepare(&e,cfg);
  if(error) return -error;
  sweep_engine_start(&e);
  while(e.state==SWEEP_RUNNING && n<500000u) {
    float x, perturbation;
    seed=1664525u*seed+1013904223u;
    perturbation=((float)((int32_t)(seed>>8)-8388608)/8388608.0f)*noise;
    x=sweep_engine_step(&e,y+0.04f+perturbation);
    trace[n++]=x;
    y=a*y+(1.0f-a)*x;
  }
  if(e.state!=SWEEP_DONE) return -3;
  for(i=0;i<cfg->count;++i) if(sweep_engine_result(&e,i,results+i)) return -4;
  return (int)n;
}
''', encoding='utf-8')
vc = Path('C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Auxiliary/Build/vcvars64.bat')
class Config(ct.Structure):
    _fields_ = [('method',ct.c_uint),('count',ct.c_uint),('low',ct.c_float),('high',ct.c_float),('peak',ct.c_float),('fs',ct.c_float)]
class Result(ct.Structure):
    _fields_ = [(name,ct.c_float) for name in ('hz','input','output','real','imag','coherence')] + [('blocks',ct.c_uint16)]
libraries=[]
for name, root in [('before',a.baseline_root.resolve()),('after',current)]:
    folder=work/name
    folder.mkdir(exist_ok=True)
    sweep=root/'project/APP/FOC/sweep'
    dll=folder/(name+'.dll')
    cmd=['cl','/nologo','/LD','/TC','/std:c11','/O2','/I'+str(sweep),str(wrapper),str(sweep/'sweep_engine.c'),'/Fe:'+str(dll)]
    batch=folder/'build.cmd'
    batch.write_text('@echo off\ncall "'+str(vc)+'" >nul\n'+subprocess.list2cmdline(cmd)+'\nexit /b %errorlevel%\n',encoding='utf-8')
    subprocess.run(['cmd','/d','/c',str(batch)],cwd=folder,check=True)
    lib=ct.CDLL(str(dll))
    lib.simulate.argtypes=[ct.POINTER(Config),ct.c_uint,ct.c_float,ct.POINTER(ct.c_float),ct.POINTER(Result)]
    libraries.append(lib)
summary=dict(before_bytes=libraries[0].engine_bytes(),after_bytes=libraries[1].engine_bytes(),cases=0,
             max_waveform_delta_a=0.0,max_gain_delta_db=0.0,max_phase_delta_deg=0.0,max_coherence_delta=0.0)
for fs in (10000,20000,40000):
    for method in (1,2):
        for count in range(3,17):
            cfg=Config(method,count,20,min(2000,fs/10),.005 if count%2 else .5,fs)
            traces=[np.zeros(500000,dtype=np.float32) for _ in libraries]
            results=[(Result*count)() for _ in libraries]
            counts=[lib.simulate(ct.byref(cfg),314159,ct.c_float(.01),trace.ctypes.data_as(ct.POINTER(ct.c_float)),result)
                    for lib,trace,result in zip(libraries,traces,results)]
            assert counts[0]>0 and counts[0]==counts[1],(fs,method,count,counts)
            wave=float(np.max(np.abs(traces[0][:counts[0]]-traces[1][:counts[0]])))
            assert wave<2e-6,(cfg.method,count,wave)
            summary['max_waveform_delta_a']=max(summary['max_waveform_delta_a'],wave)
            for old,new in zip(*results):
                assert old.hz==new.hz and old.blocks==new.blocks
                ratio=complex(new.real,new.imag)/complex(old.real,old.imag)
                gain=abs(20*np.log10(abs(ratio)))
                phase=abs(np.angle(ratio,deg=True))
                coherence=abs(new.coherence-old.coherence)
                assert gain<.001 and phase<.001 and coherence<1e-5,(fs,method,count,gain,phase,coherence)
                for key,value in [('max_gain_delta_db',gain),('max_phase_delta_deg',phase),('max_coherence_delta',coherence)]:
                    summary[key]=max(summary[key],float(value))
            summary['cases']+=1
(work/'comparison.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
print(json.dumps(summary,indent=2))
