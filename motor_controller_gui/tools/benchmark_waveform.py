"""Repeatable synthetic benchmark; does not claim hardware or GUI frame rate."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import json
import time
import numpy as np
from core.waveform_buffer import WaveformBuffer
from core.waveform_analysis import peak_envelope

b = WaveformBuffer()
sample = dict(position_plot=1, velocity=100, iq_ma=200, id_ma=0, ia_ma=10, ib_ma=-10, ic_ma=0)
t0 = time.perf_counter()
for i in range(240000):
    b.append_sample(sample, i*.001)
ingest = time.perf_counter()-t0
durations = []
for _ in range(100):
    t0 = time.perf_counter()
    data = b.snapshot()
    for key in data:
        if key != 'times':
            peak_envelope(data['times'], data[key], 2000)
    durations.append((time.perf_counter()-t0)*1000)
print(json.dumps(dict(ingest_samples_per_second=round(240000/ingest),
    cached_points=len(b), ring_bytes=b._data.nbytes,
    ten_channel_120k_snapshot_envelope_ms_p50=round(float(np.median(durations)), 2),
    ten_channel_120k_snapshot_envelope_ms_p95=round(float(np.percentile(durations,95)),2)), indent=2))
