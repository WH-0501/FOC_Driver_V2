"""Unit-independent step measurements; legacy *_ma keys use the selected unit."""
from dataclasses import dataclass
import math
import numpy as np


@dataclass
class StepSettings:
    min_step_ma: float = 10.0
    band_percent: float = 2.0
    absolute_band_ma: float = 5.0
    hold_ms: float = 5.0
    noise_ma: float = 5.0
    target_relative: bool = True
    window_ms: float = 0.0  # 0: until next command / end of capture
    crossing_hold_ms: float = 0.0


def analyze_steps(times, reference, current, settings=None, manual_start_s=None, manual_end_s=None):
    """Segment at EVERY reference change; never bridge invalid samples or steps.

    Rise is directional 10--90% of measured baseline -> requested final value.
    Settling is the final in-band suffix, observed for at least hold_ms.
    Oscillation cycles are pairs of alternating significant lobes AFTER the
    first overshoot lobe; amplitude excludes that initial overshoot.
    """
    cfg = settings or StepSettings()
    params = (cfg.min_step_ma, cfg.band_percent, cfg.absolute_band_ma,
              cfg.hold_ms, cfg.noise_ma, cfg.window_ms, cfg.crossing_hold_ms)
    if not all(np.isfinite(params)) or min(params) < 0 or cfg.min_step_ma <= 0 or cfg.hold_ms <= 0:
        raise ValueError('阈值必须有限且非负，最小阶跃和保持时间必须大于零')
    t, r, y = [np.asarray(v, dtype=float) for v in (times, reference, current)]
    if any(v.ndim != 1 for v in (t, r, y)) or not (len(t) == len(r) == len(y)):
        raise ValueError('时间、参考及电流必须是一维等长数组')
    if not np.all(np.isfinite(t)) or np.any(np.diff(t) <= 0):
        raise ValueError('时间必须有限且严格递增')
    if not np.all(np.isfinite(r)):
        raise ValueError('目标值包含无效数据')
    changes = np.flatnonzero(np.diff(r) != 0) + 1
    manual = manual_start_s is not None
    if manual:
        if not len(t) or not np.isfinite(manual_start_s) or not t[0] < manual_start_s <= t[-1]:
            raise ValueError('手动起点必须在缓存内，并保留阶跃前数据')
        start_index = int(np.searchsorted(t, manual_start_s))
        if manual_end_s is not None:
            if not np.isfinite(manual_end_s) or not manual_start_s < manual_end_s <= t[-1]:
                raise ValueError('观察终点必须晚于起点且位于缓存内')
            stop = int(np.searchsorted(t, manual_end_s, side='right'))
            t, r, y = t[:stop], r[:stop], y[:stop]
        # Preserve the real target; a later command still bounds the observation.
        following = changes[changes > start_index]
        if len(following):
            t, r, y = t[:following[0]], r[:following[0]], y[:following[0]]
        changes = np.array([start_index])
    results = []
    for j, start in enumerate(changes):
        end = int(changes[j+1]) if j+1 < len(changes) else len(t)
        if cfg.window_ms > 0:
            end = min(end, int(np.searchsorted(t, t[start]+cfg.window_ms/1000, side='right')))
        prev = int(changes[j-1]) if j else 0
        if not manual and abs(r[start] - r[start-1]) < cfg.min_step_ma:
            continue
        row = dict(start_s=float(t[start]), from_ma=float(r[start-1]), to_ma=float(r[start]),
                   baseline_ma=None, rise_ms=None, qualified_rise_ms=None, t10_s=None, t90_s=None,
                   level10=None, level90=None, overshoot_pct=None, settling_ms=None,
                   oscillations=None, oscillation_ma=None, oscillation_pct=None,
                   band_ma=None, sample_ms=None, window_ms=float((t[end-1]-t[start])*1000),
                   source='手动响应起点（非指令时刻）' if manual else '目标指令变化',
                   tail_mean=None, tail_error=None, tail_pp=None, status='')
        results.append(row)
        pre = y[max(prev, start-20):start]
        v = y[start:end]
        if len(pre) < 5 or len(v) < 3 or not np.all(np.isfinite(np.r_[pre, v])):
            row['status'] = '无效：阶跃前不足5点、响应不足3点或电流缺失'
            continue
        baseline = float(np.median(pre))
        delta = float(r[start]-baseline)
        row['baseline_ma'] = baseline
        if abs(delta) < cfg.min_step_ma:
            row['status'] = '无效：实际初值到目标的幅值过小'
            continue
        scale = abs(float(r[start])) if cfg.target_relative else abs(delta)
        band = max(cfg.absolute_band_ma, cfg.band_percent/100*scale)
        row['band_ma'] = band
        warnings = ['手动起点估算；未确认指令生效时刻'] if manual else []
        tail = v[-max(5, int(np.ceil(len(v)*.2))):]
        row['tail_mean'] = float(np.mean(tail))
        row['tail_error'] = row['tail_mean']-float(r[start])
        row['tail_pp'] = float(np.ptp(tail))
        if abs(row['tail_error']) > band:
            warnings.append('末段均值偏离目标（非已确认稳态）')
        if np.ptp(pre) > 2*max(band, cfg.noise_ma):
            warnings.append('初始波动较大')
        if not manual and abs(baseline-r[start-1]) > band:
            warnings.append('初值偏离原指令')
        tx = t[start:end]
        dt = np.diff(t[start-1:end])
        median_dt = float(np.median(dt))
        row['sample_ms'] = median_dt*1000
        if np.max(dt) > 2*median_dt or np.min(dt) < .5*median_dt:
            warnings.append('采样不均匀/疑似丢点')
        z = (v-baseline)/delta

        row["level10"] = baseline + .1*delta
        row["level90"] = baseline + .9*delta

        def crossing(level, hold_ms=0.0):
            above = z >= level
            starts = np.flatnonzero(above & np.r_[True, ~above[:-1]])
            stops = np.flatnonzero(above & np.r_[~above[1:], True])
            for k, stop in zip(starts, stops):
                if tx[stop]-tx[k]+1e-12 < hold_ms/1000:
                    continue
                if k == 0:
                    before = (y[start-1]-baseline)/delta
                    if before >= level:
                        return None  # crossing was already outside this segment
                    return float(t[start-1]+(level-before)/(z[0]-before)*(tx[0]-t[start-1]))
                return float(tx[k-1]+(level-z[k-1])/(z[k]-z[k-1])*(tx[k]-tx[k-1]))
            return None

        a, b = crossing(.1), crossing(.9)
        row['t10_s'], row['t90_s'] = a, b
        qa, qb = crossing(.1, cfg.crossing_hold_ms), crossing(.9, cfg.crossing_hold_ms)
        if qa is not None and qb is not None and qb >= qa and qb > tx[0]:
            row['qualified_rise_ms'] = (qb-qa)*1000
        if cfg.crossing_hold_ms > 0:
            warnings.append('首次交越可能含噪声；保持交越时间另列')
        if a is not None and b is not None and b > tx[0]:
            row['rise_ms'] = (b-a)*1000
            if b-a < 5*median_dt:
                warnings.append('上升段少于5个采样间隔')
        else:
            warnings.append('首次10%/90%交越未完整捕获')
        row['overshoot_pct'] = max(0.0, float(np.max(z)-1)*100)
        outside = np.flatnonzero(np.abs(v-r[start]) > band)
        settle = int(outside[-1]+1) if len(outside) else 0
        if settle < len(tx) and tx[-1]-tx[settle] >= cfg.hold_ms/1000:
            row['settling_ms'] = float((tx[settle]-t[start])*1000)
        else:
            warnings.append('未证实收敛：末段保持时间不足或仍超差')

        # Hysteretic lobes: deadband samples do not create noise crossings.
        error = (v-r[start])*np.sign(delta)
        threshold = max(cfg.noise_ma, np.ptp(pre)/2)
        lobes = []
        for value in error:
            if abs(value) <= threshold:
                continue
            sign = 1 if value > 0 else -1
            if not lobes or lobes[-1][0] != sign:
                lobes.append([sign, abs(float(value))])
            else:
                lobes[-1][1] = max(lobes[-1][1], abs(float(value)))
        first_over = next((k for k, lobe in enumerate(lobes) if lobe[0] > 0), None)
        after = lobes[first_over+1:] if first_over is not None else []
        row['oscillations'] = math.ceil(len(after)/2)  # incomplete final cycle counts
        row['oscillation_ma'] = max((lobe[1] for lobe in after), default=0.0)
        row['oscillation_pct'] = row['oscillation_ma']/scale*100 if scale > 0 else None
        if scale == 0:
            warnings.append('目标为零：幅值百分比不适用，使用mA')
        if cfg.window_ms > 0:
            warnings.append('按限定观察窗统计；原始峰值可能含尖峰')
        row['status'] = '；'.join(warnings) if warnings else '观察窗内已收敛'
    return results


def summarize(rows):
    """Per-metric available count, mean and worst (maximum), no implicit zeros."""
    result = {}
    for key in ('rise_ms', 'overshoot_pct', 'settling_ms', 'oscillations', 'oscillation_ma', 'oscillation_pct'):
        values = [r[key] for r in rows if r[key] is not None]
        result[key] = (len(values), float(np.mean(values)), float(np.max(values))) if values else (0, None, None)
    return result
