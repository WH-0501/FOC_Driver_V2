"""Array analysis on raw samples, independent of plot decimation."""
import numpy as np


def peak_envelope(xs, ys, max_points):
    xs, ys = np.asarray(xs), np.asarray(ys)
    if len(xs) <= max_points:
        return xs, ys
    buckets = max(1, (max_points - 2) // 2)
    width = int(np.ceil(len(xs) / buckets))
    padded = np.full(buckets * width, np.nan)
    padded[:len(ys)] = ys
    blocks = padded.reshape(buckets, width)
    valid = np.any(np.isfinite(blocks), axis=1)
    low = np.argmin(np.where(np.isfinite(blocks), blocks, np.inf), axis=1)
    high = np.argmax(np.where(np.isfinite(blocks), blocks, -np.inf), axis=1)
    base = np.arange(buckets)[valid] * width
    indices = np.unique(np.concatenate(([0, len(xs)-1], base + low[valid], base + high[valid])))
    return xs[indices], ys[indices]


def spectrum(values, points=4096, sample_period=0.001):
    """Hann-window one-sided amplitude using explicit device sample interval."""
    y = np.asarray(values)[-points:]
    if len(y) < points or not np.all(np.isfinite(y)):
        return np.empty(0), np.empty(0)
    window = np.hanning(points)
    amp = np.abs(np.fft.rfft((y - np.mean(y)) * window)) * 2 / window.sum()
    amp[0] *= 0.5
    if points % 2 == 0:
        amp[-1] *= 0.5
    return np.fft.rfftfreq(points, sample_period), amp
