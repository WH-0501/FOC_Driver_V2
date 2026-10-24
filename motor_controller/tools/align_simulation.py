#!/usr/bin/env python3
"""
FOC Ud 电角对齐 — 公式与三次修复阶段的数值仿真/复现。

用法:
    cd motor_controller
    python tools/align_simulation.py

依赖: numpy, matplotlib（可选，无 matplotlib 时仅打印文本）
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

try:
    import matplotlib.pyplot as plt
    import numpy as np

    HAS_MPL = True
except ImportError:
    HAS_MPL = False

# --- 与 firmware common.h / 实测一致 ---
THETA_OPEN = 3.0 * math.pi / 2.0  # _3PI_2
POLE_PAIRS = 5
POLARITY = -1.0
IQ_MIN_A = 0.03
ENGAGE_TOL = 0.35
PI = math.pi
TWO_PI = 2.0 * PI


def angle_normalize(a: float) -> float:
    x = math.fmod(a, TWO_PI)
    return x if x >= 0.0 else x + TWO_PI


def cal_polarity_probe_needs_flip(iq: float, polarity: float = POLARITY) -> bool:
    if abs(iq) < IQ_MIN_A:
        return False
    if polarity < 0.0:
        return iq > 0.0
    return iq < 0.0


def cal_polarity_probe_aligned(iq: float, polarity: float = POLARITY) -> bool:
    if abs(iq) < IQ_MIN_A:
        return True
    if polarity < 0.0:
        return iq < 0.0
    return iq > 0.0


def ud_offset(raw_locked: float, theta_open: float = THETA_OPEN) -> float:
    return angle_normalize(raw_locked - theta_open)


def theta_elec(raw_elec: float, offset: float) -> float:
    return angle_normalize(raw_elec - offset)


def engage_delta(raw_locked: float, raw_engage: float) -> float:
    return abs(angle_normalize(raw_engage - raw_locked))


def engage_pass(raw_locked: float, raw_engage: float, tol: float = ENGAGE_TOL) -> bool:
    d = engage_delta(raw_locked, raw_engage)
    return d >= (PI - tol)


@dataclass
class AlignCase:
    name: str
    mech_rad: float
    raw_locked: float
    raw_engage: float
    iq_probe: float
    offset_manual: float | None = None


def run_phase1_analysis() -> None:
    print("=" * 60)
    print("阶段一：整合回归 — offset 与 raw 关系")
    print("=" * 60)

    # 实测失败案例（约）
    raw_fail = 5.125
    off_fail = angle_normalize(raw_fail - THETA_OPEN)
    print(f"  raw_locked={raw_fail:.3f} → offset={off_fail:.3f}  (整合后失败案例)")

    # 实测成功案例
    raw_ok = 7.51219
    off_ok = ud_offset(raw_ok)
    print(f"  raw_locked={raw_ok:.3f} → offset={off_ok:.3f}  (修复后实测)")
    print(f"  锁定验算 theta_elec={theta_elec(raw_ok, off_ok):.3f}, 期望 θ_open={THETA_OPEN:.3f}")

    manual = 3.19311523
    print(f"  手动 offset={manual:.3f}, 同 raw 下 theta={theta_elec(raw_ok, manual):.3f}")
    print()


def run_phase2_polarity_table() -> None:
    print("=" * 60)
    print("阶段二：极性探针 / direction 翻转 (POLARITY=-1)")
    print("=" * 60)
    print(f"  {'iq_probe':>10}  {'flip?':>6}  {'aligned?':>8}  说明")
    for iq, note in [
        (-0.18, "方向正确典型值"),
        (+0.18, "方向错误 / 误用旧判据会翻"),
        (+0.01, "电流过小，不翻转，视为通过"),
    ]:
        flip = cal_polarity_probe_needs_flip(iq)
        ok = cal_polarity_probe_aligned(iq)
        print(f"  {iq:>10.2f}  {str(flip):>6}  {str(ok):>8}  {note}")
    print()


def run_phase3_engage() -> None:
    print("=" * 60)
    print("阶段三：吸合验证（实测 vs 理想 π 步进）")
    print("=" * 60)

    cases = [
        AlignCase("实测-吸合失败", 1.503, 7.511, 7.548, -0.15),
        AlignCase("理想-π步进", 1.503, 7.512, 7.512 - PI, -0.15),
        AlignCase("修复后-同锁定", 1.503, 7.512, 7.512, -0.15),
    ]
    for c in cases:
        d = engage_delta(c.raw_locked, c.raw_engage)
        passed = engage_pass(c.raw_locked, c.raw_engage)
        off = ud_offset(c.raw_locked)
        print(f"  [{c.name}]")
        print(f"    raw_locked={c.raw_locked:.3f}, raw_engage={c.raw_engage:.3f}")
        print(f"    |Δraw|={d:.3f} rad, engage_pass={passed}, offset={off:.3f}")
    print()


def plot_offset_comparison(out_path: Path) -> None:
    if not HAS_MPL:
        print("(跳过绘图: 请 pip install matplotlib numpy)")
        return

    raw_locked = 7.512
    offsets = np.linspace(0.0, TWO_PI, 500)
    theta = np.array([theta_elec(raw_locked, o) for o in offsets])
    err = np.abs(theta - THETA_OPEN)
    err = np.minimum(err, TWO_PI - err)

    off_auto = ud_offset(raw_locked)
    off_manual = 3.19311523

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))

    ax = axes[0]
    ax.plot(offsets, err, "b-", lw=1.5, label="|theta_elec - theta_open| @ lock")
    ax.axvline(off_auto, color="g", ls="--", label=f"auto offset={off_auto:.3f}")
    ax.axvline(off_manual, color="r", ls="--", label=f"manual offset={off_manual:.3f}")
    ax.set_xlabel("theta_elec_offset [rad]")
    ax.set_ylabel("elec angle error [rad]")
    ax.set_title("Lock-point error vs offset")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    mech = np.linspace(0, TWO_PI, 200)
    raw = mech * POLE_PAIRS
    th_auto = np.array([theta_elec(r, off_auto) for r in raw])
    th_man = np.array([theta_elec(r, off_manual) for r in raw])
    ax.plot(mech, th_auto, "g-", label=f"auto offset ({off_auto:.2f})")
    ax.plot(mech, th_man, "r--", label=f"manual offset ({off_manual:.2f})")
    ax.set_xlabel("mech angle [rad]")
    ax.set_ylabel("theta_elec [rad]")
    ax.set_title("Full-turn theta_elec (d/q coupling hint)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    fig.suptitle("FOC Ud align: offset 2.80 vs 3.19", fontsize=12)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150)
    print(f"已保存: {out_path}")
    plt.close(fig)


def plot_engaged_step(out_path: Path) -> None:
    if not HAS_MPL:
        return

    raw0 = 7.512
    # 理想：π 电角步进
    raw1_ideal = raw0 - PI
    # 实测：几乎不动
    raw1_real = 7.548

    labels = ["theta_open lock", "theta+pi ideal", "theta+pi measured"]
    raws = [raw0, raw1_ideal, raw1_real]
    norms = [angle_normalize(r) for r in raws]
    colors = ["#2196F3", "#4CAF50", "#F44336"]

    fig, ax = plt.subplots(figsize=(8, 4))
    x = np.arange(len(labels))
    ax.bar(x - 0.2, raws, width=0.35, label="raw (unnorm)", color=colors, alpha=0.7)
    ax.bar(x + 0.2, norms, width=0.35, label="normalize(raw)", color=colors, alpha=0.4)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("elec angle [rad]")
    ax.set_title("Engage check: ideal pi step vs measured (phase 3)")
    for i, (r, n) in enumerate(zip(raws, norms)):
        ax.text(i - 0.2, r + 0.2, f"{r:.2f}", ha="center", fontsize=9)
        ax.text(i + 0.2, n + 0.2, f"{n:.2f}", ha="center", fontsize=9)
    d_real = engage_delta(raw0, raw1_real)
    d_ideal = engage_delta(raw0, raw1_ideal)
    ax.text(0.5, 0.95, f"d_meas={d_real:.3f} (fail)  d_ideal={d_ideal:.3f} (pass)",
            transform=ax.transAxes, ha="center", fontsize=10,
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5))
    ax.legend()
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"已保存: {out_path}")
    plt.close(fig)


def run_phase4_verification() -> None:
    print("=" * 60)
    print("阶段四：验收 — 自动 2.80 vs 手动 3.19")
    print("=" * 60)
    raw = 7.51219
    off_auto = ud_offset(raw)
    off_manual = 3.19311523
    th_auto = theta_elec(raw, off_auto)
    th_manual = theta_elec(raw, off_manual)
    err_auto = min(abs(th_auto - THETA_OPEN), TWO_PI - abs(th_auto - THETA_OPEN))
    err_manual = min(abs(th_manual - THETA_OPEN), TWO_PI - abs(th_manual - THETA_OPEN))
    print(f"  raw_locked={raw:.5f}")
    print(f"  auto   offset={off_auto:.4f}  lock err={err_auto:.4f} rad  (空载 ~1.8W)")
    print(f"  manual offset={off_manual:.4f}  lock err={err_manual:.4f} rad  (空载 ~1.9W)")
    print()


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    docs = root / "docs"

    run_phase1_analysis()
    run_phase2_polarity_table()
    run_phase3_engage()
    run_phase4_verification()

    plot_offset_comparison(docs / "align_simulation_offset.png")
    plot_engaged_step(docs / "align_simulation_engage.png")

    print("完成。详见 docs/电角对齐问题排查与修复记录.md")
    return 0


if __name__ == "__main__":
    sys.exit(main())
