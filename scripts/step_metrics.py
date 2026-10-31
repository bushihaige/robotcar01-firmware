#!/usr/bin/env python3
"""闭环阶跃指标（Iteration 006.5 验收 A3）。

用途：板上做"固定目标速度 → 观察跟随"后，把日志整理成每个目标段的指标表：
上升时间、超调、稳态误差、稳态波动，直接作为 A3 的证据，并给 PI 粗调提供依据。

输入：BoardApp 状态行日志（只用 enc 行与 tgt 行）：
    t=<ms> enc: L c=.. v=<mm/s> q=.. pm=.. R c=.. v=<mm/s> q=.. pm=..
    tgt: L <mm/s> R <mm/s> mode=<closed|open> wheel=<both|left|right> od=<permille>

用法：
    python3 scripts/step_metrics.py <uart_log.txt> [--min-step-mmps 20] [--settle-frac 0.3]

判定口径（写在输出里，便于写 result.md 时引用）：
    - 段起点 = 目标发生显著变化的那一帧；段终点 = 下一次变化或日志结束
    - 上升时间 = 首次达到 90% |target| 所用时间
    - 超调 = (峰值 - |target|) / |target|
    - 稳态 = 段尾 settle_frac 比例样本的均值；稳态误差 = 稳态 - |target|
    - 稳态波动 = 段尾样本的标准差
退出码：0 正常；2 没有可分析的闭阶段。
"""

from __future__ import annotations

import argparse
import re
import statistics
import sys

RE_ENC = re.compile(r"t=(\d+) enc: L c=-?\d+ v=(-?\d+) q=0x[0-9A-Fa-f]+ pm=-?\d+ "
                    r"R c=-?\d+ v=(-?\d+) q=0x[0-9A-Fa-f]+ pm=-?\d+")
RE_TGT = re.compile(r"tgt: L (-?\d+) R (-?\d+) mode=(closed|open) wheel=(both|left|right) od=(\d+)")


def parse(path: str):
    """返回 [(t_ms, target_left, target_right, mode, v_left, v_right)] 按时间顺序。"""
    rows = []
    pending = None
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            if match := RE_ENC.search(line):
                pending = (int(match.group(1)), int(match.group(2)), int(match.group(3)))
                continue
            if pending is None:
                continue
            if match := RE_TGT.search(line):
                rows.append((pending[0], int(match.group(1)), int(match.group(2)),
                             match.group(3), pending[1], pending[2]))
                pending = None
    return rows


def analyze_segment(segment, target: int, settle_frac: float):
    """segment: [(t_ms, v_mmps)]；target: 本段目标（mm/s，可正可负）。"""
    magnitude = abs(target)
    if magnitude == 0 or len(segment) < 3:
        return None
    t0 = segment[0][0]
    rise = None
    for t_ms, value in segment:
        if (value > 0) == (target > 0) and abs(value) >= 0.9 * magnitude:
            rise = t_ms - t0
            break
    peak = max(segment, key=lambda item: (item[1] * (1 if target > 0 else -1)))[1]
    overshoot = (abs(peak) - magnitude) / magnitude
    keep_from = int(len(segment) * (1.0 - settle_frac))
    tail = [value for _, value in segment[keep_from:]] or [segment[-1][1]]
    steady = statistics.mean(tail)
    steady_err = steady * (1 if target > 0 else -1) - magnitude
    ripple = statistics.pstdev(tail) if len(tail) > 1 else 0.0
    return {
        "target": target,
        "samples": len(segment),
        "duration_ms": segment[-1][0] - t0,
        "rise_ms": rise,
        "peak": peak,
        "overshoot_pct": overshoot * 100.0,
        "steady": steady,
        "steady_err": steady_err,
        "ripple": ripple,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log")
    parser.add_argument("--min-step-mmps", type=int, default=20,
                        help="目标变化超过该值才算一个新段（mm/s）")
    parser.add_argument("--settle-frac", type=float, default=0.3)
    args = parser.parse_args()

    rows = parse(args.log)
    if len(rows) < 6:
        print(f"FAIL: 有效状态组太少（{len(rows)}）。")
        return 2

    # 切段：mode=closed 且目标发生显著变化
    segments = []
    current = None
    for t_ms, tgt_l, tgt_r, mode, v_l, v_r in rows:
        if mode != "closed":
            current = None
            continue
        target = tgt_l  # 以左轮目标为段定义（双轮同目标时等价）
        if current is None or abs(target - current["target"]) >= args.min_step_mmps:
            if current is not None:
                segments.append(current)
            current = {"target": target, "samples": [(t_ms, v_l)], "v_right": [(t_ms, v_r)]}
        else:
            current["samples"].append((t_ms, v_l))
            current["v_right"].append((t_ms, v_r))
    if current is not None:
        segments.append(current)

    results = []
    for segment in segments:
        if abs(segment["target"]) == 0:
            continue
        left = analyze_segment(segment["samples"], segment["target"], args.settle_frac)
        right = analyze_segment(segment["v_right"], segment["target"], args.settle_frac)
        if left and right:
            results.append((left, right))

    if not results:
        print("FAIL: 没有可分析的闭阶段（需要 mode=closed 且目标非零、并保持足够长的样本）。")
        return 2

    print("## 闭环阶跃指标（左/右轮分别统计）\n")
    print("| 目标(mm/s) | 轮 | 样本 | 时长(ms) | 上升(ms) | 峰值 | 超调 | 稳态 | 稳态误差 | 波动 |")
    print("|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|")
    for left, right in results:
        for name, item in (("左", left), ("右", right)):
            rise = "未达 90%" if item["rise_ms"] is None else f"{item['rise_ms']}"
            print(f"| {item['target']} | {name} | {item['samples']} | {item['duration_ms']} | "
                  f"{rise} | {item['peak']} | {item['overshoot_pct']:.1f}% | "
                  f"{item['steady']:.0f} | {item['steady_err']:+.0f} | {item['ripple']:.1f} |")

    print("\n判定口径：上升时间=首次到 90% 目标；超调=(峰值-|目标|)/|目标|；"
          "稳态=段尾 30% 样本均值；稳态误差=稳态-|目标|；波动=段尾标准差（mm/s）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
