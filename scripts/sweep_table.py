#!/usr/bin/env python3
"""开环扫点表（Iteration 006.5 验收 A2）。

用途：板上做占空比扫点后（`]`/`[` 调占空比、`o` 切到开环），把串口日志整理成
「占空比 → 稳态轮速」表，用于确定死区与增益初值，并直接作为 A2 的证据。

输入：BoardApp 状态行日志（每 5 行一组，本脚本只依赖 enc 行与 tgt 行）：
    t=<ms> enc: L c=.. v=<mm/s> q=.. pm=.. R c=.. v=<mm/s> q=.. pm=..
    tgt: L <mm/s> R <mm/s> mode=<closed|open> wheel=<both|left|right> od=<permille>

用法：
    python3 scripts/sweep_table.py <uart_log.txt> [--settle-frac 0.4] [--csv out.csv] [--min-samples 3]

输出：markdown 表格（占空比 | 左稳态 | 右稳态 | 样本数 | 极差）+ 死区/增益的一次线性拟合估算。
退出码：0 正常；2 可用样本不足。
"""

from __future__ import annotations

import argparse
import re
import statistics
import sys

RE_ENC = re.compile(r"t=(\d+) enc: L c=-?\d+ v=(-?\d+) q=0x[0-9A-Fa-f]+ pm=-?\d+ "
                    r"R c=-?\d+ v=(-?\d+) q=0x[0-9A-Fa-f]+ pm=-?\d+")
RE_TGT = re.compile(r"tgt: L -?\d+ R -?\d+ mode=(closed|open) wheel=(both|left|right) od=(\d+)")


def parse(path: str):
    """返回 [(od, mode, v_left, v_right)]，enc 行与紧随其后的 tgt 行配对。"""
    rows = []
    pending = None
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            if match := RE_ENC.search(line):
                pending = (int(match.group(2)), int(match.group(3)))
                continue
            if pending is None:
                continue
            if match := RE_TGT.search(line):
                mode = match.group(1)
                od = int(match.group(3))
                rows.append((od, mode, pending[0], pending[1]))
                pending = None
    return rows


def settled_values(values: list[int], settle_frac: float) -> list[int]:
    """丢掉前 settle_frac 的过渡样本（起动斜坡），返回用于统计的稳态样本。"""
    if not values:
        return []
    keep_from = int(len(values) * settle_frac)
    return values[keep_from:] or values


def median_speed(values: list[int], settle_frac: float) -> float:
    tail = settled_values(values, settle_frac)
    return float(statistics.median(tail)) if tail else 0.0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log")
    parser.add_argument("--settle-frac", type=float, default=0.4,
                        help="每组丢弃前多少比例的样本作为过渡（默认 0.4）")
    parser.add_argument("--min-samples", type=int, default=3)
    parser.add_argument("--csv")
    args = parser.parse_args()

    rows = parse(args.log)
    open_rows = [(od, vl, vr) for od, mode, vl, vr in rows if mode == "open"]
    if len(open_rows) < args.min_samples:
        print(f"FAIL: 开环样本不足（{len(open_rows)} < {args.min_samples}）。"
              f"请先用 'o' 切到开环、用 ']' 逐档加占空比。")
        return 2

    grouped: dict[int, list[tuple[int, int]]] = {}
    for od, vl, vr in open_rows:
        grouped.setdefault(od, []).append((vl, vr))

    print("## 开环扫点表（占空比 → 稳态轮速）\n")
    print("| 占空比(‰) | 左轮稳态(mm/s) | 右轮稳态(mm/s) | 样本数 | 左极差 | 右极差 |")
    print("|---:|---:|---:|---:|---:|---:|")
    table = []
    for od in sorted(grouped):
        samples = grouped[od]
        if len(samples) < args.min_samples:
            continue
        lefts = [s[0] for s in samples]
        rights = [s[1] for s in samples]
        left_med = median_speed(lefts, args.settle_frac)
        right_med = median_speed(rights, args.settle_frac)
        # 极差只统计稳态段：含起动斜坡会把它放大成"满量程"，失去诊断意义
        left_tail = settled_values(lefts, args.settle_frac)
        right_tail = settled_values(rights, args.settle_frac)
        left_range = max(left_tail) - min(left_tail)
        right_range = max(right_tail) - min(right_tail)
        table.append((od, left_med, right_med, len(samples), left_range, right_range))
        print(f"| {od} | {left_med:.0f} | {right_med:.0f} | {len(samples)} | "
              f"{left_range} | {right_range} |")

    if not table:
        print("FAIL: 每档样本数都不足，请延长每档保持时间。")
        return 2

    # 死区与增益的粗估：取稳态速度首次显著非零的档位为死区上界，再最小二乘拟合。
    # 注意：只对"有信号的轮"拟合。若一侧未接线（速度恒 0），取左右平均会把增益算小一半
    # —— 这是真实踩过的坑（右电机未接时增益被误算成 0.33，实际 0.68）。
    left_amp = max((abs(row[1]) for row in table), default=0)
    right_amp = max((abs(row[2]) for row in table), default=0)
    use_left = left_amp >= right_amp
    side_name = "左轮" if use_left else "右轮"
    silent = None
    if min(left_amp, right_amp) == 0 and max(left_amp, right_amp) > 0:
        silent = "右轮" if use_left else "左轮"
    moving = [row for row in table if abs(row[1] if use_left else row[2]) > 0]
    if len(moving) >= 3:
        deadband = min(row[0] for row in moving)
        fit = [row for row in moving if row[0] >= deadband]
        xs = [row[0] for row in fit]
        ys = [float(row[1] if use_left else row[2]) for row in fit]
        n = len(xs)
        mean_x = sum(xs) / n
        mean_y = sum(ys) / n
        denom = sum((x - mean_x) ** 2 for x in xs)
        if denom > 0:
            slope = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / denom
            intercept = mean_y - slope * mean_x
            zero_duty = -intercept / slope if slope != 0 else 0.0
            if silent is not None:
                print(f"\n- 注意：{silent}全程速度为 0（未接线或未驱动），以下拟合**只使用{side_name}**")
            print(f"- 起转死区（首个有速度的档位）：**{deadband} ‰**")
            print(f"- 线性拟合 v ≈ {slope:.4f}·duty + {intercept:.1f}（mm/s）"
                  f" ⇒ 外推零点 **{zero_duty:.0f} ‰**")
            print(f"- 增益（{side_name}）：每 100‰ 占空比约 **{slope * 100:.1f} mm/s**")
            if slope > 0:
                kff = 1000.0 / slope  # permille per (mm/s) → 乘以 1000 得 permille per m/s
                print(f"- 前馈换算：kff_permille_per_mps ≈ **{kff:.0f}**"
                      f"（当前配置 1600，若明显偏小则前馈不足、PI 负担过重）")
            print("\n> 提示：拟合仅作粗调起点；006 的 `min_effective_duty_permille` 应取"
                  "死区上界附近并留余量，`kff_permille_per_mps` 可由 1/增益 换算。")
    else:
        print("\n- 有速度的档位不足 3 个，无法拟合增益；请补做更高档位。")

    if args.csv:
        with open(args.csv, "w", encoding="utf-8") as handle:
            handle.write("duty_permille,left_mmps,right_mmps,samples\n")
            for od, left_med, right_med, count, _, _ in table:
                handle.write(f"{od},{left_med:.0f},{right_med:.0f},{count}\n")
        print(f"\nCSV 已写入 {args.csv}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
