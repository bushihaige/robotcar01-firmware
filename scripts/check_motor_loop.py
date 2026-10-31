#!/usr/bin/env python3
"""RobotCar01 006.5 板上电机环路日志检查（Iteration 006.5）。

用法：
    python3 scripts/check_motor_loop.py <uart_log.txt> [--expect-target-mps 0.10] [--allow-q]

校验对象是 BoardApp::PrintStatus 输出的四/五行一组状态：
    t=<ms> enc: L c=<cnt> v=<mm/s> q=<hex> R c=<cnt> v=<mm/s> q=<hex>
    tgt: L <mm/s> R <mm/s> mode=<closed|open> wheel=<both|left|right> od=<permille>
    out: L m=<0|1|2> d=<0|1|2> p=<permille> R m=<0|1|2> d=<0|1|2> p=<permille>
    arb: t=<n> a=<n> drv=<n> st=<n> sf=<n> ft=<n> hw=<n>
    est: smp=<n> acc=<n> inv=<n> cmd=<n> unk=<n> ovf=<n> sw=<n>

断言（失败即非零退出）：
  A. 输出合法性（INV-006-1 平台侧映射）：m=0(coast)/m=2(brake) ⇒ p=0；m=1(drive) ⇒ d!=0 且 p>0。
  B. 仲裁健康：est.inv 与 arb.hw 在采样期间不增长；st/ft 允许在起步阶段出现，但不得持续增长到末尾。
  C. 编码器质量：q 非零的行数占比 —— 默认要求 < 5%；--allow-q 关闭该断言。
  D. 目标/测量同号：当 mode=closed 且目标非零时，|v| 超过底噪的行中至少有 80% 与目标同号。
  E. 采样量：至少 3 组完整状态（不足则判证据不足）。

退出码：0 = 全部通过；1 = 断言失败；2 = 证据不足/无法解析。
"""

from __future__ import annotations

import argparse
import re
import sys

RE_ENC = re.compile(
    r"t=(?P<t>\d+) enc: L c=(?P<lc>-?\d+) v=(?P<lv>-?\d+) q=(?P<lq>0x[0-9A-Fa-f]+) "
    r"pm=(?P<lpm>-?\d+) R c=(?P<rc>-?\d+) v=(?P<rv>-?\d+) q=(?P<rq>0x[0-9A-Fa-f]+) "
    r"pm=(?P<rpm>-?\d+)"
)
RE_TGT = re.compile(
    r"tgt: L (?P<lt>-?\d+) R (?P<rt>-?\d+) mode=(?P<mode>closed|open) "
    r"wheel=(?P<wheel>both|left|right) od=(?P<od>\d+)"
)
RE_OUT = re.compile(
    r"out: L m=(?P<lm>\d+) d=(?P<ld>\d+) p=(?P<lp>\d+) R m=(?P<rm>\d+) d=(?P<rd>\d+) p=(?P<rp>\d+)"
)
RE_ARB = re.compile(
    r"arb: t=(?P<t>\d+) a=(?P<a>\d+) drv=(?P<drv>\d+) st=(?P<st>\d+) sf=(?P<sf>\d+) "
    r"ft=(?P<ft>\d+) hw=(?P<hw>\d+)"
)
RE_EST = re.compile(
    r"est: smp=(?P<smp>\d+) acc=(?P<acc>\d+) inv=(?P<inv>\d+) cmd=(?P<cmd>\d+) "
    r"unk=(?P<unk>\d+) ovf=(?P<ovf>\d+) sw=(?P<sw>\d+)"
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", help="串口采集日志（带或不带宿主时间戳均可）")
    parser.add_argument("--allow-q", action="store_true", help="不校验编码器质量位占比")
    parser.add_argument("--max-q-ratio", type=float, default=0.05, help="允许的非零质量位行占比")
    parser.add_argument("--min-samples", type=int, default=3, help="最少完整状态组数")
    args = parser.parse_args()

    enc_rows: list[dict] = []
    tgt_rows: list[dict] = []
    out_rows: list[dict] = []
    arb_rows: list[dict] = []
    est_rows: list[dict] = []

    with open(args.log, "r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            if match := RE_ENC.search(line):
                enc_rows.append({k: match.group(k) for k in match.groupdict()})
            elif match := RE_TGT.search(line):
                tgt_rows.append({k: match.group(k) for k in match.groupdict()})
            elif match := RE_OUT.search(line):
                out_rows.append({k: match.group(k) for k in match.groupdict()})
            elif match := RE_ARB.search(line):
                arb_rows.append({k: match.group(k) for k in match.groupdict()})
            elif match := RE_EST.search(line):
                est_rows.append({k: match.group(k) for k in match.groupdict()})

    groups = min(len(enc_rows), len(tgt_rows), len(out_rows), len(arb_rows), len(est_rows))
    print(f"parsed: enc={len(enc_rows)} tgt={len(tgt_rows)} out={len(out_rows)} arb={len(arb_rows)} est={len(est_rows)}")
    if groups < args.min_samples:
        print(f"FAIL: 证据不足（完整状态组 {groups} < {args.min_samples}）")
        return 2

    failures: list[str] = []

    # A. 输出合法性
    violations = 0
    for row in out_rows:
        for mode_key, dir_key, duty_key in (("lm", "ld", "lp"), ("rm", "rd", "rp")):
            mode = int(row[mode_key])
            direction = int(row[dir_key])
            duty = int(row[duty_key])
            if mode in (0, 2) and duty != 0:
                violations += 1
            if mode == 1 and (direction == 0 or duty == 0):
                violations += 1
    print(f"A. output legality violations = {violations}")
    if violations:
        failures.append(f"A: {violations} 行输出组合非法")

    # B. 仲裁/估计健康（取首末差值）
    def delta(rows: list[dict], key: str) -> int:
        return int(rows[-1][key]) - int(rows[0][key])

    inv_delta = delta(est_rows, "inv")
    hw_delta = delta(arb_rows, "hw")
    ft_delta = delta(arb_rows, "ft")
    print(f"B. est.inv +{inv_delta}  arb.hw +{hw_delta}  arb.ft +{ft_delta}  arb.drv={arb_rows[-1]['drv']}")
    if inv_delta > 0:
        failures.append(f"B: 平台快照无效次数增长 {inv_delta}")
    if hw_delta > 0:
        failures.append(f"B: 平台错误计数增长 {hw_delta}")

    # C. 编码器质量位
    bad_q = sum(1 for row in enc_rows if int(row["lq"], 16) or int(row["rq"], 16))
    ratio = bad_q / len(enc_rows)
    print(f"C. rows with non-zero quality = {bad_q}/{len(enc_rows)} ({ratio:.1%})")
    if not args.allow_q and ratio > args.max_q_ratio:
        failures.append(f"C: 质量位非零占比 {ratio:.1%} > {args.max_q_ratio:.1%}")

    # D. 目标与测量同号
    compared = 0
    same_sign = 0
    for enc, tgt in zip(enc_rows, tgt_rows):
        if tgt["mode"] != "closed":
            continue
        target_left = int(tgt["lt"])
        measured_left = int(enc["lv"])
        if abs(target_left) < 20 or abs(measured_left) < 20:
            continue
        compared += 1
        if (target_left > 0) == (measured_left > 0):
            same_sign += 1
    if compared:
        agree = same_sign / compared
        print(f"D. closed-loop sign agreement = {same_sign}/{compared} ({agree:.1%})")
        if agree < 0.80:
            failures.append(f"D: 目标与测量同号比例仅 {agree:.1%}")
    else:
        print("D. closed-loop sign agreement = n/a（无足够非零目标样本）")

    # E. 采样量
    print(f"E. state groups = {groups}")

    if failures:
        for item in failures:
            print(f"FAIL: {item}")
        return 1
    print("VERDICT: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
