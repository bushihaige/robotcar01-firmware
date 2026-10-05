#!/usr/bin/env python3
"""心跳日志离线分析（Iteration 002.5 验收 4/5，评审 H-5 量化判定）。

输入：scripts/capture_uart.sh 产出的日志，每行形如
      "<elapsed_ms>\\t[hb] n=<n> tick=<tick> assert=<k>"
      也兼容没有宿主时间戳、只有 "[hb] n=.. tick=.." 的日志（此时只做设备侧 tick 分析）。

判定（阈值来自详细设计验收 4，可依实测重定但要显式记录）：
  1) 宿主侧：相邻心跳的 elapsed_ms 间隔 mean ∈ [990, 1010] ms，且 max-min < 30 ms
  2) 设备侧：相邻心跳的 tick 差值应恰为 1000 ms（SysTick 1 ms 基准确）

用法： python3 scripts/check_heartbeat.py <log> [--mean-lo 990] [--mean-hi 1010] [--jitter 30]
退出码：0 = 通过；1 = 不通过；2 = 输入不足/无法解析
"""

from __future__ import annotations

import argparse
import re
import statistics
import sys


HB_RE = re.compile(r"\[hb\]\s+n=(\d+)\s+tick=(\d+)")
LINE_RE = re.compile(r"^(\d+)\t(.*)$")


def parse(path: str):
    host_ms: list[tuple[int, int]] = []  # (elapsed_ms, heartbeat_index)
    device: list[tuple[int, int]] = []   # (tick, heartbeat_index)
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            line = raw.rstrip("\n")
            elapsed = None
            m = LINE_RE.match(line)
            if m:
                elapsed = int(m.group(1))
                line = m.group(2)
            hb = HB_RE.search(line)
            if not hb:
                continue
            idx = int(hb.group(1))
            tick = int(hb.group(2))
            device.append((tick, idx))
            if elapsed is not None:
                host_ms.append((elapsed, idx))
    return host_ms, device


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--mean-lo", type=float, default=990.0)
    ap.add_argument("--mean-hi", type=float, default=1010.0)
    ap.add_argument("--jitter", type=float, default=30.0, help="max-min 上限 (ms)")
    args = ap.parse_args()

    host_ms, device = parse(args.log)
    if len(host_ms) < 3 and len(device) < 3:
        print(f"FAIL: 心跳样本不足（宿主 {len(host_ms)} / 设备 {len(device)}，需 >= 3）")
        return 2

    ok = True

    if len(host_ms) >= 3:
        intervals = [b[0] - a[0] for a, b in zip(host_ms, host_ms[1:])]
        mean = statistics.fmean(intervals)
        jitter = max(intervals) - min(intervals)
        print("宿主侧心跳间隔 (ms):")
        print(f"  样本 {len(intervals)}  值 {intervals}")
        print(f"  min={min(intervals)} max={max(intervals)} mean={mean:.2f} jitter={jitter}")
        host_ok = (args.mean_lo <= mean <= args.mean_hi) and (jitter < args.jitter)
        print(f"  判定 mean∈[{args.mean_lo},{args.mean_hi}] 且 jitter<{args.jitter}: "
              f"{'PASS' if host_ok else 'FAIL'}")
        ok = ok and host_ok
    else:
        print("宿主侧：日志无时间戳列，跳过间隔量化（无法满足验收 4，仅设备侧可判）")
        ok = False

    if len(device) >= 3:
        deltas = [b[0] - a[0] for a, b in zip(device, device[1:])]
        bad = [d for d in deltas if d != 1000]
        print("设备侧 tick 差值 (ms):")
        print(f"  样本 {len(deltas)}  值 {deltas}")
        print(f"  非 1000ms 的样本数: {len(bad)}")
        dev_ok = len(bad) == 0
        print(f"  判定 全部 == 1000: {'PASS' if dev_ok else 'FAIL'}")
        ok = ok and dev_ok
    else:
        print("设备侧：心跳样本不足")

    print("VERDICT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
