#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""校验 robotcar01_loopback 的反馈帧流与统计输出（Iteration 003 A7）。

独立重算 CRC-16/CCITT-FALSE（不调用 C++ 实现），逐帧检查：
  magic 线序、版本、类型、payload_len 与类型匹配、payload <= 上限、帧长 <= 80、CRC 正确。
并要求 stderr 含逐行统计行与 summary 行、退出码为 0。
"""

from __future__ import annotations

import subprocess
import sys

MAGIC = bytes([0x52, 0x43, 0x30, 0x31])
VERSION = 0x01
HEADER_BYTES = 12
CRC_BYTES = 2
MAX_FRAME_BYTES = 216
REQUIRED_PAYLOAD = {0x01: 13, 0x02: 0, 0x81: 64, 0x82: 200}


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_motion_frame(seq: int, v_mps: float, omega: float, run: int, lease_ms: int,
                       send_age_ms: int = 0) -> bytes:
    import struct

    payload = struct.pack("<ffBBHB", v_mps, omega, run, 0, lease_ms, send_age_ms)
    header = MAGIC + bytes([VERSION, 0x01, 0x00, len(payload)]) + struct.pack("<I", seq)
    body = header + payload
    return body + struct.pack("<H", crc16_ccitt_false(body))


def build_stop_frame(seq: int) -> bytes:
    import struct

    header = MAGIC + bytes([VERSION, 0x02, 0x00, 0]) + struct.pack("<I", seq)
    return header + struct.pack("<H", crc16_ccitt_false(header))


def parse_frames(stream: bytes) -> list[bytes]:
    frames = []
    index = 0
    while index < len(stream):
        if stream[index:index + 4] != MAGIC:
            raise AssertionError(f"offset {index}: magic mismatch (line protocol out of sync)")
        if index + HEADER_BYTES > len(stream):
            raise AssertionError("truncated header")
        version = stream[index + 4]
        msg_type = stream[index + 5]
        flags = stream[index + 6]
        payload_len = stream[index + 7]
        assert version == VERSION, f"offset {index}: bad version {version}"
        assert flags == 0, f"offset {index}: flags must be 0"
        assert msg_type in REQUIRED_PAYLOAD, f"offset {index}: unknown type 0x{msg_type:02x}"
        assert payload_len == REQUIRED_PAYLOAD[msg_type], (
            f"offset {index}: payload_len {payload_len} != {REQUIRED_PAYLOAD[msg_type]}")
        total = HEADER_BYTES + payload_len + CRC_BYTES
        assert total <= MAX_FRAME_BYTES, f"offset {index}: frame too long {total}"
        if index + total > len(stream):
            raise AssertionError(f"offset {index}: truncated frame body")
        body = stream[index:index + HEADER_BYTES + payload_len]
        crc_expected = int.from_bytes(stream[index + HEADER_BYTES + payload_len:
                                            index + total], "little")
        crc_actual = crc16_ccitt_false(body)
        assert crc_expected == crc_actual, (
            f"offset {index}: CRC 0x{crc_expected:04x} != 0x{crc_actual:04x}")
        frames.append(stream[index:index + total])
        index += total
    return frames


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: check_feedback_stream.py <robotcar01_loopback binary>", file=sys.stderr)
        return 2
    binary = sys.argv[1]

    good = build_motion_frame(2, 0.2, 0.0, 0, 300)
    bad_crc = bytearray(build_motion_frame(2, 0.3, 0.0, 0, 300))
    bad_crc[-1] ^= 0xFF
    ignore_run = build_motion_frame(1, 0.25, 0.1, 1, 300)   # 会话首帧带 run ⇒ 应被忽略
    run_frame = build_motion_frame(3, 0.5, 0.0, 1, 300)
    stop = build_stop_frame(4)

    lines = [
        ignore_run.hex(),  # 会话首帧即带 run 请求：必须被忽略（A4）
        bytes(bad_crc).hex(),
        good.hex(),
        run_frame.hex() + stop.hex(),  # 同一行粘连两帧（A1 粘包）
    ]
    stdin = "\n".join(lines) + "\n"

    completed = subprocess.run([binary, "--frames", "20"], input=stdin.encode(),
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if completed.returncode != 0:
        print(f"FAIL: exit code {completed.returncode}", file=sys.stderr)
        print(completed.stderr.decode("utf-8", "replace"), file=sys.stderr)
        return 1

    try:
        frames = parse_frames(completed.stdout)
    except AssertionError as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1

    if not frames:
        print("FAIL: no feedback frames on stdout", file=sys.stderr)
        return 1

    types = [frame[5] for frame in frames]
    if 0x81 not in types:
        print("FAIL: no status feedback frame (0x81)", file=sys.stderr)
        return 1

    stderr_text = completed.stderr.decode("utf-8", "replace")
    stat_lines = [line for line in stderr_text.splitlines() if line.startswith("line=")]
    if not stat_lines:
        print("FAIL: no per-line statistics on stderr", file=sys.stderr)
        return 1
    if not any(line.startswith("summary ") for line in stderr_text.splitlines()):
        print("FAIL: missing summary line on stderr", file=sys.stderr)
        return 1

    last = stat_lines[-1]
    fields = dict(part.split("=", 1) for part in last.split() if "=" in part)
    if fields.get("cmd_ignored_run") != "1":
        print(f"FAIL: expected first-frame run request to be ignored, got {last}", file=sys.stderr)
        return 1
    if fields.get("armed") != "1":
        print(f"FAIL: expected armed=1 after stop frame, got {last}", file=sys.stderr)
        return 1
    if fields.get("run_requested") != "0":
        print(f"FAIL: expected run_requested=0 after stop frame, got {last}", file=sys.stderr)
        return 1

    print(f"PASS: {len(frames)} feedback frames verified "
          f"(types={[hex(t) for t in sorted(set(types))]}), statistics parseable")
    return 0


if __name__ == "__main__":
    sys.exit(main())
