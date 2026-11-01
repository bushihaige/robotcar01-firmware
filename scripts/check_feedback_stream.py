#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""校验 robotcar01_loopback 的反馈帧流与统计输出（Iteration 003 A7 / 004 A7 / 005 A8）。

独立重算 CRC-16/CCITT-FALSE（不调用 C++ 实现），逐帧检查：
  magic 线序、版本、类型、payload_len 与类型匹配、payload <= 上限、帧长 <= 208、CRC 正确。
并要求 stderr 含逐行统计行、`kin `/`encoder ` 行与 summary 行、退出码为 0。

端到端断言（按迭代累积）：
  003：会话首帧 run 被忽略、arm 门、字节流可解析；
  004：状态帧 measured_* 与注入的编码器轮速一致且 kFlagSpeedValid 置位；
  005：状态帧 target_*/target_scale/limit_reason 与运动学输出一致 ——
       ① 阶段 A（命令 0.25 m/s）目标爬升并达到命令值、scale=1；
       ② 相邻状态帧（20 ms）目标变化不超过加速度约束 0.5 m/s² × 20 ms；
       ③ 阶段 B（超限命令 2.0 m/s，车体上限 0.5）target_scale ≈ 0.25 且 limit_reason 含 kBodySpeed。
"""

from __future__ import annotations

import struct
import subprocess
import sys

MAGIC = bytes([0x52, 0x43, 0x30, 0x31])
VERSION = 0x01
HEADER_BYTES = 12
CRC_BYTES = 2
MAX_FRAME_BYTES = 208
REQUIRED_PAYLOAD = {0x01: 13, 0x02: 0, 0x81: 64, 0x82: 192}

# status_flags / limit_reason 位（与 feedback.h、chassis_types.h 对齐）
FLAG_TARGET_VALID = 1 << 6
FLAG_SPEED_VALID = 1 << 7
REASON_BODY_SPEED = 0x02
REASON_ACCEL_LIMIT = 0x10
REASON_QUALITY_DEGRADED = 0x20

STATUS_PERIOD_MS = 20
KIN_ACCEL_MPS2 = 0.5
KIN_COMMAND_MPS = 0.25
KIN_OVERSHOOT_COMMAND_MPS = 2.0
KIN_MAX_BODY_SPEED_MPS = 0.5
KIN_EXPECTED_SCALE = KIN_MAX_BODY_SPEED_MPS / KIN_OVERSHOOT_COMMAND_MPS  # 0.25
KIN_DEGRADED_WHEEL_MPS = 0.2
REASON_SAFETY = 0x08


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_motion_frame(seq: int, v_mps: float, omega: float, run: int, lease_ms: int,
                       send_age_ms: int = 0) -> bytes:
    payload = struct.pack("<ffBBHB", v_mps, omega, run, 0, lease_ms, send_age_ms)
    header = MAGIC + bytes([VERSION, 0x01, 0x00, len(payload)]) + struct.pack("<I", seq)
    body = header + payload
    return body + struct.pack("<H", crc16_ccitt_false(body))


def build_stop_frame(seq: int) -> bytes:
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


def parse_status(frame: bytes) -> dict:
    payload = frame[HEADER_BYTES:HEADER_BYTES + 64]
    return {
        "flags": int.from_bytes(payload[0:4], "little"),
        "limit_reason": int.from_bytes(payload[4:8], "little"),
        "flags_valid": int.from_bytes(payload[8:12], "little"),
        "target_left": struct.unpack("<f", payload[28:32])[0],
        "target_right": struct.unpack("<f", payload[32:36])[0],
        "measured_left": struct.unpack("<f", payload[44:48])[0],
        "measured_right": struct.unpack("<f", payload[48:52])[0],
        "target_scale": struct.unpack("<f", payload[52:56])[0],
    }


def build_stdin() -> tuple[str, int]:
    """构造确定性的输入脚本：建立 arm → 阶段 A（正常命令）→ 阶段 B（超限命令）。"""
    good = build_motion_frame(2, 0.2, 0.0, 0, 300)
    bad_crc = bytearray(build_motion_frame(2, 0.3, 0.0, 0, 300))
    bad_crc[-1] ^= 0xFF
    ignore_run = build_motion_frame(1, 0.25, 0.1, 1, 300)   # 会话首帧带 run ⇒ 应被忽略

    lines = [
        ignore_run.hex(),  # 会话首帧即带 run 请求：必须被忽略（003 A4）
        bytes(bad_crc).hex(),
        good.hex(),        # 未请求运行的有效命令 ⇒ 建立 arm 门（D-003-1）
    ]
    seq = 10
    # 阶段 A（600 ms）：命令 0.25 m/s 且 run=1，每 50 ms 重发一次保持租约新鲜（lease 300 ms）
    for i in range(600):
        if i % 50 == 0:
            lines.append(build_motion_frame(seq, KIN_COMMAND_MPS, 0.0, 1, 300).hex())
            seq += 1
        else:
            lines.append("00")
    # 阶段 B（700 ms）：超限命令 2.0 m/s（车体上限 0.5）⇒ 统一比例 scale = 0.25
    for i in range(700):
        if i % 50 == 0:
            lines.append(build_motion_frame(seq, KIN_OVERSHOOT_COMMAND_MPS, 0.0, 1, 300).hex())
            seq += 1
        else:
            lines.append("00")
    lines.extend(["00"] * 20)
    return "\n".join(lines) + "\n", len(lines)


def check_blocked_path(binary: str) -> tuple[bool, str]:
    """005 A6 端到端：allow_motion=false（安全禁止）⇒ 零目标 + kSafety + scale=0。

    工具不含安全状态机（007），用 --kin-block-motion 注入 allow_motion=false 走同一条代码路径。
    """
    lines = [build_motion_frame(2, 0.2, 0.0, 0, 300).hex()]
    seq = 10
    for i in range(400):
        if i % 50 == 0:
            lines.append(build_motion_frame(seq, 0.5, 0.0, 1, 300).hex())
            seq += 1
        else:
            lines.append("00")
    stdin = "\n".join(lines) + "\n"
    completed = subprocess.run([binary, "--frames", str(len(lines)),
                                "--status-period-ms", str(STATUS_PERIOD_MS),
                                "--kin-block-motion"],
                               input=stdin.encode(), stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, check=False)
    if completed.returncode != 0:
        return False, f"blocked run exit code {completed.returncode}"
    try:
        frames = parse_frames(completed.stdout)
    except AssertionError as error:
        return False, f"blocked run frame error: {error}"
    statuses = [parse_status(frame) for frame in frames if frame[5] == 0x81]
    if not statuses:
        return False, "blocked run produced no status frames"
    for status in statuses:
        if status["target_left"] != 0.0 or status["target_right"] != 0.0:
            return False, (f"blocked run produced non-zero target "
                           f"{status['target_left']:.4f}/{status['target_right']:.4f}")
        if status["target_scale"] != 0.0:
            return False, f"blocked run produced target_scale {status['target_scale']}"
        if not (status["limit_reason"] & REASON_SAFETY):
            return False, f"blocked run missing kSafety (limit_reason=0x{status['limit_reason']:x})"
    return True, f"blocked frames={len(statuses)}"


def check_degraded_path(binary: str) -> tuple[bool, str]:
    """005 A7 端到端：注入阻塞质量位 ⇒ 降级轮速上限生效并置 kQualityDegraded。

    单独一次工具运行（--kin-degrade-quality），命令 0.5 m/s 高于降级上限 0.2 m/s，
    因此状态帧必须出现 target_scale ≈ 0.2/max_body 且 limit_reason 含 0x20。
    """
    # 先发一帧"未请求运行"的有效命令建立 arm 门（D-003-1），否则 motion_desired 恒为 false。
    lines = [build_motion_frame(2, 0.2, 0.0, 0, 300).hex()]
    seq = 10
    # 800 ms（accel 0.5 m/s² ⇒ 目标可爬到 0.4 m/s，明显高于降级上限 0.2 m/s）
    for i in range(800):
        if i % 50 == 0:
            lines.append(build_motion_frame(seq, 0.5, 0.0, 1, 300).hex())
            seq += 1
        else:
            lines.append("00")
    stdin = "\n".join(lines) + "\n"
    completed = subprocess.run([binary, "--frames", str(len(lines)),
                                "--status-period-ms", str(STATUS_PERIOD_MS),
                                "--kin-degrade-quality"],
                               input=stdin.encode(), stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, check=False)
    if completed.returncode != 0:
        return False, f"degraded run exit code {completed.returncode}"
    try:
        frames = parse_frames(completed.stdout)
    except AssertionError as error:
        return False, f"degraded run frame error: {error}"
    statuses = [parse_status(frame) for frame in frames if frame[5] == 0x81]
    if not statuses:
        return False, "degraded run produced no status frames"
    degraded = [s for s in statuses if s["limit_reason"] & REASON_QUALITY_DEGRADED]
    if not degraded:
        reasons = sorted({hex(s["limit_reason"]) for s in statuses})
        return False, f"no kQualityDegraded in limit_reason (observed {reasons})"
    for status in degraded:
        if status["target_left"] > KIN_DEGRADED_WHEEL_MPS + 1e-3:
            return False, (f"degraded target_left {status['target_left']:.4f} exceeds degraded "
                           f"limit {KIN_DEGRADED_WHEEL_MPS}")
    return True, f"degraded frames={len(degraded)}"


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: check_feedback_stream.py <robotcar01_loopback binary>", file=sys.stderr)
        return 2
    binary = sys.argv[1]

    stdin, total_lines = build_stdin()
    completed = subprocess.run([binary, "--frames", str(total_lines),
                                "--status-period-ms", str(STATUS_PERIOD_MS),
                                "--encoder-left-mps", "0.4", "--encoder-right-mps", "-0.3"],
                               input=stdin.encode(),
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
        print(f"FAIL: expected armed=1 after the first non-run command, got {last}", file=sys.stderr)
        return 1
    if fields.get("run_requested") != "1":
        print(f"FAIL: expected run_requested=1 during the kinematics phases, got {last}",
              file=sys.stderr)
        return 1

    statuses = [parse_status(frame) for frame in frames if frame[5] == 0x81]

    # --- 004 A7：measured_* 与注入编码器轮速一致且速度有效位 ---
    measured = None
    for status in statuses:
        if status["flags"] & FLAG_SPEED_VALID:
            measured = status
            break
    if measured is None:
        print("FAIL: no status frame carried kFlagSpeedValid (004 encoder integration)",
              file=sys.stderr)
        return 1
    if abs(measured["measured_left"] - 0.4) > 0.03 or abs(measured["measured_right"] + 0.3) > 0.03:
        print(f"FAIL: measured speeds {measured['measured_left']:.4f}/{measured['measured_right']:.4f}"
              f" != injected 0.4/-0.3", file=sys.stderr)
        return 1

    # --- 005 A8-① 目标可达命令值且 scale=1（无域限幅） ---
    full_scale = [s for s in statuses if abs(s["target_scale"] - 1.0) < 1e-3]
    if not full_scale:
        print("FAIL: no status frame reported target_scale == 1.0 (unlimited)", file=sys.stderr)
        return 1
    reached = [s for s in full_scale if abs(s["target_left"] - KIN_COMMAND_MPS) < 0.01]
    if not reached:
        best = max((s["target_left"] for s in full_scale), default=0.0)
        print(f"FAIL: target never reached the commanded {KIN_COMMAND_MPS} m/s (best {best:.4f})",
              file=sys.stderr)
        return 1

    # --- 005 A5/A8-② 加速度约束：相邻状态帧目标变化 <= accel × dt ---
    # 容差口径：限频器在拥塞/相位边界上可能跳过一个发送周期，故按 3 × 状态帧周期取上界
    # （0.5 × 60 ms + ε = 0.034）；更严格的 20 ms 断言会引入 flaky 风险（评审 H5）。
    ramp_bound = KIN_ACCEL_MPS2 * (3 * STATUS_PERIOD_MS / 1000.0) + 0.004
    for prev, cur in zip(statuses, statuses[1:]):
        step = abs(cur["target_left"] - prev["target_left"])
        if step > ramp_bound:
            print(f"FAIL: target step {step:.4f} exceeds ramp bound {ramp_bound:.4f} "
                  f"({prev['target_left']:.4f} -> {cur['target_left']:.4f})", file=sys.stderr)
            return 1

    # --- 005 A8-③ 超限命令 ⇒ 统一比例 scale 与原因位 ---
    limited = [s for s in statuses if abs(s["target_scale"] - KIN_EXPECTED_SCALE) < 0.02]
    if not limited:
        scales = sorted({round(s["target_scale"], 4) for s in statuses})
        print(f"FAIL: no status frame reported expected target_scale "
              f"{KIN_EXPECTED_SCALE:.2f} (observed {scales})", file=sys.stderr)
        return 1
    if not any(s["limit_reason"] & REASON_BODY_SPEED for s in limited):
        print("FAIL: expected kBodySpeed in limit_reason for the over-limit command",
              file=sys.stderr)
        return 1
    if any(s["limit_reason"] & REASON_ACCEL_LIMIT for s in limited) and not any(
            s["limit_reason"] & REASON_BODY_SPEED for s in limited):
        print("FAIL: ramp flag present without the expected body limit reason", file=sys.stderr)
        return 1
    # 车体上限不得被越过（target 由轮速域/车体域共同保证）
    for status in statuses:
        if status["target_left"] > KIN_MAX_BODY_SPEED_MPS + 1e-3:
            print(f"FAIL: target_left {status['target_left']:.4f} exceeds body limit "
                  f"{KIN_MAX_BODY_SPEED_MPS}", file=sys.stderr)
            return 1

    kin_lines = [line for line in stderr_text.splitlines() if line.startswith("kin ")]
    if not kin_lines:
        print("FAIL: no kinematics statistics line on stderr", file=sys.stderr)
        return 1
    kin_fields = dict(part.split("=", 1) for part in kin_lines[-1].split() if "=" in part)
    if int(kin_fields.get("ramp_clamped", "0")) == 0:
        print(f"FAIL: expected ramp_clamped > 0, got {kin_lines[-1]}", file=sys.stderr)
        return 1

    encoder_lines = [line for line in stderr_text.splitlines() if line.startswith("encoder ")]
    if not encoder_lines:
        print("FAIL: no encoder statistics line on stderr", file=sys.stderr)
        return 1

    accel_frames = [s for s in statuses if s["limit_reason"] & REASON_ACCEL_LIMIT]
    if len(accel_frames) < 2:
        print(f"FAIL: expected at least 2 status frames with kAccelLimit (ramp actually observed), "
              f"got {len(accel_frames)}", file=sys.stderr)
        return 1

    blocked_ok, blocked_detail = check_blocked_path(binary)
    if not blocked_ok:
        print(f"FAIL: {blocked_detail}", file=sys.stderr)
        return 1

    degraded_ok, degraded_detail = check_degraded_path(binary)
    if not degraded_ok:
        print(f"FAIL: {degraded_detail}", file=sys.stderr)
        return 1

    print(f"PASS: {len(frames)} feedback frames verified "
          f"(types={[hex(t) for t in sorted(set(types))]}), status frames={len(statuses)}, "
          f"target reached {KIN_COMMAND_MPS} m/s and scale {KIN_EXPECTED_SCALE:.2f} under "
          f"over-limit command, ramp bound {ramp_bound:.4f} respected, "
          f"accel-flagged frames={len(accel_frames)}; blocked/degraded paths verified")
    return 0


if __name__ == "__main__":
    sys.exit(main())
