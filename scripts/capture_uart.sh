#!/usr/bin/env bash
# RobotCar01 串口采集（Mac 侧驱动 Honor PC 上的 COM 口）——Iteration 002.5
#
# 用法： scripts/capture_uart.sh [--port COM15] [--seconds 15] [--out /tmp/uart_log.txt]
#
# 流程： 上传 capture_uart_ps.ps1 → 远端运行（带宿主时间戳）→ 回传日志到本地
# 注意： **先启动采集、再复位板子**（否则漏掉启动横幅，评审 C-6 第⑦项）。

set -euo pipefail

FW_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_ROOT="$(cd "$FW_DIR/.." && pwd)"
REMOTE_PY="$WS_ROOT/.agents/skills/honor-ssh-access/scripts/remote.py"

PORT="COM15"
SECONDS_TO_CAPTURE=15
LOCAL_OUT="/tmp/uart_log_$(date +%Y%m%d_%H%M%S).txt"
while [ $# -gt 0 ]; do
  case "$1" in
    --port) PORT="$2"; shift 2 ;;
    --seconds) SECONDS_TO_CAPTURE="$2"; shift 2 ;;
    --out) LOCAL_OUT="$2"; shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 64 ;;
  esac
done

[ -f "$REMOTE_PY" ] || { echo "ERROR: remote.py not found: $REMOTE_PY" >&2; exit 2; }

REMOTE_DIR="E:/File/robotcar01_bringup"
REMOTE_PS="$REMOTE_DIR/capture_uart_ps.ps1"
REMOTE_LOG="$REMOTE_DIR/uart_log.txt"

echo "== 上传采集脚本 =="
python3 "$REMOTE_PY" ps "New-Item -ItemType Directory -Force -Path 'E:\File\robotcar01_bringup' | Out-Null; 'ok'"
python3 "$REMOTE_PY" put "$FW_DIR/scripts/capture_uart_ps.ps1" "$REMOTE_PS"

echo "== 远端采集 ${SECONDS_TO_CAPTURE}s on ${PORT}（现在可以复位板子） =="
python3 "$REMOTE_PY" ps "powershell -ExecutionPolicy Bypass -File '$REMOTE_PS' -Port $PORT -Baud 115200 -Seconds $SECONDS_TO_CAPTURE -Out 'E:\File\robotcar01_bringup\uart_log.txt'"

echo "== 回传日志 =="
python3 "$REMOTE_PY" get "$REMOTE_LOG" "$LOCAL_OUT"
echo "本地日志: $LOCAL_OUT"
wc -l "$LOCAL_OUT" || true
