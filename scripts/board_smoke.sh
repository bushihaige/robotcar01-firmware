#!/usr/bin/env bash
# RobotCar01 006.5 一键板级冒烟：远端构建 → 烧录 → 采集 → 判定 —— Iteration 006.5
#
# 用法：
#   scripts/board_smoke.sh                    # 构建+烧录+采集 12s+判定
#   scripts/board_smoke.sh --build-only       # 只远端构建（不碰板子）
#   scripts/board_smoke.sh --flash-only       # 只烧录+采集（用现有 hex）
#   scripts/board_smoke.sh --port COM7        # 显式指定串口（默认自动探测 CH340）
#   scripts/board_smoke.sh --seconds 20
#
# 判定链：设备启动横幅 → 1s 心跳（check_heartbeat.py）→ 电机环路状态行（check_motor_loop.py）
# 安全：本脚本只写目标板并在结束时复位运行固件；**上电后 EN 由固件保持失能**，
#       直到串口收到 ']' 或 '+' 命令。烧录阶段请保持机械急停在手边。
set -euo pipefail

FW_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_ROOT="$(cd "$FW_DIR/.." && pwd)"
REMOTE_PY="$WS_ROOT/.agents/skills/honor-ssh-access/scripts/remote.py"
HOST="${HONOR_HOST:-192.168.1.19}"

REMOTE_SRC='E:\File\robotcar01_bringup\src'
REMOTE_HEX="${REMOTE_SRC}\\build\\fw\\robotcar01_fw.hex"
SECONDS_TO_CAPTURE=12
PORT="auto"
DO_BUILD=1
DO_BOARD=1
STAMP="$(date +%Y%m%d_%H%M%S)"
LOCAL_LOG="/tmp/uart_0065_${STAMP}.txt"

while [ $# -gt 0 ]; do
  case "$1" in
    --build-only) DO_BOARD=0; shift ;;
    --flash-only) DO_BUILD=0; shift ;;
    --port) PORT="$2"; shift 2 ;;
    --seconds) SECONDS_TO_CAPTURE="$2"; shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 64 ;;
  esac
done

[ -f "$REMOTE_PY" ] || { echo "ERROR: remote.py not found: $REMOTE_PY" >&2; exit 2; }
remote_ps() { python3 "$REMOTE_PY" --host "$HOST" ps "$1"; }

# 远端工具链（2026-10-25 实测路径；CubeIDE 1.19.0）
TOOLCHAIN='E:\App\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344\tools\bin'
PYSITE='C:\Users\LHL\AppData\Roaming\Python\Python313\Scripts'
PROGRAMMER='E:\App\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.200.202503041107\tools\bin\STM32_Programmer_CLI.exe'

echo "== 0) 确认远端主机与工具链 =="
remote_ps "
  \$tool = '${TOOLCHAIN}'
  \$py = '${PYSITE}'
  if (-not (Test-Path \$tool)) { Write-Output 'MISSING_TOOLCHAIN'; exit 3 }
  \$env:Path = \"\$tool;\$py;\" + \$env:Path
  Write-Output ('gcc: ' + ((& arm-none-eabi-gcc.exe --version | Select-Object -First 1)))
" | tail -2

if [ "$DO_BUILD" = "1" ]; then
  echo "== 1) 远端构建（ROBOTCAR01_BUILD_FW=ON） =="
  remote_ps "
    \$tool = '${TOOLCHAIN}'
    \$py = '${PYSITE}'
    \$env:Path = \"\$tool;\$py;\" + \$env:Path
    Set-Location '${REMOTE_SRC}'
    & cmake.exe -S . -B build/fw -G Ninja -DROBOTCAR01_BUILD_FW=ON '-DCMAKE_TOOLCHAIN_FILE=${REMOTE_SRC}\\cmake\\arm-none-eabi.cmake' 2>&1 | Select-Object -Last 2
    & cmake.exe --build build/fw 2>&1 | Select-String -Pattern 'error|FAILED' | Select-Object -First 5
    if (\$LASTEXITCODE -ne 0) { Write-Output 'BUILD_FAILED'; exit 1 }
    & arm-none-eabi-size.exe build\\fw\\robotcar01_fw.elf
  " | tail -6
fi

[ "$DO_BOARD" = "1" ] || { echo "(build-only：未触碰板子)"; exit 0; }

echo "== 2) 串口探测 =="
if [ "$PORT" = "auto" ]; then
  PORT="$(remote_ps "
    \$p = Get-CimInstance Win32_SerialPort -ErrorAction SilentlyContinue | Where-Object { \$_.Name -match 'CH340|CP210|USB-SERIAL' } | Select-Object -First 1
    if (\$p) { (\$p.DeviceID -replace '^.*\(','' -replace '\).*$','') } else { 'NONE' }
  " | tr -d '\r' | tail -1)"
fi
if [ "$PORT" = "NONE" ] || [ -z "$PORT" ]; then
  echo "ERROR: 未检测到 USB-TTL（CH340）串口。请把 USB-TTL 插到 Honor PC 后重试，或用 --port 显式指定。" >&2
  exit 4
fi
echo "  串口: $PORT"

echo "== 3) 后台启动采集（先开采集，烧录时复位才不会漏掉横幅） =="
remote_ps "New-Item -ItemType Directory -Force -Path 'E:\File\robotcar01_bringup' | Out-Null; 'ok'" >/dev/null
python3 "$REMOTE_PY" --host "$HOST" put "$FW_DIR/scripts/capture_uart_ps.ps1" "E:/File/robotcar01_bringup/capture_uart_ps.ps1" >/dev/null
# remote.py 只提供 check/ps/cmd/raw/put/get，后台采集用远端的 Start-Process 实现
remote_ps "Start-Process powershell -ArgumentList '-ExecutionPolicy','Bypass','-File','E:\File\robotcar01_bringup\capture_uart_ps.ps1','-Port','$PORT','-Baud','115200','-Seconds','$SECONDS_TO_CAPTURE','-Out','E:\File\robotcar01_bringup\uart_log.txt' -WindowStyle Hidden; 'capture started'" | tail -1
sleep 2

echo "== 4) 板卡信息（写入证据） =="
remote_ps "& '$PROGRAMMER' -c port=SWD mode=UR reset=HWrst freq=4000 | Select-String -Pattern 'Device ID|Flash size|Voltage|ST-LINK SN|Firmware version'" | tail -6

echo "== 5) 烧录 + 校验 + 复位 =="
remote_ps "& '$PROGRAMMER' -c port=SWD mode=UR reset=HWrst freq=4000 -w '${REMOTE_HEX}' -v -rst; exit \$LASTEXITCODE"

echo "== 6) 等待采集结束并回传 =="
sleep "$((SECONDS_TO_CAPTURE + 2))"
python3 "$REMOTE_PY" --host "$HOST" get "E:/File/robotcar01_bringup/uart_log.txt" "$LOCAL_LOG" >/dev/null
echo "  本地日志: $LOCAL_LOG ($(wc -l < "$LOCAL_LOG" | tr -d ' ') 行)"

echo "== 7) 判定 =="
python3 "$FW_DIR/scripts/check_heartbeat.py" "$LOCAL_LOG" || echo "WARN: 心跳判定未通过（见上）"
python3 "$FW_DIR/scripts/check_motor_loop.py" "$LOCAL_LOG" || echo "WARN: 电机环路判定未通过（见上；零输出固件下 D 段通常为 n/a）"

echo "== 完成 =="
echo "期望（零输出固件）：横幅含 'motor loop on board'；hb 每秒一行且 sched 每秒约 +1000；"
echo "                    enc 行计数随手转车轮变化；out 行 m=0 且 p=0（EN 失能）；arb.a 持续增长。"
