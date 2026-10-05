#!/usr/bin/env bash
# RobotCar01 固件烧录（Mac 侧驱动 Honor PC 的 ST-Link）——Iteration 002.5
#
# 本迭代的构建发生在 Honor PC 上（终端有完整 GNU Tools for STM32 + cmake/ninja），
# 因此默认直接烧录**终端上**的 HEX；仅当显式给 --local-hex 时才上传本地文件。
#
# 用法：
#   scripts/flash_stlink.sh                              # 烧录终端构建产物（默认路径）
#   scripts/flash_stlink.sh --remote-hex 'E:\...\x.hex'  # 指定终端上的 hex
#   scripts/flash_stlink.sh --local-hex /path/to/x.hex   # 上传后烧录（唯一文件名 + sha256 校验）
#   scripts/flash_stlink.sh --dry-run                    # 只读板卡信息，不写入
#
# 安全：仅写目标板；STM32_Programmer_CLI 会写入、校验并复位（固件随即运行）。
#       工程师已授权覆盖当前板上固件（002.5 目标板）。

set -euo pipefail

FW_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_ROOT="$(cd "$FW_DIR/.." && pwd)"
REMOTE_PY="$WS_ROOT/.agents/skills/honor-ssh-access/scripts/remote.py"

REMOTE_HEX='E:\File\robotcar01_bringup\src\build\fw\robotcar01_fw.hex'
LOCAL_HEX=""
DRY_RUN=0
while [ $# -gt 0 ]; do
  case "$1" in
    --remote-hex) REMOTE_HEX="$2"; shift 2 ;;
    --local-hex) LOCAL_HEX="$2"; shift 2 ;;
    --dry-run) DRY_RUN=1; shift ;;
    *) echo "unknown arg: $1" >&2; exit 64 ;;
  esac
done

[ -f "$REMOTE_PY" ] || { echo "ERROR: remote.py not found: ${REMOTE_PY}" >&2; exit 2; }

PROGRAMMER='E:\App\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.200.202503041107\tools\bin\STM32_Programmer_CLI.exe'

if [ -n "$LOCAL_HEX" ]; then
  [ -f "$LOCAL_HEX" ] || { echo "ERROR: local hex not found: ${LOCAL_HEX}" >&2; exit 2; }
  STAMP="$(date +%Y%m%d_%H%M%S)"
  TARGET="E:\\File\\robotcar01_bringup\\robotcar01_fw_${STAMP}.hex"
  LOCAL_SHA="$(shasum -a 256 "$LOCAL_HEX" | cut -d' ' -f1)"
  echo "== 上传本地 hex（sha256 ${LOCAL_SHA}） =="
  python3 "$REMOTE_PY" put "$LOCAL_HEX" "E:/File/robotcar01_bringup/robotcar01_fw_${STAMP}.hex"
  REMOTE_SHA="$(python3 "$REMOTE_PY" ps "(Get-FileHash -Algorithm SHA256 '${TARGET}').Hash.ToLower()" | tr -d '\r' | tail -1)"
  [ "$REMOTE_SHA" = "$LOCAL_SHA" ] || { echo "ERROR: sha256 不一致，停止烧录" >&2; exit 3; }
  REMOTE_HEX="$TARGET"
fi

echo "== 远端 hex: ${REMOTE_HEX} =="
echo "== 板卡信息（写入证据） =="
python3 "$REMOTE_PY" ps "& '${PROGRAMMER}' -c port=SWD mode=UR reset=HWrst freq=4000 | Select-String -Pattern 'Device ID|Flash size|Voltage|ST-LINK SN|Firmware version'"

if [ "$DRY_RUN" = "1" ]; then
  echo "(dry-run: 不写入)"
  exit 0
fi

echo "== ST-Link 写入 + 校验 + 复位 =="
python3 "$REMOTE_PY" ps "& '${PROGRAMMER}' -c port=SWD mode=UR reset=HWrst freq=4000 -w '${REMOTE_HEX}' -v -rst; exit \$LASTEXITCODE"
