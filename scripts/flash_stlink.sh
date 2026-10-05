#!/usr/bin/env bash
# RobotCar01 固件烧录（Mac 侧驱动 Honor PC 的 ST-Link）——Iteration 002.5
#
# 用法： scripts/flash_stlink.sh [--hex <本地hex>] [--dry-run]
#   默认 hex： firmware/build/fw/robotcar01_fw.hex
#
# 流程： 本地 hex → scp 到终端（唯一文件名，不覆盖既有产物）→ 经 ssh 调用
#        STM32_Programmer_CLI（CubeIDE 自带）写入 + 校验 + 复位 → 回传日志。
#
# 安全： 仅写目标板，不触碰终端上的 FOC_PID 项目；不传 --motor-safe 类参数给无关项目。
#        需要工程师已授权烧录（本轮已授权：目标板允许覆盖）。

set -euo pipefail

FW_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_ROOT="$(cd "$FW_DIR/.." && pwd)"
REMOTE_PY="$WS_ROOT/.agents/skills/honor-ssh-access/scripts/remote.py"

HEX="${FW_DIR}/build/fw/robotcar01_fw.hex"
DRY_RUN=0
while [ $# -gt 0 ]; do
  case "$1" in
    --hex) HEX="$2"; shift 2 ;;
    --dry-run) DRY_RUN=1; shift ;;
    *) echo "unknown arg: $1" >&2; exit 64 ;;
  esac
done

[ -f "$HEX" ] || { echo "ERROR: hex not found: $HEX（先运行 scripts/build_fw.sh）" >&2; exit 2; }
[ -f "$REMOTE_PY" ] || { echo "ERROR: remote.py not found: $REMOTE_PY" >&2; exit 2; }

STAMP="$(date +%Y%m%d_%H%M%S)"
HEX_NAME="robotcar01_fw_${STAMP}.hex"
REMOTE_DIR="E:/File/robotcar01_bringup"
REMOTE_HEX="$REMOTE_DIR/$HEX_NAME"
LOCAL_SHA="$(shasum -a 256 "$HEX" | cut -d' ' -f1)"

PROGRAMMER='E:\App\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.200.202503041107\tools\bin\STM32_Programmer_CLI.exe'

echo "== 本地 hex: $HEX =="
echo "   sha256: $LOCAL_SHA"

if [ "$DRY_RUN" = "1" ]; then
  echo "(dry-run: 不传输、不烧录)"
  exit 0
fi

echo "== 1/3 确保远端目录并上传 =="
python3 "$REMOTE_PY" ps "New-Item -ItemType Directory -Force -Path '$REMOTE_DIR' | Out-Null; 'ok'"
python3 "$REMOTE_PY" put "$HEX" "$REMOTE_HEX"

echo "== 2/3 校验远端 sha256 =="
REMOTE_SHA="$(python3 "$REMOTE_PY" ps "(Get-FileHash -Algorithm SHA256 '$REMOTE_HEX').Hash.ToLower()" | tr -d '\r' | tail -1)"
echo "   remote sha256: $REMOTE_SHA"
[ "$REMOTE_SHA" = "$LOCAL_SHA" ] || { echo "ERROR: sha256 不一致，停止烧录" >&2; exit 3; }

echo "== 3/3 ST-Link 写入 + 校验 + 复位 =="
python3 "$REMOTE_PY" ps "& '$PROGRAMMER' -c port=SWD mode=UR reset=HWrst freq=4000 -w '$REMOTE_HEX' -v -rst; exit \$LASTEXITCODE"
