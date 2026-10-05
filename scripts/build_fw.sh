#!/usr/bin/env bash
# RobotCar01 固件构建（Mac 侧交叉编译预检）——Iteration 002.5
#
# 用法： scripts/build_fw.sh
# 产物： firmware/build/fw/robotcar01_fw.elf | .hex | .bin | .map
# 证据： 末尾打印 arm-none-eabi-size 与 elf sha256（写入 run-results 用）
#
# 说明：工具链优先使用 ARM 官方 GNU Toolchain（含 newlib），其次 PATH 中的 arm-none-eabi-gcc。
#       若两者都没有，脚本会给出明确的安装指引并退出非 0。

set -euo pipefail

FW_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$FW_DIR"

# ---- 选择工具链（必须带 newlib；brew 的 arm-none-eabi-gcc 公式不含 newlib，无法编译裸机） ----
ARM_OFFICIAL="$(ls -d "$HOME"/tools/arm-gnu-toolchain-*/bin 2>/dev/null | head -1 || true)"
if [ -n "$ARM_OFFICIAL" ] && [ -x "$ARM_OFFICIAL/arm-none-eabi-gcc" ]; then
  export PATH="$ARM_OFFICIAL:$PATH"
  TOOLCHAIN_SRC="ARM 官方工具链 ($ARM_OFFICIAL)"
elif command -v arm-none-eabi-gcc >/dev/null 2>&1 && arm-none-eabi-gcc -print-file-name=libc.a >/dev/null 2>&1 \
     && [ -f "$(arm-none-eabi-gcc -print-file-name=libc.a)" ]; then
  TOOLCHAIN_SRC="PATH 中的 arm-none-eabi-gcc（含 newlib）"
else
  echo "ERROR: 未找到带 newlib 的 arm-none-eabi 工具链。" >&2
  echo "  推荐：安装 ARM 官方 GNU Toolchain（免 sudo，解压即用）：" >&2
  echo "    mkdir -p ~/tools && cd ~/tools" >&2
  echo "    curl -L -o t.tar.xz https://developer.arm.com/-/media/Files/downloads/gnu/13.3.rel1/binrel/arm-gnu-toolchain-13.3.rel1-darwin-arm64-arm-none-eabi.tar.xz" >&2
  echo "    tar -xf t.tar.xz" >&2
  echo "  注意：brew 公式 arm-none-eabi-gcc 不含 newlib，会报 'stdint.h: No such file'。" >&2
  exit 2
fi

echo "== 工具链: $TOOLCHAIN_SRC =="
arm-none-eabi-gcc --version | head -1

echo "== 配置 + 构建 fw target =="
cmake --preset fw
cmake --build --preset fw

ELF="build/fw/robotcar01_fw.elf"
HEX="build/fw/robotcar01_fw.hex"
echo "== 产物 =="
ls -l "$ELF" "$HEX"
echo "== 架构检查（应出现 v7E-M / VFPv4-D16 / hard-float） =="
arm-none-eabi-readelf -A "$ELF" | grep -E "Tag_CPU_arch|FP_arch|Tag_ABI_VFP_args|Tag_ABI_FP" || true
echo "== size =="
arm-none-eabi-size "$ELF"
echo "== sha256 =="
if command -v shasum >/dev/null 2>&1; then
  shasum -a 256 "$ELF" "$HEX"
else
  sha256sum "$ELF" "$HEX"
fi
