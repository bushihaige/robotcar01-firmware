# RobotCar01 交叉编译工具链（Iteration 002.5 起启用）
#
# 用法（推荐 preset）：cmake --preset fw && cmake --build --preset fw
# 或显式：cmake -S . -B build/fw -DROBOTCAR01_BUILD_FW=ON \
#           -DCMAKE_TOOLCHAIN_FILE="$PWD/cmake/arm-none-eabi.cmake"
#
# 变更记录（评审 C-1/C-2）：
#   - 补 CMAKE_ASM_FLAGS_INIT：startup_stm32f407xx.s 是**不预处理**的 .s，
#     但汇编阶段仍需与 C 一致的 CPU/FPU 参数，否则 ABI/指令集不一致；
#   - 补 CMAKE_EXE_LINKER_FLAGS_INIT：链接脚本 -T、--gc-sections、Map、nano/nosys specs；
#   - 补 -ffunction-sections -fdata-sections：与 --gc-sections 配套，丢弃未引用函数
#     （HAL 中如 HAL_UART_DMAStop 会引用 HAL_DMA_Abort，评审 H-4）。

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy CACHE FILEPATH "objcopy")
set(CMAKE_SIZE arm-none-eabi-size CACHE FILEPATH "size")

# 交叉编译不做可执行文件试链接（否则会因缺少启动/链接脚本而在探测阶段失败）
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(_ROBOTCAR01_CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(_ROBOTCAR01_SECTIONS_FLAGS "-ffunction-sections -fdata-sections")

set(CMAKE_C_FLAGS_INIT "${_ROBOTCAR01_CPU_FLAGS} ${_ROBOTCAR01_SECTIONS_FLAGS}")
set(CMAKE_CXX_FLAGS_INIT "${_ROBOTCAR01_CPU_FLAGS} ${_ROBOTCAR01_SECTIONS_FLAGS}")
set(CMAKE_ASM_FLAGS_INIT "${_ROBOTCAR01_CPU_FLAGS}")

# 链接脚本与链接选项：路径用工具链文件所在目录推导，避免相对 build 目录解析（评审 C-2）
get_filename_component(_ROBOTCAR01_FW_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(CMAKE_EXE_LINKER_FLAGS_INIT "-T${_ROBOTCAR01_FW_ROOT}/linker/STM32F407ZGTx_FLASH.ld -Wl,--gc-sections -Wl,-Map=robotcar01_fw.map -specs=nano.specs -specs=nosys.specs")

# Windows 无 PATH 场景兜底（不设置则依赖 PATH 中的 arm-none-eabi-*）：
#   -DCMAKE_C_COMPILER=<CubeIDE>/tools/bin/arm-none-eabi-gcc.exe 等（见迭代文档 §跨机链路）
