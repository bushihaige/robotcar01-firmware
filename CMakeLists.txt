cmake_minimum_required(VERSION 3.24)
project(robotcar01 LANGUAGES C CXX ASM)

# RobotCar01 底盘固件
#
# 双 target 策略：
#   - host：本机 AppleClang + GoogleTest，用于单元测试（默认构建）；
#   - fw  ：arm-none-eabi-gcc 交叉编译，产出固件（**Iteration 002.5 起启用**：
#           默认 OFF，通过 ROBOTCAR01_BUILD_FW=ON + preset `fw` 或工具链文件启用）。
#
# 平台无关业务（app/chassis/ 等）以静态库 robotcar01_chassis 编入 host；
# MCU OS Lite 核心（app/mcu_os_lite/）以静态库 robotcar01_mcu_os_lite 编入 host；
# fw 分支的源列表与 vendor（HAL/CMSIS）引入方式由 Iteration 002.5 落地
# （D-001-3 → B-2.5-x）：fw 编 platform_stm32/ + third_party/，入口为
# platform_stm32/main_stm32.cc，**不使用 CubeMX 生成物**。
#
# 评审 C-1：语言必须含 C（HAL 是 C 源）与 ASM（startup_stm32f407xx.s），
# 否则 CMake 报 "Cannot determine link language for target"。

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ROBOTCAR01_BUILD_FW "Build firmware cross target (needs arm-none-eabi-gcc)" OFF)

set(ROBOTCAR01_VENDOR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

if(ROBOTCAR01_BUILD_FW)
  # --- vendor 库：HAL/CMSIS（只读、不参与 -Werror，评审 H-4/S-6） ---
  set(ROBOTCAR01_HAL_SOURCES
    ${ROBOTCAR01_VENDOR_DIR}/stm32f4-hal/Src/stm32f4xx_hal.c
    ${ROBOTCAR01_VENDOR_DIR}/stm32f4-hal/Src/stm32f4xx_hal_cortex.c
    ${ROBOTCAR01_VENDOR_DIR}/stm32f4-hal/Src/stm32f4xx_hal_rcc.c
    ${ROBOTCAR01_VENDOR_DIR}/stm32f4-hal/Src/stm32f4xx_hal_rcc_ex.c
    ${ROBOTCAR01_VENDOR_DIR}/stm32f4-hal/Src/stm32f4xx_hal_gpio.c
    ${ROBOTCAR01_VENDOR_DIR}/stm32f4-hal/Src/stm32f4xx_hal_uart.c
    ${ROBOTCAR01_VENDOR_DIR}/stm32f4-hal/Src/stm32f4xx_hal_dma.c
    ${ROBOTCAR01_VENDOR_DIR}/stm32f4-hal/Src/stm32f4xx_hal_dma_ex.c
    ${ROBOTCAR01_VENDOR_DIR}/stm32f4-hal/Src/stm32f4xx_hal_pwr.c
    ${ROBOTCAR01_VENDOR_DIR}/stm32f4-hal/Src/stm32f4xx_hal_pwr_ex.c
    ${ROBOTCAR01_VENDOR_DIR}/stm32f4-hal/Src/stm32f4xx_hal_exti.c
    ${ROBOTCAR01_VENDOR_DIR}/stm32f4-hal/Src/stm32f4xx_hal_flash.c
    ${ROBOTCAR01_VENDOR_DIR}/stm32f4-hal/Src/stm32f4xx_hal_flash_ex.c
    ${ROBOTCAR01_VENDOR_DIR}/stm32f4-hal/Src/stm32f4xx_hal_flash_ramfunc.c
    ${ROBOTCAR01_VENDOR_DIR}/cmsis-device-f4/Source/Templates/system_stm32f4xx.c
  )
  add_library(robotcar01_hal STATIC ${ROBOTCAR01_HAL_SOURCES})
  target_include_directories(robotcar01_hal SYSTEM PUBLIC
    ${ROBOTCAR01_VENDOR_DIR}/stm32f4-hal/Inc
    ${ROBOTCAR01_VENDOR_DIR}/cmsis-device-f4/Include
    ${ROBOTCAR01_VENDOR_DIR}/cmsis-core/Include
  )
  target_include_directories(robotcar01_hal PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/platform_stm32/inc)
  target_compile_definitions(robotcar01_hal PUBLIC STM32F407xx USE_HAL_DRIVER)
  target_compile_options(robotcar01_hal PRIVATE -w)  # vendor 不参与告警/错误门禁

  # --- 固件 target（Iteration 002.5：最小编译单元，不含调度器与业务） ---
  add_executable(robotcar01_fw
    platform_stm32/main_stm32.cc
    platform_stm32/clock_init.cc
    platform_stm32/uart_console.cc
    platform_stm32/diag.cc
    ${ROBOTCAR01_VENDOR_DIR}/cmsis-device-f4/Source/Templates/gcc/startup_stm32f407xx.s
  )
  target_include_directories(robotcar01_fw PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/platform_stm32/inc
    ${CMAKE_CURRENT_SOURCE_DIR}
  )
  target_compile_definitions(robotcar01_fw PRIVATE STM32F407xx USE_HAL_DRIVER)
  target_compile_options(robotcar01_fw PRIVATE -Wall -Wextra -Werror)
  target_link_libraries(robotcar01_fw PRIVATE robotcar01_hal)
  # Generic 工具链下 CMake 不追加平台后缀，显式固定 ELF 产物名（验收 1 要求 robotcar01_fw.elf）
  set_target_properties(robotcar01_fw PROPERTIES SUFFIX ".elf")

  # 产物：ELF + HEX（供 ST-Link 烧录）+ BIN + size 报告
  add_custom_command(TARGET robotcar01_fw POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:robotcar01_fw> ${CMAKE_BINARY_DIR}/robotcar01_fw.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:robotcar01_fw> ${CMAKE_BINARY_DIR}/robotcar01_fw.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:robotcar01_fw>
    COMMENT "Generating robotcar01_fw.hex/.bin and size report"
  )
else()
  # --- 平台无关业务库（host 编译，供单测链接；fw 复用机制见 002.5） ---
  add_library(robotcar01_chassis STATIC
    app/chassis/kinematics.cc
    app/chassis/safety_gate.cc
    app/chassis/walking_skeleton.cc
  )
  target_include_directories(robotcar01_chassis PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_options(robotcar01_chassis PRIVATE -Wall -Wextra -Werror)

  # --- MCU OS Lite 核心库（002；源列表变量供 fw 分支复用，D-001-3 机制预留） ---
  set(ROBOTCAR01_MCU_OS_LITE_SOURCES
    app/mcu_os_lite/task_descriptor.cc
    app/mcu_os_lite/scheduler.cc
    app/mcu_os_lite/health_monitor.cc
  )
  add_library(robotcar01_mcu_os_lite STATIC ${ROBOTCAR01_MCU_OS_LITE_SOURCES})
  target_include_directories(robotcar01_mcu_os_lite PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_options(robotcar01_mcu_os_lite PRIVATE -Wall -Wextra -Werror)

  # --- 协议库（003；源列表变量供 fw 分支 008 复用，D-003-8 本轮不编入 fw） ---
  set(ROBOTCAR01_PROTOCOL_SOURCES
    app/protocol/byte_ring.cc
    app/protocol/frame.cc
    app/protocol/decoder.cc
    app/protocol/session.cc
    app/protocol/command_manager.cc
    app/protocol/feedback.cc
    app/protocol/feedback_limiter.cc
  )
  # --- 编码器估计库（004）---
  set(ROBOTCAR01_ENCODER_SOURCES
    app/encoder/encoder_config.cc
    app/encoder/wheel_state.cc
    app/encoder/encoder_estimator.cc
  )
  add_library(robotcar01_encoder STATIC ${ROBOTCAR01_ENCODER_SOURCES})
  target_include_directories(robotcar01_encoder PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_options(robotcar01_encoder PRIVATE -Wall -Wextra -Werror)
  target_link_libraries(robotcar01_encoder PUBLIC robotcar01_mcu_os_lite)

  # --- 差速运动学库（005）：依赖方向 kinematics → chassis + encoder（无环） ---
  add_library(robotcar01_kinematics STATIC
    app/kinematics/kinematics_config.cc
    app/kinematics/body_state.cc
    app/kinematics/differential_kinematics.cc
  )
  target_include_directories(robotcar01_kinematics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_options(robotcar01_kinematics PRIVATE -Wall -Wextra -Werror)
  target_link_libraries(robotcar01_kinematics PUBLIC robotcar01_chassis robotcar01_encoder)

  add_library(robotcar01_protocol STATIC ${ROBOTCAR01_PROTOCOL_SOURCES})
  target_include_directories(robotcar01_protocol PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_options(robotcar01_protocol PRIVATE -Wall -Wextra -Werror)
  # 反馈诊断帧消费 TaskSet（002），命令快照承载 chassis 纯数据（001）
  # 依赖方向（D-004-10/评审 S-8）：protocol → encoder（反馈组装需要 WheelState）；
  # encoder 不得依赖 protocol，避免循环。
  target_link_libraries(robotcar01_protocol PUBLIC robotcar01_chassis robotcar01_mcu_os_lite
                                                robotcar01_encoder)

  # --- host 单元测试 target ---
  enable_testing()
  add_subdirectory(tests)

  # --- host 调试工具（003 A7：真实字节流端到端验证） ---
  add_subdirectory(tools)
endif()
