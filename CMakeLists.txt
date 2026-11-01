cmake_minimum_required(VERSION 3.24)
project(robotcar01 LANGUAGES C CXX ASM)

# RobotCar01 底盘固件
#
# 双 target 策略：
#   - host：本机 AppleClang + GoogleTest，用于单元测试（默认构建）；
#   - fw  ：arm-none-eabi-gcc 交叉编译，产出固件（**Iteration 002.5 起启用**：
#           默认 OFF，通过 ROBOTCAR01_BUILD_FW=ON + preset `fw` 或工具链文件启用）。
#
# 平台无关业务（app/chassis/ 等）以静态库编入 host；
# MCU OS Lite 核心（app/mcu_os_lite/）以静态库 robotcar01_mcu_os_lite 编入 host；
# fw 分支的源列表与 vendor（HAL/CMSIS）引入方式由 Iteration 002.5 落地
# （D-001-3 → B-2.5-x）：fw 编 platform_stm32/ + third_party/，入口为
# platform_stm32/main_stm32.cc，**不使用 CubeMX 生成物**。
#
# Iteration 006.5（T1）调整：
#   - app 静态库（chassis/mcu_os_lite/encoder/kinematics/control）移到**公共区**，
#     host 与 fw 共用同一份源列表与编译选项（fw 侧的真正垂直切片：平台层装配 004/006 的算法）；
#   - protocol 仍只编 host（D-003-8：协议帧不进 fw，008 接线启用）；
#   - vendor 源列表新增 TIM HAL（编码器模式 TIM2/TIM3 + PWM TIM4）。
#
# 评审 C-1：语言必须含 C（HAL 是 C 源）与 ASM（startup_stm32f407xx.s），
# 否则 CMake 报 "Cannot determine link language for target"。

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ROBOTCAR01_BUILD_FW "Build firmware cross target (needs arm-none-eabi-gcc)" OFF)

set(ROBOTCAR01_VENDOR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

# =====================================================================================
# app 源列表（公共区，006.5 T1）：host 与 fw 共用，禁止在分支内各写一份
# =====================================================================================
set(ROBOTCAR01_CHASSIS_SOURCES
  app/chassis/kinematics.cc
  app/chassis/safety_gate.cc
)

set(ROBOTCAR01_MCU_OS_LITE_SOURCES
  app/mcu_os_lite/task_descriptor.cc
  app/mcu_os_lite/scheduler.cc
  app/mcu_os_lite/health_monitor.cc
)

set(ROBOTCAR01_ENCODER_SOURCES
  app/encoder/encoder_config.cc
  app/encoder/wheel_state.cc
  app/encoder/encoder_estimator.cc
)

set(ROBOTCAR01_KINEMATICS_SOURCES
  app/kinematics/kinematics_config.cc
  app/kinematics/body_state.cc
  app/kinematics/differential_kinematics.cc
)

set(ROBOTCAR01_CONTROL_SOURCES
  app/control/controller_config.cc
  app/control/wheel_speed_controller.cc
  app/control/output_arbitration.cc
  app/control/control_tasks.cc
)

# 协议库源列表（003；供 host 与 008 的 fw 复用）
set(ROBOTCAR01_PROTOCOL_SOURCES
  app/protocol/byte_ring.cc
  app/protocol/frame.cc
  app/protocol/decoder.cc
  app/protocol/session.cc
  app/protocol/command_manager.cc
  app/protocol/feedback.cc
  app/protocol/feedback_limiter.cc
)

# =====================================================================================
# app 静态库（公共区）：host 与 fw 都编译同一份，平台差异只由 compiler/toolchain 决定
# =====================================================================================
add_library(robotcar01_chassis STATIC ${ROBOTCAR01_CHASSIS_SOURCES})
target_include_directories(robotcar01_chassis PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(robotcar01_chassis PRIVATE -Wall -Wextra -Werror)

add_library(robotcar01_mcu_os_lite STATIC ${ROBOTCAR01_MCU_OS_LITE_SOURCES})
target_include_directories(robotcar01_mcu_os_lite PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(robotcar01_mcu_os_lite PRIVATE -Wall -Wextra -Werror)

# --- 编码器估计库（004）---
add_library(robotcar01_encoder STATIC ${ROBOTCAR01_ENCODER_SOURCES})
target_include_directories(robotcar01_encoder PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(robotcar01_encoder PRIVATE -Wall -Wextra -Werror)
target_link_libraries(robotcar01_encoder PUBLIC robotcar01_mcu_os_lite)

# --- 差速运动学库（005）：依赖方向 kinematics → chassis + encoder（无环） ---
add_library(robotcar01_kinematics STATIC ${ROBOTCAR01_KINEMATICS_SOURCES})
target_include_directories(robotcar01_kinematics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(robotcar01_kinematics PRIVATE -Wall -Wextra -Werror)
target_link_libraries(robotcar01_kinematics PUBLIC robotcar01_chassis robotcar01_encoder)

# --- 轮速控制与输出仲裁库（006）：依赖 chassis/encoder/mcu_os_lite，不依赖 protocol ---
add_library(robotcar01_control STATIC ${ROBOTCAR01_CONTROL_SOURCES})
target_include_directories(robotcar01_control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(robotcar01_control PRIVATE -Wall -Wextra -Werror)
target_link_libraries(robotcar01_control PUBLIC robotcar01_chassis robotcar01_encoder
                                             robotcar01_mcu_os_lite)

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
    # Iteration 006.5：TIM（编码器模式 TIM2/TIM3 + PWM 输出 TIM4），Cube FW_F4 V1.28.3
    ${ROBOTCAR01_VENDOR_DIR}/stm32f4-hal/Src/stm32f4xx_hal_tim.c
    ${ROBOTCAR01_VENDOR_DIR}/stm32f4-hal/Src/stm32f4xx_hal_tim_ex.c
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

  # --- 固件 target ---
  # 002.5：最小启动单元；006.5 T1：追加平台装配文件并链接 app 算法库（首个真实垂直切片）。
  add_executable(robotcar01_fw
    platform_stm32/main_stm32.cc
    platform_stm32/clock_init.cc
    platform_stm32/uart_console.cc
    platform_stm32/diag.cc
    # Iteration 006.5：时基/编码器/驱动/命令/装配（首个真实垂直切片）
    platform_stm32/clock_source.cc
    platform_stm32/encoder_input.cc
    platform_stm32/motor_pwm.cc
    platform_stm32/command_console.cc
    platform_stm32/board_app.cc
    ${ROBOTCAR01_VENDOR_DIR}/cmsis-device-f4/Source/Templates/gcc/startup_stm32f407xx.s
  )
  target_include_directories(robotcar01_fw PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/platform_stm32/inc
    ${CMAKE_CURRENT_SOURCE_DIR}
  )
  target_compile_definitions(robotcar01_fw PRIVATE STM32F407xx USE_HAL_DRIVER)
  target_compile_options(robotcar01_fw PRIVATE -Wall -Wextra -Werror)
  # 006.5：fw 侧的真实装配 = 006 的轮速控制/仲裁 + 004 的估计器 + 002 的调度器
  target_link_libraries(robotcar01_fw PRIVATE robotcar01_hal
                                                robotcar01_control
                                                robotcar01_kinematics
                                                robotcar01_encoder
                                                robotcar01_chassis
                                                robotcar01_mcu_os_lite)
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
  # --- 协议库（003）：host only（D-003-8：协议帧不进 fw，008 接线启用） ---
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
