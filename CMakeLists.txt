cmake_minimum_required(VERSION 3.24)
project(robotcar01 LANGUAGES CXX)

# RobotCar01 底盘固件
#
# 双 target 策略：
#   - host：本机 AppleClang + GoogleTest，用于单元测试（默认构建）；
#   - fw  ：arm-none-eabi-gcc 交叉编译，产出固件（工具链延后至 Iteration 008，
#           默认 OFF，通过 ROBOTCAR01_BUILD_FW=ON + 工具链文件启用）。
#
# 平台无关业务（app/chassis/ 等）以静态库 robotcar01_chassis 编入 host；
# fw 分支共享同一源列表的机制由 Iteration 002.5 决定（D-001-3），
# 001 不引入未验证的交叉构建改动。

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ROBOTCAR01_BUILD_FW "Build firmware cross target (needs arm-none-eabi-gcc)" OFF)

if(ROBOTCAR01_BUILD_FW)
  # --- 固件 target（预留；Iteration 008 接入 CubeMX/HAL 后扩展） ---
  add_executable(robotcar01_fw
    app/main.cc
  )
  target_compile_definitions(robotcar01_fw PRIVATE STM32F407xx)
  target_compile_options(robotcar01_fw PRIVATE -Wall -Wextra)
else()
  # --- 平台无关业务库（host 编译，供单测链接；fw 分支源共享待 002.5） ---
  add_library(robotcar01_chassis STATIC
    app/chassis/kinematics.cc
    app/chassis/safety_gate.cc
    app/chassis/walking_skeleton.cc
  )
  target_include_directories(robotcar01_chassis PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_options(robotcar01_chassis PRIVATE -Wall -Wextra -Werror)

  # --- host 单元测试 target ---
  enable_testing()
  add_subdirectory(tests)
endif()
