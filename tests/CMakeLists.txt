find_package(GTest CONFIG REQUIRED)

add_executable(robotcar01_tests
  smoke_test.cc
  kinematics_test.cc
  safety_gate_test.cc
  walking_skeleton_test.cc
  scheduler_test.cc
  health_monitor_test.cc
  protocol_frame_test.cc
  protocol_codec_test.cc
  byte_ring_test.cc
  command_manager_test.cc
  feedback_test.cc
  wheel_state_test.cc
  encoder_estimator_test.cc
  differential_kinematics_test.cc
)

target_compile_options(robotcar01_tests PRIVATE -Wall -Wextra -Werror)
# robotcar01_protocol 的 PUBLIC 依赖已传递 chassis 与 mcu_os_lite，无需重复列出（避免 ld 重复库告警）
target_link_libraries(robotcar01_tests PRIVATE
  GTest::gtest_main GTest::gtest
  robotcar01_protocol
  robotcar01_kinematics
)
target_include_directories(robotcar01_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../host_fakes)

include(GoogleTest)
gtest_discover_tests(robotcar01_tests)
