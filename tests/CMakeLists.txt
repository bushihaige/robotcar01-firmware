find_package(GTest CONFIG REQUIRED)

add_executable(robotcar01_tests
  smoke_test.cc
  kinematics_test.cc
  safety_gate_test.cc
  walking_skeleton_test.cc
)

target_compile_options(robotcar01_tests PRIVATE -Wall -Wextra -Werror)
target_link_libraries(robotcar01_tests PRIVATE
  GTest::gtest_main GTest::gtest
  robotcar01_chassis
)
target_include_directories(robotcar01_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../host_fakes)

include(GoogleTest)
gtest_discover_tests(robotcar01_tests)
