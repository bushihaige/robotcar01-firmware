# host 字节流调试工具（Iteration 003 A7）
#
# 用途：以真实字节流（stdin/stdout）完成"命令帧 → 反馈帧"往返，用于本地开发调试与脚本化验证。
# 明确边界：本工具是 host 证据，**不构成 USB 真机链路证据**（USB CDC 接线属 Iteration 008）。

add_executable(robotcar01_loopback loopback_main.cc)
target_compile_options(robotcar01_loopback PRIVATE -Wall -Wextra -Werror)
target_link_libraries(robotcar01_loopback PRIVATE robotcar01_protocol)

# A7 端到端用例：hex 行输入 → 确定性反馈帧流 + 可解析统计行；由 Python 校验帧自洽性。
find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
  add_test(NAME loopback_end_to_end
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/check_feedback_stream.py
            $<TARGET_FILE:robotcar01_loopback>)
endif()
