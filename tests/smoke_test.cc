// RobotCar01 host 单元测试冒烟用例（Iteration 000）
//
// 目的：验证 GoogleTest 接入、ctest 发现与命令行退出码可用。
// 业务断言从 Iteration 001 起加入。

#include <gtest/gtest.h>

namespace robotcar01 {
namespace {

TEST(SmokeTest, Sanity) {
  EXPECT_EQ(1 + 1, 2);
  EXPECT_TRUE(true);
}

}  // namespace
}  // namespace robotcar01
