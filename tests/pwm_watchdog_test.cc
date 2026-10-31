// PwmWatchdog 判定测试（Iteration 006.5，INV-006.5-6）
//
// 覆盖纯判定逻辑（ISR 内的断电动作本身在 motor_pwm.cc，依赖 HAL 无法 host 测）。
// 重点：uint32 毫秒回绕（49.7 天 ≈ 43 亿 ms，长时间运行必然跨越）不得导致误判。

#include <gtest/gtest.h>

#include <cstdint>

#include "platform_stm32/pwm_watchdog.h"

namespace {

using robotcar01::platform::PwmWatchdogExpired;
using robotcar01::platform::PwmWatchdogShouldTrip;

TEST(PwmWatchdogTest, DisabledNeverExpires) {
  EXPECT_FALSE(PwmWatchdogExpired(1'000'000U, 0U, 0U));
  EXPECT_FALSE(PwmWatchdogShouldTrip(true, true, 1'000'000U, 0U, 0U));
}

TEST(PwmWatchdogTest, FreshApplyDoesNotTrip) {
  EXPECT_FALSE(PwmWatchdogExpired(10'050U, 10'000U, 50U));
  EXPECT_FALSE(PwmWatchdogShouldTrip(true, true, 10'050U, 10'000U, 50U));
}

// 边界必须是严格大于：恰好等于阈值说明主循环仍在周期内刷新，不能误断电。
TEST(PwmWatchdogTest, BoundaryIsStrictlyGreater) {
  EXPECT_FALSE(PwmWatchdogExpired(10'050U, 10'000U, 50U));
  EXPECT_TRUE(PwmWatchdogExpired(10'051U, 10'000U, 50U));
}

TEST(PwmWatchdogTest, UnarmedDoesNotTrip) {
  EXPECT_TRUE(PwmWatchdogExpired(1000U, 0U, 50U));
  EXPECT_FALSE(PwmWatchdogShouldTrip(false, true, 1000U, 0U, 50U));
}

TEST(PwmWatchdogTest, DisabledSwitchDoesNotTrip) {
  EXPECT_FALSE(PwmWatchdogShouldTrip(true, false, 1000U, 0U, 50U));
}

// 回绕：last=0xFFFFFFF0、now=0x10 ⇒ 差 32 ms（不是 4e9），不应误判超时。
TEST(PwmWatchdogTest, WraparoundWithinTimeoutDoesNotTrip) {
  constexpr uint32_t last = 0xFFFFFFF0U;
  constexpr uint32_t now = 0x00000010U;  // 32 ms 后
  EXPECT_FALSE(PwmWatchdogExpired(now, last, 50U));
  EXPECT_TRUE(PwmWatchdogExpired(now, last, 20U));  // 同一差值，阈值更小则会超时
}

// 回绕后长时间不刷新：仍应判定超时。
TEST(PwmWatchdogTest, WraparoundLongStallTrips) {
  constexpr uint32_t last = 0xFFFFF000U;
  constexpr uint32_t now = 0x00001000U;  // 0x2000 = 8192 ms 后
  EXPECT_TRUE(PwmWatchdogExpired(now, last, 50U));
}

// 与真实配置一致：50 ms 阈值下，5 ms 控制周期 / 1-2 ms 分帧打印都不会触发。
TEST(PwmWatchdogTest, RealisticRefreshPatternNeverTrips) {
  uint32_t last_ok = 0U;
  for (uint32_t t = 0U; t < 100'000U; t += 5U) {  // 每 5 ms 一次 Apply，持续 100 s
    EXPECT_FALSE(PwmWatchdogExpired(t, last_ok, 50U)) << "t=" << t;
    last_ok = t;
  }
}

}  // namespace
