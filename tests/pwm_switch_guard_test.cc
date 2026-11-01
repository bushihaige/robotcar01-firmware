// PwmSwitchGuard 行为测试（Iteration 006.5）
//
// 目的：INV-006.5-3（方向变化必须经过 ≥1 个仲裁周期的零输出）此前只靠人工推演，
// 现在由本测试覆盖。任何"为了省一个周期而直接换向"的改动都会在这里变红。

#include <gtest/gtest.h>

#include <random>

#include "app/control/motor_output.h"
#include "platform_stm32/pwm_switch_guard.h"

namespace {

using robotcar01::control::MotorDirection;
using robotcar01::platform::PwmSwitchGuard;
using robotcar01::platform::SwitchDecision;

constexpr MotorDirection kForward = MotorDirection::kForward;
constexpr MotorDirection kReverse = MotorDirection::kReverse;

// 首次 drive（此前从未施加）必须立即施加，否则起步会白等一个周期。
TEST(PwmSwitchGuardTest, FirstDriveAppliesImmediately) {
  PwmSwitchGuard guard;
  EXPECT_EQ(guard.OnDriveRequest(kForward), SwitchDecision::kApplyNow);
  EXPECT_EQ(guard.last_direction(), kForward);
  EXPECT_EQ(guard.zero_windows(), 0U);
}

// 同方向重复请求不应触发零窗口（否则每次控制周期都会丢一个周期的输出）。
TEST(PwmSwitchGuardTest, SameDirectionNeverTriggersWindow) {
  PwmSwitchGuard guard;
  EXPECT_EQ(guard.OnDriveRequest(kForward), SwitchDecision::kApplyNow);
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(guard.OnDriveRequest(kForward), SwitchDecision::kApplyNow);
  }
  EXPECT_EQ(guard.zero_windows(), 0U);
}

// 换向必须"先零一个周期、再施加"。
TEST(PwmSwitchGuardTest, DirectionReversalNeedsOneZeroWindow) {
  PwmSwitchGuard guard;
  EXPECT_EQ(guard.OnDriveRequest(kForward), SwitchDecision::kApplyNow);
  EXPECT_EQ(guard.OnDriveRequest(kReverse), SwitchDecision::kZeroOnly);
  EXPECT_TRUE(guard.pending());
  EXPECT_EQ(guard.pending_direction(), kReverse);
  EXPECT_EQ(guard.OnDriveRequest(kReverse), SwitchDecision::kApplyNow);
  EXPECT_FALSE(guard.pending());
  EXPECT_EQ(guard.last_direction(), kReverse);
  EXPECT_EQ(guard.zero_windows(), 1U);
}

// 零窗口内方向又变：继续保持零，且不会提前施加。
TEST(PwmSwitchGuardTest, RapidReversalKeepsZeroUntilStable) {
  PwmSwitchGuard guard;
  EXPECT_EQ(guard.OnDriveRequest(kForward), SwitchDecision::kApplyNow);
  EXPECT_EQ(guard.OnDriveRequest(kReverse), SwitchDecision::kZeroOnly);
  EXPECT_EQ(guard.OnDriveRequest(kForward), SwitchDecision::kZeroOnly);  // 又变回去
  EXPECT_EQ(guard.OnDriveRequest(kForward), SwitchDecision::kApplyNow);
  EXPECT_EQ(guard.last_direction(), kForward);
  EXPECT_EQ(guard.zero_windows(), 2U);
}

// coast/brake（该侧已归零）之后，再次 drive 必须立即施加 —— 此时物理上已是零，不需要再等。
TEST(PwmSwitchGuardTest, ZeroOutputClearsDirectionMemory) {
  PwmSwitchGuard guard;
  EXPECT_EQ(guard.OnDriveRequest(kForward), SwitchDecision::kApplyNow);
  guard.OnZeroOutput();  // 平台把它写成 coast/brake
  EXPECT_EQ(guard.last_direction(), MotorDirection::kNone);
  EXPECT_FALSE(guard.pending());
  EXPECT_EQ(guard.OnDriveRequest(kReverse), SwitchDecision::kApplyNow);
  EXPECT_EQ(guard.zero_windows(), 0U);
}

// 停在零窗口中途收到 coast：pending 必须被清掉，不能"记忆"到下一次 drive。
TEST(PwmSwitchGuardTest, CoastDuringPendingWindowResetsGuard) {
  PwmSwitchGuard guard;
  EXPECT_EQ(guard.OnDriveRequest(kForward), SwitchDecision::kApplyNow);
  EXPECT_EQ(guard.OnDriveRequest(kReverse), SwitchDecision::kZeroOnly);
  guard.OnZeroOutput();
  EXPECT_FALSE(guard.pending());
  EXPECT_EQ(guard.OnDriveRequest(kReverse), SwitchDecision::kApplyNow);
}

// 属性测试：任意随机方向序列下，"每一次真正换向的施加之前必然出现过一次零窗口"。
// 注意：换向的**请求**会先得到 kZeroOnly，下一次同方向请求才允许施加 —— 两次调用之间
// 可能插入其他方向的请求，因此用 zero_owed_for 跟踪"零窗口已为哪个方向建立"。
TEST(PwmSwitchGuardTest, EveryDirectionChangeIsPrecededByZeroWindow) {
  PwmSwitchGuard guard;
  std::mt19937 rng(20261025U);
  MotorDirection applied = MotorDirection::kNone;      // 最后一次真正施加的方向
  MotorDirection zero_owed_for = MotorDirection::kNone;  // 零窗口已建立、等待施加的方向

  for (int step = 0; step < 5000; ++step) {
    const MotorDirection requested = (rng() % 2U == 0U) ? kForward : kReverse;
    const SwitchDecision decision = guard.OnDriveRequest(requested);

    if (decision == SwitchDecision::kZeroOnly) {
      // 零窗口期间该侧必须处于"无方向记忆"状态，且记录待施加方向
      EXPECT_EQ(guard.last_direction(), MotorDirection::kNone) << "step=" << step;
      zero_owed_for = guard.pending_direction();
      EXPECT_EQ(zero_owed_for, requested) << "step=" << step;
      continue;
    }

    // kApplyNow：若是换向施加，则必须先有一个针对该方向的零窗口
    const bool is_direction_change = (applied != MotorDirection::kNone) && (requested != applied);
    if (is_direction_change) {
      EXPECT_EQ(zero_owed_for, requested)
          << "step=" << step << "：换向施加前没有该方向的零窗口";
    }
    EXPECT_EQ(guard.last_direction(), requested) << "step=" << step;
    applied = requested;
    zero_owed_for = MotorDirection::kNone;
  }
}

}  // namespace
