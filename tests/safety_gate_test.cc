// safety_gate 单元测试（Iteration 001 walking skeleton）
//
// 覆盖 valid×放行 / invalid×放行 / valid×禁止 / invalid×禁止 四组合，
// 钉死组合规则：
//   force_disable = !allow_motion || !valid
//   reason = target.limit_reason | (allow_motion ? kNone : kSafety)

#include <gtest/gtest.h>

#include "app/chassis/chassis_types.h"
#include "app/chassis/safety_gate.h"

namespace robotcar01::chassis {
namespace {

constexpr float kEps = 1e-6f;

WheelTarget ValidTarget(float left_mps = 0.4f, float right_mps = 0.4f) {
  WheelTarget target;
  target.raw_left_mps = left_mps;
  target.raw_right_mps = right_mps;
  target.left_mps = left_mps;
  target.right_mps = right_mps;
  target.scale = 1.0f;
  target.limit_reason = LimitReason::kNone;
  target.valid = true;
  return target;
}

WheelTarget InvalidTarget() {
  WheelTarget target = ValidTarget();
  target.left_mps = 0.0f;
  target.right_mps = 0.0f;
  target.limit_reason = LimitReason::kInvalid;
  target.valid = false;
  return target;
}

SafetyStatus Allowed() {
  SafetyStatus safety;
  safety.allow_motion = true;
  return safety;
}

SafetyStatus Blocked() {
  SafetyStatus safety;
  safety.allow_motion = false;
  return safety;
}

TEST(SafetyGateTest, ValidAndAllowedPassesThrough) {
  const WheelSpeedRequest request = RunSafetyGate(ValidTarget(), Allowed());

  EXPECT_FALSE(request.force_disable);
  EXPECT_NEAR(request.left_mps, 0.4f, kEps);
  EXPECT_NEAR(request.right_mps, 0.4f, kEps);
  EXPECT_EQ(request.limit_reason, LimitReason::kNone);
}

TEST(SafetyGateTest, InvalidAndAllowedForcesDisableWithInvalidReason) {
  const WheelSpeedRequest request = RunSafetyGate(InvalidTarget(), Allowed());

  EXPECT_TRUE(request.force_disable);
  EXPECT_NEAR(request.left_mps, 0.0f, kEps);
  EXPECT_NEAR(request.right_mps, 0.0f, kEps);
  EXPECT_EQ(request.limit_reason, LimitReason::kInvalid);
}

TEST(SafetyGateTest, ValidAndBlockedForcesDisableWithSafetyReason) {
  const WheelSpeedRequest request = RunSafetyGate(ValidTarget(), Blocked());

  EXPECT_TRUE(request.force_disable);
  EXPECT_NEAR(request.left_mps, 0.0f, kEps);
  EXPECT_NEAR(request.right_mps, 0.0f, kEps);
  EXPECT_EQ(request.limit_reason, LimitReason::kSafety);
}

TEST(SafetyGateTest, InvalidAndBlockedCombinesReasons) {
  const WheelSpeedRequest request = RunSafetyGate(InvalidTarget(), Blocked());

  EXPECT_TRUE(request.force_disable);
  EXPECT_NEAR(request.left_mps, 0.0f, kEps);
  EXPECT_NEAR(request.right_mps, 0.0f, kEps);
  EXPECT_EQ(request.limit_reason,
            LimitReason::kInvalid | LimitReason::kSafety);
}

TEST(SafetyGateTest, LimitedButValidTargetReasonPassesThroughWhenAllowed) {
  WheelTarget limited = ValidTarget(0.3f, 0.3f);
  limited.raw_left_mps = 0.6f;
  limited.raw_right_mps = 0.6f;
  limited.scale = 0.5f;
  limited.limit_reason = LimitReason::kWheelSpeed;

  const WheelSpeedRequest request = RunSafetyGate(limited, Allowed());

  EXPECT_FALSE(request.force_disable);
  EXPECT_EQ(request.limit_reason, LimitReason::kWheelSpeed);
}

TEST(SafetyGateTest, LimitedValidTargetBlockedReasonIsSafetyOrLimit) {
  WheelTarget limited = ValidTarget();
  limited.limit_reason = LimitReason::kWheelSpeed;

  const WheelSpeedRequest request = RunSafetyGate(limited, Blocked());

  EXPECT_TRUE(request.force_disable);
  EXPECT_EQ(request.limit_reason,
            LimitReason::kWheelSpeed | LimitReason::kSafety);
}

}  // namespace
}  // namespace robotcar01::chassis
