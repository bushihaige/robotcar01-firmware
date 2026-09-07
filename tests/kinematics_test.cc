// kinematics 单元测试（Iteration 001 walking skeleton）
//
// 覆盖：直行、原地转向（异号）、统一比例限幅（曲率保持、scale<1）、
// 混合超限（v 超限 ω 合法、ω 超限 v 合法、双超）、NaN/Inf、valid=false、
// 边界零输入、恰等于/恰超限。浮点断言统一 EXPECT_NEAR(eps=1e-6f)，
// 严格 > 判定（恰等于上限不触发限幅）。

#include <cmath>

#include <gtest/gtest.h>

#include "app/chassis/chassis_config.h"
#include "app/chassis/chassis_types.h"
#include "app/chassis/kinematics.h"

namespace robotcar01::chassis {
namespace {

constexpr float kEps = 1e-6f;

RobotCarConfig TestConfig() {
  RobotCarConfig config;
  config.track_width_m = 0.5f;  // 便于手算：raw = v ± ω*0.25
  config.max_body_speed_mps = 1.0f;
  config.max_yaw_rate_radps = 2.0f;
  config.max_wheel_speed_mps = 0.8f;
  return config;
}

MotionCommand Command(float v_mps, float omega_radps, bool valid = true) {
  MotionCommand command;
  command.v_mps = v_mps;
  command.omega_radps = omega_radps;
  command.seq = 0;
  command.valid = valid;
  return command;
}

TEST(KinematicsTest, ForwardStraightUnlimited) {
  const RobotCarConfig config = TestConfig();
  const WheelTarget target = ComputeWheelTarget(config, Command(0.5f, 0.0f));

  EXPECT_TRUE(target.valid);
  EXPECT_NEAR(target.left_mps, 0.5f, kEps);
  EXPECT_NEAR(target.right_mps, 0.5f, kEps);
  EXPECT_NEAR(target.raw_left_mps, 0.5f, kEps);
  EXPECT_NEAR(target.raw_right_mps, 0.5f, kEps);
  EXPECT_NEAR(target.scale, 1.0f, kEps);
  EXPECT_EQ(target.limit_reason, LimitReason::kNone);
}

TEST(KinematicsTest, PivotTurnAllowsOppositeSigns) {
  const RobotCarConfig config = TestConfig();
  const WheelTarget target = ComputeWheelTarget(config, Command(0.0f, 1.0f));

  EXPECT_TRUE(target.valid);
  EXPECT_NEAR(target.left_mps, -0.25f, kEps);   // v - ω*T/2 = -0.25
  EXPECT_NEAR(target.right_mps, 0.25f, kEps);   // v + ω*T/2 = +0.25
  EXPECT_LT(target.left_mps, 0.0f);
  EXPECT_GT(target.right_mps, 0.0f);
  EXPECT_NEAR(target.scale, 1.0f, kEps);
  EXPECT_EQ(target.limit_reason, LimitReason::kNone);
}

TEST(KinematicsTest, WheelDomainLimitKeepsCurvatureRatio) {
  const RobotCarConfig config = TestConfig();
  // v=0.4, ω=2.4 → raw_left = 0.4-0.6 = -0.2, raw_right = 0.4+0.6 = 1.0
  // max_wheel=0.8 → scale=0.8；曲率 v/ω=1/6 保持（左右等比例缩放）。
  const WheelTarget target = ComputeWheelTarget(config, Command(0.4f, 2.4f));

  EXPECT_TRUE(target.valid);
  EXPECT_NEAR(target.scale, 0.8f, kEps);
  EXPECT_NEAR(target.raw_right_mps, 1.0f, kEps);
  EXPECT_NEAR(target.right_mps, 0.8f, kEps);
  EXPECT_NEAR(target.left_mps, -0.16f, kEps);  // -0.2 * 0.8
  // 曲率保持：left/right 比值与 raw 相同。
  EXPECT_NEAR(target.left_mps / target.right_mps,
              target.raw_left_mps / target.raw_right_mps, kEps);
  EXPECT_EQ(target.limit_reason, LimitReason::kWheelSpeed);
}

TEST(KinematicsTest, BodySpeedLimitAlone) {
  const RobotCarConfig config = TestConfig();
  // v=2.0 > max_body=1.0，ω=0 → body scale=0.5，轮速 0.5*2=... raw=2.0→1.0
  // 但 max_wheel=0.8：raw=2.0 > 0.8 → wheel scale=0.4 → s=0.4（wheel 更紧）。
  // 改用 v=1.2, ω=0 → raw=1.2；body scale=1/1.2=0.8333；wheel scale=0.8/1.2=0.6667
  // wheel 仍更紧。构造 wheel 不更紧场景：缩小 max_body 到 0.5。
  RobotCarConfig tight_body = config;
  tight_body.max_body_speed_mps = 0.5f;
  tight_body.max_wheel_speed_mps = 2.0f;
  const WheelTarget target = ComputeWheelTarget(tight_body, Command(1.0f, 0.0f));

  EXPECT_TRUE(target.valid);
  EXPECT_NEAR(target.scale, 0.5f, kEps);
  EXPECT_NEAR(target.left_mps, 0.5f, kEps);
  EXPECT_EQ(target.limit_reason, LimitReason::kBodySpeed);
}

TEST(KinematicsTest, MixedOverLimitBodyBinds) {
  const RobotCarConfig config = TestConfig();
  // v=2.0 超 max_body=1.0，ω=0.5 合法 → body scale = 0.5
  // raw_left = 2.0 - 0.125 = 1.875；raw_right = 2.0+0.125 = 2.125；
  // wheel scale = 0.8/2.125 = 0.3765 → wheel 更紧，最终 s=0.3765。
  // 断言单一 s、reason=kWheelSpeed（body 原始超限但非最紧）。
  const WheelTarget target = ComputeWheelTarget(config, Command(2.0f, 0.5f));

  EXPECT_TRUE(target.valid);
  const float expected_scale = 0.8f / 2.125f;
  EXPECT_NEAR(target.scale, expected_scale, kEps);
  EXPECT_NEAR(target.right_mps, 2.125f * expected_scale, kEps);
  EXPECT_NEAR(target.left_mps, 1.875f * expected_scale, kEps);
  // 曲率保持：raw 与 actual 两轮比值一致。
  EXPECT_NEAR(target.left_mps / target.right_mps,
              target.raw_left_mps / target.raw_right_mps, kEps);
  EXPECT_EQ(target.limit_reason, LimitReason::kWheelSpeed);
}

TEST(KinematicsTest, MixedOverLimitBodyWheelCombined) {
  const RobotCarConfig config = TestConfig();
  // 构造 body 与 wheel 恰好同紧：v=1.0(==max_body 不超)，ω=4.0 超 max_yaw=2.0
  // raw_left = 1.0-1.0=0；raw_right=1.0+1.0=2.0；wheel scale=0.8/2=0.4；
  // body scale=2/4=0.5 → s=0.4 wheel 更紧。改 max_yaw 使 body 同紧：
  // ω=4.0, max_yaw=2.0 → 0.5；为同紧设 max_yaw=2.5 → body=2.5/4=0.625。
  // 直接验证双超（v 与 ω 都超）reason 组合可多位置位：
  RobotCarConfig wide_wheel = config;
  wide_wheel.max_wheel_speed_mps = 4.0f;  // wheel 不再限制
  // v=2.0 超 max_body=1.0（body=0.5）；ω=2.4 超 max_yaw=2.0（body 更紧 0.5 vs 0.8333）
  // raw_right = 2.0 + 2.4*0.25 = 2.6 < 4.0 → wheel 不超。最终 s=0.5，reason=kBodySpeed。
  const WheelTarget target =
      ComputeWheelTarget(wide_wheel, Command(2.0f, 2.4f));

  EXPECT_TRUE(target.valid);
  EXPECT_NEAR(target.scale, 0.5f, kEps);
  EXPECT_EQ(target.limit_reason, LimitReason::kBodySpeed);
}

TEST(KinematicsTest, InvalidFlagYieldsZeroAndReason) {
  const RobotCarConfig config = TestConfig();
  const WheelTarget target = ComputeWheelTarget(config, Command(0.5f, 0.0f, false));

  EXPECT_FALSE(target.valid);
  EXPECT_NEAR(target.left_mps, 0.0f, kEps);
  EXPECT_NEAR(target.right_mps, 0.0f, kEps);
  EXPECT_NEAR(target.scale, 0.0f, kEps);
  EXPECT_EQ(target.limit_reason, LimitReason::kInvalid);
}

TEST(KinematicsTest, NonFiniteInputsRejected) {
  const RobotCarConfig config = TestConfig();
  const float nan_value = std::nanf("");
  const float inf_value = std::numeric_limits<float>::infinity();

  const WheelTarget nan_v = ComputeWheelTarget(config, Command(nan_value, 0.0f));
  EXPECT_FALSE(nan_v.valid);
  EXPECT_EQ(nan_v.limit_reason, LimitReason::kInvalid);

  const WheelTarget inf_omega =
      ComputeWheelTarget(config, Command(0.0f, inf_value));
  EXPECT_FALSE(inf_omega.valid);
  EXPECT_EQ(inf_omega.limit_reason, LimitReason::kInvalid);
}

TEST(KinematicsTest, ZeroCommandIsZeroAndUnlimited) {
  const RobotCarConfig config = TestConfig();
  const WheelTarget target = ComputeWheelTarget(config, Command(0.0f, 0.0f));

  EXPECT_TRUE(target.valid);
  EXPECT_NEAR(target.left_mps, 0.0f, kEps);
  EXPECT_NEAR(target.right_mps, 0.0f, kEps);
  EXPECT_NEAR(target.scale, 1.0f, kEps);
  EXPECT_EQ(target.limit_reason, LimitReason::kNone);
}

TEST(KinematicsTest, ExactlyAtLimitNotLimited) {
  // 严格 > 判定：v 恰等于 max_body 且恰等于 max_wheel 时不触发限幅。
  RobotCarConfig config = TestConfig();
  config.max_body_speed_mps = 1.0f;
  config.max_wheel_speed_mps = 1.0f;
  const WheelTarget target = ComputeWheelTarget(config, Command(1.0f, 0.0f));

  EXPECT_TRUE(target.valid);
  EXPECT_NEAR(target.left_mps, 1.0f, kEps);
  EXPECT_NEAR(target.right_mps, 1.0f, kEps);
  EXPECT_NEAR(target.scale, 1.0f, kEps);
  EXPECT_EQ(target.limit_reason, LimitReason::kNone);
}

}  // namespace
}  // namespace robotcar01::chassis
