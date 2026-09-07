// walking skeleton 端到端测试（Iteration 001）
//
// 四类场景经 RunMotionSliceOnce + OutputSink 断言：
// 有效前进、有效原地旋转、无效零输出（force_disable=true）、安全禁止零输出。

#include <gtest/gtest.h>

#include "app/chassis/chassis_config.h"
#include "app/chassis/chassis_types.h"
#include "app/chassis/walking_skeleton.h"
#include "host_fakes/fake_output.h"

namespace robotcar01::chassis {
namespace {

constexpr float kEps = 1e-6f;

RobotCarConfig TestConfig() {
  RobotCarConfig config;
  config.track_width_m = 0.5f;
  config.max_body_speed_mps = 1.0f;
  config.max_yaw_rate_radps = 2.0f;
  config.max_wheel_speed_mps = 0.8f;
  return config;
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

TEST(WalkingSkeletonTest, ValidForwardEndToEnd) {
  const RobotCarConfig config = TestConfig();
  host_fakes::OutputSink sink;

  MotionCommand command;
  command.v_mps = 0.5f;
  command.omega_radps = 0.0f;
  command.seq = 1;
  command.valid = true;

  const WheelSpeedRequest request = RunMotionSliceOnce(config, command, Allowed());
  sink.Write(request);

  ASSERT_EQ(sink.writes().size(), 1u);
  const WheelSpeedRequest& recorded = sink.writes().front();
  EXPECT_FALSE(recorded.force_disable);
  EXPECT_NEAR(recorded.left_mps, 0.5f, kEps);
  EXPECT_NEAR(recorded.right_mps, 0.5f, kEps);
  EXPECT_EQ(recorded.limit_reason, LimitReason::kNone);
}

TEST(WalkingSkeletonTest, ValidPivotTurnEndToEnd) {
  const RobotCarConfig config = TestConfig();
  host_fakes::OutputSink sink;

  MotionCommand command;
  command.v_mps = 0.0f;
  command.omega_radps = 1.0f;
  command.seq = 2;
  command.valid = true;

  const WheelSpeedRequest request = RunMotionSliceOnce(config, command, Allowed());
  sink.Write(request);

  ASSERT_EQ(sink.writes().size(), 1u);
  const WheelSpeedRequest& recorded = sink.writes().front();
  EXPECT_FALSE(recorded.force_disable);
  EXPECT_LT(recorded.left_mps, 0.0f);    // 原地转向允许异号
  EXPECT_GT(recorded.right_mps, 0.0f);
  EXPECT_NEAR(recorded.left_mps, -recorded.right_mps, kEps);
}

TEST(WalkingSkeletonTest, InvalidCommandZeroOutputAndForceDisable) {
  const RobotCarConfig config = TestConfig();
  host_fakes::OutputSink sink;

  MotionCommand command;
  command.v_mps = 0.5f;
  command.omega_radps = 0.0f;
  command.seq = 3;
  command.valid = false;  // 无效命令，即使 allow_motion=true

  const WheelSpeedRequest request = RunMotionSliceOnce(config, command, Allowed());
  sink.Write(request);

  ASSERT_EQ(sink.writes().size(), 1u);
  const WheelSpeedRequest& recorded = sink.writes().front();
  EXPECT_TRUE(recorded.force_disable);
  EXPECT_NEAR(recorded.left_mps, 0.0f, kEps);
  EXPECT_NEAR(recorded.right_mps, 0.0f, kEps);
  EXPECT_EQ(recorded.limit_reason, LimitReason::kInvalid);
}

TEST(WalkingSkeletonTest, SafetyBlockedZeroOutputAndForceDisable) {
  const RobotCarConfig config = TestConfig();
  host_fakes::OutputSink sink;

  MotionCommand command;
  command.v_mps = 0.5f;
  command.omega_radps = 0.0f;
  command.seq = 4;
  command.valid = true;

  const WheelSpeedRequest request = RunMotionSliceOnce(config, command, Blocked());
  sink.Write(request);

  ASSERT_EQ(sink.writes().size(), 1u);
  const WheelSpeedRequest& recorded = sink.writes().front();
  EXPECT_TRUE(recorded.force_disable);
  EXPECT_NEAR(recorded.left_mps, 0.0f, kEps);
  EXPECT_NEAR(recorded.right_mps, 0.0f, kEps);
  EXPECT_EQ(recorded.limit_reason, LimitReason::kSafety);
}

TEST(WalkingSkeletonTest, OverLimitCommandReflectedInRequest) {
  const RobotCarConfig config = TestConfig();
  host_fakes::OutputSink sink;

  MotionCommand command;
  command.v_mps = 0.4f;
  command.omega_radps = 2.4f;  // raw_right=1.0 > max_wheel=0.8
  command.seq = 5;
  command.valid = true;

  const WheelSpeedRequest request = RunMotionSliceOnce(config, command, Allowed());
  sink.Write(request);

  ASSERT_EQ(sink.writes().size(), 1u);
  const WheelSpeedRequest& recorded = sink.writes().front();
  EXPECT_FALSE(recorded.force_disable);
  EXPECT_NEAR(recorded.right_mps, 0.8f, kEps);
  EXPECT_NEAR(recorded.left_mps, -0.16f, kEps);
  EXPECT_EQ(recorded.limit_reason, LimitReason::kWheelSpeed);
}

}  // namespace
}  // namespace robotcar01::chassis
