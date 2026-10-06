// 编码器估计器测试（Iteration 004）：A4 低速判零与累加、A6 停滞与急停、A7 反馈闭环；
// 含评审 S-5 要求的坏用例（计数抖动、整窗丢失、连续两窗无快照）
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "app/encoder/encoder_config.h"
#include "app/encoder/encoder_estimator.h"
#include "app/encoder/wheel_state.h"
#include "app/protocol/feedback.h"
#include "host_fakes/fake_encoder_source.h"

namespace robotcar01::encoder {
namespace {

EncoderConfig MakeConfig() {
  EncoderConfig config{};
  config.location = EncoderLocation::kMotorShaft;
  config.counts_per_encoder_rev = 44.0f;
  config.gear_ratio = 90.0f;
  config.wheel_radius_m = 0.05f;
  config.left.counter_bits = 16;
  config.right.counter_bits = 16;
  config.low_speed_zero_threshold_mps = 0.05f;
  config.max_estimation_delay_us = 20000;
  config.stall_windows_threshold = 8;
  config.low_speed_accumulate_counts = 5;
  config.max_wheel_speed_mps = 0.5f;
  return config;
}

constexpr uint32_t kWindowUs = 5000;

// A4：单窗口速度高于阈值时直接发布（不做累加）
TEST(EncoderEstimatorTest, HighSpeedPublishesSingleWindow) {
  EncoderEstimator estimator;
  ASSERT_TRUE(estimator.Init(MakeConfig()));
  host_fakes::FakeEncoderSource source;
  source.Init(MakeConfig());
  source.SetWheelSpeeds(0.5f, 0.5f);

  ASSERT_TRUE(estimator.OnSample(source.Step(kWindowUs), false));  // 建立基线
  ASSERT_TRUE(estimator.OnSample(source.Step(kWindowUs), false));
  const WheelState state = estimator.state();
  EXPECT_TRUE(state.left.valid);
  EXPECT_EQ(state.left.accumulated_windows, 1u);
  EXPECT_NEAR(state.left.speed_mps, 0.5f, 0.06f);
  EXPECT_NEAR(state.right.speed_mps, 0.5f, 0.06f);
}

// A4：低速时走多窗口累加路径，累加期间 valid=false 且置 kQualityAccumulating
TEST(EncoderEstimatorTest, LowSpeedAccumulatesThenPublishes) {
  EncoderEstimator estimator;
  ASSERT_TRUE(estimator.Init(MakeConfig()));
  host_fakes::FakeEncoderSource source;
  source.Init(MakeConfig());
  source.SetWheelSpeeds(0.07f, 0.07f);  // 单窗口 ≈4.4 counts < 5 阈值（发布值须高于判零阈值 0.05）

  ASSERT_TRUE(estimator.OnSample(source.Step(kWindowUs), false));  // 基线
  bool saw_accumulating = false;
  bool saw_publish = false;
  for (int i = 0; i < 6; ++i) {
    estimator.OnSample(source.Step(kWindowUs), false);
    const WheelState state = estimator.state();
    if (!state.left.valid && (state.left.quality & kQualityAccumulating) != 0u) {
      saw_accumulating = true;
    }
    if (state.left.valid && state.left.accumulated_windows > 1u) {
      saw_publish = true;
      EXPECT_NEAR(state.left.speed_mps, 0.07f, 0.02f);
      break;
    }
  }
  EXPECT_TRUE(saw_accumulating);
  EXPECT_TRUE(saw_publish);
}

// A4：累加到期未达阈值 ⇒ 判零 + kQualityLowSpeedTimeout
TEST(EncoderEstimatorTest, LowSpeedTimeoutZeroesAndFlags) {
  EncoderConfig config = MakeConfig();
  config.low_speed_accumulate_counts = 40;  // 高于单窗口可累加量 ⇒ 只能在到期时发布
  EncoderEstimator estimator;
  ASSERT_TRUE(estimator.Init(config));
  host_fakes::FakeEncoderSource source;
  source.Init(config);
  source.SetWheelSpeeds(0.03f, 0.03f);  // ≈1.9 counts/窗口；15 ms 内达不到 40 counts

  ASSERT_TRUE(estimator.OnSample(source.Step(kWindowUs), false));
  bool saw_timeout = false;
  for (int i = 0; i < 12; ++i) {
    estimator.OnSample(source.Step(kWindowUs), false);
    const WheelState state = estimator.state();
    if ((state.left.quality & kQualityLowSpeedTimeout) != 0u) {
      saw_timeout = true;
      EXPECT_EQ(state.left.speed_mps, 0.0f);  // 到期未达阈值 ⇒ 判零
      break;
    }
  }
  EXPECT_TRUE(saw_timeout);
  EXPECT_GE(estimator.stats().low_speed_timeouts, 1u);
}

// A6：motion_desired=true 且连续零增量 ⇒ 停滞候选；false ⇒ 不置位
TEST(EncoderEstimatorTest, StallCandidateRequiresMotionDesired) {
  EncoderEstimator estimator;
  ASSERT_TRUE(estimator.Init(MakeConfig()));
  host_fakes::FakeEncoderSource source;
  source.Init(MakeConfig());
  source.SetWheelSpeeds(0.0f, 0.0f);

  ASSERT_TRUE(estimator.OnSample(source.Step(kWindowUs), true));  // 基线
  for (int i = 0; i < 10; ++i) {
    estimator.OnSample(source.Step(kWindowUs), true);
  }
  EXPECT_NE(estimator.state().left.quality & kQualityStallCandidate, 0u);
  EXPECT_GE(estimator.stats().stall_candidates, 1u);

  EncoderEstimator idle;
  ASSERT_TRUE(idle.Init(MakeConfig()));
  idle.OnSample(source.Step(kWindowUs), false);
  for (int i = 0; i < 10; ++i) {
    idle.OnSample(source.Step(kWindowUs), false);
  }
  EXPECT_EQ(idle.state().left.quality & kQualityStallCandidate, 0u);
  EXPECT_NE(idle.state().left.quality & kQualityNoCountChange, 0u);
}

// A5/坏用例：dt 边界（==max 接受；>max 拒绝）
TEST(EncoderEstimatorTest, TimestampBoundaries) {
  EncoderEstimator estimator;
  ASSERT_TRUE(estimator.Init(MakeConfig()));
  host_fakes::FakeEncoderSource source;
  source.Init(MakeConfig());
  source.SetWheelSpeeds(0.5f, 0.5f);

  ASSERT_TRUE(estimator.OnSample(source.Step(kWindowUs), false));
  // dt = 20000（== max）应被接受
  ASSERT_TRUE(estimator.OnSample(source.Step(20000), false));
  EXPECT_EQ(estimator.stats().timestamp_invalid, 0u);
  // dt = 20001 应被拒绝并计 timestamp_invalid
  estimator.OnSample(source.Step(20001), false);
  EXPECT_GE(estimator.stats().timestamp_invalid, 1u);
}

// 坏用例（评审 S-5）：连续两窗无有效快照 ⇒ 不产出新速度、计数可诊断
TEST(EncoderEstimatorTest, TwoInvalidSnapshotsAreRejectedAndCounted) {
  EncoderEstimator estimator;
  ASSERT_TRUE(estimator.Init(MakeConfig()));
  host_fakes::FakeEncoderSource source;
  source.Init(MakeConfig());
  source.SetWheelSpeeds(0.5f, 0.5f);
  ASSERT_TRUE(estimator.OnSample(source.Step(kWindowUs), false));
  ASSERT_TRUE(estimator.OnSample(source.Step(kWindowUs), false));
  const float speed_before = estimator.state().left.speed_mps;

  EncoderSnapshot invalid{};
  invalid.valid = false;
  EXPECT_FALSE(estimator.OnSample(invalid, false));
  EXPECT_FALSE(estimator.OnSample(invalid, false));
  EXPECT_EQ(estimator.stats().snapshot_invalid, 2u);
  EXPECT_FLOAT_EQ(estimator.state().left.speed_mps, speed_before);  // 保留上次值
  EXPECT_GT(estimator.state().samples_submitted, estimator.state().samples_accepted);
}

// 坏用例（评审 S-5）：整窗丢失（OnSample 被跳过 ⇒ dt 加倍）仍可被接受
TEST(EncoderEstimatorTest, SkippedWindowIsAcceptedWithLargerDt) {
  EncoderEstimator estimator;
  ASSERT_TRUE(estimator.Init(MakeConfig()));
  host_fakes::FakeEncoderSource source;
  source.Init(MakeConfig());
  source.SetWheelSpeeds(0.5f, 0.5f);
  ASSERT_TRUE(estimator.OnSample(source.Step(kWindowUs), false));
  ASSERT_TRUE(estimator.OnSample(source.Step(15000), false));  // 丢了两个窗口
  EXPECT_EQ(estimator.stats().timestamp_invalid, 0u);
  EXPECT_TRUE(estimator.state().left.valid);
  EXPECT_NEAR(estimator.state().left.speed_mps, 0.5f, 0.1f);
}

// 基线重建契约：count_baseline_reset ⇒ 只重建基线、不产出速度
TEST(EncoderEstimatorTest, BaselineResetRebuildsWithoutSpeed) {
  EncoderEstimator estimator;
  ASSERT_TRUE(estimator.Init(MakeConfig()));
  host_fakes::FakeEncoderSource source;
  source.Init(MakeConfig());
  source.SetWheelSpeeds(0.5f, 0.5f);
  ASSERT_TRUE(estimator.OnSample(source.Step(kWindowUs), false));
  ASSERT_TRUE(estimator.OnSample(source.Step(kWindowUs), false));

  EncoderSnapshot reset = source.Step(kWindowUs);
  reset.left.count_baseline_reset = true;
  estimator.OnSample(reset, false);
  EXPECT_GE(estimator.stats().baseline_resets, 1u);
  EXPECT_NE(estimator.state().left.quality & kQualityHardwareFault, 0u);

  // 契约：reset 帧之后的第一份样本只重建基线（不产出速度），第二份才产出
  ASSERT_TRUE(estimator.OnSample(source.Step(kWindowUs), false));
  EXPECT_FALSE(estimator.state().left.valid);
  ASSERT_TRUE(estimator.OnSample(source.Step(kWindowUs), false));
  EXPECT_TRUE(estimator.state().left.valid);
}

// A7：WheelState → 003 反馈状态帧（measured_* 与两个有效位）
TEST(EncoderFeedbackIntegrationTest, MeasuredSpeedsReachStatusFrame) {
  EncoderEstimator estimator;
  ASSERT_TRUE(estimator.Init(MakeConfig()));
  host_fakes::FakeEncoderSource source;
  source.Init(MakeConfig());
  source.SetWheelSpeeds(0.4f, -0.3f);
  ASSERT_TRUE(estimator.OnSample(source.Step(kWindowUs), false));
  ASSERT_TRUE(estimator.OnSample(source.Step(kWindowUs), false));
  ASSERT_TRUE(estimator.OnSample(source.Step(kWindowUs), false));

  const WheelState wheel_state = estimator.state();
  protocol::FeedbackInputs inputs{};
  inputs.wheel_state = &wheel_state;
  const protocol::FeedbackStatusPayload payload = protocol::AssembleStatusPayload(inputs, 1000);

  EXPECT_NE(payload.status_flags & protocol::kFlagSpeedValid, 0u);
  EXPECT_NE(payload.status_flags & protocol::kFlagEncoderValid, 0u);
  EXPECT_NEAR(payload.measured_left_mps, wheel_state.left.speed_mps, 1e-6f);
  EXPECT_NEAR(payload.measured_right_mps, wheel_state.right.speed_mps, 1e-6f);
  EXPECT_TRUE(payload.flags_valid & protocol::kFlagSpeedValid);
}

// A7：无数据源时清位且不写数值（003 的缺失语义）
TEST(EncoderFeedbackIntegrationTest, MissingSourceClearsFlagsAndZeroesValues) {
  protocol::FeedbackInputs inputs{};
  inputs.wheel_state = nullptr;
  const protocol::FeedbackStatusPayload payload = protocol::AssembleStatusPayload(inputs, 1000);
  EXPECT_EQ(payload.status_flags & protocol::kFlagSpeedValid, 0u);
  EXPECT_EQ(payload.status_flags & protocol::kFlagEncoderValid, 0u);
  EXPECT_EQ(payload.measured_left_mps, 0.0f);
}

}  // namespace
}  // namespace robotcar01::encoder
