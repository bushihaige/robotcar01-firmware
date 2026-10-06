// 轮速/位置状态与单窗口估计测试（Iteration 004）：A1 安装模型、A2 回绕差分、A3 方向与位置、
// A5 时间戳质量；配合 encoder_estimator_test 覆盖 A4/A6/A7
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "app/encoder/encoder_config.h"
#include "app/encoder/wheel_state.h"
#include "host_fakes/fake_encoder_source.h"

namespace robotcar01::encoder {
namespace {

// 测试配置：11 线 ×4 = 44 counts/电机转、90:1、100 mm 轮（占位非标定，与详设核算一致）
EncoderConfig MakeConfig(EncoderLocation location = EncoderLocation::kMotorShaft) {
  EncoderConfig config{};
  config.location = location;
  config.counts_per_encoder_rev = 44.0f;
  config.gear_ratio = 90.0f;
  config.wheel_radius_m = 0.05f;
  config.left.counter_bits = 16;
  config.right.counter_bits = 16;
  config.low_speed_zero_threshold_mps = 0.05f;   // ≈3.2 × 单 count 当量（0.0159 m/s）
  config.max_estimation_delay_us = 20000;
  config.stall_windows_threshold = 8;
  config.low_speed_accumulate_counts = 5;        // ceil(0.05/0.01587)+1 = 5
  config.max_wheel_speed_mps = 0.5f;
  return config;
}

// ---- A2：回绕安全差分（双向 + 符号锁定，评审 H-2）----
TEST(SignedCountDeltaTest, ForwardAndBackward) {
  int32_t delta = 0;
  ASSERT_TRUE(SignedCountDelta(100, 50, 16, delta));
  EXPECT_EQ(delta, 50);
  ASSERT_TRUE(SignedCountDelta(50, 100, 16, delta));
  EXPECT_EQ(delta, -50);
  ASSERT_TRUE(SignedCountDelta(7, 7, 16, delta));
  EXPECT_EQ(delta, 0);
}

TEST(SignedCountDeltaTest, WrapBothDirectionsSixteenBit) {
  int32_t delta = 0;
  ASSERT_TRUE(SignedCountDelta(5, 65530, 16, delta));  // 正向跨回绕
  EXPECT_EQ(delta, 11);
  ASSERT_TRUE(SignedCountDelta(65530, 5, 16, delta));  // 反向跨回绕
  EXPECT_EQ(delta, -11);
  ASSERT_TRUE(SignedCountDelta(5, 65530, 32, delta));  // 32 位下是普通递减
  EXPECT_EQ(delta, -65525);
}

TEST(SignedCountDeltaTest, ExactlyHalfRangeIsIndistinguishable) {
  int32_t delta = 123;
  EXPECT_FALSE(SignedCountDelta(32768, 0, 16, delta));   // 正向恰好半程
  EXPECT_FALSE(SignedCountDelta(0, 32768, 16, delta));   // 反向恰好半程（双向均不可分辨）
  EXPECT_FALSE(SignedCountDelta(0x80000000u, 0, 32, delta));
}

TEST(SignedCountDeltaTest, JustBeyondHalfRangeSignsAreLocked) {
  int32_t delta = 0;
  ASSERT_TRUE(SignedCountDelta(32769, 0, 16, delta));
  EXPECT_EQ(delta, -32767);  // 逾半程按反向解释（D-004-3）
  ASSERT_TRUE(SignedCountDelta(0, 32769, 16, delta));
  EXPECT_EQ(delta, 32767);
}

TEST(SignedCountDeltaTest, UnsupportedWidthRejected) {
  int32_t delta = 0;
  EXPECT_FALSE(SignedCountDelta(1, 0, 8, delta));
  EXPECT_FALSE(SignedCountDelta(1, 0, 24, delta));
}

// ---- A1：两种安装模型（输入不同、输出相同，评审 H-1）----
TEST(WheelStateConversionTest, TwoMountingModelsAgreeOnSpeed) {
  // 电机轴模型：CPR 44（电机轴侧）、减速比 90 ⇒ cpr_eff = 44 × 90 = 3960
  EncoderConfig motor = MakeConfig(EncoderLocation::kMotorShaft);
  // 输出轴模型：等价条件 cpr_motor = cpr_output × gear_ratio ⇒ 输出轴侧 3960，gear_ratio = 1
  EncoderConfig output = MakeConfig(EncoderLocation::kOutputShaft);
  output.gear_ratio = 1.0f;
  output.counts_per_encoder_rev = 44.0f * 90.0f;

  ASSERT_TRUE(motor.IsValid());
  ASSERT_TRUE(output.IsValid());
  EXPECT_NEAR(PerCountDistanceMeters(motor), PerCountDistanceMeters(output), 1e-9f);

  const int32_t counts = 32;  // 0.5 m/s 下的单窗口计数（两模型相同，因为 cpr_eff 相同）
  WheelSideInputs motor_inputs{};
  motor_inputs.count_delta = counts;
  motor_inputs.dt_us = 5000;
  motor_inputs.is_left = true;
  motor_inputs.config = &motor;
  const WheelSideState motor_state = EstimateSide(motor_inputs);

  WheelSideInputs output_inputs = motor_inputs;
  output_inputs.config = &output;
  const WheelSideState output_state = EstimateSide(output_inputs);

  EXPECT_TRUE(motor_state.valid);
  EXPECT_TRUE(output_state.valid);
  EXPECT_NEAR(motor_state.speed_mps, output_state.speed_mps, 1e-5f);

  // 断言"输入不同、输出相同"（防止用例退化为同义反复，评审 H-1）：
  // 输出轴侧计 90 倍 counts 时速度必须是电机轴侧的 90 倍。
  WheelSideInputs scaled = output_inputs;
  scaled.count_delta = static_cast<int32_t>(counts * 90);
  const WheelSideState scaled_state = EstimateSide(scaled);
  EXPECT_NEAR(scaled_state.speed_mps, output_state.speed_mps * 90.0f, 1e-3f);
}

TEST(WheelStateConversionTest, PerCountHelpers) {
  const EncoderConfig motor = MakeConfig();
  const float per_count = PerCountDistanceMeters(motor);
  EXPECT_NEAR(per_count, 0.31415926f / (44.0f * 90.0f), 1e-9f);
  EXPECT_NEAR(PerCountSpeedMps(motor), per_count / 0.005f, 1e-7f);
}

// ---- A3：方向极性与位置增量 ----
TEST(WheelStateConversionTest, PolarityFlipsSign) {
  EncoderConfig config = MakeConfig();
  WheelSideInputs inputs{};
  inputs.count_delta = 10;
  inputs.dt_us = 5000;
  inputs.is_left = true;
  inputs.config = &config;

  const WheelSideState normal = EstimateSide(inputs);
  config.left.polarity = EncoderPolarity::kInverted;
  const WheelSideState inverted = EstimateSide(inputs);
  EXPECT_GT(normal.speed_mps, 0.0f);
  EXPECT_LT(inverted.speed_mps, 0.0f);
  EXPECT_NEAR(normal.speed_mps, -inverted.speed_mps, 1e-6f);
}

TEST(WheelStateConversionTest, PositionDeltaScalesWithCounts) {
  const EncoderConfig config = MakeConfig();
  WheelSideInputs inputs{};
  inputs.count_delta = 100;
  inputs.dt_us = 5000;
  inputs.is_left = true;
  inputs.config = &config;
  const WheelSideState state = EstimateSide(inputs);
  EXPECT_NEAR(state.position_delta_m, 100.0f * PerCountDistanceMeters(config), 1e-9f);
}

// ---- A5：时间戳与质量（含判零）----
TEST(WheelStateConversionTest, TimestampInvalidProducesNoSpeed) {
  const EncoderConfig config = MakeConfig();
  WheelSideInputs inputs{};
  inputs.count_delta = 10;
  inputs.dt_us = 0;
  inputs.quality = kQualityTimestampInvalid;
  inputs.is_left = true;
  inputs.config = &config;
  const WheelSideState state = EstimateSide(inputs);
  EXPECT_FALSE(state.valid);
  EXPECT_EQ(state.speed_mps, 0.0f);
  EXPECT_NE(state.quality & kQualityTimestampInvalid, 0u);
}

TEST(WheelStateConversionTest, LowSpeedIsZeroedBelowThreshold) {
  const EncoderConfig config = MakeConfig();
  // 2 counts / 5 ms ⇒ 0.0317 m/s < 0.05 阈值 ⇒ 判零但 valid（发布值判零，D-004-4）
  WheelSideInputs inputs{};
  inputs.count_delta = 2;
  inputs.dt_us = 5000;
  inputs.is_left = true;
  inputs.config = &config;
  const WheelSideState state = EstimateSide(inputs);
  EXPECT_TRUE(state.valid);
  EXPECT_EQ(state.speed_mps, 0.0f);
  EXPECT_GT(state.raw_speed_mps, 0.0f);  // 原始值仍可观测
}

// ---- 配置 fail-closed 校验（评审 C-4/H-8）----
TEST(EncoderConfigTest, ValidConfigPasses) {
  EncoderConfig config = MakeConfig();
  EXPECT_TRUE(config.IsValid());
}

TEST(EncoderConfigTest, RejectsInvalidCombinations) {
  {
    EncoderConfig config = MakeConfig();
    config.counts_per_encoder_rev = 0.0f;
    EXPECT_FALSE(config.IsValid());
  }
  {
    EncoderConfig config = MakeConfig();
    config.left.counter_bits = 24;
    EXPECT_FALSE(config.IsValid());
  }
  {
    // 输出轴模型必须 gear_ratio == 1
    EncoderConfig config = MakeConfig(EncoderLocation::kOutputShaft);
    config.gear_ratio = 90.0f;
    EXPECT_FALSE(config.IsValid());
  }
  {
    // 判零阈值低于单 count 当量
    EncoderConfig config = MakeConfig();
    config.low_speed_zero_threshold_mps = 0.001f;
    EXPECT_FALSE(config.IsValid());
  }
  {
    // 累加阈值未按取值式给出
    EncoderConfig config = MakeConfig();
    config.low_speed_accumulate_counts = 2;
    EXPECT_FALSE(config.IsValid());
  }
  {
    // 估计延迟预算不足
    EncoderConfig config = MakeConfig();
    config.max_estimation_delay_us = 9000;
    EXPECT_FALSE(config.IsValid());
  }
  {
    // 停滞阈值低于 2 × 50 Hz 命令间隔
    EncoderConfig config = MakeConfig();
    config.stall_windows_threshold = 4;
    EXPECT_FALSE(config.IsValid());
  }
  {
    // 评审 C-4 反例：把编码器侧 CPR 误当输出轴 CPR（比真实值小 90 倍）⇒ 量纲自检必须拦住
    EncoderConfig config = MakeConfig(EncoderLocation::kOutputShaft);
    config.gear_ratio = 1.0f;
    config.counts_per_encoder_rev = 44.0f;  // 实际应为 44×90 = 3960
    EXPECT_FALSE(config.IsValid());         // 单窗口计数被放大 90 倍，半程余量校验失败
    EXPECT_NEAR(PerCountDistanceMeters(config), 0.31415926f / 44.0f, 1e-6f);
    // 该量纲下半程只需约 0.06 s 即触达（远快于正常工况）——量纲自检正是必需的那道门
    const float per_count = PerCountDistanceMeters(config);
    // 单 count 当量被放大 90 倍（0.00714 m vs 正确值 7.9e-5 m）：真实的编码器侧计数若按此量纲
    // 换算，单窗口增量会达到 2^15 量级 ⇒ 恰好半程/越界；量纲自检（校验项 4）据此拒绝该配置。
    EXPECT_GT(per_count, 80.0f * PerCountDistanceMeters(MakeConfig(EncoderLocation::kMotorShaft)));
  }
}

}  // namespace
}  // namespace robotcar01::encoder
