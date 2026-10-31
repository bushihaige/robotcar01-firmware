// 平台配置常量与上游配置契约的一致性测试（Iteration 006.5）
//
// 目的：把"上板才不会发现"的失败提前到 host。
//   - BoardApp::Init 是 fail-closed 的：EncoderConfig / WheelControllerConfig 任一 IsValid() 失败，
//     固件会打印 FATAL 且电机永久失能。若平台常量不自洽（CPR/半径/底噪/阈值），实机将根本无法启动。
//   - 本测试用 `platform_stm32/platform_config.h` 的常量按 BoardApp 相同方式构造两个配置，
//     断言它们必须通过（任何修改常量导致的失配都会在这里先红）。

#include <gtest/gtest.h>

#include "app/control/controller_config.h"
#include "app/encoder/encoder_config.h"
#include "platform_stm32/platform_config.h"

namespace {

using robotcar01::control::WheelControllerConfig;
using robotcar01::encoder::EncoderConfig;
using robotcar01::encoder::EncoderLocation;
using robotcar01::encoder::EncoderPolarity;
using robotcar01::platform::kEncoderCounterBits;
using robotcar01::platform::kEncoderCountsPerWheelRev;
using robotcar01::platform::kGearRatio;
using robotcar01::platform::kMeasurementFloorMps;
using robotcar01::platform::kPwmFrequencyHz;
using robotcar01::platform::kPwmPeriodCounts;
using robotcar01::platform::kWheelRadiusM;

// 与 BoardApp::Init 完全一致的构造方式（输出轴模型，D-006.5-2/D-006.5-8）。
EncoderConfig MakeEncoderConfig() {
  EncoderConfig config{};
  config.location = EncoderLocation::kOutputShaft;
  config.counts_per_encoder_rev = kEncoderCountsPerWheelRev;
  config.gear_ratio = kGearRatio;
  config.wheel_radius_m = kWheelRadiusM;
  config.left.counter_bits = kEncoderCounterBits;
  config.right.counter_bits = kEncoderCounterBits;
  config.left.polarity = EncoderPolarity::kNormal;
  config.right.polarity = EncoderPolarity::kNormal;
  config.low_speed_zero_threshold_mps = kMeasurementFloorMps;
  config.max_estimation_delay_us = 20000U;
  config.stall_windows_threshold = 8U;
  config.low_speed_accumulate_counts = 4U;
  config.max_wheel_speed_mps = 0.5f;
  return config;
}

WheelControllerConfig MakeControllerConfig() {
  WheelControllerConfig config{};  // 006 冻结默认（占位非标定）
  config.measurement_floor_mps = kMeasurementFloorMps;  // D-006-7：与 004 底噪绑定
  return config;
}

TEST(PlatformConfigTest, EncoderConfigFromPlatformConstantsIsValid) {
  const EncoderConfig config = MakeEncoderConfig();
  EXPECT_TRUE(config.IsValid()) << "平台常量构造的 EncoderConfig 未通过 fail-closed 校验";
}

TEST(PlatformConfigTest, ControllerConfigFromPlatformConstantsIsValid) {
  const WheelControllerConfig config = MakeControllerConfig();
  EXPECT_TRUE(config.IsValid()) << "平台常量构造的 WheelControllerConfig 未通过 fail-closed 校验";
}

// 量纲核算：输出轴 3960 counts/轮转、100 mm 轮径、5 ms 窗口 ⇒ 单 count 当量 ≈ 7.93e-5 m。
TEST(PlatformConfigTest, PerCountScaleMatchesSizing) {
  const EncoderConfig config = MakeEncoderConfig();
  const float per_count_m = robotcar01::encoder::PerCountDistanceMeters(config);
  EXPECT_NEAR(per_count_m, 7.93e-5f, 1e-6f);

  const float per_count_mps = robotcar01::encoder::PerCountSpeedMps(config);
  EXPECT_NEAR(per_count_mps, 0.01587f, 1e-4f);
}

// 底噪必须高于单 count 当量（006 的"低于底噪目标按零"才有意义）。
TEST(PlatformConfigTest, MeasurementFloorAboveSingleCountQuantum) {
  const EncoderConfig config = MakeEncoderConfig();
  const float per_count_mps = robotcar01::encoder::PerCountSpeedMps(config);
  EXPECT_GE(kMeasurementFloorMps, per_count_mps);
}

// PWM 时基：千分比分辨率与 BTS7960 上限（20 kHz，手册 f_pwm ≤ 25 kHz）。
TEST(PlatformConfigTest, PwmTimingWithinLimits) {
  EXPECT_GE(kPwmPeriodCounts, 1000U);
  EXPECT_LE(kPwmFrequencyHz, 25000U);
  // 84 MHz / 4200 = 20 kHz 的静态核算
  EXPECT_EQ(kPwmPeriodCounts, 4200U);
}

}  // namespace
