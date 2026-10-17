// 轮速控制器测试（Iteration 006）：A1 收敛、A2 抗饱和、A3 零目标、A4 换向（物理口径）、
// A5 质量/新鲜度/停滞、A9 亚分辨率目标、A10 死区 > min_effective、配置 fail-closed。
// 闭环串联真实 004 估计器：FakeMotorPlant → FakeEncoderSource → EncoderEstimator → 控制器（评审 H1）。
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "app/control/controller_config.h"
#include "app/control/wheel_speed_controller.h"
#include "app/encoder/encoder_config.h"
#include "app/encoder/encoder_estimator.h"
#include "host_fakes/fake_encoder_source.h"
#include "host_fakes/fake_motor_plant.h"

namespace robotcar01::control {
namespace {

constexpr uint32_t kPeriodUs = WheelControllerConfig::kNominalPeriodUs;

encoder::EncoderConfig MakeEncoderConfig() {
  encoder::EncoderConfig config{};
  config.location = encoder::EncoderLocation::kMotorShaft;
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

// 闭环夹具：单轮（左）控制器 + plant + 004 估计器
class Loop {
 public:
  Loop() {
    EXPECT_TRUE(controller.Init(WheelControllerConfig{}));
    EXPECT_TRUE(estimator.Init(MakeEncoderConfig()));
    source.Init(MakeEncoderConfig());
  }

  // 推进一个控制周期：plant 按上周期输出运动 → 编码器采样 → 控制器更新
  MotorOutputRequest Step(float target, bool allow_motion = true) {
    plant.Step(last, kPeriodUs);
    now_us += kPeriodUs;
    source.SetWheelSpeeds(plant.speed(), 0.0f);
    encoder::EncoderSnapshot snapshot = source.Step(kPeriodUs);
    snapshot.timestamp = mcu_os_lite::TimePoint{now_us};
    estimator.OnSample(snapshot, allow_motion && target != 0.0f);
    const encoder::WheelState state = estimator.state();
    last = controller.Update(target, state.left, state.timestamp.us, allow_motion, now_us);
    trace.push_back(last);
    return last;
  }

  // 逐周期检查 INV-006-2（物理形式）：方向切换且 duty>0 时，此前必有 dwell 停止确认。
  // 这里用 plant 的带载换向计数作为物理判据（|v|>0.05 且施加反向 duty）。
  uint32_t plugging() const { return plant.plugging_events(); }

  WheelSpeedController controller;
  encoder::EncoderEstimator estimator;
  host_fakes::FakeEncoderSource source;
  host_fakes::FakeMotorPlant plant;
  MotorOutputRequest last{};
  uint64_t now_us = 0;
  std::vector<MotorOutputRequest> trace;
};

// 模拟 005 的意图域斜坡：目标以 rate_mps2 逼近 goal
float RampToward(float current, float goal, float rate_mps2) {
  const float step = rate_mps2 * (static_cast<float>(kPeriodUs) / 1e6f);
  if (goal > current) return std::fmin(goal, current + step);
  return std::fmax(goal, current - step);
}

// ---- 配置 fail-closed（14 条，抽查关键规则）----
TEST(WheelControllerConfigTest, DefaultIsValid) { EXPECT_TRUE(WheelControllerConfig{}.IsValid()); }

TEST(WheelControllerConfigTest, RejectsInvalidRules) {
  {
    WheelControllerConfig c{};
    c.kp_permille_per_mps = -1.0f;  // 规则 1
    EXPECT_FALSE(c.IsValid());
  }
  {
    WheelControllerConfig c{};
    c.ki_permille_per_mps_s = std::numeric_limits<float>::quiet_NaN();  // 规则 1
    EXPECT_FALSE(c.IsValid());
  }
  {
    WheelControllerConfig c{};
    c.min_effective_duty_permille = 900;  // 规则 3：min_effective < max_duty
    EXPECT_FALSE(c.IsValid());
  }
  {
    WheelControllerConfig c{};
    c.max_duty_permille = 1001;  // 规则 3
    EXPECT_FALSE(c.IsValid());
  }
  {
    WheelControllerConfig c{};
    c.zero_target_threshold_mps = 0.01f;  // 规则 6：低于测量底噪（C2）
    EXPECT_FALSE(c.IsValid());
  }
  {
    WheelControllerConfig c{};
    c.stop_speed_threshold_mps = 0.02f;  // 规则 7
    EXPECT_FALSE(c.IsValid());
  }
  {
    WheelControllerConfig c{};
    c.stop_dwell_us = 15000;  // 规则 8：必须 > 004 最坏发布间隔
    EXPECT_FALSE(c.IsValid());
  }
  {
    WheelControllerConfig c{};
    c.kff_permille_per_mps = 2000.0f;  // 规则 12：kff·max_target(0.5)=1000 > max_duty 900
    EXPECT_FALSE(c.IsValid());
  }
  {
    WheelControllerConfig c{};
    c.integral_limit_permille = 50.0f;  // 规则 14：< min_effective
    EXPECT_FALSE(c.IsValid());
  }
  {
    WheelControllerConfig c{};
    c.max_target_speed_mps = 1.0f;  // 规则 11：必须 < max_measured
    EXPECT_FALSE(c.IsValid());
  }
  {
    WheelControllerConfig c{};
    c.stall_integral_cap_permille = 400.0f;  // 规则 15：> integral_limit
    EXPECT_FALSE(c.IsValid());
  }
  {
    WheelControllerConfig c{};
    c.stall_fault_timeout_us = 20000;  // 规则 16：≤ stop_dwell
    EXPECT_FALSE(c.IsValid());
  }
}

TEST(WheelSpeedControllerTest, UninitializedOutputsFaultCoast) {
  WheelSpeedController controller;
  WheelControllerConfig bad{};
  bad.max_duty_permille = 2000;
  EXPECT_FALSE(controller.Init(bad));
  encoder::WheelSideState side{};
  side.valid = true;
  const MotorOutputRequest out = controller.Update(0.3f, side, 1000, true, 1000);
  EXPECT_EQ(out.mode, OutputMode::kCoast);
  EXPECT_EQ(out.duty_permille, 0u);
  EXPECT_TRUE(out.fault);
  EXPECT_NE(out.reason & kOutputControllerFault, 0u);
}

// ---- A1：闭环收敛（串联真实 004 估计器）----
TEST(WheelSpeedControllerTest, StepConvergesWithinBand) {
  Loop loop;
  int settle_index = -1;
  int inside = 0;
  for (int k = 0; k < 600; ++k) {  // 3 s
    loop.Step(0.3f);
    const float err = std::fabs(loop.plant.speed() - 0.3f);
    if (err <= 0.06f) {
      if (++inside >= 100 && settle_index < 0) settle_index = k;  // 保持 0.5 s
    } else {
      inside = 0;
    }
  }
  ASSERT_GE(settle_index, 0) << "never settled";
  EXPECT_LE(settle_index * kPeriodUs, 1500000u + 500000u);  // 1.5 s 进入 + 0.5 s 保持
  EXPECT_NEAR(loop.plant.speed(), 0.3f, 0.06f);
}

// ---- A2：抗饱和（积分轨迹性质）----
TEST(WheelSpeedControllerTest, IntegratorDoesNotWindUpUnderSaturation) {
  // 判定口径（INV-006-3，详设 §10）：输出被夹在 max_duty 的周期内积分不得增长；全程 |I| ≤ 上限。
  // 饱和边界附近的非饱和周期允许正常积分（条件积分本义），故不断言"首次饱和后永不增长"。
  Loop loop;
  loop.plant.load_bias_mps = 0.15f;  // 负载使 0.5 m/s 不可达 ⇒ 持续饱和（饱和稳态 ≈0.34）
  uint32_t saturated_cycles = 0;
  float prev_integral = 0.0f;
  for (int k = 0; k < 400; ++k) {  // 目标 0.5 持续 2 s
    const MotorOutputRequest out = loop.Step(0.5f);
    const float integral = loop.controller.integral_permille();
    if ((out.reason & kOutputSaturated) != 0u) {
      ++saturated_cycles;
      EXPECT_LE(integral, prev_integral + 1e-4f) << "integrator grew in a saturated cycle at k=" << k;
    }
    EXPECT_LE(std::fabs(integral), 350.0f + 1e-3f);
    prev_integral = integral;
  }
  ASSERT_GT(saturated_cycles, 300u) << "test never exercised sustained saturation";
  // 负载解除 + 目标降到 0.2：积分未发散 ⇒ 1 s 内速度不超过 0.30（无大超调）
  loop.plant.load_bias_mps = 0.0f;
  float peak = 0.0f;
  for (int k = 0; k < 400; ++k) {
    loop.Step(0.2f);
    if (k >= 100 && k <= 300) peak = std::fmax(peak, loop.plant.speed());
  }
  EXPECT_LE(peak, 0.30f);
  EXPECT_NEAR(loop.plant.speed(), 0.2f, 0.06f);
}

// ---- A3：零目标 ----
TEST(WheelSpeedControllerTest, ZeroTargetClearsIntegratorAndOutputsZero) {
  Loop loop;
  for (int k = 0; k < 200; ++k) loop.Step(0.3f);
  EXPECT_GT(std::fabs(loop.controller.integral_permille()), 0.0f);
  for (int k = 0; k < 200; ++k) {
    const MotorOutputRequest out = loop.Step(0.0f);
    EXPECT_EQ(out.duty_permille, 0u);
  }
  EXPECT_EQ(loop.controller.integral_permille(), 0.0f);
  EXPECT_EQ(loop.controller.state(), ControlState::kIdle);  // 停止已确认，锁存释放
  EXPECT_NEAR(loop.plant.speed(), 0.0f, 0.05f);
}

// ---- A4：换向（经 005 斜坡的连续目标，物理口径）----
TEST(WheelSpeedControllerTest, RampedReversalNeverPlugs) {
  Loop loop;
  float target = 0.0f;
  for (int k = 0; k < 300; ++k) {
    target = RampToward(target, 0.3f, 0.83f);
    loop.Step(target);
  }
  ASSERT_GT(loop.plant.speed(), 0.2f);
  for (int k = 0; k < 800; ++k) {
    target = RampToward(target, -0.3f, 1.295f);
    loop.Step(target);
  }
  EXPECT_EQ(loop.plugging(), 0u) << "drove against motion (plugging)";
  EXPECT_LT(loop.plant.speed(), -0.2f) << "reversal never completed";
  EXPECT_GE(loop.controller.stats().stopping_entries, 1u);
}

TEST(WheelSpeedControllerTest, StepReversalWaitsForConfirmedStop) {
  Loop loop;
  for (int k = 0; k < 300; ++k) loop.Step(0.3f);
  const size_t before = loop.trace.size();
  bool saw_stopping = false;
  for (int k = 0; k < 600; ++k) {
    const MotorOutputRequest out = loop.Step(-0.3f);
    if ((out.reason & kOutputStopping) != 0u) {
      saw_stopping = true;
      EXPECT_EQ(out.duty_permille, 0u);
    }
  }
  EXPECT_TRUE(saw_stopping);
  EXPECT_EQ(loop.plugging(), 0u);
  // 相邻周期不存在"方向翻转且两周期 duty 均非零"
  for (size_t i = before + 1; i < loop.trace.size(); ++i) {
    const auto& a = loop.trace[i - 1];
    const auto& b = loop.trace[i];
    if (a.duty_permille > 0u && b.duty_permille > 0u) {
      EXPECT_EQ(a.direction, b.direction);
    }
  }
  EXPECT_LT(loop.plant.speed(), -0.2f);
}

TEST(WheelSpeedControllerTest, AllowMotionToggleDuringReversalKeepsLatch) {
  Loop loop;
  for (int k = 0; k < 300; ++k) loop.Step(0.3f);
  loop.Step(-0.3f, false);  // 换向中安全禁止
  loop.Step(-0.3f, false);
  for (int k = 0; k < 600; ++k) loop.Step(-0.3f, true);  // 恢复
  EXPECT_EQ(loop.plugging(), 0u);
  EXPECT_LT(loop.plant.speed(), -0.2f);
}

TEST(WheelSpeedControllerTest, ResumeWhileStillSpinningOppositeStopsFirst) {
  Loop loop;
  for (int k = 0; k < 300; ++k) loop.Step(0.3f);
  // 安全禁止，状态机被外部复位（模拟重新装配），轮子仍正向滑行
  loop.controller.Reset();
  for (int k = 0; k < 400; ++k) loop.Step(-0.3f);
  EXPECT_EQ(loop.plugging(), 0u) << "resumed against residual motion";
}

// ---- A5：质量 / 新鲜度 / 停滞 ----
TEST(WheelSpeedControllerTest, BlockingQualityZeroesAndClearsIntegrator) {
  WheelSpeedController controller;
  ASSERT_TRUE(controller.Init(WheelControllerConfig{}));
  encoder::WheelSideState side{};
  side.valid = true;
  side.speed_mps = 0.1f;
  uint64_t now = 0;
  for (int k = 0; k < 50; ++k) {
    now += kPeriodUs;
    controller.Update(0.3f, side, now, true, now);
  }
  EXPECT_GT(controller.integral_permille(), 0.0f);
  side.quality = encoder::kQualityHardwareFault;
  now += kPeriodUs;
  const MotorOutputRequest out = controller.Update(0.3f, side, now, true, now);
  EXPECT_EQ(out.duty_permille, 0u);
  EXPECT_NE(out.reason & kOutputMeasurementInvalid, 0u);
  EXPECT_EQ(controller.integral_permille(), 0.0f);
}

TEST(WheelSpeedControllerTest, StaleMeasurementZeroesOutput) {
  WheelSpeedController controller;
  ASSERT_TRUE(controller.Init(WheelControllerConfig{}));
  encoder::WheelSideState side{};
  side.valid = true;
  side.speed_mps = 0.0f;
  const uint64_t measured_at = 5000;
  uint64_t now = measured_at;
  uint32_t stale = 0;
  for (int k = 0; k < 40; ++k) {  // 测量冻结 200 ms
    now += kPeriodUs;
    const MotorOutputRequest out = controller.Update(0.3f, side, measured_at, true, now);
    if ((out.reason & kOutputMeasurementStale) != 0u) {
      ++stale;
      EXPECT_EQ(out.duty_permille, 0u);
    }
  }
  EXPECT_GT(stale, 30u);
  EXPECT_EQ(controller.integral_permille(), 0.0f);
}

TEST(WheelSpeedControllerTest, StallCandidateBoundsIntegratorAndTimesOut) {
  // C3（实现期修订）：停滞候选 ⇒ 积分 ≤ 起步上限（有界），且停滞超时 ⇒ 零输出 + 停滞故障。
  WheelSpeedController controller;
  const WheelControllerConfig config{};
  ASSERT_TRUE(controller.Init(config));
  encoder::WheelSideState side{};
  side.valid = true;
  side.speed_mps = 0.0f;
  side.quality = encoder::kQualityStallCandidate;  // 编码器断线/轮子卡死：测量恒 0
  uint64_t now = 0;
  uint16_t max_duty = 0;
  bool faulted = false;
  uint64_t fault_at = 0;
  for (int k = 0; k < 400; ++k) {  // 2 s
    now += kPeriodUs;
    const MotorOutputRequest out = controller.Update(0.3f, side, now, true, now);
    EXPECT_LE(controller.integral_permille(), config.stall_integral_cap_permille + 1e-3f);
    if (out.duty_permille > max_duty) max_duty = out.duty_permille;
    if (out.fault && (out.reason & kOutputStallFault) != 0u && !faulted) {
      faulted = true;
      fault_at = now;
    }
    if (faulted) EXPECT_EQ(out.duty_permille, 0u);
  }
  // FF + P + 起步上限：1600·0.3 + 800·0.3 + 150 = 870 < max_duty 900（盲驱幅值有界）
  EXPECT_LE(max_duty, 871u);
  ASSERT_TRUE(faulted);
  EXPECT_LE(fault_at, static_cast<uint64_t>(config.stall_fault_timeout_us) + 2u * kPeriodUs);
  // 目标归零后清除故障，可重新起步
  now += kPeriodUs;
  side.quality = encoder::kQualityValid;
  controller.Update(0.0f, side, now, true, now);
  EXPECT_EQ(controller.stats().stall_faults, 1u);
}

TEST(WheelSpeedControllerTest, AccumulatingKeepsUsingLastPublishedSpeed) {
  WheelSpeedController controller;
  ASSERT_TRUE(controller.Init(WheelControllerConfig{}));
  encoder::WheelSideState side{};
  side.valid = false;
  side.quality = encoder::kQualityAccumulating;
  side.speed_mps = 0.1f;
  uint64_t now = 0;
  MotorOutputRequest out{};
  for (int k = 0; k < 10; ++k) {
    now += kPeriodUs;
    out = controller.Update(0.3f, side, now, true, now);
  }
  EXPECT_EQ(out.reason & kOutputMeasurementInvalid, 0u);
  EXPECT_GT(out.duty_permille, 0u);
}

TEST(WheelSpeedControllerTest, OverspeedMeasurementIsInvalid) {
  WheelSpeedController controller;
  ASSERT_TRUE(controller.Init(WheelControllerConfig{}));
  encoder::WheelSideState side{};
  side.valid = true;
  side.speed_mps = 5.0f;  // > max_measured 1.0
  const MotorOutputRequest out = controller.Update(0.3f, side, 1000, true, 1000);
  EXPECT_NE(out.reason & kOutputMeasurementInvalid, 0u);
  EXPECT_EQ(out.duty_permille, 0u);
}

TEST(WheelSpeedControllerTest, NonFiniteTargetIsControllerFault) {
  WheelSpeedController controller;
  ASSERT_TRUE(controller.Init(WheelControllerConfig{}));
  encoder::WheelSideState side{};
  side.valid = true;
  const MotorOutputRequest out = controller.Update(std::numeric_limits<float>::infinity(), side,
                                                   1000, true, 1000);
  EXPECT_TRUE(out.fault);
  EXPECT_EQ(out.duty_permille, 0u);
}

// ---- A9：亚分辨率目标不漂移 ----
TEST(WheelSpeedControllerTest, BelowFloorTargetIsTreatedAsZero) {
  Loop loop;
  bool saw_below_floor = false;
  for (int k = 0; k < 2000; ++k) {  // 10 s @ 0.03 m/s
    const MotorOutputRequest out = loop.Step(0.03f);
    saw_below_floor |= (out.reason & kOutputBelowFloor) != 0u;
    EXPECT_LE(std::fabs(loop.plant.speed()), 0.06f);
  }
  EXPECT_TRUE(saw_below_floor);
  EXPECT_LE(std::fabs(loop.plant.position_m()), 0.01);
}

// ---- A10：死区 > min_effective，靠积分起步 ----
TEST(WheelSpeedControllerTest, StartsAgainstDeadbandAboveMinEffective) {
  Loop loop;
  loop.plant.deadband_permille = 250.0f;  // 远大于 min_effective 80‰
  int started = -1;
  for (int k = 0; k < 600; ++k) {
    loop.Step(0.1f);
    if (started < 0 && loop.plant.speed() > 0.05f) started = k;
    EXPECT_LE(std::fabs(loop.controller.integral_permille()), 350.0f + 1e-3f);
  }
  ASSERT_GE(started, 0);
  EXPECT_LE(started * kPeriodUs, 2000000u);
}

}  // namespace
}  // namespace robotcar01::control
