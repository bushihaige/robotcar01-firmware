// 差速运动学单元测试（Iteration 005）
//
// 覆盖：A1 正逆一致 + 与 001 chassis::ComputeWheelTarget 交叉一致、A2 统一比例限幅、
// A3 原地转向异号、A4 边界输入与 fail-closed、A5 加减速斜坡与 dt 语义（含跨零换向）、
// A6 立即零与基线语义、A7 质量降级、正运动学 FK 三态与质量聚合。
//
// 两套夹具：
//   TestConfig() —— 真实占位加速度（0.5 / 0.8 / 2.0 / 3.0），用于斜坡与 dt 语义用例；
//   FastConfig() —— 极大加速度（1000），用于"静态"限幅/交叉一致性用例，使首周期后即达目标，
//                   从而与 001 的纯函数（无时间维）逐字段可比。
// 浮点断言统一 EXPECT_NEAR；严格 > 判定（恰在限上不触发限幅）。

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "app/chassis/kinematics.h"
#include "app/encoder/wheel_state.h"
#include "app/kinematics/body_state.h"
#include "app/kinematics/differential_kinematics.h"

namespace robotcar01::kinematics {
namespace {

constexpr float kEps = 1e-6f;
constexpr uint32_t kPeriodUs = 5000;  // 5 ms（arch §6 KinematicsTask 周期）

KinematicsConfig BaseConfig() {
  KinematicsConfig config;  // drive 默认：T=0.33, v_max=0.5, ω_max=2.0, wheel_max=1.0
  config.drive.track_width_m = 0.5f;  // 便于手算：raw = v ± ω*0.25
  config.drive.max_body_speed_mps = 1.0f;
  config.drive.max_yaw_rate_radps = 2.0f;
  config.drive.max_wheel_speed_mps = 0.8f;
  config.max_body_accel_mps2 = 0.5f;
  config.max_body_decel_mps2 = 0.8f;
  config.max_yaw_accel_radps2 = 2.0f;
  config.max_yaw_decel_radps2 = 3.0f;
  config.degraded_max_wheel_speed_mps = 0.2f;
  config.max_update_interval_us = 20000;
  return config;
}

KinematicsConfig TestConfig() { return BaseConfig(); }

// 大加速度（50 m/s²，仍满足校验 10 的单周期可达性上界）：用若干周期即到达"静态"目标，
// 用于域限幅与 001 交叉一致性；RunCycles 需给足周期数（5 周期 × 0.25 m/s = 1.25 m/s 覆盖量程）。
KinematicsConfig FastConfig() {
  KinematicsConfig config = BaseConfig();
  config.max_body_accel_mps2 = 50.0f;
  config.max_body_decel_mps2 = 50.0f;
  config.max_yaw_accel_radps2 = 50.0f;
  config.max_yaw_decel_radps2 = 50.0f;
  return config;
}

// FastConfig 下到达任意域内目标所需周期数：Δv = 50×0.005 = 0.25 m/s/周期、
// Δω = 50×0.005 = 0.25 rad/s/周期 ⇒ 40 周期足以覆盖 v ≤ 1.0 m/s 与 ω ≤ 10 rad/s 的所有用例。
constexpr int kFastCycles = 40;

chassis::MotionCommand Command(float v_mps, float omega_radps, bool valid = true) {
  chassis::MotionCommand command;
  command.v_mps = v_mps;
  command.omega_radps = omega_radps;
  command.valid = valid;
  return command;
}

encoder::WheelState Measurement(float left_mps, float right_mps,
                                uint32_t left_quality = encoder::kQualityValid,
                                uint32_t right_quality = encoder::kQualityValid,
                                bool left_valid = true, bool right_valid = true) {
  encoder::WheelState state{};
  state.left.speed_mps = left_mps;
  state.right.speed_mps = right_mps;
  state.left.quality = left_quality;
  state.right.quality = right_quality;
  state.left.valid = left_valid;
  state.right.valid = right_valid;
  return state;
}

KinematicsInputs Inputs(const chassis::MotionCommand& command, const encoder::WheelState* measurement,
                        bool motion_allowed = true) {
  KinematicsInputs inputs{};
  inputs.command = command;
  inputs.motion_allowed = motion_allowed;
  inputs.measurement = measurement;
  return inputs;
}

// 首个周期只建立基线（评审 C2：不存在"直接采用目标"的旁路），其后推进 cycles 个周期。
chassis::WheelTarget RunCycles(DifferentialKinematics& kinematics, const KinematicsInputs& inputs,
                               uint64_t& now_us, int cycles) {
  chassis::WheelTarget target = kinematics.Update(inputs, now_us);  // 基线周期（输出零）
  for (int i = 0; i < cycles; ++i) {
    now_us += kPeriodUs;
    target = kinematics.Update(inputs, now_us);
  }
  return target;
}

// --- A1：正逆一致与 001 交叉一致性 ---

TEST(DifferentialKinematicsTest, CrossCheckWithChassisKinematics) {
  const KinematicsConfig config = FastConfig();
  const encoder::WheelState measurement = Measurement(0.0f, 0.0f);
  const struct {
    float v;
    float omega;
  } cases[] = {{0.5f, 0.0f}, {0.0f, 1.0f}, {2.0f, 0.0f}, {0.4f, 2.4f}};

  for (const auto& item : cases) {
    DifferentialKinematics kinematics;
    ASSERT_TRUE(kinematics.Init(config));
    uint64_t now_us = kPeriodUs;
    const chassis::MotionCommand command = Command(item.v, item.omega);
    const chassis::WheelTarget actual = RunCycles(kinematics, Inputs(command, &measurement), now_us, kFastCycles);
    const chassis::WheelTarget expected = chassis::ComputeWheelTarget(config.drive, command);

    EXPECT_NEAR(actual.raw_left_mps, expected.raw_left_mps, 1e-5f);
    EXPECT_NEAR(actual.raw_right_mps, expected.raw_right_mps, 1e-5f);
    EXPECT_NEAR(actual.left_mps, expected.left_mps, 1e-5f);
    EXPECT_NEAR(actual.right_mps, expected.right_mps, 1e-5f);
    EXPECT_NEAR(actual.scale, expected.scale, 1e-5f);
    EXPECT_EQ(actual.limit_reason, expected.limit_reason);
  }

  // 显式数值（防两份实现同源错误）：
  // ① 直行 0.5 m/s：无任何限幅；
  DifferentialKinematics straight_kin;
  ASSERT_TRUE(straight_kin.Init(config));
  uint64_t now_us = kPeriodUs;
  const chassis::WheelTarget straight =
      RunCycles(straight_kin, Inputs(Command(0.5f, 0.0f), &measurement), now_us, kFastCycles);
  EXPECT_NEAR(straight.left_mps, 0.5f, 1e-5f);
  EXPECT_NEAR(straight.right_mps, 0.5f, 1e-5f);
  EXPECT_NEAR(straight.scale, 1.0f, 1e-5f);
  EXPECT_EQ(straight.limit_reason, chassis::LimitReason::kNone);

  // ② v=0.4, ω=2.4（T=0.5, ω_max=2.0, wheel_max=0.8）：raw=(-0.2, 1.0)、
  //    body_scale=2/2.4=0.8333、候选轮域=0.8、最终 scale=0.8。
  DifferentialKinematics limited_kin;
  ASSERT_TRUE(limited_kin.Init(config));
  now_us = kPeriodUs;
  const chassis::WheelTarget limited =
      RunCycles(limited_kin, Inputs(Command(0.4f, 2.4f), &measurement), now_us, kFastCycles);
  EXPECT_NEAR(limited.scale, 0.8f, 1e-5f);
  EXPECT_NEAR(limited.left_mps, -0.16f, 1e-5f);
  EXPECT_NEAR(limited.right_mps, 0.8f, 1e-5f);
  EXPECT_TRUE(chassis::IsSet(limited.limit_reason, chassis::LimitReason::kWheelSpeed));
  // 曲率保持：actual 左右比值 == raw 左右比值
  EXPECT_NEAR(limited.left_mps / limited.right_mps,
              limited.raw_left_mps / limited.raw_right_mps, 1e-5f);
}

TEST(DifferentialKinematicsTest, ForwardInverseTransformRoundTrip) {
  const KinematicsConfig config = FastConfig();
  DifferentialKinematics kinematics;
  ASSERT_TRUE(kinematics.Init(config));
  const encoder::WheelState measurement = Measurement(0.0f, 0.0f);
  uint64_t now_us = kPeriodUs;

  const chassis::WheelTarget target =
      RunCycles(kinematics, Inputs(Command(0.3f, 0.8f), &measurement), now_us, kFastCycles);
  ASSERT_TRUE(target.valid);
  // 逆 → 正：从左右轮目标恢复车体速度，应与命令一致（T=0.5 ⇒ ω=(r−l)/0.5）
  const float v_back = 0.5f * (target.left_mps + target.right_mps);
  const float omega_back = (target.right_mps - target.left_mps) / config.drive.track_width_m;
  EXPECT_NEAR(v_back, 0.3f, 1e-5f);
  EXPECT_NEAR(omega_back, 0.8f, 1e-5f);
}

// --- A2：统一比例限幅 ---

TEST(DifferentialKinematicsTest, BodyDomainLimitKeepsCurvature) {
  KinematicsConfig config = FastConfig();
  config.drive.max_wheel_speed_mps = 4.0f;  // 让车体域成为唯一约束（仍满足可达性校验）
  DifferentialKinematics kinematics;
  ASSERT_TRUE(kinematics.Init(config));
  const encoder::WheelState measurement = Measurement(0.0f, 0.0f);
  uint64_t now_us = kPeriodUs;

  const chassis::WheelTarget target =
      RunCycles(kinematics, Inputs(Command(2.0f, 2.4f), &measurement), now_us, kFastCycles);
  EXPECT_NEAR(target.scale, 0.5f, 1e-5f);  // min(1/2.0, 2.0/2.4)=0.5
  EXPECT_TRUE(chassis::IsSet(target.limit_reason, chassis::LimitReason::kBodySpeed));
  EXPECT_FALSE(chassis::IsSet(target.limit_reason, chassis::LimitReason::kWheelSpeed));
  EXPECT_NEAR(target.left_mps / target.right_mps, target.raw_left_mps / target.raw_right_mps,
              1e-5f);
}

TEST(DifferentialKinematicsTest, ExactlyAtLimitsNotLimited) {
  KinematicsConfig config = FastConfig();
  config.drive.max_body_speed_mps = 1.0f;
  config.drive.max_wheel_speed_mps = 1.0f;
  DifferentialKinematics kinematics;
  ASSERT_TRUE(kinematics.Init(config));
  const encoder::WheelState measurement = Measurement(0.0f, 0.0f);
  uint64_t now_us = kPeriodUs;

  const chassis::WheelTarget target =
      RunCycles(kinematics, Inputs(Command(1.0f, 0.0f), &measurement), now_us, kFastCycles);
  EXPECT_NEAR(target.scale, 1.0f, kEps);
  EXPECT_EQ(target.limit_reason, chassis::LimitReason::kNone);
  EXPECT_NEAR(target.left_mps, 1.0f, 1e-5f);
}

// --- A3：原地转向 ---

TEST(DifferentialKinematicsTest, PivotTurnAllowsOppositeSignsAndIgnoresTwoVOverT) {
  const KinematicsConfig config = FastConfig();
  DifferentialKinematics kinematics;
  ASSERT_TRUE(kinematics.Init(config));
  const encoder::WheelState measurement = Measurement(0.0f, 0.0f);
  uint64_t now_us = kPeriodUs;

  // v=0 且 ω=5.0 > ω_max=2.0：只受角速度上限约束（不使用 |ω| ≤ 2v/T），左右仍异号。
  const chassis::WheelTarget target =
      RunCycles(kinematics, Inputs(Command(0.0f, 5.0f), &measurement), now_us, kFastCycles);
  EXPECT_TRUE(target.valid);
  EXPECT_NEAR(target.scale, 0.4f, 1e-5f);  // 2.0/5.0
  EXPECT_NEAR(target.left_mps, -0.5f, 1e-5f);  // ω_body=2.0 ⇒ ±2.0*0.25
  EXPECT_NEAR(target.right_mps, 0.5f, 1e-5f);
  EXPECT_LT(target.left_mps, 0.0f);
  EXPECT_GT(target.right_mps, 0.0f);
  EXPECT_TRUE(chassis::IsSet(target.limit_reason, chassis::LimitReason::kBodySpeed));
}

// --- A4：边界输入与 fail-closed ---

TEST(DifferentialKinematicsTest, NonFiniteAndInvalidCommandsRejected) {
  const KinematicsConfig config = TestConfig();
  DifferentialKinematics kinematics;
  ASSERT_TRUE(kinematics.Init(config));
  const encoder::WheelState measurement = Measurement(0.0f, 0.0f);
  const float nan_value = std::nanf("");
  const float inf_value = std::numeric_limits<float>::infinity();
  uint64_t now_us = kPeriodUs;

  for (const chassis::MotionCommand& command :
       {Command(nan_value, 0.0f), Command(0.0f, nan_value), Command(inf_value, 0.0f),
        Command(0.0f, inf_value), Command(0.5f, 0.0f, false)}) {
    now_us += kPeriodUs;
    const chassis::WheelTarget target = kinematics.Update(Inputs(command, &measurement), now_us);
    EXPECT_FALSE(target.valid);
    EXPECT_EQ(target.limit_reason, chassis::LimitReason::kInvalid);
    EXPECT_NEAR(target.left_mps, 0.0f, kEps);
    EXPECT_NEAR(target.right_mps, 0.0f, kEps);
    EXPECT_NEAR(target.scale, 0.0f, kEps);
  }
  EXPECT_EQ(kinematics.stats().invalid_commands, 5u);
}

TEST(DifferentialKinematicsTest, InvalidConfigFailsClosed) {
  const KinematicsConfig good = TestConfig();
  const struct {
    const char* name;
    KinematicsConfig config;
  } cases[] = {
      {"zero_track", [] { KinematicsConfig c = TestConfig(); c.drive.track_width_m = 0.0f; return c; }()},
      {"nan_body_speed", [] { KinematicsConfig c = TestConfig(); c.drive.max_body_speed_mps = std::nanf(""); return c; }()},
      {"zero_wheel_speed", [] { KinematicsConfig c = TestConfig(); c.drive.max_wheel_speed_mps = 0.0f; return c; }()},
      {"negative_accel", [] { KinematicsConfig c = TestConfig(); c.max_body_accel_mps2 = -1.0f; return c; }()},
      {"zero_yaw_decel", [] { KinematicsConfig c = TestConfig(); c.max_yaw_decel_radps2 = 0.0f; return c; }()},
      {"degraded_above_wheel", [] { KinematicsConfig c = TestConfig(); c.degraded_max_wheel_speed_mps = 0.9f; return c; }()},
      {"yaw_unreachable", [] { KinematicsConfig c = TestConfig(); c.drive.max_yaw_rate_radps = 10.0f; return c; }()},
      {"dt_bound_too_small", [] { KinematicsConfig c = TestConfig(); c.max_update_interval_us = 500; return c; }()},
  };

  for (const auto& item : cases) {
    EXPECT_FALSE(item.config.IsValid()) << item.name;
    DifferentialKinematics kinematics;
    EXPECT_FALSE(kinematics.Init(item.config)) << item.name;
    EXPECT_FALSE(kinematics.initialized()) << item.name;
    const chassis::WheelTarget target =
        kinematics.Update(Inputs(Command(0.5f, 0.0f), nullptr), kPeriodUs);
    EXPECT_FALSE(target.valid) << item.name;
    EXPECT_EQ(target.limit_reason, chassis::LimitReason::kInvalid) << item.name;
    EXPECT_NEAR(target.left_mps, 0.0f, kEps) << item.name;
  }
  EXPECT_TRUE(good.IsValid());
  // "冗余但安全"的形态必须被接受（评审 H4 triage）：轮速上限低于车体上限只是让车体上限失效。
  KinematicsConfig redundant = TestConfig();
  redundant.drive.max_body_speed_mps = 0.5f;
  redundant.drive.max_yaw_rate_radps = 1.0f;  // 1.0 × 0.25 = 0.25 ≤ 0.4
  redundant.drive.max_wheel_speed_mps = 0.4f;
  EXPECT_TRUE(redundant.IsValid());

  // 失败后可用合法配置重新 Init（不留半初始化状态）
  DifferentialKinematics kinematics;
  ASSERT_FALSE(kinematics.Init(cases[0].config));
  ASSERT_TRUE(kinematics.Init(TestConfig()));
  EXPECT_TRUE(kinematics.initialized());
}

TEST(DifferentialKinematicsTest, NotInitializedFailsClosed) {
  DifferentialKinematics kinematics;
  const chassis::WheelTarget target =
      kinematics.Update(Inputs(Command(0.5f, 0.0f), nullptr), kPeriodUs);
  EXPECT_FALSE(target.valid);
  EXPECT_EQ(target.limit_reason, chassis::LimitReason::kInvalid);
  // 未 Init 的 Update 不污染统计（详设 §7 真值表）
  EXPECT_EQ(kinematics.stats().updates, 0u);
  EXPECT_EQ(kinematics.stats().invalid_commands, 0u);
}

// --- A5：斜坡语义（评审 C2：首个周期只建基线，不存在无约束起点） ---

TEST(DifferentialKinematicsTest, FirstCycleEstablishesBaselineOnly) {
  const KinematicsConfig config = TestConfig();
  DifferentialKinematics kinematics;
  ASSERT_TRUE(kinematics.Init(config));
  const encoder::WheelState measurement = Measurement(0.0f, 0.0f);

  const chassis::WheelTarget first =
      kinematics.Update(Inputs(Command(0.5f, 0.0f), &measurement), kPeriodUs);
  EXPECT_NEAR(first.left_mps, 0.0f, kEps);
  EXPECT_NEAR(first.right_mps, 0.0f, kEps);
  EXPECT_TRUE(first.valid);
  EXPECT_TRUE(chassis::IsSet(first.limit_reason, chassis::LimitReason::kAccelLimit));
  EXPECT_NEAR(kinematics.ramped_body_speed_mps(), 0.0f, kEps);
}

TEST(DifferentialKinematicsTest, RampLimitsAccelerationAndReachesTarget) {
  const KinematicsConfig config = TestConfig();
  DifferentialKinematics kinematics;
  ASSERT_TRUE(kinematics.Init(config));
  const encoder::WheelState measurement = Measurement(0.0f, 0.0f);
  const KinematicsInputs inputs = Inputs(Command(0.5f, 0.0f), &measurement);

  uint64_t now_us = kPeriodUs;
  kinematics.Update(inputs, now_us);  // 基线周期
  const float max_step = config.max_body_accel_mps2 * (static_cast<float>(kPeriodUs) / 1e6f);
  float previous = 0.0f;
  uint32_t ramp_limited_periods = 0;
  for (int i = 0; i < 200; ++i) {  // 200 × 5 ms = 1.0 s
    now_us += kPeriodUs;
    const chassis::WheelTarget target = kinematics.Update(inputs, now_us);
    const float step = target.left_mps - previous;
    EXPECT_LE(step, max_step + 1e-6f) << "cycle " << i;
    EXPECT_GE(step, 0.0f) << "cycle " << i;
    if (chassis::IsSet(target.limit_reason, chassis::LimitReason::kAccelLimit)) {
      ++ramp_limited_periods;
    }
    previous = target.left_mps;
  }
  EXPECT_NEAR(previous, 0.5f, 1e-3f);  // 0.5 m/s² × 1 s 到 0.5 m/s
  EXPECT_GT(ramp_limited_periods, 0u);
  EXPECT_GT(kinematics.stats().ramp_clamped, 0u);

  // 到达后保持：不超调、不再置 kAccelLimit
  now_us += kPeriodUs;
  const chassis::WheelTarget settled = kinematics.Update(inputs, now_us);
  EXPECT_NEAR(settled.left_mps, 0.5f, 1e-4f);
  EXPECT_FALSE(chassis::IsSet(settled.limit_reason, chassis::LimitReason::kAccelLimit));
}

TEST(DifferentialKinematicsTest, RampLimitsDecelerationOnLowerTarget) {
  const KinematicsConfig config = FastConfig();  // 先快速建立高速状态
  DifferentialKinematics kinematics;
  ASSERT_TRUE(kinematics.Init(config));
  const encoder::WheelState measurement = Measurement(0.0f, 0.0f);

  uint64_t now_us = kPeriodUs;
  const chassis::WheelTarget fast =
      RunCycles(kinematics, Inputs(Command(0.8f, 0.0f), &measurement), now_us, kFastCycles);
  ASSERT_NEAR(fast.left_mps, 0.8f, 1e-5f);

  // 切换到真实减速度：命令仍有效但目标变小 ⇒ 受控降速（D-005-2）
  KinematicsConfig slow = TestConfig();
  DifferentialKinematics decelerating;
  ASSERT_TRUE(decelerating.Init(slow));
  uint64_t slow_now = kPeriodUs;
  decelerating.Update(Inputs(Command(0.8f, 0.0f), &measurement), slow_now);  // 基线
  slow_now += kPeriodUs;
  const chassis::WheelTarget top =
      decelerating.Update(Inputs(Command(0.8f, 0.0f), &measurement), slow_now);
  EXPECT_NEAR(top.left_mps, 0.8f * (slow.max_body_accel_mps2 * 0.005f) / 0.8f, 1e-3f);

  const float max_step = slow.max_body_decel_mps2 * (static_cast<float>(kPeriodUs) / 1e6f);
  float previous = top.left_mps;
  for (int i = 0; i < 5; ++i) {
    slow_now += kPeriodUs;
    const chassis::WheelTarget target =
        decelerating.Update(Inputs(Command(0.0f, 0.0f), &measurement), slow_now);
    EXPECT_LE(previous - target.left_mps, max_step + 1e-6f) << "cycle " << i;
    EXPECT_LE(target.left_mps, previous);
    previous = target.left_mps;
  }
}

TEST(DifferentialKinematicsTest, CrossZeroReversalStaysWithinRateBounds) {
  // 评审 H2：车体域斜坡在换向时会让左右轮速跨零；变化率上界
  // |Δv_wheel| ≤ (a_v + a_ω·T/2)·Δt 必须成立（换向的过零保护属 006 输出域）。
  const KinematicsConfig config = TestConfig();
  DifferentialKinematics kinematics;
  ASSERT_TRUE(kinematics.Init(config));
  const encoder::WheelState measurement = Measurement(0.0f, 0.0f);
  const float dt_s = static_cast<float>(kPeriodUs) / 1e6f;
  const float bound =
      (config.max_body_decel_mps2 + config.max_yaw_decel_radps2 * config.drive.track_width_m / 2.0f) *
      dt_s;

  uint64_t now_us = kPeriodUs;
  const chassis::WheelTarget forward =
      RunCycles(kinematics, Inputs(Command(0.5f, 0.0f), &measurement), now_us, 220);
  ASSERT_GT(forward.left_mps, 0.4f);

  float previous = forward.left_mps;
  uint32_t crossings = 0;
  for (int i = 0; i < 300; ++i) {  // 1.5 s
    now_us += kPeriodUs;
    const chassis::WheelTarget target =
        kinematics.Update(Inputs(Command(-0.5f, 0.0f), &measurement), now_us);
    EXPECT_LE(std::fabs(target.left_mps - previous), bound + 1e-6f) << "cycle " << i;
    if ((previous > 0.0f) != (target.left_mps > 0.0f)) {
      ++crossings;
    }
    previous = target.left_mps;
  }
  EXPECT_GE(crossings, 1u);        // 确实跨过零（换向场景被覆盖）
  EXPECT_LT(previous, -0.4f);      // 最终到达反向目标
}

TEST(DifferentialKinematicsTest, DtZeroDoesNotAdvanceAndIsCounted) {
  const KinematicsConfig config = TestConfig();
  DifferentialKinematics kinematics;
  ASSERT_TRUE(kinematics.Init(config));
  const encoder::WheelState measurement = Measurement(0.0f, 0.0f);

  uint64_t now_us = kPeriodUs;
  kinematics.Update(Inputs(Command(0.0f, 0.0f), &measurement), now_us);
  now_us += kPeriodUs;
  const chassis::WheelTarget first =
      kinematics.Update(Inputs(Command(0.5f, 0.0f), &measurement), now_us);
  const chassis::WheelTarget second =
      kinematics.Update(Inputs(Command(0.5f, 0.0f), &measurement), now_us);  // dt == 0
  EXPECT_NEAR(second.left_mps, first.left_mps, kEps);
  EXPECT_EQ(kinematics.stats().timestamp_anomalies, 1u);
}

TEST(DifferentialKinematicsTest, DtClampedAndBackwardsTimeCountedAndBounded) {
  const KinematicsConfig config = TestConfig();
  DifferentialKinematics kinematics;
  ASSERT_TRUE(kinematics.Init(config));
  const encoder::WheelState measurement = Measurement(0.0f, 0.0f);

  uint64_t now_us = kPeriodUs;
  kinematics.Update(Inputs(Command(0.0f, 0.0f), &measurement), now_us);
  now_us += 1'000'000;  // 1 s：超过 max_update_interval_us = 20 ms
  const chassis::WheelTarget clamped =
      kinematics.Update(Inputs(Command(0.5f, 0.0f), &measurement), now_us);
  const float bound = config.max_body_accel_mps2 * 0.02f;
  EXPECT_LE(clamped.left_mps, bound + 1e-6f);
  EXPECT_EQ(kinematics.stats().dt_clamped, 1u);

  const float before = clamped.left_mps;
  now_us -= kPeriodUs;  // 时间倒退（早于上次 Update 时刻）
  const chassis::WheelTarget backwards =
      kinematics.Update(Inputs(Command(0.5f, 0.0f), &measurement), now_us);
  EXPECT_NEAR(backwards.left_mps, before, kEps);
  EXPECT_EQ(kinematics.stats().timestamp_anomalies, 1u);
}

// --- A6：立即零语义 ---

TEST(DifferentialKinematicsTest, MotionGateFalseYieldsImmediateSafetyZero) {
  const KinematicsConfig config = TestConfig();
  DifferentialKinematics kinematics;
  ASSERT_TRUE(kinematics.Init(config));
  const encoder::WheelState measurement = Measurement(0.0f, 0.0f);

  uint64_t now_us = kPeriodUs;
  const chassis::WheelTarget moving =
      RunCycles(kinematics, Inputs(Command(0.5f, 0.0f), &measurement), now_us, 220);
  ASSERT_GT(moving.left_mps, 0.4f);

  now_us += kPeriodUs;
  const chassis::WheelTarget stopped =
      kinematics.Update(Inputs(Command(0.5f, 0.0f), &measurement, /*motion_allowed=*/false), now_us);
  EXPECT_NEAR(stopped.left_mps, 0.0f, kEps);
  EXPECT_NEAR(stopped.right_mps, 0.0f, kEps);
  EXPECT_NEAR(stopped.scale, 0.0f, kEps);
  EXPECT_TRUE(stopped.valid);  // 命令语义有效，禁止来自运行门/安全
  EXPECT_EQ(stopped.limit_reason, chassis::LimitReason::kSafety);
  EXPECT_EQ(kinematics.stats().blocked_by_motion_gate, 1u);

  // 恢复后必须从零重新加速（不残留停机前的目标）
  now_us += kPeriodUs;
  const chassis::WheelTarget resumed =
      kinematics.Update(Inputs(Command(0.5f, 0.0f), &measurement), now_us);
  const float max_step = config.max_body_accel_mps2 * (static_cast<float>(kPeriodUs) / 1e6f);
  EXPECT_LE(resumed.left_mps, max_step + 1e-6f);
  EXPECT_GT(resumed.left_mps, 0.0f);
}

TEST(DifferentialKinematicsTest, InvalidCommandZeroesAndResetsRamp) {
  const KinematicsConfig config = FastConfig();
  DifferentialKinematics kinematics;
  ASSERT_TRUE(kinematics.Init(config));
  const encoder::WheelState measurement = Measurement(0.0f, 0.0f);

  uint64_t now_us = kPeriodUs;
  const chassis::WheelTarget moving =
      RunCycles(kinematics, Inputs(Command(0.5f, 0.0f), &measurement), now_us, kFastCycles);
  ASSERT_NEAR(moving.left_mps, 0.5f, 1e-5f);

  now_us += kPeriodUs;
  const chassis::WheelTarget invalid =
      kinematics.Update(Inputs(Command(0.5f, 0.0f, false), &measurement), now_us);
  EXPECT_FALSE(invalid.valid);
  EXPECT_EQ(invalid.limit_reason, chassis::LimitReason::kInvalid);
  EXPECT_NEAR(invalid.left_mps, 0.0f, kEps);
}

// --- A7：质量降级 ---

TEST(DifferentialKinematicsTest, MissingMeasurementDegradesWheelLimit) {
  const KinematicsConfig config = FastConfig();
  DifferentialKinematics kinematics;
  ASSERT_TRUE(kinematics.Init(config));
  uint64_t now_us = kPeriodUs;

  const chassis::WheelTarget target =
      RunCycles(kinematics, Inputs(Command(1.0f, 0.0f), nullptr), now_us, kFastCycles);
  // 关键断言是"发布目标被降级上限硬约束"（INV-005-1）。
  // 注意 scale 的语义：它是"域限幅比例"（作用在斜坡后的车体意图上），斜坡滞后时并不等于
  // degraded/command（详设 D-005-6 修订 + 评审 C3），因此这里只断言 <1 与原因位。
  EXPECT_NEAR(target.left_mps, 0.2f, 1e-5f);
  EXPECT_NEAR(target.right_mps, 0.2f, 1e-5f);
  EXPECT_LT(target.scale, 1.0f);
  EXPECT_TRUE(chassis::IsSet(target.limit_reason, chassis::LimitReason::kQualityDegraded));
  EXPECT_TRUE(chassis::IsSet(target.limit_reason, chassis::LimitReason::kWheelSpeed));
  EXPECT_EQ(kinematics.stats().quality_degraded, static_cast<uint32_t>(kFastCycles) + 1u);
}

TEST(DifferentialKinematicsTest, BlockingQualityBitDegrades) {
  const KinematicsConfig config = FastConfig();
  DifferentialKinematics kinematics;
  ASSERT_TRUE(kinematics.Init(config));
  const encoder::WheelState measurement = Measurement(
      0.0f, 0.0f, encoder::kQualityCountOutOfRange, encoder::kQualityValid, false, true);
  uint64_t now_us = kPeriodUs;

  const chassis::WheelTarget target =
      RunCycles(kinematics, Inputs(Command(1.0f, 0.0f), &measurement), now_us, kFastCycles);
  EXPECT_NEAR(target.left_mps, 0.2f, 1e-5f);
  EXPECT_TRUE(chassis::IsSet(target.limit_reason, chassis::LimitReason::kQualityDegraded));
}

TEST(DifferentialKinematicsTest, NonBlockingQualityBitsDoNotDegrade) {
  const KinematicsConfig config = FastConfig();
  const struct {
    const char* name;
    uint32_t left_quality;
    bool left_valid;
  } cases[] = {
      {"accumulating", encoder::kQualityAccumulating, false},
      {"stall_candidate", encoder::kQualityStallCandidate, true},
      {"no_count_change", encoder::kQualityNoCountChange, true},
      {"low_speed_timeout", encoder::kQualityLowSpeedTimeout, true},
  };

  for (const auto& item : cases) {
    DifferentialKinematics kinematics;
    ASSERT_TRUE(kinematics.Init(config));
    const encoder::WheelState measurement =
        Measurement(0.0f, 0.0f, item.left_quality, encoder::kQualityValid, item.left_valid, true);
    uint64_t now_us = kPeriodUs;
    const chassis::WheelTarget target =
        RunCycles(kinematics, Inputs(Command(1.0f, 0.0f), &measurement), now_us, kFastCycles);
    // 正常上限 0.8（而非降级上限 0.2），且不得置降级位
    EXPECT_NEAR(target.left_mps, 0.8f, 1e-5f) << item.name;
    EXPECT_FALSE(chassis::IsSet(target.limit_reason, chassis::LimitReason::kQualityDegraded))
        << item.name;
  }
}

TEST(DifferentialKinematicsTest, DegradeRecoveryIsRampLimited) {
  const KinematicsConfig config = TestConfig();
  DifferentialKinematics kinematics;
  ASSERT_TRUE(kinematics.Init(config));

  uint64_t now_us = kPeriodUs;
  // 第一周期：无测量 ⇒ 降级；基线周期输出零，但仍处于降级状态
  const chassis::WheelTarget degraded =
      kinematics.Update(Inputs(Command(0.5f, 0.0f), nullptr), now_us);
  EXPECT_NEAR(degraded.left_mps, 0.0f, kEps);
  EXPECT_EQ(kinematics.stats().quality_degraded, 1u);

  // 推进若干周期直到降级上限（0.2 m/s）被触及
  for (int i = 0; i < 200; ++i) {
    now_us += kPeriodUs;
    kinematics.Update(Inputs(Command(0.5f, 0.0f), nullptr), now_us);
  }
  const chassis::WheelTarget at_degrade =
      kinematics.Update(Inputs(Command(0.5f, 0.0f), nullptr), now_us + kPeriodUs);
  EXPECT_NEAR(at_degrade.left_mps, 0.2f, 1e-4f);
  EXPECT_TRUE(chassis::IsSet(at_degrade.limit_reason, chassis::LimitReason::kQualityDegraded));

  // 质量恢复：上限回到 0.8，但目标上升仍受加速斜坡约束（发布值反写斜坡状态）
  const encoder::WheelState measurement = Measurement(0.0f, 0.0f);
  const float max_step = config.max_body_accel_mps2 * (static_cast<float>(kPeriodUs) / 1e6f);
  const uint64_t recover_us = now_us + 2 * kPeriodUs;
  const chassis::WheelTarget recovering =
      kinematics.Update(Inputs(Command(0.5f, 0.0f), &measurement), recover_us);
  EXPECT_GT(recovering.left_mps, 0.2f);
  EXPECT_LE(recovering.left_mps, 0.2f + max_step + 1e-6f);
  EXPECT_FALSE(chassis::IsSet(recovering.limit_reason, chassis::LimitReason::kQualityDegraded));
}

// --- 正运动学 FK 与质量三态 ---

TEST(BodyStateTest, EstimateBodyVelocityStraightAndTurn) {
  const KinematicsConfig config = TestConfig();
  const BodyVelocityEstimate forward = EstimateBodyVelocity(config.drive.track_width_m, Measurement(0.3f, 0.3f));
  EXPECT_TRUE(forward.valid);
  EXPECT_EQ(forward.state, MeasurementState::kFresh);
  EXPECT_NEAR(forward.v_mps, 0.3f, kEps);
  EXPECT_NEAR(forward.omega_radps, 0.0f, kEps);

  const BodyVelocityEstimate turning = EstimateBodyVelocity(config.drive.track_width_m, Measurement(0.1f, 0.3f));
  EXPECT_TRUE(turning.valid);
  EXPECT_NEAR(turning.v_mps, 0.2f, kEps);
  EXPECT_NEAR(turning.omega_radps, 0.4f, kEps);  // (0.3−0.1)/0.5

  const BodyVelocityEstimate in_place = EstimateBodyVelocity(config.drive.track_width_m, Measurement(-0.2f, 0.2f));
  EXPECT_TRUE(in_place.valid);
  EXPECT_NEAR(in_place.v_mps, 0.0f, kEps);
  EXPECT_NEAR(in_place.omega_radps, 0.8f, kEps);
}

TEST(BodyStateTest, MeasurementStateTripleAndSideInformation) {
  const KinematicsConfig config = TestConfig();

  // 阻塞位：kUnavailable —— "没有测量"，数值字段不得被读作"测到零"
  const BodyVelocityEstimate blocked = EstimateBodyVelocity(
      config.drive.track_width_m, Measurement(0.3f, 0.3f, encoder::kQualityHardwareFault, encoder::kQualityValid, false,
                          true));
  EXPECT_FALSE(blocked.valid);
  EXPECT_EQ(blocked.state, MeasurementState::kUnavailable);
  EXPECT_NEAR(blocked.v_mps, 0.0f, kEps);
  EXPECT_FALSE(IsMeasurementTrustworthy(blocked));

  // 低速累加中：kStale（值为上次发布值，有界：≤ max_estimation_delay 后必转 fresh 或发布零）
  const BodyVelocityEstimate accumulating = EstimateBodyVelocity(
      config.drive.track_width_m, Measurement(0.07f, 0.07f, encoder::kQualityAccumulating, encoder::kQualityAccumulating,
                          false, false));
  EXPECT_FALSE(accumulating.valid);
  EXPECT_EQ(accumulating.state, MeasurementState::kStale);
  EXPECT_TRUE(IsMeasurementTrustworthy(accumulating));
  EXPECT_NEAR(accumulating.v_mps, 0.07f, kEps);

  // 单侧无效：保留逐侧信息，不能只报"整体坏"
  const BodyVelocityEstimate one_side = EstimateBodyVelocity(
      config.drive.track_width_m, Measurement(0.2f, 0.2f, encoder::kQualityStallCandidate, encoder::kQualityValid, true,
                          false));
  EXPECT_FALSE(one_side.valid);
  EXPECT_EQ(one_side.state, MeasurementState::kStale);
  EXPECT_TRUE(one_side.left_valid);
  EXPECT_FALSE(one_side.right_valid);
  EXPECT_NE(one_side.quality & encoder::kQualityStallCandidate, 0u);
  EXPECT_EQ(one_side.left_quality, encoder::kQualityStallCandidate);
  EXPECT_EQ(one_side.right_quality, encoder::kQualityValid);
  EXPECT_TRUE(IsMeasurementTrustworthy(one_side));
}

TEST(BodyStateTest, InvalidGeometryPublishesNothing) {
  KinematicsConfig config = TestConfig();
  config.drive.track_width_m = 0.0f;  // 非法几何（防御路径）
  const BodyVelocityEstimate estimate = EstimateBodyVelocity(config.drive.track_width_m, Measurement(0.3f, 0.3f));
  EXPECT_FALSE(estimate.valid);
  EXPECT_EQ(estimate.state, MeasurementState::kUnavailable);
  EXPECT_NEAR(estimate.v_mps, 0.0f, kEps);
  EXPECT_NEAR(estimate.omega_radps, 0.0f, kEps);
}


// --- 评审驱动的补充用例（H2/H4/H5/S1/S7） ---

TEST(DifferentialKinematicsTest, ConfigRejectsSilentlyIneffectiveDegrade) {
  // H4-1：降级上限必须小于车体线速度上限，否则"降级"在直线行驶时永不生效（配置合法但功能静默失效）。
  KinematicsConfig config = TestConfig();
  config.drive.max_body_speed_mps = 0.6f;
  config.degraded_max_wheel_speed_mps = 0.6f;  // == max_body_speed_mps ⇒ 降级永不生效
  EXPECT_FALSE(config.IsValid());
  config.degraded_max_wheel_speed_mps = 0.61f;
  EXPECT_FALSE(config.IsValid());
  config.degraded_max_wheel_speed_mps = 0.59f;
  EXPECT_TRUE(config.IsValid());
}

TEST(DifferentialKinematicsTest, ConfigRejectsRampStepLargerThanDomain) {
  // H4-4：dt 钳制到上界时单周期变化量不得跨过整个速度域（否则"已知上界"退化）。
  KinematicsConfig config = TestConfig();
  config.max_body_accel_mps2 = 60.0f;   // 60 × 0.02 s = 1.2 m/s > max_body 1.0
  EXPECT_FALSE(config.IsValid());
  config = TestConfig();
  config.drive.max_yaw_rate_radps = 0.5f;  // 2.0 × 0.02 = 0.04 ≤ 0.5 ✔
  config.max_yaw_accel_radps2 = 30.0f;     // 30 × 0.02 = 0.6 > 0.5 ⇒ 拒绝
  EXPECT_FALSE(config.IsValid());
}

TEST(DifferentialKinematicsTest, ConfigRejectsTooNarrowTrack) {
  // H4-5：ω = (right − left)/T 会把量化噪声放大 1/T 倍；< 5 cm 不构成差速底盘。
  KinematicsConfig config = TestConfig();
  config.drive.track_width_m = 0.01f;
  EXPECT_FALSE(config.IsValid());
  config.drive.track_width_m = 0.05f;
  EXPECT_TRUE(config.IsValid());
}

TEST(DifferentialKinematicsTest, ClosedFormScaleMatchesPipeline) {
  // H5-③：用独立解析式做第二判据，避免"两份实现同源错误"的自证。
  const KinematicsConfig config = FastConfig();
  DifferentialKinematics kinematics;
  ASSERT_TRUE(kinematics.Init(config));
  const encoder::WheelState measurement = Measurement(0.0f, 0.0f);
  const chassis::MotionCommand command = Command(0.4f, 2.4f);
  uint64_t now_us = kPeriodUs;

  const chassis::WheelTarget target =
      RunCycles(kinematics, Inputs(command, &measurement), now_us, kFastCycles);
  const float raw_left = command.v_mps - command.omega_radps * config.drive.track_width_m / 2.0f;
  const float raw_right = command.v_mps + command.omega_radps * config.drive.track_width_m / 2.0f;
  const float max_raw = std::max(std::fabs(raw_left), std::fabs(raw_right));
  const float expected = std::min({1.0f, config.drive.max_body_speed_mps / std::fabs(command.v_mps),
                                   config.drive.max_yaw_rate_radps / std::fabs(command.omega_radps),
                                   config.drive.max_wheel_speed_mps / max_raw});
  EXPECT_NEAR(target.scale, expected, 1e-5f);
  EXPECT_NEAR(target.left_mps, raw_left * expected, 1e-5f);
  EXPECT_NEAR(target.right_mps, raw_right * expected, 1e-5f);
}

TEST(DifferentialKinematicsTest, SameValueCommandDoesNotSetAccelLimit) {
  // S7-4：到达目标后再给一个"值相同但 seq 递增"的命令 ⇒ 目标不变、不得置 kAccelLimit。
  const KinematicsConfig config = FastConfig();
  DifferentialKinematics kinematics;
  ASSERT_TRUE(kinematics.Init(config));
  const encoder::WheelState measurement = Measurement(0.0f, 0.0f);
  chassis::MotionCommand command = Command(0.3f, 0.0f);
  uint64_t now_us = kPeriodUs;

  chassis::WheelTarget target =
      RunCycles(kinematics, Inputs(command, &measurement), now_us, kFastCycles);
  ASSERT_NEAR(target.left_mps, 0.3f, 1e-5f);
  EXPECT_FALSE(chassis::IsSet(target.limit_reason, chassis::LimitReason::kAccelLimit));

  command.seq += 1;
  now_us += kPeriodUs;
  target = kinematics.Update(Inputs(command, &measurement), now_us);
  EXPECT_NEAR(target.left_mps, 0.3f, 1e-6f);
  EXPECT_FALSE(chassis::IsSet(target.limit_reason, chassis::LimitReason::kAccelLimit));
}

TEST(DifferentialKinematicsTest, PivotTurnSwitchMidRampCrossesZeroWithinRateBound) {
  // H2-②：直行中途切到原地转向 —— 左轮必然穿越零，且单轮变化率不超过车体域上界
  // (a_v + a_ω·T/2)·dt。
  const KinematicsConfig config = TestConfig();
  DifferentialKinematics kinematics;
  ASSERT_TRUE(kinematics.Init(config));
  const encoder::WheelState measurement = Measurement(0.0f, 0.0f);
  const float dt_s = static_cast<float>(kPeriodUs) / 1e6f;
  const float bound =
      (config.max_body_decel_mps2 + config.max_yaw_accel_radps2 * config.drive.track_width_m / 2.0f) *
      dt_s;

  uint64_t now_us = kPeriodUs;
  const chassis::WheelTarget straight =
      RunCycles(kinematics, Inputs(Command(0.3f, 0.0f), &measurement), now_us, 200);
  ASSERT_NEAR(straight.left_mps, 0.3f, 1e-3f);

  const KinematicsInputs pivot = Inputs(Command(0.0f, 2.0f), &measurement);
  float previous = straight.left_mps;
  uint32_t crossings = 0;
  for (int i = 0; i < 400; ++i) {
    now_us += kPeriodUs;
    const chassis::WheelTarget target = kinematics.Update(pivot, now_us);
    EXPECT_LE(std::fabs(target.left_mps - previous), bound + 1e-6f) << "cycle " << i;
    // 车体域不变量：目标轮速始终在轮速上限内（INV-005-1）
    EXPECT_LE(std::fabs(target.left_mps), config.drive.max_wheel_speed_mps + 1e-6f);
    EXPECT_LE(std::fabs(target.right_mps), config.drive.max_wheel_speed_mps + 1e-6f);
    if ((previous > 0.0f) != (target.left_mps > 0.0f)) {
      ++crossings;
    }
    previous = target.left_mps;
  }
  EXPECT_GE(crossings, 1u);
  EXPECT_NEAR(previous, -0.5f, 1e-3f);  // ω=2.0 ⇒ left = −2.0×0.25
}

TEST(DifferentialKinematicsTest, RepeatedInitAndResetSemantics) {
  // S7-7/S7-8：重复 Init 清统计、Reset 后首周期建立基线、now_us 从 0 开始无哨兵二义。
  const KinematicsConfig config = TestConfig();
  DifferentialKinematics kinematics;
  ASSERT_TRUE(kinematics.Init(config));
  const encoder::WheelState measurement = Measurement(0.0f, 0.0f);

  // now_us 从 0 开始：首周期建立基线（输出零），第二周期起爬升
  const chassis::WheelTarget at_zero = kinematics.Update(Inputs(Command(0.5f, 0.0f), &measurement), 0);
  EXPECT_NEAR(at_zero.left_mps, 0.0f, kEps);
  const chassis::WheelTarget next = kinematics.Update(Inputs(Command(0.5f, 0.0f), &measurement),
                                                      kPeriodUs);
  EXPECT_GT(next.left_mps, 0.0f);
  EXPECT_GT(kinematics.stats().updates, 0u);

  // 重复 Init：统计与斜坡状态清零
  ASSERT_TRUE(kinematics.Init(config));
  EXPECT_EQ(kinematics.stats().updates, 0u);
  EXPECT_EQ(kinematics.stats().ramp_clamped, 0u);
  EXPECT_NEAR(kinematics.ramped_body_speed_mps(), 0.0f, kEps);
  EXPECT_EQ(kinematics.stats().max_ramp_lag_mps, 0.0f);

  // Reset 后首周期同样只建立基线；未 Init 的 Update 不计入 updates（fail-closed 且不污染统计）

  DifferentialKinematics uninitialized;
  uninitialized.Update(Inputs(Command(0.5f, 0.0f), nullptr), kPeriodUs);
  EXPECT_EQ(uninitialized.stats().updates, 0u);
  EXPECT_EQ(uninitialized.stats().invalid_commands, 0u);
}

}  // namespace
}  // namespace robotcar01::kinematics
