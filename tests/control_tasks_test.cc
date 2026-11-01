// 控制任务体装配测试（Iteration 006，A7）：TaskSet + CooperativeScheduler + FakeClock 驱动
// 「命令 → 005 运动学 → 006 控制 → 仲裁 → FakePwmSink → FakeMotorPlant → 004 估计器」垂直切片，
// 并承接退役的 001 walking_skeleton 五个用例（详设 §9.1）。
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "app/chassis/chassis_types.h"
#include "app/control/control_tasks.h"
#include "app/control/output_arbitration.h"
#include "app/control/wheel_speed_controller.h"
#include "app/encoder/encoder_estimator.h"
#include "app/kinematics/differential_kinematics.h"
#include "app/kinematics/kinematics_config.h"
#include "app/mcu_os_lite/health_monitor.h"
#include "app/mcu_os_lite/scheduler.h"
#include "host_fakes/fake_clock.h"
#include "host_fakes/fake_encoder_source.h"
#include "host_fakes/fake_motor_plant.h"
#include "host_fakes/fake_pwm_sink.h"

namespace robotcar01::control {
namespace {

using chassis::LimitReason;

encoder::EncoderConfig MakeEncoderConfig() {
  encoder::EncoderConfig config{};
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

// 装配夹具：真实调度器 + 运动学/编码器"装配层"任务 + 006 两个任务体
class Vehicle {
 public:
  Vehicle() {
    EXPECT_TRUE(kinematics.Init(kinematics::KinematicsConfig{}));
    EXPECT_TRUE(estimator.Init(MakeEncoderConfig()));
    source.Init(MakeEncoderConfig());
    EXPECT_TRUE(left.Init(WheelControllerConfig{}));
    EXPECT_TRUE(right.Init(WheelControllerConfig{}));
    EXPECT_TRUE(arbitration.Init(sink, ArbitrationConfig{}));

    ctx.clock = &clock;
    ctx.target = &target;
    ctx.wheel_state = &wheel_state;
    ctx.safety = &safety;
    ctx.left = &left;
    ctx.right = &right;
    ctx.arbitration = &arbitration;

    scheduler.Init(mcu_os_lite::TickConfig{}, clock, task_set, health);
    health.Init(mcu_os_lite::HealthConfig{}, task_set);
    // 装配层任务（相位 1）：plant 推进 + 编码器采样 + 005 运动学（006 只装配，不改 004/005）
    sense_config.name = "SenseAndKinematics";
    sense_config.period_ticks = 5;
    sense_config.phase_ticks = 1;
    EXPECT_TRUE(task_set.Register(sense_config, &Vehicle::SenseRun, this, 100));
    EXPECT_TRUE(RegisterControlTasks(task_set, ctx, 100));
  }

  static void SenseRun(void* self) { static_cast<Vehicle*>(self)->Sense(); }

  void Sense() {
    // plant 由仲裁后真实写入 sink 的最后一次输出驱动（证明 PWM 只经仲裁到达"电机"）
    if (!sink.records().empty()) {
      left_plant.Step(sink.records().back().left, 5000);
      right_plant.Step(sink.records().back().right, 5000);
    }
    source.SetWheelSpeeds(left_plant.speed(), right_plant.speed());
    encoder::EncoderSnapshot snapshot = source.Step(5000);
    snapshot.timestamp = clock.Now();
    estimator.OnSample(snapshot, safety.allow_motion && command.valid);
    wheel_state = estimator.state();

    kinematics::KinematicsInputs inputs{};
    inputs.command = command;
    inputs.motion_allowed = safety.allow_motion;
    inputs.measurement = &wheel_state;
    target = kinematics.Update(inputs, clock.Now().us);
  }

  void RunMs(uint32_t ms) {
    for (uint32_t i = 0; i < ms; ++i) {
      ASSERT_TRUE(clock.AdvanceUs(1000));
      scheduler.TickLoopOnce();
    }
  }

  void Command(float v, float omega, bool valid = true) {
    command.v_mps = v;
    command.omega_radps = omega;
    command.valid = valid;
  }

  host_fakes::FakeClock clock;
  mcu_os_lite::TaskSet task_set;
  mcu_os_lite::HealthMonitor health;
  mcu_os_lite::CooperativeScheduler scheduler;
  mcu_os_lite::TaskConfig sense_config{};

  kinematics::DifferentialKinematics kinematics;
  encoder::EncoderEstimator estimator;
  host_fakes::FakeEncoderSource source;
  host_fakes::FakeMotorPlant left_plant;
  host_fakes::FakeMotorPlant right_plant;
  host_fakes::FakePwmSink sink;
  WheelSpeedController left;
  WheelSpeedController right;
  OutputArbitration arbitration;

  chassis::MotionCommand command{};
  chassis::WheelTarget target{};
  encoder::WheelState wheel_state{};
  chassis::SafetyStatus safety{};
  ControlContext ctx{};
};

TEST(ControlTasksTest, TaskConfigsMatchRegisteredTable) {
  EXPECT_EQ(WheelControlTaskConfig().period_ticks, 5u);
  EXPECT_EQ(WheelControlTaskConfig().phase_ticks, 2u);
  EXPECT_EQ(WheelControlTaskConfig().criticality, mcu_os_lite::TaskCriticality::kCritical);
  EXPECT_EQ(OutputArbitrationTaskConfig().period_ticks, 1u);
  EXPECT_EQ(OutputArbitrationTaskConfig().criticality, mcu_os_lite::TaskCriticality::kCritical);
}

TEST(ControlTasksTest, PeriodsAreFiveAndOneMilliseconds) {
  Vehicle car;
  car.safety.allow_motion = true;
  car.RunMs(100);
  EXPECT_EQ(car.left.stats().updates, 20u);       // 5 ms
  EXPECT_EQ(car.arbitration.stats().ticks, 100u);  // 1 ms
  EXPECT_EQ(car.sink.records().size(), car.arbitration.stats().ticks);  // 唯一写入口
}

// 承接 001 ValidSmallCommandProducesLimitedTarget + 闭环收敛（A7）
TEST(ControlTasksTest, ValidCommandDrivesBothWheelsToTarget) {
  Vehicle car;
  car.safety.allow_motion = true;
  car.Command(0.25f, 0.0f);
  car.RunMs(3000);
  EXPECT_NEAR(car.left_plant.speed(), 0.25f, 0.06f);
  EXPECT_NEAR(car.right_plant.speed(), 0.25f, 0.06f);
  EXPECT_GT(car.sink.records().back().left.duty_permille, 0u);
  EXPECT_FALSE(car.ctx.speed_request.force_disable);
  EXPECT_NEAR(car.ctx.speed_request.left_mps, car.target.left_mps, 1e-6f);
}

// 承接 001 OverLimitCommandIsScaled
TEST(ControlTasksTest, OverLimitCommandIsScaledAndReported) {
  Vehicle car;
  car.safety.allow_motion = true;
  car.Command(2.0f, 0.0f);  // > max_body_speed 0.5
  car.RunMs(3000);
  EXPECT_LE(car.target.left_mps, 0.5f + 1e-4f);
  EXPECT_NE(static_cast<uint32_t>(car.ctx.speed_request.limit_reason) &
                static_cast<uint32_t>(LimitReason::kBodySpeed),
            0u);
  EXPECT_LE(car.left_plant.speed(), 0.55f);
}

// 承接 001 InvalidCommandForcesDisable
TEST(ControlTasksTest, InvalidCommandForcesDisableAndCoast) {
  Vehicle car;
  car.safety.allow_motion = true;
  car.Command(0.25f, 0.0f, /*valid=*/false);
  car.RunMs(200);
  EXPECT_TRUE(car.ctx.speed_request.force_disable);
  EXPECT_NE(static_cast<uint32_t>(car.ctx.speed_request.limit_reason) &
                static_cast<uint32_t>(LimitReason::kInvalid),
            0u);
  for (const auto& r : car.sink.records()) {
    EXPECT_EQ(r.left.duty_permille, 0u);
    EXPECT_EQ(r.right.duty_permille, 0u);
  }
}

// 承接 001 SafetyBlockedForcesDisable（A6 端到端）
TEST(ControlTasksTest, SafetyBlockedForcesCoastWithinOneTick) {
  Vehicle car;
  car.safety.allow_motion = true;
  car.Command(0.25f, 0.0f);
  car.RunMs(1000);
  ASSERT_GT(car.sink.records().back().left.duty_permille, 0u);
  car.safety.allow_motion = false;
  car.RunMs(1);  // 仲裁 1 ms 周期：下一次 Tick 即零
  EXPECT_EQ(car.sink.records().back().left.duty_permille, 0u);
  EXPECT_EQ(car.sink.records().back().right.duty_permille, 0u);
  car.RunMs(500);
  for (size_t i = car.sink.records().size() - 500; i < car.sink.records().size(); ++i) {
    EXPECT_EQ(car.sink.records()[i].left.duty_permille, 0u);
  }
  EXPECT_TRUE(car.ctx.speed_request.force_disable);
}

// 承接 001 SinkReceivesArbitratedRequest：sink 只收到仲裁后输出
TEST(ControlTasksTest, SinkOnlyReceivesArbitratedOutput) {
  Vehicle car;
  car.safety.allow_motion = true;
  car.Command(0.25f, 0.0f);
  car.RunMs(500);
  EXPECT_EQ(car.sink.records().size(), car.arbitration.stats().apply_calls);
  EXPECT_EQ(car.arbitration.last_left().duty_permille, car.sink.records().back().left.duty_permille);
}

// 原地转向：左右异号，经过零保护无带载换向
TEST(ControlTasksTest, SpinInPlaceAndReverseWithoutPlugging) {
  Vehicle car;
  car.safety.allow_motion = true;
  car.Command(0.0f, 1.5f);
  car.RunMs(2000);
  EXPECT_LT(car.left_plant.speed(), -0.1f);
  EXPECT_GT(car.right_plant.speed(), 0.1f);
  car.Command(0.0f, -1.5f);  // 反向原地转
  car.RunMs(3000);
  EXPECT_GT(car.left_plant.speed(), 0.1f);
  EXPECT_LT(car.right_plant.speed(), -0.1f);
  EXPECT_EQ(car.left_plant.plugging_events(), 0u);
  EXPECT_EQ(car.right_plant.plugging_events(), 0u);
}

TEST(ControlTasksTest, MissingWheelStateKeepsOutputsZero) {
  Vehicle car;
  car.ctx.wheel_state = nullptr;  // 无测量数据源
  car.safety.allow_motion = true;
  car.Command(0.25f, 0.0f);
  car.RunMs(200);
  for (const auto& r : car.sink.records()) {
    EXPECT_EQ(r.left.duty_permille, 0u);
  }
}

TEST(ControlTasksTest, HealthStaysGoodWithinBudget) {
  Vehicle car;
  car.safety.allow_motion = true;
  car.Command(0.25f, 0.0f);
  car.RunMs(1000);
  EXPECT_FALSE(car.health.snapshot().stop_active);
  EXPECT_FALSE(car.health.snapshot().fault_latched);
}

}  // namespace
}  // namespace robotcar01::control
