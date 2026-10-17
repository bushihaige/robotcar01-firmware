// 输出仲裁测试（Iteration 006）：A6 安全/超龄/故障保持与回差、唯一写入口、形状校正。
#include <gtest/gtest.h>

#include <cstdint>

#include "app/control/motor_output.h"
#include "app/control/output_arbitration.h"
#include "host_fakes/fake_pwm_sink.h"

namespace robotcar01::control {
namespace {

MotorOutputRequest Drive(MotorDirection dir, uint16_t duty) {
  MotorOutputRequest r{};
  r.mode = OutputMode::kDrive;
  r.direction = dir;
  r.duty_permille = duty;
  return r;
}

MotorOutputPair Pair(uint64_t at, uint16_t duty = 400, bool fault = false) {
  MotorOutputPair p{};
  p.left = Drive(MotorDirection::kForward, duty);
  p.right = Drive(MotorDirection::kReverse, duty);
  p.left.fault = fault;
  p.produced_at_us = at;
  return p;
}

class ArbitrationTest : public ::testing::Test {
 protected:
  void SetUp() override { ASSERT_TRUE(arbitration.Init(sink, ArbitrationConfig{})); }
  host_fakes::FakePwmSink sink;
  OutputArbitration arbitration;
};

TEST_F(ArbitrationTest, PassesFreshAllowedRequest) {
  arbitration.Submit(Pair(1000));
  arbitration.Tick(true, 2000);
  ASSERT_EQ(sink.records().size(), 1u);
  EXPECT_EQ(sink.records()[0].left.duty_permille, 400u);
  EXPECT_EQ(sink.records()[0].right.direction, MotorDirection::kReverse);
}

TEST_F(ArbitrationTest, SafetyBlockedForcesCoast) {
  arbitration.Submit(Pair(1000));
  arbitration.Tick(false, 2000);
  ASSERT_EQ(sink.records().size(), 1u);
  EXPECT_EQ(sink.records()[0].left.mode, OutputMode::kCoast);
  EXPECT_EQ(sink.records()[0].left.duty_permille, 0u);
  EXPECT_EQ(sink.records()[0].right.duty_permille, 0u);
  EXPECT_NE(sink.records()[0].left.reason & kOutputSafetyBlocked, 0u);
  EXPECT_EQ(arbitration.stats().safety_zero, 1u);
}

TEST_F(ArbitrationTest, NeverReceivedRequestIsStale) {
  arbitration.Tick(true, 1000);
  ASSERT_EQ(sink.records().size(), 1u);
  EXPECT_EQ(sink.records()[0].left.duty_permille, 0u);
  EXPECT_NE(sink.records()[0].left.reason & kOutputStale, 0u);
}

TEST_F(ArbitrationTest, AgedRequestIsStaleWithStrictBoundary) {
  arbitration.Submit(Pair(1000));
  arbitration.Tick(true, 1000 + 10000);  // 恰好等于上限：仍有效（严格 >）
  arbitration.Tick(true, 1000 + 10001);  // 超龄
  ASSERT_EQ(sink.records().size(), 2u);
  EXPECT_EQ(sink.records()[0].left.duty_permille, 400u);
  EXPECT_EQ(sink.records()[1].left.duty_permille, 0u);
  EXPECT_EQ(arbitration.stats().stale_zero, 1u);
}

TEST_F(ArbitrationTest, FaultHoldsAndRequiresGoodSubmitsToRecover) {
  arbitration.Submit(Pair(1000, 400, /*fault=*/true));
  arbitration.Tick(true, 1500);
  EXPECT_EQ(sink.records().back().left.duty_permille, 0u);
  EXPECT_EQ(sink.records().back().right.duty_permille, 0u);  // 双侧
  // 保持期内即使有好请求也保持零
  for (uint64_t t = 2000; t < 40000; t += 5000) {
    arbitration.Submit(Pair(t));
    arbitration.Tick(true, t + 500);
    EXPECT_EQ(sink.records().back().left.duty_permille, 0u);
  }
  // 保持期满（≥ 50 ms）且已连续 ≥ 3 个好请求 ⇒ 恢复
  arbitration.Submit(Pair(60000));
  arbitration.Tick(true, 60500);
  EXPECT_EQ(sink.records().back().left.duty_permille, 400u);
}

TEST_F(ArbitrationTest, FaultRecoveryNeedsConsecutiveGoodSubmits) {
  arbitration.Submit(Pair(1000, 400, true));
  arbitration.Submit(Pair(2000));
  arbitration.Submit(Pair(3000, 400, true));  // 再次故障：计数清零
  arbitration.Submit(Pair(60000));
  arbitration.Tick(true, 60500);  // 只有 1 个好请求 ⇒ 仍保持
  EXPECT_EQ(sink.records().back().left.duty_permille, 0u);
}

TEST_F(ArbitrationTest, SanitizesMalformedRequests) {
  MotorOutputPair p{};
  p.left.mode = OutputMode::kCoast;
  p.left.duty_permille = 500;  // coast 却带 duty
  p.right = Drive(MotorDirection::kNone, 300);  // drive 却无方向
  p.produced_at_us = 1000;
  arbitration.Submit(p);
  arbitration.Tick(true, 1500);
  EXPECT_EQ(sink.records().back().left.duty_permille, 0u);
  EXPECT_EQ(sink.records().back().right.duty_permille, 0u);
  EXPECT_NE(sink.records().back().right.mode, OutputMode::kDrive);
}

TEST_F(ArbitrationTest, ApplyCalledExactlyOncePerTick) {
  for (uint64_t t = 1000; t < 21000; t += 1000) {
    if (t % 5000 == 0) arbitration.Submit(Pair(t));
    arbitration.Tick(true, t);
  }
  EXPECT_EQ(sink.records().size(), arbitration.stats().ticks);
  EXPECT_EQ(arbitration.stats().apply_calls, arbitration.stats().ticks);
}

TEST_F(ArbitrationTest, PlatformErrorIsCounted) {
  arbitration.Submit(Pair(1000));
  sink.FailNext();
  arbitration.Tick(true, 1500);
  EXPECT_EQ(arbitration.stats().platform_errors, 1u);
}

TEST(ArbitrationInitTest, InvalidConfigRejected) {
  host_fakes::FakePwmSink sink;
  OutputArbitration arbitration;
  ArbitrationConfig bad{};
  bad.max_request_age_us = 0;
  EXPECT_FALSE(arbitration.Init(sink, bad));
  arbitration.Tick(true, 1000);  // 未初始化：不得写 PWM
  EXPECT_TRUE(sink.records().empty());
}

}  // namespace
}  // namespace robotcar01::control
