// 反馈测试（Iteration 003）：A6 往返一致性、flags_valid 子集语义、limit_reason 合成、
// 限频相位与失败计数、A3.7 诊断帧截断规模
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "app/chassis/chassis_types.h"
#include "app/encoder/wheel_state.h"
#include "app/mcu_os_lite/clock.h"
#include "app/mcu_os_lite/task_descriptor.h"
#include "app/protocol/byte_ring.h"
#include "app/protocol/command_manager.h"
#include "app/protocol/decoder.h"
#include "app/protocol/feedback.h"
#include "app/protocol/feedback_limiter.h"
#include "app/protocol/motion_desired.h"

namespace robotcar01::protocol {
namespace {

// A6：状态帧往返逐字段一致
TEST(FeedbackTest, StatusFrameRoundTrip) {
  FeedbackInputs inputs{};
  CommandSnapshot snapshot{};
  snapshot.command.seq = 42;
  snapshot.command.v_mps = 0.25f;
  snapshot.command.omega_radps = -0.5f;
  snapshot.command.valid = true;
  snapshot.has_command = true;
  snapshot.armed = true;
  snapshot.run_requested = true;
  snapshot.session_generation = 3;
  snapshot.received_at_us = 1'000'000;
  snapshot.valid_until_us = 1'300'000;
  inputs.command = &snapshot;

  chassis::WheelTarget target{};
  target.left_mps = 0.2f;
  target.right_mps = 0.3f;
  target.limit_reason = chassis::LimitReason::kWheelSpeed;
  inputs.target = &target;

  chassis::WheelSpeedRequest output{};
  output.left_mps = 0.2f;
  output.right_mps = 0.3f;
  output.force_disable = false;
  output.limit_reason = chassis::LimitReason::kWheelSpeed | chassis::LimitReason::kSafety;
  inputs.output = &output;

  const FeedbackStatusPayload payload = AssembleStatusPayload(inputs, 1'100'000);
  uint8_t frame[256] = {};
  const size_t size = EncodeStatusFrame(payload, 7, frame, sizeof(frame));
  ASSERT_EQ(size, kFrameHeaderBytes + kStatusPayloadBytes + kCrcBytes);

  StreamDecoder decoder;
  decoder.Init();
  FeedbackStatusPayload decoded{};
  bool ok = false;
  for (size_t i = 0; i < size; ++i) {
    if (decoder.ConsumeByte(frame[i]) == DecodeStatus::kFrameReady) {
      ok = DecodeStatusPayload(decoder.last_frame(), decoded);
    }
  }
  ASSERT_TRUE(ok);
  EXPECT_EQ(decoded.command_seq, 42u);
  EXPECT_EQ(decoded.session_generation, 3u);
  EXPECT_EQ(decoded.command_age_us, 100'000u);
  EXPECT_FLOAT_EQ(decoded.target_left_mps, 0.2f);
  EXPECT_FLOAT_EQ(decoded.target_right_mps, 0.3f);
  EXPECT_FLOAT_EQ(decoded.output_left_mps, 0.2f);
  EXPECT_FLOAT_EQ(decoded.output_right_mps, 0.3f);
  EXPECT_EQ(decoded.limit_reason, static_cast<uint32_t>(chassis::LimitReason::kWheelSpeed |
                                                        chassis::LimitReason::kSafety));
  EXPECT_NE(decoded.status_flags & kFlagCommandFresh, 0u);
  EXPECT_NE(decoded.status_flags & kFlagRunRequested, 0u);
  EXPECT_NE(decoded.status_flags & kFlagArmed, 0u);
  EXPECT_EQ(decoded.status_flags & kFlagStopRequested, 0u);
  EXPECT_EQ(decoded.command_flags & 1u, 1u);  // run_requested
}

// A6：缺失数据源以清位表达，不得以 0 冒充测量值
TEST(FeedbackTest, MissingSourcesClearValidityBits) {
  FeedbackInputs inputs{};
  const FeedbackStatusPayload payload = AssembleStatusPayload(inputs, 0);
  EXPECT_EQ(payload.status_flags & kFlagTargetValid, 0u);
  EXPECT_EQ(payload.status_flags & kFlagSpeedValid, 0u);
  EXPECT_EQ(payload.status_flags & kFlagEncoderValid, 0u);
  EXPECT_EQ(payload.flags_valid & kFlagSpeedValid, 0u);
  EXPECT_EQ(payload.flags_valid & kFlagEncoderValid, 0u);
  // flags_valid 必须是 status_flags 的子集
  EXPECT_EQ(payload.status_flags & ~payload.flags_valid, 0u);
  EXPECT_EQ(payload.command_age_us, 0u);
}

// A6：过期命令不得被标记 fresh；Stop 与过期可区分
TEST(FeedbackTest, ExpiredCommandIsNotFreshButStopIsDistinguishable) {
  CommandSnapshot snapshot{};
  snapshot.has_command = true;
  snapshot.command.valid = true;
  snapshot.stop_requested = true;
  snapshot.received_at_us = 1'000'000;
  snapshot.valid_until_us = 1'300'000;
  FeedbackInputs inputs{};
  inputs.command = &snapshot;

  const FeedbackStatusPayload fresh = AssembleStatusPayload(inputs, 1'100'000);
  EXPECT_NE(fresh.status_flags & kFlagCommandFresh, 0u);
  EXPECT_NE(fresh.status_flags & kFlagStopRequested, 0u);

  const FeedbackStatusPayload expired = AssembleStatusPayload(inputs, 1'400'000);
  EXPECT_EQ(expired.status_flags & kFlagCommandFresh, 0u);
  EXPECT_NE(expired.status_flags & kFlagStopRequested, 0u);  // 仍可区分"已请求停止"
  EXPECT_EQ(expired.command_age_us, 0u);
}

// A3.7：诊断帧往返 + 任务槽截断规模
TEST(FeedbackTest, DiagFrameRoundTripAndTruncation) {
  mcu_os_lite::TaskSet tasks;
  constexpr uint32_t kWindowTicks = 100;
  std::vector<mcu_os_lite::TaskConfig> configs(10);
  for (size_t i = 0; i < configs.size(); ++i) {
    configs[i].name = "task";
    configs[i].period_ticks = 5;
    ASSERT_TRUE(tasks.Register(configs[i], [](void*) {}, nullptr, kWindowTicks));
  }
  ProtocolStats stats{};
  stats.frames_accepted = 11;
  stats.bad_crc = 2;
  stats.ring_overflow_bytes = 7;
  ByteRingStatistics ring{};
  ring.overflow_drop_bytes = 7;
  ring.high_water_bytes = 100;

  CommandManagerStats command_stats{};
  command_stats.accepted = 5;
  command_stats.seq_rejected = 2;
  command_stats.stale_frames = 1;
  command_stats.session_resets = 1;
  const FeedbackDiagPayload payload =
      AssembleDiagPayload(&tasks, stats, command_stats, ring, /*limits_config_valid=*/true,
                          /*encoder_stats=*/nullptr, 12345);
  EXPECT_EQ(payload.task_count, kMaxDiagTaskSlots);
  EXPECT_EQ(payload.dropped_task_count, 10u - kMaxDiagTaskSlots);
  EXPECT_EQ(payload.command_accepted, 5u);
  EXPECT_EQ(payload.seq_rejected, 2u);
  EXPECT_EQ(payload.stale_frames, 1u);
  EXPECT_EQ(payload.session_resets, 1u);
  EXPECT_EQ(payload.limits_config_valid, 1u);
  EXPECT_EQ(payload.flag_bits & 1u, 1u);  // truncated

  uint8_t frame[256] = {};
  const size_t size = EncodeDiagFrame(payload, 3, frame, sizeof(frame));
  ASSERT_EQ(size, kFrameHeaderBytes + kDiagPayloadBytes + kCrcBytes);

  StreamDecoder decoder;
  decoder.Init();
  FeedbackDiagPayload decoded{};
  bool ok = false;
  for (size_t i = 0; i < size; ++i) {
    if (decoder.ConsumeByte(frame[i]) == DecodeStatus::kFrameReady) {
      ok = DecodeDiagPayload(decoder.last_frame(), decoded);
    }
  }
  ASSERT_TRUE(ok);
  EXPECT_EQ(decoded.protocol_frames_accepted, 11u);
  EXPECT_EQ(decoded.bad_crc, 2u);
  EXPECT_EQ(decoded.ring_overflow_bytes, 7u);
  EXPECT_EQ(decoded.task_count, kMaxDiagTaskSlots);
  EXPECT_EQ(decoded.dropped_task_count, 10u - kMaxDiagTaskSlots);
  EXPECT_EQ(decoded.command_accepted, 5u);
  EXPECT_EQ(decoded.seq_rejected, 2u);
  EXPECT_EQ(decoded.limits_config_valid, 1u);
}

// 限频：Init 后首次即到期；失败只计数且不推进相位
TEST(FeedbackLimiterTest, PhaseAndFailureCounting) {
  FeedbackLimiter limiter;
  FeedbackRateConfig config{};
  config.status_period_ms = 20;
  config.diag_period_ms = 100;
  limiter.Init(config, 0);

  EXPECT_TRUE(limiter.ShouldSendStatus(0));       // 首次即到期
  EXPECT_FALSE(limiter.ShouldSendStatus(19'000)); // 未到周期
  EXPECT_TRUE(limiter.ShouldSendStatus(20'000));
  EXPECT_FALSE(limiter.ShouldSendStatus(20'000)); // 同一时刻不重复发

  limiter.OnSendFailed(true);
  limiter.OnSendFailed(true);
  limiter.OnSendFailed(false);
  EXPECT_EQ(limiter.status_dropped(), 2u);
  EXPECT_EQ(limiter.diag_dropped(), 1u);
  EXPECT_TRUE(limiter.ShouldSendStatus(40'000));  // 失败未改变相位推进规则

  EXPECT_TRUE(limiter.ShouldSendDiag(40'000));
  EXPECT_FALSE(limiter.ShouldSendDiag(100'000));
  EXPECT_TRUE(limiter.ShouldSendDiag(200'000));
}

// 005 A8：target_scale（域限幅比例）往返一致 + 无数据源时写 0（不得被读作"未限幅"）
TEST(FeedbackTest, TargetScaleRoundTripAndMissingSourceSemantics) {
  chassis::WheelTarget target{};
  target.left_mps = 0.1f;
  target.right_mps = 0.2f;
  target.scale = 0.4f;
  target.limit_reason = chassis::LimitReason::kQualityDegraded | chassis::LimitReason::kWheelSpeed;
  target.valid = true;

  FeedbackInputs inputs{};
  inputs.target = &target;
  const FeedbackStatusPayload payload = AssembleStatusPayload(inputs, 1'000);
  EXPECT_NEAR(payload.target_scale, 0.4f, 1e-6f);
  EXPECT_NE(payload.status_flags & kFlagTargetValid, 0u);
  EXPECT_EQ(payload.limit_reason, static_cast<uint32_t>(target.limit_reason));

  uint8_t frame[256] = {};
  const size_t size = EncodeStatusFrame(payload, 9, frame, sizeof(frame));
  ASSERT_EQ(size, kFrameHeaderBytes + kStatusPayloadBytes + kCrcBytes);
  StreamDecoder decoder;
  decoder.Init();
  FeedbackStatusPayload decoded{};
  bool ok = false;
  for (size_t i = 0; i < size; ++i) {
    if (decoder.ConsumeByte(frame[i]) == DecodeStatus::kFrameReady) {
      ok = DecodeStatusPayload(decoder.last_frame(), decoded);
    }
  }
  ASSERT_TRUE(ok);
  EXPECT_NEAR(decoded.target_scale, 0.4f, 1e-6f);
  EXPECT_NEAR(decoded.target_left_mps, 0.1f, 1e-6f);
  EXPECT_NEAR(decoded.target_right_mps, 0.2f, 1e-6f);

  // 无 target 数据源：清位 + scale 写 0（而不是结构体默认的 1.0）
  const FeedbackInputs empty{};
  const FeedbackStatusPayload missing = AssembleStatusPayload(empty, 1'000);
  EXPECT_EQ(missing.status_flags & kFlagTargetValid, 0u);
  EXPECT_NEAR(missing.target_scale, 0.0f, 1e-6f);
  EXPECT_NEAR(missing.target_left_mps, 0.0f, 1e-6f);
}

// 005：逐侧编码器质量位直达状态帧（004 详设的指派，005 落地补齐）
TEST(FeedbackTest, EncoderQualityFieldsPerSide) {
  encoder::WheelState wheel{};
  wheel.left.quality = encoder::kQualityStallCandidate;
  wheel.right.quality = encoder::kQualityCountOutOfRange;
  wheel.left.valid = true;
  wheel.right.valid = false;

  FeedbackInputs inputs{};
  inputs.wheel_state = &wheel;
  const FeedbackStatusPayload payload = AssembleStatusPayload(inputs, 1'000);
  EXPECT_EQ(payload.encoder_quality_left, encoder::kQualityStallCandidate);
  EXPECT_EQ(payload.encoder_quality_right, encoder::kQualityCountOutOfRange);
  // 阻塞位存在 ⇒ kFlagEncoderValid 必须清位（004 逐位置位表不变）
  EXPECT_EQ(payload.status_flags & kFlagEncoderValid, 0u);

  uint8_t frame[256] = {};
  const size_t size = EncodeStatusFrame(payload, 11, frame, sizeof(frame));
  ASSERT_EQ(size, kFrameHeaderBytes + kStatusPayloadBytes + kCrcBytes);
  StreamDecoder decoder;
  decoder.Init();
  FeedbackStatusPayload decoded{};
  bool ok = false;
  for (size_t i = 0; i < size; ++i) {
    if (decoder.ConsumeByte(frame[i]) == DecodeStatus::kFrameReady) {
      ok = DecodeStatusPayload(decoder.last_frame(), decoded);
    }
  }
  ASSERT_TRUE(ok);
  EXPECT_EQ(decoded.encoder_quality_left, encoder::kQualityStallCandidate);
  EXPECT_EQ(decoded.encoder_quality_right, encoder::kQualityCountOutOfRange);

  // 无数据源时两侧质量位写 0
  const FeedbackInputs empty{};
  const FeedbackStatusPayload missing = AssembleStatusPayload(empty, 1'000);
  EXPECT_EQ(missing.encoder_quality_left, 0u);
  EXPECT_EQ(missing.encoder_quality_right, 0u);
}

// 005 A6 前置：motion_desired 唯一构造式（004 D-004-8）的各分支
TEST(MotionDesiredTest, ConstructionRuleBranches) {
  const chassis::SafetyStatus allowed{true};
  const chassis::SafetyStatus blocked{false};
  CommandSnapshot snapshot{};
  snapshot.command.valid = true;
  snapshot.has_command = true;
  snapshot.armed = true;
  snapshot.run_requested = true;
  snapshot.received_at_us = 1'000'000;
  snapshot.valid_until_us = 1'300'000;

  EXPECT_TRUE(MakeMotionDesired(snapshot, 1'100'000, allowed));
  EXPECT_FALSE(MakeMotionDesired(snapshot, 1'100'000, blocked));   // 安全禁止
  EXPECT_FALSE(MakeMotionDesired(snapshot, 1'400'000, allowed));   // 租约过期
  EXPECT_TRUE(MakeMotionDesired(snapshot, 1'300'000, allowed));    // 租约边界（<= 有效）

  CommandSnapshot not_armed = snapshot;
  not_armed.armed = false;
  EXPECT_FALSE(MakeMotionDesired(not_armed, 1'100'000, allowed));

  CommandSnapshot not_running = snapshot;
  not_running.run_requested = false;
  EXPECT_FALSE(MakeMotionDesired(not_running, 1'100'000, allowed));

  CommandSnapshot stopped = snapshot;
  stopped.stop_requested = true;
  EXPECT_FALSE(MakeMotionDesired(stopped, 1'100'000, allowed));

  CommandSnapshot no_command = snapshot;
  no_command.has_command = false;
  EXPECT_FALSE(MakeMotionDesired(no_command, 1'100'000, allowed));
}

}  // namespace
}  // namespace robotcar01::protocol
