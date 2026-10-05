// 命令管理测试（Iteration 003）：arm 运行门、会话重置、租约判定（A5）、迟到帧（A3.5）、
// 挂起（A3.8）、时钟回绕（A3.6）、快照不变性（A2）
#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <vector>

#include "app/protocol/byte_ring.h"
#include "app/protocol/command_manager.h"
#include "app/protocol/decoder.h"
#include "app/protocol/frame.h"
#include "app/protocol/session.h"

namespace robotcar01::protocol {
namespace {

class Harness {
 public:
  Harness() {
    CommandLeaseConfig lease{};
    lease.default_lease_ms = 300;
    lease.min_lease_ms = 50;
    lease.max_lease_ms = 2000;
    lease.max_send_age_ms = 100;
    manager_.Init(lease, CommandLimits{});
  }

  bool SendMotion(uint32_t seq, float v, float omega, bool run, uint16_t lease_ms = 300,
                  uint8_t send_age_ms = 0, uint64_t now_us = 1000) {
    MotionPayload payload{};
    payload.v_mps = v;
    payload.omega_radps = omega;
    payload.run_requested = run;
    payload.lease_ms = lease_ms;
    payload.send_age_ms = send_age_ms;
    return Feed(MessageType::kMotionCommand, payload, seq, now_us);
  }

  bool SendStop(uint32_t seq, uint64_t now_us = 1000) {
    MotionPayload payload{};
    return Feed(MessageType::kStopCommand, payload, seq, now_us);
  }

  void Reset() { ResetCommandSession(ring_, decoder_, manager_); }

  CommandManager& manager() { return manager_; }

 private:
  bool Feed(MessageType type, const MotionPayload& payload, uint32_t seq, uint64_t now_us) {
    uint8_t frame[64] = {};
    const size_t size = (type == MessageType::kMotionCommand)
                            ? EncodeMotionFrame(payload, seq, frame, sizeof(frame))
                            : EncodeStopFrame(seq, frame, sizeof(frame));
    StreamDecoder decoder;
    decoder.Init();
    for (size_t i = 0; i < size; ++i) {
      if (decoder.ConsumeByte(frame[i]) == DecodeStatus::kFrameReady) {
        return manager_.OnFrame(decoder.last_frame(), now_us);
      }
    }
    return false;
  }

  ByteRing ring_{};
  StreamDecoder decoder_{};
  CommandManager manager_{};
};

// A4 断言 ①：会话首帧即带 run 请求也不得使能，且计数忽略
TEST(CommandManagerTest, FirstFrameNeverEnablesRun) {
  Harness h;
  EXPECT_TRUE(h.SendMotion(1, 0.2f, 0.0f, /*run=*/true));
  EXPECT_FALSE(h.manager().snapshot().run_requested);
  EXPECT_TRUE(h.manager().snapshot().armed);
  EXPECT_EQ(h.manager().stats().run_requests_ignored, 1u);

  // 第 2 帧显式请求才生效
  EXPECT_TRUE(h.SendMotion(2, 0.2f, 0.0f, /*run=*/true));
  EXPECT_TRUE(h.manager().snapshot().run_requested);
}

// A4 断言 ②：Stop 之后紧接单帧 run 请求不得使能（arm 门需要一帧未请求运行的有效命令）
TEST(CommandManagerTest, RunAfterStopRequiresNonRunFrame) {
  Harness h;
  EXPECT_TRUE(h.SendMotion(1, 0.0f, 0.0f, false));
  EXPECT_TRUE(h.SendMotion(2, 0.2f, 0.0f, true));
  EXPECT_TRUE(h.manager().snapshot().run_requested);

  EXPECT_TRUE(h.SendStop(3));
  EXPECT_FALSE(h.manager().snapshot().run_requested);
  EXPECT_TRUE(h.manager().snapshot().stop_requested);
  EXPECT_EQ(h.manager().snapshot().command.v_mps, 0.0f);

  // arm 门已由显式 Stop 置位 ⇒ 下一帧 run 请求可生效
  EXPECT_TRUE(h.SendMotion(4, 0.2f, 0.0f, true));
  EXPECT_TRUE(h.manager().snapshot().run_requested);
}

// A4 断言 ③：ResetCommandSession 后清空，前一帧 run 请求必须重新建立
TEST(CommandManagerTest, ResetSessionClearsEverythingAndBlocksAutoRun) {
  Harness h;
  EXPECT_TRUE(h.SendMotion(1, 0.1f, 0.0f, false));
  EXPECT_TRUE(h.SendMotion(2, 0.2f, 0.0f, true));
  ASSERT_TRUE(h.manager().snapshot().run_requested);
  const uint32_t generation = h.manager().snapshot().session_generation;

  h.Reset();
  EXPECT_EQ(h.manager().snapshot().session_generation, generation + 1);
  EXPECT_FALSE(h.manager().snapshot().has_command);
  EXPECT_FALSE(h.manager().snapshot().run_requested);
  EXPECT_FALSE(h.manager().snapshot().armed);
  EXPECT_FALSE(h.manager().snapshot().command.valid);
  EXPECT_EQ(h.manager().snapshot().received_at_us, 0u);

  // 重连后上位机持续按 20Hz 发 run 请求：首帧被忽略、第二帧才生效
  EXPECT_TRUE(h.SendMotion(1, 0.2f, 0.0f, true, 300, 0, 10000));
  EXPECT_FALSE(h.manager().snapshot().run_requested);
  EXPECT_TRUE(h.SendMotion(2, 0.2f, 0.0f, true, 300, 0, 20000));
  EXPECT_TRUE(h.manager().snapshot().run_requested);
}

// A5：租约判定在边界 ±1 µs 翻转；原始快照 command.valid 不随时间变化
TEST(CommandManagerTest, LeaseValidityBoundary) {
  Harness h;
  const uint64_t now = 1'000'000ull;  // 1s
  ASSERT_TRUE(h.SendMotion(1, 0.2f, 0.0f, false, 300, 0, now));
  const uint64_t expires = now + 300ull * 1000ull;

  // 判定式 now < valid_until_us：恰好到期的时刻仍有效，过期 1µs 起无效
  EXPECT_TRUE(h.manager().CommandValidAt(expires - 1));
  EXPECT_TRUE(h.manager().CommandValidAt(expires));
  EXPECT_FALSE(h.manager().CommandValidAt(expires + 1));
  EXPECT_FALSE(h.manager().CommandValidAt(expires + 1'000'000ull));

  // 原始快照：command.valid 恒表示"会话内已接受过合法命令"
  EXPECT_TRUE(h.manager().snapshot().command.valid);
  EXPECT_TRUE(h.manager().EffectiveSnapshot(expires - 1).command.valid);
  EXPECT_FALSE(h.manager().EffectiveSnapshot(expires + 1).command.valid);
  EXPECT_EQ(h.manager().snapshot().valid_until_us, expires);
  EXPECT_EQ(h.manager().AgeUs(now + 5000), 5000u);
}

// A2.2：租约 clamp（0 → 默认；超上限 → 上限；低于下限 → 下限）
TEST(CommandManagerTest, LeaseClamping) {
  {
    Harness h;
    ASSERT_TRUE(h.SendMotion(1, 0.1f, 0.0f, false, /*lease_ms=*/0));
    EXPECT_EQ(h.manager().snapshot().lease_ms, 300u);
  }
  {
    Harness h;
    ASSERT_TRUE(h.SendMotion(1, 0.1f, 0.0f, false, /*lease_ms=*/5000));
    EXPECT_EQ(h.manager().snapshot().lease_ms, 2000u);
  }
  {
    Harness h;
    ASSERT_TRUE(h.SendMotion(1, 0.1f, 0.0f, false, /*lease_ms=*/1));
    EXPECT_EQ(h.manager().snapshot().lease_ms, 50u);
  }
}

// A3.5：迟到帧被拒绝且不刷新时间戳（防止环内积压旧命令重置租约）
TEST(CommandManagerTest, StaleFrameRejectedWithoutRefreshingTimestamp) {
  Harness h;
  ASSERT_TRUE(h.SendMotion(1, 0.1f, 0.0f, false, 300, 0, 1000));
  const uint64_t received = h.manager().snapshot().received_at_us;
  const uint32_t seq = h.manager().snapshot().command.seq;

  EXPECT_FALSE(h.SendMotion(2, 0.9f, 0.0f, false, 300, /*send_age_ms=*/250, 500000));
  EXPECT_EQ(h.manager().stats().stale_frames, 1u);
  EXPECT_EQ(h.manager().snapshot().received_at_us, received);
  EXPECT_EQ(h.manager().snapshot().command.seq, seq);
  EXPECT_EQ(h.manager().snapshot().command.v_mps, 0.1f);
}

// A2：坏帧（数值非法 / 超范围）不改动已发布快照
TEST(CommandManagerTest, InvalidValuesDoNotChangeSnapshot) {
  Harness h;
  ASSERT_TRUE(h.SendMotion(1, 0.1f, 0.05f, false, 300, 0, 1000));
  const CommandSnapshot before = h.manager().snapshot();

  EXPECT_FALSE(h.SendMotion(2, std::numeric_limits<float>::quiet_NaN(), 0.0f, false, 300, 0, 2000));
  EXPECT_FALSE(h.SendMotion(3, std::numeric_limits<float>::infinity(), 0.0f, false, 300, 0, 3000));
  EXPECT_EQ(h.manager().stats().rejected, 2u);
  EXPECT_EQ(h.manager().snapshot().command.seq, before.command.seq);
  EXPECT_EQ(h.manager().snapshot().received_at_us, before.received_at_us);
  EXPECT_EQ(h.manager().snapshot().command.v_mps, before.command.v_mps);

  // 超范围（注入限值后）
  CommandManager limited;
  CommandLeaseConfig lease{};
  CommandLimits limits{};
  limits.max_body_speed_mps = 0.5f;
  limits.max_yaw_rate_radps = 2.0f;
  limited.Init(lease, limits);
  uint8_t frame[64] = {};
  MotionPayload payload{};
  payload.v_mps = 0.6f;  // 超范围
  const size_t size = EncodeMotionFrame(payload, 1, frame, sizeof(frame));
  StreamDecoder decoder;
  decoder.Init();
  for (size_t i = 0; i < size; ++i) {
    if (decoder.ConsumeByte(frame[i]) == DecodeStatus::kFrameReady) {
      EXPECT_FALSE(limited.OnFrame(decoder.last_frame(), 1000));
    }
  }
  EXPECT_EQ(limited.stats().rejected, 1u);
  EXPECT_FALSE(limited.snapshot().has_command);
}

// A3.8：SUSPEND 期间运动帧被拒绝且不刷新时间戳；Stop 仍接受；Resume 后 arm 复位
TEST(CommandManagerTest, SuspendRejectsMotionAndResumeResetsArm) {
  Harness h;
  ASSERT_TRUE(h.SendMotion(1, 0.1f, 0.0f, false, 300, 0, 1000));
  const uint64_t received = h.manager().snapshot().received_at_us;

  h.manager().Suspend();
  EXPECT_FALSE(h.SendMotion(2, 0.9f, 0.0f, true, 300, 0, 200000));
  EXPECT_EQ(h.manager().stats().suspended_rejected, 1u);
  EXPECT_EQ(h.manager().snapshot().received_at_us, received);
  EXPECT_FALSE(h.manager().snapshot().run_requested);

  EXPECT_TRUE(h.SendStop(3, 300000));
  EXPECT_TRUE(h.manager().snapshot().stop_requested);
  EXPECT_EQ(h.manager().snapshot().received_at_us, received);  // 挂起期 Stop 不刷时间戳

  h.manager().Resume();
  EXPECT_FALSE(h.manager().snapshot().armed);  // arm 复位，必须重新建立
  EXPECT_TRUE(h.SendMotion(4, 0.2f, 0.0f, true, 300, 0, 400000));
  EXPECT_FALSE(h.manager().snapshot().run_requested);  // 复位后首帧请求被忽略并置 arm
  EXPECT_TRUE(h.manager().snapshot().armed);
  EXPECT_TRUE(h.SendMotion(5, 0.2f, 0.0f, true, 300, 0, 500000));
  EXPECT_TRUE(h.manager().snapshot().run_requested);
}

// A3.6：单调时钟推进跨越 32 位边界（uint64 微秒）不影响年龄与租约判定
TEST(CommandManagerTest, AgeAndValiditySurviveLow32BitWrap) {
  Harness h;
  const uint64_t now = 0xFFFFFFFFull;  // 低 32 位回绕点附近
  ASSERT_TRUE(h.SendMotion(1, 0.1f, 0.0f, false, 300, 0, now));
  EXPECT_EQ(h.manager().AgeUs(now + 1234), 1234u);
  EXPECT_TRUE(h.manager().CommandValidAt(now + 299'999));
  EXPECT_FALSE(h.manager().CommandValidAt(now + 300'001));
  EXPECT_EQ(h.manager().snapshot().received_at_us, now);
}

}  // namespace
}  // namespace robotcar01::protocol
