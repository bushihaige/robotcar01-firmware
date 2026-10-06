// 协议编解码测试（Iteration 003）：A1 拆包粘包与重同步、A1.1/A1.2 部分 magic、A2.1 预算跨界、
// A3.1 序号边界、A3.4 反馈帧误消费、A3.6 时钟回绕相关的租约判定
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "app/protocol/command_manager.h"
#include "app/protocol/decoder.h"
#include "app/protocol/feedback.h"
#include "app/protocol/frame.h"
#include "app/protocol/session.h"
#include "host_fakes/fake_byte_channel.h"

namespace robotcar01::protocol {
namespace {

using host_fakes::FakeByteChannel;

std::vector<uint8_t> BuildStream(const std::vector<size_t>& seqs) {
  std::vector<uint8_t> stream;
  for (size_t seq : seqs) {
    uint8_t frame[64] = {};
    MotionPayload payload{};
    payload.v_mps = 0.1f * static_cast<float>(seq);
    payload.lease_ms = 300;
    const size_t size = EncodeMotionFrame(payload, static_cast<uint32_t>(seq), frame, sizeof(frame));
    stream.insert(stream.end(), frame, frame + size);
  }
  return stream;
}

// 用给定切片把字节流喂给解码器，返回成功解出的帧序号序列
std::vector<uint32_t> DecodeWithChunks(const std::vector<uint8_t>& stream,
                                       const std::vector<size_t>& chunks) {
  StreamDecoder decoder;
  decoder.Init();
  std::vector<uint32_t> seqs;
  FakeByteChannel channel;
  channel.Ingest(stream);
  channel.DeliverInChunks(chunks, [&decoder, &seqs](const uint8_t* data, size_t size) {
    for (size_t i = 0; i < size; ++i) {
      if (decoder.ConsumeByte(data[i]) == DecodeStatus::kFrameReady) {
        seqs.push_back(decoder.last_frame().header.seq);
      }
    }
  });
  return seqs;
}

// A1：任意切分下解码结果一致
TEST(CodecTest, ArbitraryChunkingYieldsSameFrames) {
  const std::vector<uint8_t> stream = BuildStream({1, 2, 3, 4, 5});
  const std::vector<std::vector<size_t>> chunkings = {
      {1}, {2}, {3}, {7}, {13}, {1, 2, 3, 5, 8, 13, 21}, {100}, {26},
  };
  const std::vector<uint32_t> expected = {1, 2, 3, 4, 5};
  for (const auto& chunks : chunkings) {
    EXPECT_EQ(DecodeWithChunks(stream, chunks), expected) << "chunk size " << chunks.front();
  }
}

// A1：噪声前缀 + 多帧粘连
TEST(CodecTest, NoisePrefixAndStickyFrames) {
  std::vector<uint8_t> stream = {0x00, 0xFF, 0x12, 0x34};
  const std::vector<uint8_t> frames = BuildStream({9, 10});
  stream.insert(stream.end(), frames.begin(), frames.end());
  StreamDecoder decoder;
  decoder.Init();
  std::vector<uint32_t> seqs;
  for (uint8_t byte : stream) {
    if (decoder.ConsumeByte(byte) == DecodeStatus::kFrameReady) {
      seqs.push_back(decoder.last_frame().header.seq);
    }
  }
  EXPECT_EQ(seqs.size(), 2u);
  EXPECT_EQ(seqs[0], 9u);
  EXPECT_EQ(seqs[1], 10u);
}

// A1.1：噪声以 0x52 结尾后紧跟真帧 —— 失配字节必须重当候选首字节，真帧不得被吞
TEST(CodecTest, PartialMagicTailThenRealFrame) {
  const std::vector<uint8_t> frame = BuildStream({1});
  // 噪声：0x52 0x11（0x52 匹配首字节、0x11 失配）→ 之后是真帧
  std::vector<uint8_t> stream = {0x99, 0x52, 0x11};
  stream.insert(stream.end(), frame.begin(), frame.end());
  StreamDecoder decoder;
  decoder.Init();
  std::vector<uint32_t> seqs;
  for (uint8_t byte : stream) {
    if (decoder.ConsumeByte(byte) == DecodeStatus::kFrameReady) {
      seqs.push_back(decoder.last_frame().header.seq);
    }
  }
  ASSERT_EQ(seqs.size(), 1u);  // 真帧必须被解出
  EXPECT_EQ(seqs[0], 1u);
  EXPECT_EQ(decoder.stats().bad_magic, 1u);  // 一次失配恰 +1
}

// A1.2：噪声含 0x52 0x43 0x30 前缀 + 真帧
TEST(CodecTest, PartialMagicLongPrefixThenRealFrame) {
  const std::vector<uint8_t> frame = BuildStream({7});
  std::vector<uint8_t> stream = {0x52, 0x43, 0x30, 0x00};
  stream.insert(stream.end(), frame.begin(), frame.end());
  StreamDecoder decoder;
  decoder.Init();
  std::vector<uint32_t> seqs;
  for (uint8_t byte : stream) {
    if (decoder.ConsumeByte(byte) == DecodeStatus::kFrameReady) {
      seqs.push_back(decoder.last_frame().header.seq);
    }
  }
  ASSERT_EQ(seqs.size(), 1u);
  EXPECT_EQ(seqs[0], 7u);
  EXPECT_EQ(decoder.stats().bad_magic, 1u);
}

// A1.2 变体：失配字节本身是 0x52 时必须重开匹配（不吞字节）
TEST(CodecTest, RestartOnMagicByteItself) {
  const std::vector<uint8_t> frame = BuildStream({3});
  std::vector<uint8_t> stream = {0x52, 0x43, 0x52};  // 0x52 0x43 后收到 0x52 → 重开
  stream.insert(stream.end(), frame.begin() + 1, frame.end());  // 真帧去掉首个 0x52（已被重用）
  StreamDecoder decoder;
  decoder.Init();
  std::vector<uint32_t> seqs;
  for (uint8_t byte : stream) {
    if (decoder.ConsumeByte(byte) == DecodeStatus::kFrameReady) {
      seqs.push_back(decoder.last_frame().header.seq);
    }
  }
  ASSERT_EQ(seqs.size(), 1u);
  EXPECT_EQ(seqs[0], 3u);
}

// A2.1：粘包跨解析预算（预算 5 字节/次），不丢帧、不重复、顺序不变
TEST(CodecTest, BudgetCrossingKeepsOrderAndDoesNotDropFrames) {
  const std::vector<uint8_t> stream = BuildStream({1, 2, 3});
  StreamDecoder decoder;
  decoder.Init();
  CommandParseConfig budget{};
  budget.max_bytes_per_cycle = 5;
  std::vector<uint32_t> seqs;
  size_t offset = 0;
  while (offset < stream.size()) {
    size_t consumed = 0;
    const DecodeStatus status =
        decoder.Consume(stream.data() + offset, stream.size() - offset, &consumed, budget);
    EXPECT_LE(consumed, budget.max_bytes_per_cycle);
    offset += consumed;
    if (status == DecodeStatus::kFrameReady) {
      seqs.push_back(decoder.last_frame().header.seq);
      decoder.ClearLastFrame();
    }
    if (consumed == 0) {
      break;
    }
  }
  const std::vector<uint32_t> expected = {1, 2, 3};
  EXPECT_EQ(seqs, expected);
}

// A3.1：序号五边界（重复 / 落后 / 领先 / 恰好半程 / 0xFFFFFFFF→0）
TEST(CodecTest, SequenceBoundariesAcceptedAndRejected) {
  CommandLeaseConfig lease{};
  CommandLimits limits{};

  auto feed = [](CommandManager& manager, uint32_t seq, uint64_t now_us) {
    uint8_t frame[64] = {};
    const size_t size = EncodeMotionFrame(MotionPayload{}, seq, frame, sizeof(frame));
    StreamDecoder decoder;
    decoder.Init();
    for (size_t i = 0; i < size; ++i) {
      if (decoder.ConsumeByte(frame[i]) == DecodeStatus::kFrameReady) {
        return manager.OnFrame(decoder.last_frame(), now_us);
      }
    }
    return false;
  };

  {
    CommandManager manager;
    manager.Init(lease, limits);
    EXPECT_TRUE(feed(manager, 10, 1000));
    EXPECT_FALSE(feed(manager, 10, 2000));  // 重复 diff=0
    EXPECT_EQ(manager.stats().seq_rejected, 1u);
    EXPECT_FALSE(feed(manager, 9, 3000));  // 落后 diff=0xFFFFFFFF
    EXPECT_EQ(manager.stats().seq_rejected, 2u);
    EXPECT_TRUE(feed(manager, 11, 4000));  // 领先 diff=1
  }
  {
    CommandManager manager;
    manager.Init(lease, limits);
    EXPECT_TRUE(feed(manager, 0, 1000));
    EXPECT_FALSE(feed(manager, 0x80000000u, 2000));  // 恰好半程必须拒绝
    EXPECT_EQ(manager.stats().seq_rejected, 1u);
  }
  {
    CommandManager manager;
    manager.Init(lease, limits);
    EXPECT_TRUE(feed(manager, 0xFFFFFFFFu, 1000));
    EXPECT_TRUE(feed(manager, 0, 2000));  // 回绕 diff=1 必须接受
    EXPECT_EQ(manager.stats().seq_rejected, 0u);
  }
}

// A3.4：反馈方向帧到达命令路径 → 丢弃且不计 seq 倒退
TEST(CodecTest, FeedbackFrameOnCommandPathIsDiscarded) {
  CommandManager manager;
  manager.Init(CommandLeaseConfig{}, CommandLimits{});

  FeedbackStatusPayload payload{};
  uint8_t frame[256] = {};
  const size_t size = EncodeStatusFrame(payload, 5, frame, sizeof(frame));
  ASSERT_GT(size, 0u);

  StreamDecoder decoder;
  decoder.Init();
  bool updated = false;
  for (size_t i = 0; i < size; ++i) {
    if (decoder.ConsumeByte(frame[i]) == DecodeStatus::kFrameReady) {
      updated = manager.OnFrame(decoder.last_frame(), 1000);
    }
  }
  EXPECT_FALSE(updated);
  EXPECT_EQ(manager.stats().wrong_direction, 1u);
  EXPECT_EQ(manager.stats().seq_rejected, 0u);
  EXPECT_FALSE(manager.snapshot().has_command);
}

}  // namespace
}  // namespace robotcar01::protocol
