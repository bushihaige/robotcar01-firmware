// 协议帧层测试（Iteration 003）：CRC 固定向量、命令帧线格式、长度/保留位/类型/flag 校验
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "app/protocol/crc16.h"
#include "app/protocol/decoder.h"
#include "app/protocol/frame.h"

namespace robotcar01::protocol {
namespace {

using ::testing::Test;

// A3.9：CRC 固定向量（线序 little-endian）
TEST(Crc16Test, StandardCheckVectors) {
  const char* check = "123456789";
  EXPECT_EQ(Crc16CcittFalse(reinterpret_cast<const uint8_t*>(check), 9), 0x29B1u);
  EXPECT_EQ(Crc16CcittFalse(nullptr, 0), 0xFFFFu);  // 初值自检
  const uint8_t zeros[24] = {};
  EXPECT_EQ(Crc16CcittFalse(zeros, 24), 0xEF8Au);
}

// 命令帧线格式固定向量：magic/version/type/flags/len/seq/载荷/CRC 逐字节
TEST(FrameEncodeTest, MotionFrameWireFormat) {
  MotionPayload payload{};
  payload.v_mps = 0.5f;
  payload.omega_radps = -0.25f;
  payload.run_requested = true;
  payload.lease_ms = 300;   // 0x012C
  payload.send_age_ms = 7;
  uint8_t out[64] = {};
  const size_t size = EncodeMotionFrame(payload, 0x04030201u, out, sizeof(out));

  const size_t expected_size = kFrameHeaderBytes + kMotionPayloadBytes + kCrcBytes;
  ASSERT_EQ(size, expected_size);
  EXPECT_EQ(out[0], 0x52u);
  EXPECT_EQ(out[1], 0x43u);
  EXPECT_EQ(out[2], 0x30u);
  EXPECT_EQ(out[3], 0x31u);
  EXPECT_EQ(out[4], kProtocolVersion);
  EXPECT_EQ(out[5], static_cast<uint8_t>(MessageType::kMotionCommand));
  EXPECT_EQ(out[6], 0u);
  EXPECT_EQ(out[7], kMotionPayloadBytes);
  // seq little-endian
  EXPECT_EQ(out[8], 0x01u);
  EXPECT_EQ(out[9], 0x02u);
  EXPECT_EQ(out[10], 0x03u);
  EXPECT_EQ(out[11], 0x04u);
  EXPECT_EQ(out[12], 0x00u);  // reserved0 必须为 0
  EXPECT_EQ(out[13], 0x00u);
  EXPECT_EQ(out[14], 0x00u);
  EXPECT_EQ(out[15], 0x3Fu);  // 0.5f
  EXPECT_EQ(out[20], 0x01u);  // run_requested
  EXPECT_EQ(out[21], 0x00u);  // reserved0
  EXPECT_EQ(out[22], 0x2Cu);  // lease_ms = 300 LE 低字节
  EXPECT_EQ(out[23], 0x01u);  // lease_ms 高字节
  EXPECT_EQ(out[24], 0x07u);  // send_age_ms（独立字节）
  const uint16_t crc = GetLe16(out + kFrameHeaderBytes + kMotionPayloadBytes);
  EXPECT_EQ(crc, Crc16CcittFalse(out, kFrameHeaderBytes + kMotionPayloadBytes));
}

TEST(FrameEncodeTest, StopFrameWireFormat) {
  uint8_t out[64] = {};
  const size_t size = EncodeStopFrame(9, out, sizeof(out));
  ASSERT_EQ(size, kFrameHeaderBytes + kStopPayloadBytes + kCrcBytes);
  EXPECT_EQ(out[5], static_cast<uint8_t>(MessageType::kStopCommand));
  EXPECT_EQ(out[7], 0u);
  EXPECT_EQ(out[8], 9u);
}

TEST(FrameEncodeTest, FloatRoundTrip) {
  const float values[] = {0.0f, -0.0f, 1.0f, -1.0f, 0.33f, 1e-6f, 12345.5f};
  for (float value : values) {
    EXPECT_EQ(BitsToFloat(FloatBits(value)), value);
  }
}

// 线格式拒绝分类：每类恰 +1（A3）
TEST(DecoderWireFormatTest, RejectionCategories) {
  MotionPayload payload{};
  uint8_t frame[64] = {};
  const size_t size = EncodeMotionFrame(payload, 1, frame, sizeof(frame));

  struct Case {
    const char* name;
    size_t offset;
    uint8_t value;
    uint32_t ProtocolStats::*counter;
  };
  const Case cases[] = {
      {"bad_magic", 1, 0x00, &ProtocolStats::bad_magic},
      {"bad_version", 4, 0x7F, &ProtocolStats::bad_version},
      {"bad_flag", 6, 0x01, &ProtocolStats::bad_flag},
      {"bad_type", 5, 0x77, &ProtocolStats::bad_type},
      {"bad_length", 7, 0x30, &ProtocolStats::bad_length},
      {"bad_reserved", 12 + 9, 0x01, &ProtocolStats::bad_reserved},
  };
  for (const Case& item : cases) {
    std::vector<uint8_t> damaged(frame, frame + size);
    damaged[item.offset] = item.value;
    StreamDecoder decoder;
    decoder.Init();
    for (uint8_t byte : damaged) {
      decoder.ConsumeByte(byte);
    }
    EXPECT_EQ(decoder.stats().frames_accepted, 0u) << item.name;
    EXPECT_EQ(decoder.stats().*(item.counter), 1u) << item.name;
  }

  // CRC 损坏：末字节翻转
  std::vector<uint8_t> bad_crc(frame, frame + size);
  bad_crc[size - 1] ^= 0xFFu;
  StreamDecoder decoder;
  decoder.Init();
  for (uint8_t byte : bad_crc) {
    decoder.ConsumeByte(byte);
  }
  EXPECT_EQ(decoder.stats().bad_crc, 1u);
  EXPECT_EQ(decoder.stats().frames_accepted, 0u);
}

TEST(DecoderWireFormatTest, LengthAttackRejectedBeforeBufferGrowth) {
  // payload_len = 0xFF（远超 kMaxPayloadBytes）
  const uint8_t header[kFrameHeaderBytes] = {0x52, 0x43, 0x30, 0x31, 1,
                                             static_cast<uint8_t>(MessageType::kMotionCommand), 0,
                                             0xFF, 1, 0, 0, 0};
  StreamDecoder decoder;
  decoder.Init();
  for (uint8_t byte : header) {
    decoder.ConsumeByte(byte);
  }
  EXPECT_EQ(decoder.stats().bad_length, 1u);
  EXPECT_EQ(decoder.stats().frames_accepted, 0u);
}

TEST(DecoderWireFormatTest, TypeLengthMismatchRejected) {
  // Stop 类型但声明 12 字节载荷
  const uint8_t header[kFrameHeaderBytes] = {0x52, 0x43, 0x30, 0x31, 1,
                                             static_cast<uint8_t>(MessageType::kStopCommand), 0,
                                             kMotionPayloadBytes, 1, 0, 0, 0};
  StreamDecoder decoder;
  decoder.Init();
  for (uint8_t byte : header) {
    decoder.ConsumeByte(byte);
  }
  EXPECT_EQ(decoder.stats().bad_length, 1u);
}

TEST(DecoderWireFormatTest, ValidFramesAcceptedInSequence) {
  uint8_t motion[64] = {};
  uint8_t stop[64] = {};
  const size_t motion_size = EncodeMotionFrame(MotionPayload{}, 1, motion, sizeof(motion));
  const size_t stop_size = EncodeStopFrame(2, stop, sizeof(stop));

  StreamDecoder decoder;
  decoder.Init();
  int ready = 0;
  for (size_t i = 0; i < motion_size; ++i) {
    if (decoder.ConsumeByte(motion[i]) == DecodeStatus::kFrameReady) {
      ++ready;
      EXPECT_EQ(decoder.last_frame().header.seq, 1u);
    }
  }
  for (size_t i = 0; i < stop_size; ++i) {
    if (decoder.ConsumeByte(stop[i]) == DecodeStatus::kFrameReady) {
      ++ready;
      EXPECT_EQ(decoder.last_frame().header.seq, 2u);
    }
  }
  EXPECT_EQ(ready, 2);
  EXPECT_EQ(decoder.stats().frames_accepted, 2u);
  EXPECT_EQ(decoder.stats().frames_rejected, 0u);
}

}  // namespace
}  // namespace robotcar01::protocol
