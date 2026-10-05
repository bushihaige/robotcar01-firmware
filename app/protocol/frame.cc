#include "app/protocol/frame.h"

#include <cstring>

#include "app/protocol/crc16.h"

namespace robotcar01::protocol {

namespace {

void WriteMagic(uint8_t* out) {
  out[0] = kMagicByte0;
  out[1] = kMagicByte1;
  out[2] = kMagicByte2;
  out[3] = kMagicByte3;
}

// 组装完整帧：magic + header + payload + CRC(LE)。返回总长度；容量不足返回 0。
size_t AssembleFrame(uint8_t msg_type, const uint8_t* payload, size_t payload_len, uint32_t seq,
                     uint8_t* out, size_t out_capacity) {
  const size_t total = kFrameHeaderBytes + payload_len + kCrcBytes;
  if (out == nullptr || total > out_capacity) {
    return 0;
  }
  WriteMagic(out);
  FrameHeader header{};
  header.msg_type = msg_type;
  header.payload_len = static_cast<uint8_t>(payload_len);
  header.seq = seq;
  EncodeHeader(header, out + 4);
  for (size_t i = 0; i < payload_len; ++i) {
    out[kFrameHeaderBytes + i] = payload[i];
  }
  const uint16_t crc = Crc16CcittFalse(out, kFrameHeaderBytes + payload_len);
  PutLe16(out + kFrameHeaderBytes + payload_len, crc);
  return total;
}

}  // namespace

void PutLe16(uint8_t* out, uint16_t value) {
  out[0] = static_cast<uint8_t>(value & 0xFFu);
  out[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
}

void PutLe32(uint8_t* out, uint32_t value) {
  out[0] = static_cast<uint8_t>(value & 0xFFu);
  out[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
  out[2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
  out[3] = static_cast<uint8_t>((value >> 24) & 0xFFu);
}

uint16_t GetLe16(const uint8_t* in) {
  return static_cast<uint16_t>(static_cast<uint16_t>(in[0]) |
                               static_cast<uint16_t>(static_cast<uint16_t>(in[1]) << 8));
}

uint32_t GetLe32(const uint8_t* in) {
  return static_cast<uint32_t>(in[0]) | (static_cast<uint32_t>(in[1]) << 8) |
         (static_cast<uint32_t>(in[2]) << 16) | (static_cast<uint32_t>(in[3]) << 24);
}

uint32_t FloatBits(float value) {
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

float BitsToFloat(uint32_t bits) {
  float value = 0.0f;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void EncodeHeader(const FrameHeader& header, uint8_t* out) {
  out[0] = header.version;
  out[1] = header.msg_type;
  out[2] = header.flags;
  out[3] = header.payload_len;
  PutLe32(out + 4, header.seq);
}

size_t EncodeMotionFrame(const MotionPayload& payload, uint32_t seq, uint8_t* out, size_t out_capacity) {
  uint8_t body[kMotionPayloadBytes] = {};
  PutLe32(body + 0, FloatBits(payload.v_mps));
  PutLe32(body + 4, FloatBits(payload.omega_radps));
  body[8] = payload.run_requested ? 1u : 0u;
  body[9] = 0u;  // reserved0 必须为 0
  PutLe16(body + 10, payload.lease_ms);  // [10..11] little-endian
  body[12] = payload.send_age_ms;        // [12] 独立字节，不与 lease 高字节重叠
  return AssembleFrame(static_cast<uint8_t>(MessageType::kMotionCommand), body, kMotionPayloadBytes,
                       seq, out, out_capacity);
}

size_t EncodeStopFrame(uint32_t seq, uint8_t* out, size_t out_capacity) {
  return AssembleFrame(static_cast<uint8_t>(MessageType::kStopCommand), nullptr, kStopPayloadBytes,
                       seq, out, out_capacity);
}

MotionPayload DecodeMotionPayload(const uint8_t* payload, size_t payload_len) {
  MotionPayload result{};
  if (payload == nullptr || payload_len != kMotionPayloadBytes) {
    return result;
  }
  result.v_mps = BitsToFloat(GetLe32(payload + 0));
  result.omega_radps = BitsToFloat(GetLe32(payload + 4));
  result.run_requested = payload[8] != 0u;
  result.lease_ms = GetLe16(payload + 10);
  result.send_age_ms = payload[12];
  return result;
}

}  // namespace robotcar01::protocol
