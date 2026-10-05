// 帧定义与命令帧编码（Iteration 003）
//
// 线格式（003 详细设计 §帧格式冻结）：
//   offset 0  magic 'R''C''0''1'（逐字节比较，不用整数 memcmp）
//   offset 4  version（1）
//   offset 5  msg_type
//   offset 6  flags（保留，必须为 0）
//   offset 7  payload_len（<= 64）
//   offset 8  seq（uint32 LE）
//   offset 12 payload
//   offset 12+N crc16（LE，覆盖 0..12+N-1）
//
// 序列化逐字段显式进行：多字节整数逐字节移位、float 经 memcpy 取位模式后按 LE 写，
// 避免 strict-aliasing、对齐与端序差异（host x86_64 与 arm-none-eabi 语义一致）。

#ifndef ROBOTCAR01_APP_PROTOCOL_FRAME_H_
#define ROBOTCAR01_APP_PROTOCOL_FRAME_H_

#include <cstddef>
#include <cstdint>

#include "app/protocol/protocol_constants.h"

namespace robotcar01::protocol {

struct FrameHeader {
  uint8_t version = kProtocolVersion;
  uint8_t msg_type = 0;
  uint8_t flags = 0;
  uint8_t payload_len = 0;
  uint32_t seq = 0;
};

// 运动命令载荷（线格式语义，已通过线格式校验的字段）。
struct MotionPayload {
  float v_mps = 0.0f;
  float omega_radps = 0.0f;
  bool run_requested = false;   // 客户端运行请求；是否生效取决于 arm 门（D-003-1）
  uint16_t lease_ms = 0;        // 0 = 用服务端默认；生效值 = clamp(声明, min, max)
  uint8_t send_age_ms = 0;      // 上位机填写的"发出至今"毫秒数；0 = 未提供（D-003-12）
};

// 数值/范围校验上限（由应用配置填充；协议库不内建生效值，D-003-9）。
// 0 表示该项未配置：只校验有限性，不校验范围。
struct CommandLimits {
  float max_body_speed_mps = 0.0f;
  float max_yaw_rate_radps = 0.0f;
};

// ---- 小端序列化辅助（显式、无对齐假设）----
void PutLe16(uint8_t* out, uint16_t value);
void PutLe32(uint8_t* out, uint32_t value);
uint16_t GetLe16(const uint8_t* in);
uint32_t GetLe32(const uint8_t* in);
uint32_t FloatBits(float value);
float BitsToFloat(uint32_t bits);

// 写入 12 字节帧头（不含 magic）。payload_len 由调用方保证与类型匹配。
void EncodeHeader(const FrameHeader& header, uint8_t* out);

// 运动帧编码：返回写入总字节数（头 + 载荷 + CRC），容量不足返回 0。
size_t EncodeMotionFrame(const MotionPayload& payload, uint32_t seq, uint8_t* out, size_t out_capacity);

// 停止帧编码：载荷长度 0。
size_t EncodeStopFrame(uint32_t seq, uint8_t* out, size_t out_capacity);

// 从已接受帧的载荷解析运动命令（仅在 decoder 判定 kFrameReady 且类型为运动/停止后调用）。
MotionPayload DecodeMotionPayload(const uint8_t* payload, size_t payload_len);

}  // namespace robotcar01::protocol

#endif  // ROBOTCAR01_APP_PROTOCOL_FRAME_H_
