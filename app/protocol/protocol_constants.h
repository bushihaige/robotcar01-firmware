// Protocol constants（Iteration 003，robotcar01::protocol）
//
// 线格式常量与解析预算。冻结依据：003 详细设计 §帧格式冻结（wire contract）+ D-003-9
// （参数以注入式值对象提供，本文件只放线格式常量，不放可标定的业务阈值）。
// 无动态内存、无 HAL 依赖；host 与 ARM 语义一致。

#ifndef ROBOTCAR01_APP_PROTOCOL_PROTOCOL_CONSTANTS_H_
#define ROBOTCAR01_APP_PROTOCOL_PROTOCOL_CONSTANTS_H_

#include <cstddef>
#include <cstdint>

namespace robotcar01::protocol {

// 协议版本；未知版本按 kRejectedVersion 拒绝（不静默兼容）。
inline constexpr uint8_t kProtocolVersion = 1;

// magic 线序 'R','C','0','1'。**该常量仅用于编码**：解码必须逐字节比较线序字节，
// 禁止对整数常量做 memcmp（端序陷阱，D-003-14）。
inline constexpr uint32_t kFrameMagic = 0x31304352u;
inline constexpr uint8_t kMagicByte0 = 0x52;  // 'R'
inline constexpr uint8_t kMagicByte1 = 0x43;  // 'C'
inline constexpr uint8_t kMagicByte2 = 0x30;  // '0'
inline constexpr uint8_t kMagicByte3 = 0x31;  // '1'

inline constexpr size_t kFrameHeaderBytes = 12;         // magic4 + ver1 + type1 + flags1 + len1 + seq4
inline constexpr size_t kCrcBytes = 2;
inline constexpr size_t kMaxCommandPayloadBytes = 32;   // 命令帧载荷上限（只作装配期静态上界，不参与校验）
inline constexpr size_t kMaxFeedbackPayloadBytes = 200;  // 反馈帧载荷上限（状态 64 / 诊断 200）
inline constexpr size_t kMaxPayloadBytes = 200;         // 通用上限：长度攻击在该值前拒绝
inline constexpr size_t kMaxFrameBytes = 216;           // 12 + 200 + 2 + 2 余量

inline constexpr size_t kMotionPayloadBytes = 13;       // v4 + omega4 + run1 + reserved1 + lease2 + send_age1
inline constexpr size_t kStopPayloadBytes = 0;
inline constexpr size_t kStatusPayloadBytes = 64;
inline constexpr size_t kDiagPayloadBytes = 200;       // 28 统计 + 8 元信息 + 4 保留 + 8×16 任务槽
inline constexpr size_t kMaxDiagTaskSlots = 8;

// 消息类型：命令下行（0x0x）/ 反馈上行（0x8x）方向分离，便于拒绝"反向帧"（A3.4）。
enum class MessageType : uint8_t {
  kMotionCommand = 0x01,
  kStopCommand = 0x02,
  kFeedbackStatus = 0x81,
  kFeedbackDiag = 0x82,
};

// 该类型是否为命令方向（下行）。反馈帧到达命令路径必须丢弃且不计 seq 倒退。
constexpr bool IsCommandDirection(MessageType type) {
  return type == MessageType::kMotionCommand || type == MessageType::kStopCommand;
}

// 该类型是否为反馈方向（上行）。
constexpr bool IsFeedbackDirection(MessageType type) {
  return type == MessageType::kFeedbackStatus || type == MessageType::kFeedbackDiag;
}

constexpr bool IsKnownMessageType(uint8_t raw) {
  return raw == static_cast<uint8_t>(MessageType::kMotionCommand) ||
         raw == static_cast<uint8_t>(MessageType::kStopCommand) ||
         raw == static_cast<uint8_t>(MessageType::kFeedbackStatus) ||
         raw == static_cast<uint8_t>(MessageType::kFeedbackDiag);
}

// 装配期静态断言：命令载荷的实际长度必须落在声明的上界内（校验逻辑用 RequiredPayloadBytes）。
static_assert(kMotionPayloadBytes <= kMaxCommandPayloadBytes,
              "motion payload exceeds declared command payload bound");
static_assert(kStopPayloadBytes <= kMaxCommandPayloadBytes,
              "stop payload exceeds declared command payload bound");
static_assert(kDiagPayloadBytes <= kMaxFeedbackPayloadBytes,
              "diag payload exceeds declared feedback payload bound");
static_assert(kStatusPayloadBytes <= kMaxFeedbackPayloadBytes,
              "status payload exceeds declared feedback payload bound");
static_assert(kMaxFeedbackPayloadBytes + kFrameHeaderBytes + kCrcBytes <= kMaxFrameBytes,
              "max frame constant is smaller than the largest well-formed frame");

// 类型要求的载荷长度（-1 表示未知类型）。长度不匹配按 kRejectedLength 拒绝。
constexpr int RequiredPayloadBytes(uint8_t raw) {
  switch (raw) {
    case static_cast<uint8_t>(MessageType::kMotionCommand):
      return static_cast<int>(kMotionPayloadBytes);
    case static_cast<uint8_t>(MessageType::kStopCommand):
      return static_cast<int>(kStopPayloadBytes);
    case static_cast<uint8_t>(MessageType::kFeedbackStatus):
      return static_cast<int>(kStatusPayloadBytes);
    case static_cast<uint8_t>(MessageType::kFeedbackDiag):
      return static_cast<int>(kDiagPayloadBytes);
    default:
      return -1;
  }
}

// 解析预算（D-003-6）：CommandTask 每周期最多消费的字节数；预算耗尽不是错误。
struct CommandParseConfig {
  uint32_t max_bytes_per_cycle = 256;
};

}  // namespace robotcar01::protocol

#endif  // ROBOTCAR01_APP_PROTOCOL_PROTOCOL_CONSTANTS_H_
