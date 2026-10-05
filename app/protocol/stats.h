// 协议统计（Iteration 003）
//
// 只增计数、无浮点、无 HAL 依赖。线格式错误按类目分类（每类恰 +1），供 host 逐项断言
// 并进入 FeedbackDiagPayload（arch §13 可观测性）。
// 单一真源：ring_overflow_bytes 是 ByteRingStatistics::overflow_drop_bytes 的镜像（INV-003-9）。

#ifndef ROBOTCAR01_APP_PROTOCOL_STATS_H_
#define ROBOTCAR01_APP_PROTOCOL_STATS_H_

#include <cstdint>

namespace robotcar01::protocol {

struct ProtocolStats {
  uint32_t frames_accepted = 0;
  uint32_t frames_rejected = 0;

  // 线格式拒绝分类（解码器）
  uint32_t bad_magic = 0;
  uint32_t bad_version = 0;
  uint32_t bad_flag = 0;      // flags != 0（本期保留位必须为 0）
  uint32_t bad_type = 0;
  uint32_t bad_length = 0;    // payload_len > 上限，或与类型要求不符
  uint32_t bad_crc = 0;
  uint32_t bad_reserved = 0;  // 载荷内保留字段非 0（运动帧 reserved0）

  // 会话/方向/链路
  uint32_t seq_rejected = 0;            // 序号重复/倒退/恰好半程
  uint32_t ring_overflow_bytes = 0;     // 镜像 ByteRingStatistics::overflow_drop_bytes
  uint32_t wrong_direction_frames = 0;  // 反馈方向帧到达命令路径（不计 seq_rejected）

  void Reset() { *this = ProtocolStats{}; }
};

}  // namespace robotcar01::protocol

#endif  // ROBOTCAR01_APP_PROTOCOL_STATS_H_
