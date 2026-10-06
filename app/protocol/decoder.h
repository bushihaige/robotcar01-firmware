// 流式帧解码状态机（Iteration 003）
//
// 职责边界（评审 H5 / D-003-10）：只做**线格式**校验——magic、版本、类型、flags、长度匹配、
// 保留字段、CRC。**不做**数值/范围校验（那需要应用配置限值，归 CommandManager::OnFrame）。
//
// 重同步规则（评审 C1 / D-003-2，冻结为可实现描述）：
//   SEARCH_MAGIC 之外的状态失配时，**当前字节重新作为新 magic 的首字节候选**参与匹配，
//   绝不吞掉该字节；magic 线序 0x52,0x43,0x30,0x31 的真前缀集合 {52, 5243, 524330} 与
//   真后缀集合 {31, 3031, 433031} 交集为空 ⇒ 重新匹配恒为"从当前字节重新开始"，无需回退表。
//
// 纯状态机：无时钟、无 I/O、无动态内存、无异常路径。

#ifndef ROBOTCAR01_APP_PROTOCOL_DECODER_H_
#define ROBOTCAR01_APP_PROTOCOL_DECODER_H_

#include <cstddef>
#include <cstdint>

#include "app/protocol/frame.h"
#include "app/protocol/protocol_constants.h"
#include "app/protocol/stats.h"

namespace robotcar01::protocol {

enum class DecodeStatus : uint8_t {
  kNeedMoreData = 0,
  kFrameReady,
  kRejectedMagic,
  kRejectedVersion,
  kRejectedFlag,
  kRejectedType,
  kRejectedLength,
  kRejectedCrc,
  kRejectedReserved,
};

struct DecodedFrame {
  FrameHeader header{};
  uint8_t payload[kMaxPayloadBytes] = {};
  bool valid = false;
};

class StreamDecoder {
 public:
  void Init();
  void Reset();  // 会话重连：状态与缓冲清零（stats 由调用方决定是否保留）

  // 恰好消费 1 字节。
  DecodeStatus ConsumeByte(uint8_t byte);

  // 有界批量消费：最多消费 budget.max_bytes_per_cycle 字节；遇到成帧立即返回（不再消费更多）。
  // out_consumed 返回实际消费字节数（可为 nullptr）。
  DecodeStatus Consume(const uint8_t* data, size_t size, size_t* out_consumed,
                       const CommandParseConfig& budget);

  const DecodedFrame& last_frame() const { return frame_; }
  void ClearLastFrame() { frame_.valid = false; }

  bool IsSearchingMagic() const { return state_ == State::kSearchMagic; }

  ProtocolStats& stats() { return stats_; }
  const ProtocolStats& stats() const { return stats_; }

 private:
  enum class State : uint8_t {
    kSearchMagic = 0,
    kCheckMagic,
    kReadHeader,
    kReadPayload,
    kReadCrc,
  };

  void Reject(DecodeStatus status);
  // 回到搜索态；restart_with_current_byte=true 时把当前字节重当候选首字节（D-003-2 重同步）。
  void BackToSearch(bool restart_with_current_byte, uint8_t byte);
  DecodeStatus FinishFrame();

  State state_ = State::kSearchMagic;
  uint8_t magic_index_ = 0;   // 0..2：已匹配的 magic 字节数（SEARCH/CHECK 用）
  uint8_t header_index_ = 0;  // 已收集的帧头字节数（含 magic，共 12）
  uint16_t payload_index_ = 0;
  uint16_t crc_index_ = 0;
  uint16_t frame_len_ = 0;    // 线格式上的总字节数（头 + 载荷 + CRC）
  uint8_t raw_[kMaxFrameBytes] = {};
  DecodedFrame frame_{};
  ProtocolStats stats_{};
};

}  // namespace robotcar01::protocol

#endif  // ROBOTCAR01_APP_PROTOCOL_DECODER_H_
