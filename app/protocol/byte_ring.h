// 有界字节环（Iteration 003）
//
// 边界：编译期定容、无动态内存、无 HAL 依赖。写者 = USB CDC 回调/ISR（008 接线），
// 读者 = CommandTask（单核协作模型，无抢占）；本模块只提供容量/溢出/高水位可观测性，
// 不做任何协议解析（arch §10：回调只做有限量数据入环 + 事件置位）。
//
// 满时策略冻结为"丢弃新字节 + 计数"（D-003-3）：丢弃旧字节会破坏帧同步且不可观测。
// Reset 只允许在写者已停止（USB 重新枚举已发生）后由主循环侧调用，且只应经
// ResetCommandSession() 统一入口（003 详细设计 §会话重置唯一入口）。

#ifndef ROBOTCAR01_APP_PROTOCOL_BYTE_RING_H_
#define ROBOTCAR01_APP_PROTOCOL_BYTE_RING_H_

#include <array>
#include <cstddef>
#include <cstdint>

namespace robotcar01::protocol {

// 环统计（只读快照）。
struct ByteRingStatistics {
  uint32_t overflow_drop_bytes = 0;  // 因满而被丢弃的新字节累计（单一真源，ProtocolStats 取其镜像）
  uint32_t high_water_bytes = 0;     // 历史最大占用量（标定入环预算的证据）
};

class ByteRing {
 public:
  static constexpr size_t kCapacityBytes = 512;

  // 写入 size 字节，返回实际写入数量；剩余字节被丢弃并计入 overflow_drop_bytes。
  size_t Write(const uint8_t* data, size_t size);

  // 单字节消费；空环返回 false（不修改 out）。
  bool ReadByte(uint8_t& out);

  size_t available() const { return size_; }
  bool empty() const { return size_ == 0; }
  size_t capacity() const { return kCapacityBytes; }

  // 只允许写者停止后调用；清空数据与统计（统计随会话重置归零）。
  void Reset();

  const ByteRingStatistics& statistics() const { return stats_; }

 private:
  std::array<uint8_t, kCapacityBytes> buffer_{};
  size_t head_ = 0;  // next pop position
  size_t tail_ = 0;  // next push position
  size_t size_ = 0;
  ByteRingStatistics stats_{};
};

}  // namespace robotcar01::protocol

#endif  // ROBOTCAR01_APP_PROTOCOL_BYTE_RING_H_
