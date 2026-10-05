// fake_byte_channel.h —— host 侧字节流通用替身（Iteration 003，header-only，仅 host）
//
// 用途：把一段字节流按任意切片注入被测对象，并记录其输出字节，用于
// "任意切分下解码结果一致"（A1）与端到端回环（A7）类断言。
// 不依赖 GoogleTest、不使用动态内存之外的资源；与 001/002 的 host_fakes 约定一致。

#ifndef ROBOTCAR01_HOST_FAKES_FAKE_BYTE_CHANNEL_H_
#define ROBOTCAR01_HOST_FAKES_FAKE_BYTE_CHANNEL_H_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace robotcar01::host_fakes {

class FakeByteChannel {
 public:
  void Ingest(const std::vector<uint8_t>& bytes) {
    for (uint8_t byte : bytes) {
      rx_.push_back(byte);
    }
  }

  // 把受控切片依次交给 sink；sink 每次收到一块字节（模拟 USB 回调的到达粒度）。
  template <typename Sink>
  void DeliverInChunks(const std::vector<size_t>& chunk_sizes, Sink&& sink) {
    size_t offset = 0;
    size_t index = 0;
    while (offset < rx_.size()) {
      const size_t requested = (index < chunk_sizes.size()) ? chunk_sizes[index] : chunk_sizes.back();
      const size_t take = (requested == 0) ? 0 : ((requested > rx_.size() - offset) ? (rx_.size() - offset) : requested);
      if (take == 0) {
        break;
      }
      sink(rx_.data() + offset, take);
      offset += take;
      ++index;
    }
  }

  // 逐字节交付（最坏切片粒度）。
  template <typename Sink>
  void DeliverByteByByte(Sink&& sink) {
    for (uint8_t byte : rx_) {
      sink(&byte, 1);
    }
  }

  void Emit(const uint8_t* data, size_t size) {
    for (size_t i = 0; i < size; ++i) {
      tx_.push_back(data[i]);
    }
  }

  const std::vector<uint8_t>& tx() const { return tx_; }
  const std::vector<uint8_t>& rx() const { return rx_; }
  void Clear() {
    rx_.clear();
    tx_.clear();
  }

 private:
  std::vector<uint8_t> rx_;
  std::vector<uint8_t> tx_;
};

}  // namespace robotcar01::host_fakes

#endif  // ROBOTCAR01_HOST_FAKES_FAKE_BYTE_CHANNEL_H_
