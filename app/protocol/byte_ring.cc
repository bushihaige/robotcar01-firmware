#include "app/protocol/byte_ring.h"

namespace robotcar01::protocol {

size_t ByteRing::Write(const uint8_t* data, size_t size) {
  if (data == nullptr || size == 0) {
    return 0;
  }
  const size_t space = kCapacityBytes - size_;
  const size_t accepted = (size <= space) ? size : space;
  for (size_t i = 0; i < accepted; ++i) {
    buffer_[tail_] = data[i];
    tail_ = (tail_ + 1) % kCapacityBytes;
  }
  size_ += accepted;
  const size_t dropped = size - accepted;
  if (dropped != 0) {
    stats_.overflow_drop_bytes += static_cast<uint32_t>(dropped);
  }
  if (size_ > stats_.high_water_bytes) {
    stats_.high_water_bytes = static_cast<uint32_t>(size_);
  }
  return accepted;
}

bool ByteRing::ReadByte(uint8_t& out) {
  if (size_ == 0) {
    return false;
  }
  out = buffer_[head_];
  head_ = (head_ + 1) % kCapacityBytes;
  --size_;
  return true;
}

void ByteRing::Reset() {
  head_ = 0;
  tail_ = 0;
  size_ = 0;
  stats_ = ByteRingStatistics{};
}

}  // namespace robotcar01::protocol
