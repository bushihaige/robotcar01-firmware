// 字节环测试（Iteration 003，评审 H10 ③）：容量、高水位、满时丢弃新字节、Reset、环绕顺序
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "app/protocol/byte_ring.h"

namespace robotcar01::protocol {
namespace {

TEST(ByteRingTest, FullRingDropsNewBytesAndCounts) {
  ByteRing ring;
  std::vector<uint8_t> big(ByteRing::kCapacityBytes + 100, 0xAB);
  const size_t written = ring.Write(big.data(), big.size());
  EXPECT_EQ(written, ByteRing::kCapacityBytes);
  EXPECT_EQ(ring.statistics().overflow_drop_bytes, 100u);
  EXPECT_EQ(ring.statistics().high_water_bytes, ByteRing::kCapacityBytes);
  EXPECT_EQ(ring.available(), ByteRing::kCapacityBytes);

  uint8_t byte = 0;
  ASSERT_TRUE(ring.ReadByte(byte));
  EXPECT_EQ(byte, 0xABu);
}

TEST(ByteRingTest, ResetClearsDataAndStatistics) {
  ByteRing ring;
  const uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  ring.Write(data, sizeof(data));
  EXPECT_EQ(ring.available(), sizeof(data));
  ring.Reset();
  EXPECT_EQ(ring.available(), 0u);
  EXPECT_EQ(ring.statistics().overflow_drop_bytes, 0u);
  EXPECT_EQ(ring.statistics().high_water_bytes, 0u);
  uint8_t byte = 0;
  EXPECT_FALSE(ring.ReadByte(byte));
}

TEST(ByteRingTest, WrapAroundKeepsOrder) {
  ByteRing ring;
  const uint8_t first[300] = {};
  std::vector<uint8_t> second(300);
  for (size_t i = 0; i < second.size(); ++i) {
    second[i] = static_cast<uint8_t>(i & 0xFF);
  }
  ring.Write(first, sizeof(first));
  uint8_t byte = 0;
  for (int i = 0; i < 300; ++i) {
    ASSERT_TRUE(ring.ReadByte(byte));
  }
  ring.Write(second.data(), second.size());
  for (size_t i = 0; i < second.size(); ++i) {
    ASSERT_TRUE(ring.ReadByte(byte));
    EXPECT_EQ(byte, second[i]);
  }
}

TEST(ByteRingTest, SingleByteWriteBeyondCapacityIsCounted) {
  ByteRing ring;
  std::vector<uint8_t> exact(ByteRing::kCapacityBytes, 0x11);
  EXPECT_EQ(ring.Write(exact.data(), exact.size()), ByteRing::kCapacityBytes);
  const uint8_t extra = 0x22;
  EXPECT_EQ(ring.Write(&extra, 1), 0u);
  EXPECT_EQ(ring.statistics().overflow_drop_bytes, 1u);
  EXPECT_EQ(ring.statistics().high_water_bytes, ByteRing::kCapacityBytes);
}

TEST(ByteRingTest, EmptyWriteAndNullAreNoOps) {
  ByteRing ring;
  const uint8_t data[1] = {0x5A};
  EXPECT_EQ(ring.Write(nullptr, 0), 0u);
  EXPECT_EQ(ring.Write(data, 0), 0u);
  EXPECT_EQ(ring.statistics().overflow_drop_bytes, 0u);
  EXPECT_EQ(ring.available(), 0u);
}

}  // namespace
}  // namespace robotcar01::protocol
