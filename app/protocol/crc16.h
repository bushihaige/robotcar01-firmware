// CRC-16/CCITT-FALSE（Iteration 003）
//
// 参数冻结（003 详细设计 §帧格式冻结）：多项式 0x1021、初值 0xFFFF、不反射、无末异或、
// 按字节 MSB-first。固定测试向量（host 断言，防止实现漂移）：
//   Crc16CcittFalse("123456789") == 0x29B1（标准 check 值）
//   Crc16CcittFalse(<空输入>)    == 0xFFFF（初值自检）
//   Crc16CcittFalse(<24×0x00>)   == 0xEF8A
// 编译期生成表；无动态内存、无 HAL 依赖。

#ifndef ROBOTCAR01_APP_PROTOCOL_CRC16_H_
#define ROBOTCAR01_APP_PROTOCOL_CRC16_H_

#include <array>
#include <cstddef>
#include <cstdint>

namespace robotcar01::protocol {

namespace detail {

constexpr uint16_t Crc16Byte(uint16_t crc, uint8_t byte) {
  crc ^= static_cast<uint16_t>(static_cast<uint16_t>(byte) << 8);
  for (int bit = 0; bit < 8; ++bit) {
    crc = ((crc & 0x8000u) != 0u) ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
                                  : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

constexpr std::array<uint16_t, 256> MakeCrcTable() {
  std::array<uint16_t, 256> table{};
  for (size_t i = 0; i < table.size(); ++i) {
    // 表必须按"由 0 起步"生成：更新式为 crc = (crc << 8) ^ table[(crc >> 8) ^ byte]，
    // 初值 0xFFFF 由调用方承担（若用 0xFFFF 起步建表会把初值重复叠加，check 值错误）。
    table[i] = Crc16Byte(0x0000u, static_cast<uint8_t>(i));
  }
  return table;
}

inline constexpr std::array<uint16_t, 256> kCrcTable = MakeCrcTable();

}  // namespace detail

// 计算 CRC-16/CCITT-FALSE。空输入返回初值 0xFFFF。
inline uint16_t Crc16CcittFalse(const uint8_t* data, size_t size) {
  uint16_t crc = 0xFFFFu;
  for (size_t i = 0; i < size; ++i) {
    const uint8_t index = static_cast<uint8_t>(crc >> 8) ^ data[i];
    crc = static_cast<uint16_t>(static_cast<uint16_t>(crc << 8) ^ detail::kCrcTable[index]);
  }
  return crc;
}

}  // namespace robotcar01::protocol

#endif  // ROBOTCAR01_APP_PROTOCOL_CRC16_H_
