// RobotCar01 平台参数唯一真源（Iteration 002.5，INV-2.5-3）
//
// 板级事实（B-2.5-4，变更需工程师确认）：
//   MCU  : STM32F407ZGT6（Cortex-M4F, 1 MB Flash, 128 KB SRAM + 64 KB CCM）
//   HSE  : 8 MHz 晶振
//   PLL  : M=8, N=336, P=2  → VCO_IN=1 MHz, VCO_OUT=336 MHz, SYSCLK=168 MHz
//          （F407 的 RCC_PLLCFGR 无 PLLR 字段；Q=7/Q 分频供后续 USB，本迭代不使用）
//   AHB  : /1 = 168 MHz；APB1 /4 = 42 MHz；APB2 /2 = 84 MHz
//   Flash: 5 wait states + prefetch（168 MHz @3.3 V）
//   观测 : USART1 PA9(TX)/PA10(RX) AF7 @115200 8N1（B-2.5-5）

#ifndef ROBOTCAR01_PLATFORM_STM32_PLATFORM_CONFIG_H_
#define ROBOTCAR01_PLATFORM_STM32_PLATFORM_CONFIG_H_

#include <cstdint>

namespace robotcar01::platform {

// ---- 时钟 ----
constexpr uint32_t kHseHz = 8000000U;      // 板载 HSE
constexpr uint32_t kPllM = 8U;             // VCO 输入 = HSE/M = 1 MHz
constexpr uint32_t kPllN = 336U;           // VCO 输出 = 336 MHz
constexpr uint32_t kPllP = 2U;             // SYSCLK = 168 MHz
constexpr uint32_t kPllQ = 7U;             // 48 MHz，USB 预留（本迭代未使用）
constexpr uint32_t kSysclkHz = 168000000U; // 期望 SYSCLK（供横幅比对）
constexpr uint32_t kHclkHz = 168000000U;
constexpr uint32_t kPclk1Hz = 42000000U;
constexpr uint32_t kPclk2Hz = 84000000U;

// ---- USART1 观测通道 ----
constexpr uint32_t kConsoleBaud = 115200U;
constexpr uint32_t kConsoleTxTimeoutMs = 100U;

// ---- 心跳 ----
constexpr uint32_t kHeartbeatPeriodMs = 1000U;  // 心跳周期（验收 4 的量化对象）

}  // namespace robotcar01::platform

#endif  // ROBOTCAR01_PLATFORM_STM32_PLATFORM_CONFIG_H_
