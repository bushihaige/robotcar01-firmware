// 时钟初始化（Iteration 002.5）
//
// 冻结链（详设 §时钟、启动顺序与失败语义；评审 C-5 修订）：
//   1) HAL_RCC_OscConfig：HSE ON（等待 HSERDY，HSE_STARTUP_TIMEOUT=100 ms）
//   2) HAL_RCC_OscConfig：PLL ON（M=8, N=336, P=2, Q=7 → VCO_IN=1 MHz, VCO_OUT=336 MHz）
//   3) HAL_RCC_ClockConfig(PLLCLK, AHB=/1, APB1=/4, APB2=/2, FLASH_LATENCY_5)
//      —— 该函数内部更新 SystemCoreClock 并调用 HAL_InitTick 重定时 SysTick
// 不使用 over-drive：F407 无此特性（HAL_PWREx_EnableOverDrive 仅对
// F42x/F43x/F446/F469/F479 编译；168 MHz 在 VOS=Scale1 下即可达成，见详设 B-2.5-4）。
// 外设时钟（GPIO/USART）一律在时钟切换完成后才使能。

#ifndef ROBOTCAR01_PLATFORM_STM32_CLOCK_INIT_H_
#define ROBOTCAR01_PLATFORM_STM32_CLOCK_INIT_H_

#include <cstdint>

namespace robotcar01::platform {

// 时钟失败阶段（降级诊断用；0 = 成功）。
enum class ClockStage : uint32_t {
  kOk = 0,
  kHse = 1,     // HSE 起振失败
  kPll = 2,     // PLL 锁定失败
  kSwitch = 3,  // 切频失败
};

struct ClockResult {
  ClockStage stage = ClockStage::kOk;
  uint32_t hal_code = 0;  // HAL_StatusTypeDef 原始返回
};

// 配置 HSE+PLL，切换 SYSCLK 到 168 MHz；失败时不切换（保持 HSI）并返回失败阶段。
ClockResult ClockInit();

}  // namespace robotcar01::platform

#endif  // ROBOTCAR01_PLATFORM_STM32_CLOCK_INIT_H_
