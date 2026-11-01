// SysTick 微秒时基（Iteration 006.5，mcu_os_lite::ClockSource 的 ARM adapter）
//
// 冻结语义（详设 §5.1，D-006.5-4）：
//   - Now() 单调非减：毫秒计数（ISR 单一写者，32 位原子读）+ SysTick->VAL 换算的亚毫秒余量；
//   - 若换算结果小于上次返回（读 VAL 与读毫秒之间的 1 ms 中断竞态）⇒ 钳制为上次值；
//   - 32 位毫秒计数 ⇒ 49.7 天回绕（远超本迭代运行时长；008 换 DWT/64 位方案时一并替换）；
//   - ISR 侧只做 ++，无浮点、无 HAL 调用、无锁。
// 平台层专用：不进入 host 编译单元。

#ifndef ROBOTCAR01_PLATFORM_STM32_CLOCK_SOURCE_H_
#define ROBOTCAR01_PLATFORM_STM32_CLOCK_SOURCE_H_

#include <cstdint>

#include "app/mcu_os_lite/clock.h"

namespace robotcar01::platform {

class SysTickClockSource final : public mcu_os_lite::ClockSource {
 public:
  // 清零毫秒计数与单调基线（Init 期调用一次）。
  static void Reset();

  // 由 SysTick_Handler 每 1 ms 调用。
  static void OnTickFromIsr();

  // 单调微秒时间戳。
  mcu_os_lite::TimePoint Now() const override;

  // 诊断：自启动起的毫秒数。
  static uint32_t MillisSinceBoot();

 private:
  static volatile uint32_t ms_counter_;  // ISR 唯一写者
  static uint64_t last_us_;              // 单调钳制（主循环上下文）
};

}  // namespace robotcar01::platform

#endif  // ROBOTCAR01_PLATFORM_STM32_CLOCK_SOURCE_H_
