#include "platform_stm32/clock_source.h"

#include "stm32f4xx_hal.h"

namespace robotcar01::platform {

volatile uint32_t SysTickClockSource::ms_counter_ = 0U;
uint64_t SysTickClockSource::last_us_ = 0U;

void SysTickClockSource::Reset() {
  ms_counter_ = 0U;
  last_us_ = 0U;
}

// C++20 起对 volatile 用 ++ 已弃用（-Werror=volatile），显式读-加-写。
void SysTickClockSource::OnTickFromIsr() { ms_counter_ = ms_counter_ + 1U; }

uint32_t SysTickClockSource::MillisSinceBoot() { return ms_counter_; }

mcu_os_lite::TimePoint SysTickClockSource::Now() const {
  const uint32_t ms = ms_counter_;  // 32 位对齐读在本核上是原子的
  const uint32_t load = SysTick->LOAD;
  uint32_t value = SysTick->VAL;
  if (load == 0U) {
    // SysTick 未配置：退化为纯毫秒
    return mcu_os_lite::TimePoint{static_cast<uint64_t>(ms) * 1000U};
  }
  if (value > load) {
    value = load;  // 防御：LOAD 被改写瞬间可能出现 VAL > LOAD
  }
  const uint32_t elapsed = load - value;  // 自上次 1 ms 重置以来的计数值
  const uint32_t sub_us =
      static_cast<uint32_t>((static_cast<uint64_t>(elapsed) * 1000U) / (static_cast<uint64_t>(load) + 1U));
  uint64_t candidate = static_cast<uint64_t>(ms) * 1000U + sub_us;
  if (candidate < last_us_) {
    candidate = last_us_;  // 单调钳制（D-006.5-4）
  }
  last_us_ = candidate;
  return mcu_os_lite::TimePoint{candidate};
}

}  // namespace robotcar01::platform
