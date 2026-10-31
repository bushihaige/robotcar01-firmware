// 电机输出看门狗判定（Iteration 006.5，INV-006.5-6）
//
// 真实风险：006 的"请求龄超 10 ms ⇒ coast"是在 OutputArbitration::Tick 里判定的，
// 而 Tick 由主循环调度器驱动。一旦主循环卡死（阻塞 UART 长时间不返回、断言降级、
// 死循环、栈破坏等），Tick 不再执行，PWM 会**保持最后一次占空比**，电机持续转。
// 也就是说：软件层的"超时保护"无法保护它自己。
//
// 因此平台层需要一个独立于主循环的监护：由 1 ms SysTick 中断检查
// 「距上次成功 Apply 的毫秒数」，超过阈值就在 ISR 内直接把 EN 拉低、CCR 清零。
//
// 本头文件只放判定逻辑（纯函数，可 host 穷举），ISR 动作在 motor_pwm.cc。

#ifndef ROBOTCAR01_PLATFORM_STM32_PWM_WATCHDOG_H_
#define ROBOTCAR01_PLATFORM_STM32_PWM_WATCHDOG_H_

#include <cstdint>

namespace robotcar01::platform {

// 超时判定（uint32 毫秒回绕安全：用无符号差比较）。
// timeout_ms == 0 表示禁用（永不判超时）。
inline bool PwmWatchdogExpired(uint32_t now_ms, uint32_t last_ok_ms, uint32_t timeout_ms) {
  if (timeout_ms == 0U) {
    return false;
  }
  return (now_ms - last_ok_ms) > timeout_ms;
}

// 是否应当触发强制失能。armed=false 表示还没建立基线（上电首次调用），不判超时。
inline bool PwmWatchdogShouldTrip(bool armed, bool enabled, uint32_t now_ms, uint32_t last_ok_ms,
                                  uint32_t timeout_ms) {
  return enabled && armed && PwmWatchdogExpired(now_ms, last_ok_ms, timeout_ms);
}

}  // namespace robotcar01::platform

#endif  // ROBOTCAR01_PLATFORM_STM32_PWM_WATCHDOG_H_
