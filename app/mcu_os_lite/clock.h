// MCU OS Lite 时基抽象（Iteration 002）
//
// 冻结语义（详见 detailed design §API/§平台兼容性，D-002-2）：
//   - TimePoint 为单调微秒时间戳（uint64；P0：类型承载单位，不加后缀）；
//   - ClockSource 是只读单调时基能力契约：host 用 host_fakes/fake_clock，
//     ARM 用 DWT/TIM adapter（Iteration 008）；核心不引用 HAL；
//   - OS 调度 tick 域为 uint32 模 2^32（见 task_descriptor/scheduler）；
//     本头文件只负责微秒时基，不承载 tick 回绕语义。
// 前置条件（文档化，D-002-2/S8 闭环）：ClockSource::Now() 必须单调非减，
// 008 adapter 负责在 ARM 上提供跨低 32 位回绕的单调 64 位时基。
// 纯 header；无动态内存、无全局可变状态、无异常路径。

#ifndef ROBOTCAR01_APP_MCU_OS_LITE_CLOCK_H_
#define ROBOTCAR01_APP_MCU_OS_LITE_CLOCK_H_

#include <cstdint>

namespace robotcar01::mcu_os_lite {

// 单调微秒时间戳。单位由类型承载（微秒），差分用 ElapsedUsSince。
struct TimePoint {
  uint64_t us = 0;

  // 返回自 start 起的微秒差分。前置条件：本对象单调 >= start（ClockSource 保证）。
  constexpr uint64_t ElapsedUsSince(TimePoint start) const { return us - start.us; }
};

// 时基能力契约。实现者必须提供单调非减的 Now()。
class ClockSource {
 public:
  virtual TimePoint Now() const = 0;
  virtual ~ClockSource() = default;
};

// OS 基础 tick 配置（arch A-002：默认 1 ms；所有任务周期为其整数倍）。
struct TickConfig {
  uint32_t base_tick_us = 1000;
};

}  // namespace robotcar01::mcu_os_lite

#endif  // ROBOTCAR01_APP_MCU_OS_LITE_CLOCK_H_
