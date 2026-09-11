// FakeClock：可步进的单调微秒时钟（Iteration 002；仅 host）。
//
// 用途：host 测试替代平台时基，精确驱动 scheduler 的 tick 累计、回绕与任务耗时模拟。
// 语义：Now() 返回内部单调 us 计数；测试用 SetUs/AdvanceUs 推进（单调非减，违反则返回 false）。
// header-only；不依赖 app 之外的实现。

#ifndef ROBOTCAR01_HOST_FAKES_FAKE_CLOCK_H_
#define ROBOTCAR01_HOST_FAKES_FAKE_CLOCK_H_

#include <cstdint>

#include "app/mcu_os_lite/clock.h"

namespace robotcar01::host_fakes {

class FakeClock : public mcu_os_lite::ClockSource {
 public:
  mcu_os_lite::TimePoint Now() const override { return mcu_os_lite::TimePoint{now_us_}; }

  // 绝对设定（测试用；回绕/边界用例直接设目标值）。要求 new >= now，违反返回 false。
  bool SetUs(uint64_t new_us) {
    if (new_us < now_us_) {
      return false;
    }
    now_us_ = new_us;
    return true;
  }

  // 相对推进。要求不反向，违反返回 false。
  bool AdvanceUs(uint64_t delta_us) { return SetUs(now_us_ + delta_us); }

  uint64_t now_us() const { return now_us_; }

 private:
  uint64_t now_us_ = 0;
};

}  // namespace robotcar01::host_fakes

#endif  // ROBOTCAR01_HOST_FAKES_FAKE_CLOCK_H_
