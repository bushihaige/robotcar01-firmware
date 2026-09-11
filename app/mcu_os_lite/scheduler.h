// MCU OS Lite 协作调度器（Iteration 002）
//
// 冻结语义（详见 detailed design §到期与运行语义/§时序）：
//   - TickLoopOnce = 读时钟 → 累计基础 tick（模 2^32）→ TaskSet::RunDue（按注册序）
//     → 跨度式健康窗口判定 → 更新 SchedulerStats；全路径 Run-to-Completion；
//   - 每循环至多执行一次（不追赶，D-OS-001）；overrun 只顺延（D-OS-004）；
//   - overrun = 单次执行 elapsed_us > wcet_budget_us（>0 时判定，记录 WCET 证据）；
//     顺延 last_run_tick += floor(elapsed_us / base_tick_us)，与 tick 累计同向取整
//     （ceil 会让 last_run 超前 tick 计数、模差误判为立即到期，D-OS-004 修订）；
//   - 关键任务 overrun → HealthMonitor::OnTaskOverrun 立即置 stop（D-OS-002）；
//   - ISR 不调用任务；单核主循环是唯一调度者。
// 无动态内存、无所有权转移、无全局可变状态、无异常路径。

#ifndef ROBOTCAR01_APP_MCU_OS_LITE_SCHEDULER_H_
#define ROBOTCAR01_APP_MCU_OS_LITE_SCHEDULER_H_

#include <cstdint>

#include "app/mcu_os_lite/clock.h"
#include "app/mcu_os_lite/health_monitor.h"
#include "app/mcu_os_lite/task_descriptor.h"

namespace robotcar01::mcu_os_lite {

// 调度器级统计（host 断言/诊断读取）。
struct SchedulerStats {
  uint32_t tick_count = 0;   // 当前 tick（模 2^32 计数）
  uint64_t loop_count = 0;   // TickLoopOnce 调用次数
};

// 协作调度器：不拥有被注入对象。
class CooperativeScheduler {
 public:
  // 采样时钟纪元 + Seal 任务表；随后即可 TickLoopOnce。
  void Init(const TickConfig& tick_config, ClockSource& clock, TaskSet& task_set,
            HealthMonitor& health);

  // 单核主循环每轮调用一次。
  void TickLoopOnce();

  uint32_t tick_count() const { return stats_.tick_count; }
  const SchedulerStats& stats() const { return stats_; }

 private:
  // 由 now_us 累计基础 tick（模 2^32；逐次差分，单调时基前置）。
  void AccumulateTicks(uint64_t now_us);

  TickConfig tick_config_{};
  ClockSource* clock_ = nullptr;
  TaskSet* task_set_ = nullptr;
  HealthMonitor* health_ = nullptr;
  SchedulerStats stats_{};
  uint64_t last_poll_us_ = 0;    // 上次时钟采样（Init 时置纪元）
  uint64_t tick_acc_rem_us_ = 0; // 不足一个基础 tick 的微秒余量
  uint32_t last_check_tick_ = 0; // 上次健康 Check 的 tick（跨度式窗口判定）
  bool task_set_sealed_ = false; // 首次 TickLoopOnce 时 Seal（生命周期见详设）
};

}  // namespace robotcar01::mcu_os_lite

#endif  // ROBOTCAR01_APP_MCU_OS_LITE_SCHEDULER_H_
