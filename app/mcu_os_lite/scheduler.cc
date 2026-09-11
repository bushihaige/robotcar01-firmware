#include "app/mcu_os_lite/scheduler.h"

#include <cstdint>

namespace robotcar01::mcu_os_lite {

namespace {
// overrun 顺延的上限（模 2^32 域内保证 (now - last_run) 不会因顺延越过半程而误判到期）。
constexpr uint32_t kMaxDeferTicks = 1u << 30;
}  // namespace

void CooperativeScheduler::Init(const TickConfig& tick_config, ClockSource& clock,
                                TaskSet& task_set, HealthMonitor& health) {
  tick_config_ = tick_config;
  clock_ = &clock;
  task_set_ = &task_set;
  health_ = &health;
  // 时钟纪元采样（S8）：首轮 tick 从 0 计，逐次差分累加。
  last_poll_us_ = clock_->Now().us;
  tick_acc_rem_us_ = 0;
  stats_ = SchedulerStats{};
  last_check_tick_ = 0;
  task_set_sealed_ = false;
  // 注意：不在 Init Seal——生命周期为 scheduler.Init → 任务 Register → 首次
  // TickLoopOnce 时 Seal（详设 §生命周期，D-002-1/S8）；保证 Register 与调度解耦。
}

void CooperativeScheduler::TickLoopOnce() {
  if (!task_set_sealed_) {
    task_set_->Seal();  // 冻结注册表：此后 Register 返回 false（S8/H4 修订）
    task_set_sealed_ = true;
  }
  const TimePoint now = clock_->Now();
  AccumulateTicks(now.us);
  ++stats_.loop_count;

  const uint32_t now_tick = stats_.tick_count;
  const size_t task_count = task_set_->size();
  for (size_t i = 0; i < task_count; ++i) {
    TaskEntry& entry = task_set_->at(i);
    if (!TaskSet::IsTaskDue(now_tick, entry)) {
      continue;
    }
    // 记录执行开始（overrun/耗时测量用时钟差分，独立于 tick 快照）。
    const TimePoint start = clock_->Now();

    entry.run_fn(entry.context);
    ++entry.stats.run_count;  // 心跳计数（先于 overrun 判定：即使 overrun 也计入心跳）
    entry.stats.last_run_tick = now_tick;

    const uint64_t elapsed_us = clock_->Now().ElapsedUsSince(start);
    if (elapsed_us > entry.stats.max_elapsed_us) {
      entry.stats.max_elapsed_us = elapsed_us;
    }

    const uint32_t budget_us = entry.config->wcet_budget_us;
    if (budget_us > 0u && elapsed_us > budget_us) {
      ++entry.stats.overrun_count;
      // 顺延：等效于把 last_run 推进到"任务实际完成所消耗的整 tick 数"，
      // 避免下次立即再到期。向下取整与 AccumulateTicks 一致（不会让 last_run
      // 超前 tick 计数，从而避免模差把"超前"误判成"已到期"，D-OS-004 修订）。
      uint64_t defer_ticks_u64 = elapsed_us / tick_config_.base_tick_us;  // floor
      if (defer_ticks_u64 > kMaxDeferTicks) {
        defer_ticks_u64 = kMaxDeferTicks;
      }
      entry.stats.last_run_tick += static_cast<uint32_t>(defer_ticks_u64);
      // 上报 overrun 事件（任意任务）；关键性裁决由 HealthMonitor 完成（H4 修订）。
      health_->OnTaskOverrun(static_cast<uint32_t>(i));
    }
  }

  // 跨度式健康窗口判定（H3 修订）：不依赖 now % window == 0，
  // (now - last_check) >= window 即触发，回绕与滞留下均每窗口一次。
  const uint32_t window_ticks = health_->supervision_window_ticks();
  if (now_tick - last_check_tick_ >= window_ticks) {
    health_->Check(now_tick);
    last_check_tick_ = now_tick;
  }
}

void CooperativeScheduler::AccumulateTicks(uint64_t now_us) {
  const uint64_t delta_us = now_us - last_poll_us_;
  last_poll_us_ = now_us;
  const uint64_t base_us = tick_config_.base_tick_us;
  uint64_t remainder_us = tick_acc_rem_us_ + delta_us;
  const uint64_t new_ticks = remainder_us / base_us;
  remainder_us -= new_ticks * base_us;
  tick_acc_rem_us_ = remainder_us;
  stats_.tick_count += static_cast<uint32_t>(new_ticks);  // 模 2^32 回绕
}

}  // namespace robotcar01::mcu_os_lite
