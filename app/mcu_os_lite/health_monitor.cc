#include "app/mcu_os_lite/health_monitor.h"

#include <cstddef>
#include <cstdint>

namespace robotcar01::mcu_os_lite {

void HealthMonitor::Init(const HealthConfig& config, TaskSet& task_set) {
  config_ = config;
  task_set_ = &task_set;
  state_ = HealthSnapshot{};
  overruns_in_window_ = 0;
  // 首窗宽限（H3/实现）：基线在"首次被 Check 观察"时惰性建立，与注册顺序解耦。
  baseline_valid_.fill(false);
  heartbeat_baseline_.fill(0);
  initialized_ = true;
}

void HealthMonitor::OnTaskOverrun(uint32_t task_index) {
  if (!initialized_ || task_index >= task_set_->size()) {
    return;
  }
  ++overruns_in_window_;
  // 关键任务单次 overrun → 立即置 stop（D-OS-002，不等窗口边界）。
  const TaskEntry& entry = task_set_->at(task_index);
  if (entry.config->criticality == TaskCriticality::kCritical) {
    state_.stop_active = true;
  }
  // 窗口内任意任务 overrun 累计达阈值 → 立即锁存（D-002-4；含非关键任务重复 overrun）。
  if (overruns_in_window_ >= config_.max_overruns_per_window) {
    state_.fault_latched = true;
  }
}

void HealthMonitor::Check(uint32_t /*now_tick*/) {
  if (!initialized_) {
    return;
  }
  const size_t count = task_set_->size();
  uint32_t stale_critical = 0;

  for (size_t i = 0; i < count && i < TaskSet::kMaxTasks; ++i) {
    const TaskEntry& entry = task_set_->at(i);
    const uint64_t run_count = entry.stats.run_count;

    // 首窗宽限：该任务首次被 Check 观察 → 只建立心跳基线，不判 stale。
    if (!baseline_valid_[i]) {
      heartbeat_baseline_[i] = run_count;
      baseline_valid_[i] = true;
      continue;
    }

    // 心跳新鲜度 = 自上次 Check 以来 run_count 有增量。
    const bool heartbeat_fresh = (run_count - heartbeat_baseline_[i]) != 0u;
    if (!heartbeat_fresh &&
        entry.config->criticality == TaskCriticality::kCritical) {
      ++stale_critical;
    }
    heartbeat_baseline_[i] = run_count;
  }

  if (stale_critical > 0u) {
    state_.fault_latched = true;
  }
  state_.stale_critical_count = stale_critical;
  state_.critical_heartbeat_ok = (stale_critical == 0u);
  // 窗口累计清零，作为新窗口起点。
  overruns_in_window_ = 0;
}

void HealthMonitor::ClearFault() {
  state_.stop_active = false;
  state_.fault_latched = false;
  overruns_in_window_ = 0;
  state_.stale_critical_count = 0;
  // 不清 run_count 基线（Check 自推进）；critical_heartbeat_ok 由下一次 Check 刷新。
}

HealthSnapshot HealthMonitor::snapshot() const {
  HealthSnapshot snap = state_;
  // 实时派生喂狗许可（INV-OS-8）：不依赖 Check 刷新时序。
  snap.watchdog_feed_allowed =
      !state_.stop_active && !state_.fault_latched && state_.critical_heartbeat_ok;
  snap.overruns_in_window = overruns_in_window_;
  return snap;
}

}  // namespace robotcar01::mcu_os_lite
