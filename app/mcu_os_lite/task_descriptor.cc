#include "app/mcu_os_lite/task_descriptor.h"

namespace robotcar01::mcu_os_lite {

bool TaskSet::Register(const TaskConfig& config, void (*run_fn)(void*), void* context,
                       uint32_t supervision_window_ticks) {
  if (sealed_) {
    return false;
  }
  if (count_ >= kMaxTasks) {
    return false;
  }
  if (config.period_ticks == 0u) {
    return false;
  }
  if (config.phase_ticks >= config.period_ticks) {
    return false;
  }
  if (run_fn == nullptr) {
    return false;
  }
  // INV-OS-7（H3 修订）：关键任务周期不得大于健康监督窗口，保证窗口内必有激活机会。
  if (config.criticality == TaskCriticality::kCritical &&
      config.period_ticks > supervision_window_ticks) {
    return false;
  }

  TaskEntry& entry = entries_[count_];
  entry.config = &config;
  entry.run_fn = run_fn;
  entry.context = context;
  // 激活点网格（D-OS-003）：last_run_tick 初始 = (phase - period) mod 2^32，
  // 使首个激活点恰为 tick=phase_ticks。无符号下溢即模 2^32 运算。
  entry.stats = TaskStats{};
  entry.stats.last_run_tick = config.phase_ticks - config.period_ticks;
  ++count_;
  return true;
}

void TaskSet::Seal() { sealed_ = true; }

TaskEntry& TaskSet::at(size_t index) { return entries_.at(index); }

const TaskEntry& TaskSet::at(size_t index) const { return entries_.at(index); }

bool TaskSet::IsTaskDue(uint32_t now_tick, const TaskEntry& entry) {
  // uint32 无符号差 = 模 2^32 差分，回绕安全（INV-OS-1）。
  const uint32_t elapsed_since_last = now_tick - entry.stats.last_run_tick;
  return elapsed_since_last >= entry.config->period_ticks;
}

}  // namespace robotcar01::mcu_os_lite
