// MCU OS Lite 健康监控（Iteration 002）
//
// 冻结语义（详见 detailed design §到期与运行语义 5/6、§API，H2/H3/H4 修订版）：
//   - OnTaskOverrun(index)：经绑定 TaskSet 判定关键性；关键任务 → 立即 stop_active=true；
//     任意任务窗口累计 overrun >= max_overruns_per_window → 立即 fault_latched=true；
//   - Check(now_tick)：对关键任务做 run_count 心跳增量比较；任一 stale → fault_latched；
//     心跳全新鲜且无锁存 → critical_heartbeat_ok=true；随后窗口 overrun 计数清零；
//   - 首窗宽限（实现）：每个任务在"首次被 Check 观察"时建立心跳基线、不判 stale
//     （与注册顺序解耦；Init 只需绑定 TaskSet）；
//   - snapshot() 实时派生 watchdog_feed_allowed（INV-OS-8），不依赖 Check 刷新时序；
//   - stop_active/fault_latched 只能由 ClearFault() 复位（INV-OS-3），无隐式自复位；
//     ClearFault 不清 run_count 基线（Check 自推进）。
// 无动态内存、无全局可变状态、无异常路径。

#ifndef ROBOTCAR01_APP_MCU_OS_LITE_HEALTH_MONITOR_H_
#define ROBOTCAR01_APP_MCU_OS_LITE_HEALTH_MONITOR_H_

#include <array>
#include <cstdint>

#include "app/mcu_os_lite/task_descriptor.h"

namespace robotcar01::mcu_os_lite {

// 健康监督配置（窗口与阈值入配置，可调参不改语义，D-002-4）。
struct HealthConfig {
  uint32_t supervision_window_ticks = 100;  // 默认 100 tick = 100 ms（HealthTask 周期）
  uint32_t max_overruns_per_window = 2;     // 窗口内任意任务 overrun 累计达此值 → 锁存
};

// 健康快照（只读；feed_allowed 每次 snapshot() 实时派生）。
struct HealthSnapshot {
  bool watchdog_feed_allowed = true;  // !stop_active && !fault_latched && critical_heartbeat_ok
  bool stop_active = false;           // 关键任务单次 overrun → 受控停止请求（锁存，需 ClearFault）
  bool fault_latched = false;         // 窗口重复 overrun / 关键心跳缺失 → 锁存故障（需 ClearFault）
  bool critical_heartbeat_ok = true;  // 最近一次 Check：所有关键任务心跳新鲜
  uint32_t stale_critical_count = 0;  // 心跳缺失的关键任务数（最近一次 Check）
  uint32_t overruns_in_window = 0;    // 当前窗口内 overrun 事件累计（任意任务）
};

class HealthMonitor {
 public:
  void Init(const HealthConfig& config, TaskSet& task_set);

  // 调度器在检测到 overrun 时回调（任意任务；关键性在本对象内裁决，H4 修订）。
  void OnTaskOverrun(uint32_t task_index);

  // 调度器在监督窗口边界调用（跨度式触发由 scheduler 负责）。
  void Check(uint32_t now_tick);

  // 显式清故障：清 stop/fault/窗口 overrun 计数/stale 计数；不清 run_count 心跳基线。
  void ClearFault();

  HealthSnapshot snapshot() const;

  uint32_t supervision_window_ticks() const { return config_.supervision_window_ticks; }

 private:
  HealthConfig config_{};
  TaskSet* task_set_ = nullptr;
  HealthSnapshot state_{};
  uint32_t overruns_in_window_ = 0;
  // 每任务心跳基线是否已建立（true 后该任务下一次 Check 才判 stale）。
  std::array<bool, TaskSet::kMaxTasks> baseline_valid_{};
  std::array<uint64_t, TaskSet::kMaxTasks> heartbeat_baseline_{};
  bool initialized_ = false;
};

}  // namespace robotcar01::mcu_os_lite

#endif  // ROBOTCAR01_APP_MCU_OS_LITE_HEALTH_MONITOR_H_
