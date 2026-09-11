// MCU OS Lite 静态任务表（Iteration 002）
//
// 冻结语义（详见 detailed design §到期与运行语义，D-OS-003/H1 修订版）：
//   - 激活点网格 = { phase_ticks + k*period_ticks | k=0,1,2,... }，
//     即首个激活点 = tick=phase_ticks（phase=0 即 tick0），每 period_ticks 一个激活点；
//   - 实现：Register 时置 last_run_tick = (phase_ticks - period_ticks) mod 2^32，
//     到期判定 (now_tick - last_run_tick)（uint32 无符号差）>= period_ticks 即回绕安全；
//   - 每循环至多执行一次（不追赶，D-OS-001）；overrun 顺延不在此文件，见 scheduler；
//   - Register 为 Init 期一次性：非法参数/满员/Seal 后一律返回 false；
//   - 关键任务强制 period_ticks <= supervision_window_ticks（INV-OS-7/H3 修订），
//     窗口值由调用方以 supervision_window_ticks 参数传入。
// 无动态内存、无全局可变状态、无异常路径。

#ifndef ROBOTCAR01_APP_MCU_OS_LITE_TASK_DESCRIPTOR_H_
#define ROBOTCAR01_APP_MCU_OS_LITE_TASK_DESCRIPTOR_H_

#include <array>
#include <cstddef>
#include <cstdint>

namespace robotcar01::mcu_os_lite {

// 任务关键性（决定 overrun 是否触发受控停止）。
enum class TaskCriticality : uint8_t { kNonCritical = 0, kCritical = 1 };

// 任务静态配置（Init 期冻结，运行期只读）。
struct TaskConfig {
  const char* name = "";           // 诊断名（静态字符串，OS 不拥有）
  uint32_t period_ticks = 1;       // 周期（基础 tick 整数倍；>=1）
  uint32_t phase_ticks = 0;        // 相位（< period_ticks）：首个激活点 = phase_ticks
  uint32_t wcet_budget_us = 0;     // WCET 预算（us）；0 = 未配置（不判定 overrun）
  TaskCriticality criticality = TaskCriticality::kNonCritical;
};

// 每任务运行统计（调度器更新，测试/健康读取）。
struct TaskStats {
  uint32_t last_run_tick = 0;      // 上次执行时的 tick（模 2^32）
  uint64_t run_count = 0;          // 累计执行次数（心跳的单调计数）
  uint32_t overrun_count = 0;      // 累计 overrun 次数
  uint64_t max_elapsed_us = 0;     // 单次执行最大耗时（us；0 = 尚未执行，由 run_count 消歧义）
};

// 静态任务表元素（纯数据，无所有权）。
struct TaskEntry {
  const TaskConfig* config = nullptr;  // 指向装配方持有的静态配置
  TaskStats stats;
  void (*run_fn)(void* context) = nullptr;  // 执行入口（任务体）
  void* context = nullptr;                  // 任务上下文（无所有权）
};

// 静态任务表：编译期定容、注册期填充、Seal 后只读。
class TaskSet {
 public:
  static constexpr size_t kMaxTasks = 16;   // 学习版上限

  // Init 期一次性注册。校验：!sealed、未满员、period>=1、phase<period、run_fn!=nullptr、
  // 关键任务 period<=supervision_window_ticks。任一失败返回 false。
  bool Register(const TaskConfig& config, void (*run_fn)(void*), void* context,
                uint32_t supervision_window_ticks);

  // 冻结注册表：首次调度前由 scheduler 调用，此后 Register 返回 false。
  void Seal();

  size_t size() const { return count_; }
  bool IsSealed() const { return sealed_; }

  TaskEntry& at(size_t index);              // 供调度/健康/测试读取与更新
  const TaskEntry& at(size_t index) const;

  // 到期判定（模 2^32 回绕安全，纯函数）。
  static bool IsTaskDue(uint32_t now_tick, const TaskEntry& entry);

 private:
  std::array<TaskEntry, kMaxTasks> entries_{};
  size_t count_ = 0;
  bool sealed_ = false;
};

}  // namespace robotcar01::mcu_os_lite

#endif  // ROBOTCAR01_APP_MCU_OS_LITE_TASK_DESCRIPTOR_H_
