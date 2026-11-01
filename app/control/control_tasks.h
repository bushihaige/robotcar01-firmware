// 控制任务体与注册装配（Iteration 006，robotcar01::control）
//
// 任务表（详设 §7，相位/预算登记，arch §6）：
//   WheelControlTask      period = kNominalPeriodUs/1000 = 5 tick, phase 2, wcet 400 us, kCritical
//   OutputArbitrationTask period = 1 tick,                         phase 0, wcet 100 us, kCritical
// 本文件不拥有任何对象：ControlContext 只保存装配方持有对象的指针（002 TaskSet 同约定）。
// WheelSpeedRequest 由 WheelControlTask 经 001 RunSafetyGate 生产（复用冻结组合规则，D-006-6），
// 供 003 反馈的 output_* / kFlagForceDisable 使用（含义 = 仲裁后允许的速度域目标）。

#ifndef ROBOTCAR01_APP_CONTROL_CONTROL_TASKS_H_
#define ROBOTCAR01_APP_CONTROL_CONTROL_TASKS_H_

#include <cstdint>

#include "app/chassis/chassis_types.h"
#include "app/control/output_arbitration.h"
#include "app/control/wheel_speed_controller.h"
#include "app/encoder/wheel_state.h"
#include "app/mcu_os_lite/clock.h"
#include "app/mcu_os_lite/task_descriptor.h"

namespace robotcar01::control {

// 装配方持有的数据端口（单核协作：任务边界内读写，无 ISR 读者）。
struct ControlContext {
  const mcu_os_lite::ClockSource* clock = nullptr;
  const chassis::WheelTarget* target = nullptr;      // 005 输出（装配方每周期刷新）
  const encoder::WheelState* wheel_state = nullptr;   // 004 输出
  const chassis::SafetyStatus* safety = nullptr;      // allow_motion（007 前由 MakeMotionDesired 推导）
  WheelSpeedController* left = nullptr;
  WheelSpeedController* right = nullptr;
  OutputArbitration* arbitration = nullptr;
  chassis::WheelSpeedRequest speed_request{};         // 控制任务产出（供反馈）
  MotorOutputPair last_pair{};                        // 最近一次提交（诊断）
};

// 任务体（签名满足 TaskEntry::run_fn）。
void WheelControlTaskRun(void* context);
void OutputArbitrationTaskRun(void* context);

// 任务配置（静态对象，由 TaskSet 以指针引用）。
const mcu_os_lite::TaskConfig& WheelControlTaskConfig();
const mcu_os_lite::TaskConfig& OutputArbitrationTaskConfig();

// 注册两个任务（顺序：控制在前、仲裁在后；同 tick 内按注册序执行）。
bool RegisterControlTasks(mcu_os_lite::TaskSet& task_set, ControlContext& context,
                          uint32_t supervision_window_ticks);

}  // namespace robotcar01::control

#endif  // ROBOTCAR01_APP_CONTROL_CONTROL_TASKS_H_
