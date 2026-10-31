// 板级装配（Iteration 006.5）
//
// 把 002 调度器、004 编码器估计、006 轮速控制与输出仲裁接到真机外设上（详设 §5.5）。
//   - 三个任务：EncoderTask(5 ms, phase 0) / ControlTask(5 ms, phase 2) / ArbitrationTask(1 ms, phase 0)；
//   - ControlTask 在闭环模式转调 006 的 WheelControlTaskRun，在开环模式直接经 Submit 送固定占空比
//     （D-006.5-5 / D-006-10：开环扫点同样不得绕过仲裁）；
//   - 串口单字符命令 + 200 ms 状态行（观测口径见详设 §8）；
//   - 全部对象静态持有，无动态内存。
//
// 安全：Init 任一步失败 ⇒ 返回 false 且 EN 保持失能（INV-006.5-5）；主循环不得进入。

#ifndef ROBOTCAR01_PLATFORM_STM32_BOARD_APP_H_
#define ROBOTCAR01_PLATFORM_STM32_BOARD_APP_H_

#include <cstdint>

#include "app/chassis/chassis_types.h"
#include "app/control/control_tasks.h"
#include "app/control/controller_config.h"
#include "app/control/output_arbitration.h"
#include "app/control/wheel_speed_controller.h"
#include "app/encoder/encoder_config.h"
#include "app/encoder/encoder_estimator.h"
#include "app/encoder/wheel_state.h"
#include "app/mcu_os_lite/health_monitor.h"
#include "app/mcu_os_lite/scheduler.h"
#include "app/mcu_os_lite/task_descriptor.h"
#include "platform_stm32/clock_source.h"
#include "platform_stm32/command_console.h"
#include "platform_stm32/encoder_input.h"
#include "platform_stm32/motor_pwm.h"

namespace robotcar01::platform {

// 由 SysTick_Handler（1 ms ISR）调用：把监护转发给当前活动的 Ibt2PwmSink（INV-006.5-6）。
// 主循环停止刷新输出时，本钩子会在超时后直接把电机断电。
void MotorWatchdogTickFromIsr();

enum class MotorMode : uint8_t { kClosed = 0, kOpen = 1 };
enum class WheelSelection : uint8_t { kBoth = 0, kLeft = 1, kRight = 2 };

class BoardApp {
 public:
  // 装配全部对象与任务；失败时 last_error() 给出唯一原因。
  bool Init();

  // 主循环一次：调度器 → 命令 → 周期状态行。
  void Step();

  // 启动时打印命令说明。
  void PrintHelp();

  const char* last_error() const { return last_error_; }

  // 诊断：调度器 tick 计数（心跳行使用）。
  uint32_t SchedulerTick() const { return scheduler_.tick_count(); }

 private:
  static void EncoderTaskEntry(void* arg);
  static void ControlTaskEntry(void* arg);
  static void ArbitrationTaskEntry(void* arg);

  void OnEncoderTick();
  void OnControlTick();
  control::MotorOutputRequest MakeOpenLoopRequest(bool active) const;
  void Apply(ConsoleCommand command);
  void RefreshTarget();
  void PrintStatusLine(uint8_t index);  // 分帧输出：每次 Step 只发一行（D-006.5-7 修订）
  void PrintGpioScan();   // 'g'：GPIOA/GPIOB 全输入位
  void PrintEncoderRaw();  // 'e'：原始计数 + PA0/PA1/PA6/PA7 电平
  void PrintInfo();                     // 'i'：构建信息 + 生效配置（核对固件身份，替代易丢的横幅）
  void SetError(const char* text) { last_error_ = text; }

  static constexpr uint8_t kStatusLineCount = 5U;

  // ---- 平台对象（静态持有，无动态内存） ----
  SysTickClockSource clock_{};
  EncoderInput encoder_input_{};
  Ibt2PwmSink sink_{};
  encoder::EncoderEstimator estimator_{};
  control::WheelSpeedController left_{};
  control::WheelSpeedController right_{};
  control::OutputArbitration arbitration_{};
  mcu_os_lite::TaskSet task_set_{};
  mcu_os_lite::HealthMonitor health_{};
  mcu_os_lite::CooperativeScheduler scheduler_{};
  CommandConsole console_{};

  // ---- 装配期配置 ----
  encoder::EncoderConfig encoder_config_{};
  control::WheelControllerConfig controller_config_{};
  mcu_os_lite::TaskConfig encoder_task_config_{};

  // ---- 数据端口 ----
  control::ControlContext ctx_{};
  chassis::WheelTarget target_{};
  chassis::SafetyStatus safety_{};
  encoder::WheelState wheel_state_{};

  // ---- 运行态（命令可改） ----
  MotorMode mode_ = MotorMode::kClosed;
  WheelSelection wheels_ = WheelSelection::kBoth;
  float target_mps_ = 0.0f;
  uint16_t open_duty_permille_ = 0U;
  control::MotorDirection open_direction_ = control::MotorDirection::kForward;
  uint32_t last_status_ms_ = 0U;
  uint8_t status_line_index_ = 0U;
  bool status_active_ = false;
  const char* last_error_ = "none";
};

}  // namespace robotcar01::platform

#endif  // ROBOTCAR01_PLATFORM_STM32_BOARD_APP_H_
