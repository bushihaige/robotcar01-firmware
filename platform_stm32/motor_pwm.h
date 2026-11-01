// IBT-2（BTS7960 ×2）PWM 输出（Iteration 006.5，实现 006 的 PwmSink 契约）
//
// 冻结语义（详设 §5.3、D-006.5-1、INV-006.5-1/2/3）：
//   - 每个 IBT-2 模块承载一个电机：RPWM/LPWM 双 PWM 调速，R_EN/L_EN 短接后由 MCU GPIO 控制；
//   - kCoast ⇒ EN=0 且两路 CCR=0（高阻滑行）；kBrake ⇒ EN=1 且两路 CCR=0（低侧双低制动）；
//   - kDrive ⇒ EN=1，按方向给 RPWM 或 LPWM 占空比；
//   - 方向切换必须经过 ≥1 个仲裁周期（1 ms）的全零窗口（pending_switch 机制）；
//   - 非法组合（duty>0 但 mode!=kDrive / direction=kNone）一律 coast 并返回 platform_error；
//   - 本类的 Apply 只允许由 OutputArbitration::Tick 调用（INV-006-4）。
// 平台层专用：不进入 host 编译单元。

#ifndef ROBOTCAR01_PLATFORM_STM32_MOTOR_PWM_H_
#define ROBOTCAR01_PLATFORM_STM32_MOTOR_PWM_H_

#include <cstdint>

#include "app/control/motor_output.h"
#include "platform_stm32/pwm_switch_guard.h"
#include "platform_stm32/pwm_watchdog.h"

namespace robotcar01::platform {

// PwmApplyStatus.platform_error 取值（0 = 无错误）
constexpr uint32_t kPwmErrNone = 0U;
constexpr uint32_t kPwmErrNotInitialized = 1U;
constexpr uint32_t kPwmErrInvalidRequest = 2U;

class Ibt2PwmSink final : public control::PwmSink {
 public:
  // 使能 GPIOB/TIM4 时钟、配置 4 路 PWM 与 2 路 EN；失败保持未初始化（EN 保持低）。
  bool Init();

  control::PwmApplyStatus Apply(const control::MotorOutputRequest& left,
                                const control::MotorOutputRequest& right) override;

  // ---- 主循环独立监护（INV-006.5-6）----
  // 由 1 ms SysTick 中断每拍调用；距上次成功 Apply 超过 timeout_ms 就强制 EN=0、CCR=0。
  // timeout_ms = 0 关闭。返回 true 表示本次调用触发了强制失能。
  bool ServiceWatchdogFromIsr(uint32_t now_ms);

  // 设置/关闭监护（0 = 关闭）。Init 末尾默认按 kMotorWatchdogTimeoutMs 开启。
  void SetWatchdogTimeoutMs(uint32_t timeout_ms) { watchdog_timeout_ms_ = timeout_ms; }
  uint32_t watchdog_timeout_ms() const { return watchdog_timeout_ms_; }

  // 诊断
  uint32_t watchdog_trips() const { return watchdog_trips_; }
  uint32_t apply_calls() const { return apply_calls_; }
  uint32_t switch_zero_windows() const {
    return guards_[0].zero_windows() + guards_[1].zero_windows();
  }
  uint32_t rejected_requests() const { return rejected_requests_; }
  uint16_t last_duty_permille(int side) const { return last_duty_permille_[side & 1]; }
  bool enable_state(int side) const { return enable_state_[side & 1]; }

 private:
  // Init 失败路径：把 EN 与 CCR 强制拉回失能零输出（INV-006.5-5）。
  void ForceSafeState();
  bool WriteSide(int index, const control::MotorOutputRequest& request);
  void SetCompare(int index, uint32_t rpwm_counts, uint32_t lpwm_counts);
  void SetEnable(int index, bool on);
  static uint32_t DutyToCompare(uint16_t duty_permille);

  PwmSwitchGuard guards_[2]{};
  uint16_t last_duty_permille_[2] = {0U, 0U};
  bool enable_state_[2] = {false, false};
  uint32_t apply_calls_ = 0U;
  uint32_t rejected_requests_ = 0U;
  uint32_t watchdog_timeout_ms_ = 0U;   // 0 = 禁用
  uint32_t watchdog_last_ok_ms_ = 0U;
  uint32_t watchdog_trips_ = 0U;
  uint32_t isr_ms_ = 0U;                // 最近一次 ISR 提供的毫秒
  bool watchdog_armed_ = false;
  bool initialized_ = false;
};

}  // namespace robotcar01::platform

#endif  // ROBOTCAR01_PLATFORM_STM32_MOTOR_PWM_H_
