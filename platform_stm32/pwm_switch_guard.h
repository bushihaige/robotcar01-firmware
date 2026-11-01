// 方向切换零窗口守卫（Iteration 006.5，INV-006.5-3 的可测试实现）
//
// 意图：IBT-2/BTS7960 是双 PWM 半桥驱动，从一个方向切到相反方向时若直接交换 RPWM/LPWM，
// 会出现两半桥同时导通的瞬间（直通/反向电流冲击）。因此平台层必须保证：
//   **任何一次方向变化，该侧至少经过 1 个仲裁周期（1 ms）的 EN=0 且双 CCR=0。**
//
// 本头文件只包含纯状态机（无 HAL、无外设类型），因此能在 host 单测里被穷举验证；
// `motor_pwm.cc` 负责把它翻译成真实的 CCR/EN 写入动作。
//
// 契约：
//   - 只有 "drive 且 duty>0 且 direction != kNone" 的请求才调用 OnDriveRequest；
//   - coast / brake / 非法请求一律调用 OnZeroOutput（该侧已物理归零，方向记忆随之清空）；
//   - 返回 kZeroOnly 时调用方**必须**把该侧写成零输出（EN 失能、CCR=0），不得施加占空比；
//   - 返回 kApplyNow 时调用方才可施加（仍应先写零、再使能、最后写占空比）。

#ifndef ROBOTCAR01_PLATFORM_STM32_PWM_SWITCH_GUARD_H_
#define ROBOTCAR01_PLATFORM_STM32_PWM_SWITCH_GUARD_H_

#include <cstdint>

#include "app/control/motor_output.h"

namespace robotcar01::platform {

enum class SwitchDecision : uint8_t {
  kApplyNow = 0,  // 可以施加当前方向与占空比
  kZeroOnly = 1,  // 本次只能输出零（零窗口尚未满足）
};

class PwmSwitchGuard {
 public:
  // drive 请求（duty>0、direction != kNone）。前置条件由调用方校验。
  SwitchDecision OnDriveRequest(control::MotorDirection direction) {
    if (pending_) {
      if (direction != pending_direction_) {
        // 零窗口内方向又变：继续保持零，改记新的待切换方向
        pending_direction_ = direction;
        ++zero_windows_;
        return SwitchDecision::kZeroOnly;
      }
      // 零窗口已持续至少一个仲裁周期且方向稳定 ⇒ 允许施加
      pending_ = false;
      pending_direction_ = control::MotorDirection::kNone;
    } else if (last_direction_ != control::MotorDirection::kNone &&
               last_direction_ != direction) {
      // 首次检测到方向变化：本次只输出零，等待下一个仲裁周期
      pending_ = true;
      pending_direction_ = direction;
      last_direction_ = control::MotorDirection::kNone;
      ++zero_windows_;
      return SwitchDecision::kZeroOnly;
    }
    last_direction_ = direction;
    return SwitchDecision::kApplyNow;
  }

  // 该侧已归零（coast/brake/非法请求）⇒ 清空方向记忆，下一次 drive 可立即施加。
  void OnZeroOutput() {
    last_direction_ = control::MotorDirection::kNone;
    pending_direction_ = control::MotorDirection::kNone;
    pending_ = false;
  }

  control::MotorDirection last_direction() const { return last_direction_; }
  bool pending() const { return pending_; }
  control::MotorDirection pending_direction() const { return pending_direction_; }
  uint32_t zero_windows() const { return zero_windows_; }

 private:
  control::MotorDirection last_direction_ = control::MotorDirection::kNone;
  control::MotorDirection pending_direction_ = control::MotorDirection::kNone;
  bool pending_ = false;
  uint32_t zero_windows_ = 0;
};

}  // namespace robotcar01::platform

#endif  // ROBOTCAR01_PLATFORM_STM32_PWM_SWITCH_GUARD_H_
