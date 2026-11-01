// 单轮速度控制器（Iteration 006，robotcar01::control）
//
// 门序（详设 §6，顺序固定）：
//   ① 门控：未初始化 / 目标非有限 ⇒ 故障零；allow_motion=false ⇒ 安全零（方向锁存保留）
//   ② 测量有效性：!IsTrustworthyForControl ∨ 非有限 ∨ 超速 ⇒ 零输出、清积分、dwell 复位
//   ③ 测量新鲜度：now − measured_at > max_measurement_age ⇒ 零输出、清积分、dwell 复位
//   ④ 目标归一：|target| < zero_target_threshold ⇒ 零目标（0<|t| 时附 kOutputBelowFloor）
//   ⑤ 状态机：IDLE / DRIVE(d) / STOPPING(d)；方向锁存只在"测量可信+新鲜且 |v|<阈值 持续 ≥ dwell"后释放
//   ⑥ DRIVE：幅值域 FF+PI，条件积分；停滞候选冻结积分；从零恢复单向软启动
// 不变量：INV-006-1/2/3/8（详设 §10）。无动态内存、无异常、固定计算量、不引用厂商外设头。

#ifndef ROBOTCAR01_APP_CONTROL_WHEEL_SPEED_CONTROLLER_H_
#define ROBOTCAR01_APP_CONTROL_WHEEL_SPEED_CONTROLLER_H_

#include <cstdint>

#include "app/control/controller_config.h"
#include "app/control/motor_output.h"
#include "app/encoder/wheel_state.h"

namespace robotcar01::control {

enum class ControlState : uint8_t { kIdle = 0, kDrive = 1, kStopping = 2 };

struct WheelControlStats {
  uint16_t last_duty_permille = 0;
  uint8_t last_mode = 0;
  uint8_t applied_direction = 0;  // 0 none / 1 forward / 2 reverse
  uint32_t last_reason = 0;
  uint32_t updates = 0;
  uint32_t drive_cycles = 0;
  uint32_t stopping_entries = 0;
  uint32_t stop_timeouts = 0;
  uint32_t measurement_invalid_cycles = 0;
  uint32_t measurement_stale_cycles = 0;
  uint32_t integrator_frozen_cycles = 0;
  uint32_t below_floor_cycles = 0;
  uint32_t saturated_cycles = 0;
  uint32_t direction_changes = 0;
  uint32_t stall_faults = 0;
};

class WheelSpeedController {
 public:
  bool Init(const WheelControllerConfig& config);
  bool initialized() const { return initialized_; }
  void Reset();  // 视为静止：清积分、锁存与时间基线

  // target_mps 来自 005；measured 来自 004 单侧；measured_at_us = WheelState.timestamp.us。
  MotorOutputRequest Update(float target_mps, const encoder::WheelSideState& measured,
                            uint64_t measured_at_us, bool allow_motion, uint64_t now_us);

  float integral_permille() const { return integral_; }
  ControlState state() const { return state_; }
  int applied_direction() const { return applied_dir_; }
  const WheelControlStats& stats() const { return stats_; }

 private:
  MotorOutputRequest Finish(const MotorOutputRequest& request);
  void EnterStopping();
  void LeaveDrive();

  WheelControllerConfig config_{};
  bool initialized_ = false;
  bool has_time_base_ = false;
  uint64_t last_update_us_ = 0;
  ControlState state_ = ControlState::kIdle;
  int applied_dir_ = 0;          // +1 / -1 / 0
  float integral_ = 0.0f;
  uint16_t prev_duty_ = 0;
  bool soft_start_active_ = false;
  uint32_t stop_dwell_accum_us_ = 0;
  uint32_t stopping_elapsed_us_ = 0;
  uint32_t stall_elapsed_us_ = 0;
  bool stall_fault_ = false;
  int last_drive_dir_ = 0;       // 上一次以非零 duty 驱动的方向（INV-006-2 统计用）
  WheelControlStats stats_{};
};

}  // namespace robotcar01::control

#endif  // ROBOTCAR01_APP_CONTROL_WHEEL_SPEED_CONTROLLER_H_
