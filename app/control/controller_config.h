// 轮速控制器配置（Iteration 006）
//
// 全部为注入式占位非标定参数（arch §8.3/A-004）；IsValid() 为 14 条 fail-closed 校验（详设 §5.1）。
// 阈值下限与 004 测量底噪绑定（measurement_floor_mps = EncoderConfig.low_speed_zero_threshold_mps，D-006-7）。

#ifndef ROBOTCAR01_APP_CONTROL_CONTROLLER_CONFIG_H_
#define ROBOTCAR01_APP_CONTROL_CONTROLLER_CONFIG_H_

#include <cstdint>

namespace robotcar01::control {

struct WheelControllerConfig {
  float kff_permille_per_mps = 1600.0f;
  float kp_permille_per_mps = 800.0f;
  float ki_permille_per_mps_s = 1500.0f;
  float integral_limit_permille = 350.0f;
  uint16_t max_duty_permille = 900;
  uint16_t min_effective_duty_permille = 80;
  uint16_t soft_start_step_permille = 100;
  float measurement_floor_mps = 0.05f;
  float zero_target_threshold_mps = 0.05f;
  float stop_speed_threshold_mps = 0.05f;
  uint32_t stop_dwell_us = 25000;
  uint32_t stop_timeout_us = 1500000;
  uint32_t max_measurement_age_us = 25000;
  float max_target_speed_mps = 0.5f;
  float max_measured_speed_mps = 1.0f;
  uint32_t max_period_us = 20000;
  // 停滞起步（D-006-2 实现期修订，见详设 §13）：停滞候选期间积分只能爬到该上限（起步裕量），
  // 且停滞持续超过 stall_fault_timeout_us ⇒ 停滞故障（零输出 + fault，目标归零后清除）。
  float stall_integral_cap_permille = 150.0f;
  uint32_t stall_fault_timeout_us = 1000000;

  // 单一真源：控制周期；TaskSet 注册周期由它派生（详设 §7，评审 H2）。
  static constexpr uint32_t kNominalPeriodUs = 5000;
  // 004 最坏发布间隔（低速累加 3 窗口）：停止确认 dwell 必须大于它。
  static constexpr uint32_t kEncoderWorstPublishIntervalUs = 15000;

  bool IsValid() const;
};

}  // namespace robotcar01::control

#endif  // ROBOTCAR01_APP_CONTROL_CONTROLLER_CONFIG_H_
