#include "app/control/controller_config.h"

#include <cmath>

namespace robotcar01::control {

namespace {
bool NonNegativeFinite(float value) { return std::isfinite(value) && value >= 0.0f; }
bool PositiveFinite(float value) { return std::isfinite(value) && value > 0.0f; }
}  // namespace

bool WheelControllerConfig::IsValid() const {
  // 1. 增益非负有限（允许 kff = 0 纯 PI）
  if (!NonNegativeFinite(kff_permille_per_mps) || !NonNegativeFinite(kp_permille_per_mps) ||
      !NonNegativeFinite(ki_permille_per_mps_s)) {
    return false;
  }
  // 2. 0 ≤ integral_limit ≤ max_duty
  if (!NonNegativeFinite(integral_limit_permille) ||
      integral_limit_permille > static_cast<float>(max_duty_permille)) {
    return false;
  }
  // 3. 0 < min_effective < max_duty ≤ 1000
  if (min_effective_duty_permille == 0u || min_effective_duty_permille >= max_duty_permille ||
      max_duty_permille > 1000u) {
    return false;
  }
  // 4. min_effective ≤ soft_start_step ≤ max_duty
  if (soft_start_step_permille < min_effective_duty_permille ||
      soft_start_step_permille > max_duty_permille) {
    return false;
  }
  // 5. 测量底噪正有限
  if (!PositiveFinite(measurement_floor_mps)) {
    return false;
  }
  // 6/7. 阈值 ≥ 测量底噪（C2）
  if (!std::isfinite(zero_target_threshold_mps) || zero_target_threshold_mps < measurement_floor_mps) {
    return false;
  }
  if (!std::isfinite(stop_speed_threshold_mps) || stop_speed_threshold_mps < measurement_floor_mps) {
    return false;
  }
  // 8. stop_dwell ≥ 2 周期 且 > 004 最坏发布间隔
  if (stop_dwell_us < 2u * kNominalPeriodUs || stop_dwell_us <= kEncoderWorstPublishIntervalUs) {
    return false;
  }
  // 9. 测量龄上限 > 周期 且 ≥ 20 ms（覆盖低速累加的陈旧度）
  if (max_measurement_age_us <= kNominalPeriodUs || max_measurement_age_us < 20000u) {
    return false;
  }
  // 10. dt 上限 > 周期
  if (max_period_us <= kNominalPeriodUs) {
    return false;
  }
  // 11. 0 < max_target < max_measured
  if (!PositiveFinite(max_target_speed_mps) || !PositiveFinite(max_measured_speed_mps) ||
      max_target_speed_mps >= max_measured_speed_mps) {
    return false;
  }
  // 12. 前馈不得单独饱和
  if (kff_permille_per_mps * max_target_speed_mps > static_cast<float>(max_duty_permille)) {
    return false;
  }
  // 13. stop_timeout > stop_dwell
  if (stop_timeout_us <= stop_dwell_us) {
    return false;
  }
  // 14. 积分足以爬过补偿以下死区
  if (integral_limit_permille < static_cast<float>(min_effective_duty_permille)) {
    return false;
  }
  // 15. 停滞起步上限：0 ≤ cap ≤ integral_limit
  if (!NonNegativeFinite(stall_integral_cap_permille) ||
      stall_integral_cap_permille > integral_limit_permille) {
    return false;
  }
  // 16. 停滞故障超时 > 停止 dwell（且必须 > 0）
  if (stall_fault_timeout_us <= stop_dwell_us) {
    return false;
  }
  return true;
}

}  // namespace robotcar01::control
