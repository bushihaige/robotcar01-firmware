#include "app/kinematics/kinematics_config.h"

#include <cmath>

namespace robotcar01::kinematics {

namespace {

bool IsPositiveFinite(float value) { return std::isfinite(value) && value > 0.0f; }

}  // namespace

bool KinematicsConfig::IsValid() const {
  // 1~4：几何与域上限（Fail-closed：0/负值/NaN/Inf 全部拒绝）。
  if (!IsPositiveFinite(drive.track_width_m)) {
    return false;
  }
  if (!IsPositiveFinite(drive.max_body_speed_mps)) {
    return false;
  }
  if (!IsPositiveFinite(drive.max_yaw_rate_radps)) {
    return false;
  }
  if (!IsPositiveFinite(drive.max_wheel_speed_mps)) {
    return false;
  }

  // 5：四个加减速度约束必须为正有限值（0/负值会让斜坡永远无法推进或反向）。
  if (!IsPositiveFinite(max_body_accel_mps2) || !IsPositiveFinite(max_body_decel_mps2) ||
      !IsPositiveFinite(max_yaw_accel_radps2) || !IsPositiveFinite(max_yaw_decel_radps2)) {
    return false;
  }

  // 6：降级上限必须为正且不超过正常轮速上限（否则"降级"反而放大目标，P1 语义矛盾）。
  if (!IsPositiveFinite(degraded_max_wheel_speed_mps) ||
      degraded_max_wheel_speed_mps > drive.max_wheel_speed_mps) {
    return false;
  }

  // 7：ω 上限可达性 —— 角速度上限对应的轮速不得超过轮速上限，否则配置自相矛盾
  //    （车体域永不绑定，实际角速度上限由轮速域决定，属于配置错误而非运行时策略）。
  const float yaw_wheel_speed = drive.max_yaw_rate_radps * drive.track_width_m / 2.0f;
  if (!std::isfinite(yaw_wheel_speed) || yaw_wheel_speed > drive.max_wheel_speed_mps) {
    return false;
  }

  // 8：dt 上界必须至少覆盖一个 1 ms 调度 tick（arch §5.3）。
  if (max_update_interval_us < 1000u) {
    return false;
  }

  // 9：降级上限必须**严格小于**车体线速度上限，否则"降级"在直线行驶时永不生效
  //    （配置合法但功能静默失效 = fail-open；评审 H4-1）。
  if (degraded_max_wheel_speed_mps >= drive.max_body_speed_mps) {
    return false;
  }

  // 10：单周期斜坡可达性上界（评审 H4-4/H2-③）：dt 被钳制到上界时，单周期最大变化量不得超过
  //     该域自身的上限，否则斜坡在最坏 dt 下失去平滑意义（"已知上界"退化为无界）。
  const float dt_bound_s = static_cast<float>(max_update_interval_us) / 1'000'000.0f;
  if (max_body_accel_mps2 * dt_bound_s > drive.max_body_speed_mps) {
    return false;
  }
  if (max_yaw_accel_radps2 * dt_bound_s > drive.max_yaw_rate_radps) {
    return false;
  }

  // 11：轮距下界（评审 H4-5）：差分测速 ω = (right − left)/T 会把量化噪声放大 1/T 倍；
  //     小于 5 cm 的"轮距"不构成差速底盘（比 100 mm 车轮直径还小）。
  //     不设上界：不同平台可有更大轮距，凭空给上界会把未标定假设写成硬约束（arch A-004）。
  if (drive.track_width_m < 0.05f) {
    return false;
  }

  // 说明（评审 H4 的 triage 结论）：**不**把以下"冗余但安全"的形态设为 fail-closed：
  //   - max_wheel_speed_mps < max_body_speed_mps：直线 v_max 由轮速域统一比例收紧，结果安全，
  //     只是车体上限冗余（001 的默认测试配置即为此形态）；
  //   - track_width_m / 加速度的"物理合理范围"：需要实测标定值，属 006.5/008（arch A-004），
  //     凭空给上下界会把未标定假设写成硬约束。
  // fail-closed 只拒绝"自相矛盾或量纲错误"的配置（第 1~8 条）。
  return true;
}

}  // namespace robotcar01::kinematics
