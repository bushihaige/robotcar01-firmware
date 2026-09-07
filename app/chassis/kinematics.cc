#include "app/chassis/kinematics.h"

#include <algorithm>
#include <cmath>

namespace robotcar01::chassis {

namespace {

// 计算车体 v/ω 域的统一候选缩放（仅对超限分量参与，未超限返回 1）。
float BodyDomainScale(const RobotCarConfig& config, const MotionCommand& command) {
  float scale = 1.0f;
  const float abs_v = std::fabs(command.v_mps);
  const float abs_omega = std::fabs(command.omega_radps);
  if (abs_v > config.max_body_speed_mps) {
    scale = std::min(scale, config.max_body_speed_mps / abs_v);
  }
  if (abs_omega > config.max_yaw_rate_radps) {
    scale = std::min(scale, config.max_yaw_rate_radps / abs_omega);
  }
  return scale;
}

// 计算轮速域的候选缩放（两轮任一超 max_wheel_speed 时参与，否则返回 1）。
float WheelDomainScale(const RobotCarConfig& config, float max_raw_mps) {
  if (max_raw_mps > config.max_wheel_speed_mps) {
    return config.max_wheel_speed_mps / max_raw_mps;
  }
  return 1.0f;
}

}  // namespace

WheelTarget ComputeWheelTarget(const RobotCarConfig& config,
                               const MotionCommand& command) {
  WheelTarget target;
  if (!command.valid || !std::isfinite(command.v_mps) ||
      !std::isfinite(command.omega_radps)) {
    target.scale = 0.0f;
    target.limit_reason = LimitReason::kInvalid;
    target.valid = false;
    return target;
  }

  // 差速换算（前进为正）。
  const float raw_left_mps =
      command.v_mps - command.omega_radps * config.track_width_m / 2.0f;
  const float raw_right_mps =
      command.v_mps + command.omega_radps * config.track_width_m / 2.0f;
  const float max_raw_mps = std::max(std::fabs(raw_left_mps), std::fabs(raw_right_mps));

  // 单一统一比例缩放（保曲率）。
  const float body_scale = BodyDomainScale(config, command);
  const float wheel_scale = WheelDomainScale(config, max_raw_mps);
  const float scale = std::min({1.0f, body_scale, wheel_scale});

  // reason 置位：凡"超限且候选等于最终 scale"的域置位。
  LimitReason reason = LimitReason::kNone;
  const bool body_over =
      std::fabs(command.v_mps) > config.max_body_speed_mps ||
      std::fabs(command.omega_radps) > config.max_yaw_rate_radps;
  if (body_over && body_scale <= wheel_scale) {
    reason |= LimitReason::kBodySpeed;
  }
  const bool wheel_over = max_raw_mps > config.max_wheel_speed_mps;
  if (wheel_over && wheel_scale <= body_scale) {
    reason |= LimitReason::kWheelSpeed;
  }

  target.raw_left_mps = raw_left_mps;
  target.raw_right_mps = raw_right_mps;
  target.left_mps = raw_left_mps * scale;
  target.right_mps = raw_right_mps * scale;
  target.scale = scale;
  target.limit_reason = reason;
  target.valid = true;
  return target;
}

}  // namespace robotcar01::chassis
