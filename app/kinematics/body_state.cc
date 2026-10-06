#include "app/kinematics/body_state.h"

#include <cmath>

namespace robotcar01::kinematics {

BodyVelocityEstimate EstimateBodyVelocity(float track_width_m, const encoder::WheelState& state) {
  BodyVelocityEstimate estimate{};
  estimate.left_valid = state.left.valid;
  estimate.right_valid = state.right.valid;
  estimate.left_quality = state.left.quality;
  estimate.right_quality = state.right.quality;
  estimate.quality = state.left.quality | state.right.quality;

  // 几何非法（未校验配置/被清零）时不发布数值：避免除零产生 Inf/NaN 传播到反馈。
  if (!std::isfinite(track_width_m) || track_width_m <= 0.0f) {
    return estimate;
  }
  if (!IsMeasurementTrustworthy(estimate)) {
    return estimate;  // 质量不可信：数值域保持 0
  }
  if (!std::isfinite(state.left.speed_mps) || !std::isfinite(state.right.speed_mps)) {
    return estimate;
  }

  estimate.v_mps = 0.5f * (state.left.speed_mps + state.right.speed_mps);
  estimate.omega_radps = (state.right.speed_mps - state.left.speed_mps) / track_width_m;
  estimate.valid = estimate.left_valid && estimate.right_valid;
  // 三态：两侧都 valid ⇒ 本窗口新值；否则为低速累加中的"上次发布值"（004 D-004-9）。
  estimate.state = estimate.valid ? MeasurementState::kFresh : MeasurementState::kStale;
  return estimate;
}

bool IsMeasurementTrustworthy(const BodyVelocityEstimate& estimate) {
  return (estimate.quality & encoder::kQualityBlockingMask) == 0u;
}

}  // namespace robotcar01::kinematics
