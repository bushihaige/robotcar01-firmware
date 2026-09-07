#include "app/chassis/safety_gate.h"

namespace robotcar01::chassis {

WheelSpeedRequest RunSafetyGate(const WheelTarget& target,
                                const SafetyStatus& safety) {
  WheelSpeedRequest request;
  const bool gate_open = safety.allow_motion && target.valid;
  request.force_disable = !gate_open;
  request.limit_reason = target.limit_reason;
  if (!safety.allow_motion) {
    request.limit_reason |= LimitReason::kSafety;
  }
  if (gate_open) {
    request.left_mps = target.left_mps;
    request.right_mps = target.right_mps;
  }
  // gate 关闭（禁止或无效）时 left/right 保持默认 0，force_disable=true。
  return request;
}

}  // namespace robotcar01::chassis
